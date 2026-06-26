// Copyright 2026. All Rights Reserved.
//
// @file ar_signal_pipeline_test.cpp
// @brief 验证信号处理流水线与内部配置的基础行为。

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <memory>
#include <vector>

#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "airborne_radar/config/InternalExecutionConfig.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"
#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace tests {

namespace {

using ExecutionConfig = config::execution::InternalExecutionConfig;

config::RadarSessionConfig MakeDetectionFocusedConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(config::profiles::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

config::RadarSessionConfig MakeTrackingFocusedConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kTrackStabilityPriority)
      .End()
      .Build();
}

config::RadarSessionConfig MakeRobustTrackingConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kTrackStabilityPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(config::profiles::TrackingPolicyProfile::kRobustAntiJamming)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kHighPersistence)
      .End()
      .Build();
}

float SpeedOf(const session::RadarSceneTarget& target) {
  return std::sqrt(target.velocity_x * target.velocity_x + target.velocity_y * target.velocity_y +
                   target.velocity_z * target.velocity_z);
}

session::RadarSceneTarget BuildPhysicsTarget(float range_m, float rcs) {
  session::RadarSceneTarget target(220.0f, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f);

  target.position_x = range_m;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = range_m;
  return target;
}

session::RadarSceneTarget ToSceneTarget(const session::RadarSceneTarget& target) {
  session::RadarSceneTarget out;
  out.external_target_id = target.external_target_id;
  out.velocity_x = target.velocity_x;
  out.velocity_y = target.velocity_y;
  out.velocity_z = target.velocity_z;
  out.rcs = target.rcs;
  out.range_m = target.range_m;
  out.position_x = target.position_x;
  out.position_y = target.position_y;
  out.position_z = target.position_z;
  out.target_swerling_type = target.target_swerling_type;
  return out;
}

session::RadarSceneTargetList ToSceneTargets(const session::RadarSceneTargetList& targets) {
  session::RadarSceneTargetList out;
  out.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    out.push_back(ToSceneTarget(targets[i]));
  }
  return out;
}

session::RadarSceneTarget CloneSceneTarget(const session::RadarSceneTarget& target) {
  session::RadarSceneTarget out;
  out.external_target_id = target.external_target_id;
  out.velocity_x = target.velocity_x;
  out.velocity_y = target.velocity_y;
  out.velocity_z = target.velocity_z;
  out.rcs = target.rcs;
  out.range_m = target.range_m;

  out.position_x = target.position_x;
  out.position_y = target.position_y;
  out.position_z = target.position_z;
  out.target_swerling_type = target.target_swerling_type;
  return out;
}

session::RadarSceneTargetList CloneSceneTargets(const session::RadarSceneTargetList& targets) {
  session::RadarSceneTargetList out;
  out.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    out.push_back(CloneSceneTarget(targets[i]));
  }
  return out;
}

signal::tracking::CycleContext MakeLifecycleCycle(std::uint32_t cycle_index,
                                                  std::uint64_t batch_id) {
  signal::tracking::CycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = 1.0f;
  cycle.extra_miss_tolerance = 0u;
  return cycle;
}

session::EnvironmentCycleContext MakeEnvironmentCycle(std::uint32_t cycle_index) {
  session::EnvironmentCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.dt_sec = 1.0f;
  return cycle;
}

template <typename PipelineType>
session::SignalCycleResult RunPipelineCycle(PipelineType* pipeline,
                                              const session::RadarSceneTargetList& input_state,
                                              environment::EnvironmentService* environment_service,
                                              std::uint32_t cycle_index = 1u) {
  environment_service->BeginCycle(MakeEnvironmentCycle(cycle_index));
  return pipeline->RunCycle(ToSceneTargets(input_state), *environment_service);
}

config::JammerEmitterState MakeJammerEmitter(config::JammingTechnique technique,
                                                  float power_db) {
  config::JammerEmitterState jammer;
  jammer.technique = technique;
  jammer.power_db = power_db;
  jammer.confidence = 1.0f;
  return jammer;
}

config::EnvironmentModelConfig MakeEnvironmentConfigWithJammers(
    std::initializer_list<config::JammerEmitterState> jammer_sources) {
  config::EnvironmentModelConfig config;
  config.jammer_sources.insert(config.jammer_sources.end(), jammer_sources.begin(),
                               jammer_sources.end());
  return config;
}

void ApplyHardwareProfile(config::RadarSessionConfig* config,
                          config::profiles::RadarHardwareProfile profile) {
  if (config == nullptr) {
    return;
  }
  auto& d = config->hardware;
  switch (profile) {
    case config::profiles::RadarHardwareProfile::kLongRangeHighPower:
      d.transmitter.peak_power_w = 5.0e6f;
      d.transmitter.frequency_hz = 9.3e9f;
      d.transmitter.bandwidth_hz = 3.0e6f;
      d.transmitter.pulse_width_s = 18e-6f;
      d.transmitter.prf_hz = 220.0f;
      d.antenna.main_beam_gain_db = 38.0f;
      d.receiver.noise_figure_db = 3.0f;
      break;
    case config::profiles::RadarHardwareProfile::kLightweightLpi:
      d.transmitter.peak_power_w = 3.5e5f;
      d.transmitter.frequency_hz = 10.0e9f;
      d.transmitter.bandwidth_hz = 8.0e6f;
      d.transmitter.pulse_width_s = 8e-6f;
      d.transmitter.prf_hz = 600.0f;
      d.antenna.main_beam_gain_db = 31.0f;
      d.antenna.nominal_az_beamwidth_deg = 5.0f;
      d.antenna.nominal_el_beamwidth_deg = 5.0f;
      d.receiver.noise_figure_db = 5.0f;
      break;
    case config::profiles::RadarHardwareProfile::kGenericAirborneXBand:
    default:
      break;
  }
}

void ApplyDetectionIntentProfile(config::RadarSessionConfig* config,
                                 config::profiles::DetectionIntentProfile profile) {
  if (config == nullptr) {
    return;
  }
  auto& d = config->hardware;
  switch (profile) {
    case config::profiles::DetectionIntentProfile::kDetectionPriority:
      d.pulse_count = 16;
      d.detection_policy.cfar_pfa = 2e-6f;
      d.detection_policy.min_snr_db = -12.0f;
      d.min_detection_margin_db = -100.0f;
      break;
    case config::profiles::DetectionIntentProfile::kTrackStabilityPriority:
      d.pulse_count = 8;
      d.detection_policy.cfar_pfa = 5e-7f;
      d.detection_policy.min_snr_db = -8.0f;
      d.min_detection_margin_db = -20.0f;
      break;
    case config::profiles::DetectionIntentProfile::kBalanced:
    default:
      d.min_detection_margin_db = -2.0f;
      break;
  }
}

void ApplyRcsFusionProfile(config::RadarSessionConfig* config,
                           config::profiles::RcsFusionProfile profile) {
  if (config == nullptr) {
    return;
  }
  auto& rcs = config->hardware.rcs_physics;
  switch (profile) {
    case config::profiles::RcsFusionProfile::kConservative:
      rcs.enable_physical_rcs = true;
      rcs.physics_mix_ratio = 0.25f;
      break;
    case config::profiles::RcsFusionProfile::kEnhanced:
      rcs.enable_physical_rcs = true;
      rcs.physics_mix_ratio = 0.60f;
      rcs.cylinder_weight = 0.65f;
      break;
    case config::profiles::RcsFusionProfile::kDisabled:
    default:
      rcs.enable_physical_rcs = false;
      rcs.physics_mix_ratio = 0.0f;
      break;
  }
}

void ApplyTrackingPolicyProfile(config::RadarSessionConfig* config,
                                config::profiles::TrackingPolicyProfile profile) {
  if (config == nullptr) {
    return;
  }
  auto& t = config->policy.tracking;
  switch (profile) {
    case config::profiles::TrackingPolicyProfile::kFastAssociation:
      t.kalman_measurement_noise_std = 6.0f;
      t.kalman_update_backend = config::KalmanUpdateBackend::kStandardKfJoseph;
      t.speed_decay_ratio_on_loss = 0.95f;
      t.rcs_decay_ratio_on_loss = 0.92f;
      break;
    case config::profiles::TrackingPolicyProfile::kRobustAntiJamming:
      t.kalman_measurement_noise_std = 12.0f;
      t.kalman_update_backend = config::KalmanUpdateBackend::kUdKf;
      t.speed_decay_ratio_on_loss = 0.95f;
      t.rcs_decay_ratio_on_loss = 0.92f;
      config->policy.association.unassigned_cost = 12.0f;
      break;
    case config::profiles::TrackingPolicyProfile::kBalanced:
    default:
      break;
  }
}

void ApplyLifecyclePolicyProfile(config::RadarSessionConfig* config,
                                 config::profiles::LifecyclePolicyProfile profile) {
  if (config == nullptr) {
    return;
  }
  auto& lc = config->policy.lifecycle;
  switch (profile) {
    case config::profiles::LifecyclePolicyProfile::kFastConfirm:
      lc.confirm_hits = 1U;
      lc.max_miss_before_lost = 1U;
      lc.max_lost_cycles = 3U;
      break;
    case config::profiles::LifecyclePolicyProfile::kHighPersistence:
      lc.confirm_hits = 3U;
      lc.max_miss_before_lost = 3U;
      lc.max_lost_cycles = 8U;
      break;
    case config::profiles::LifecyclePolicyProfile::kBalanced:
    default:
      break;
  }
}

}  // namespace

TEST(SignalPipelineTest, KeepsTrackStableWhenDetectionMarginIsEnough) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  session::RadarSceneTarget target(800.0f, 0.0f, 0.0f, 2.5f, 0.0f, 0.0f, 0.0f);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const session::RadarSceneTargetList input_state{target};

  const auto output_state = CloneSceneTargets(
      RunPipelineCycle(&signal_pipeline, input_state, &environment_service).updated_scene_targets);

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_FLOAT_EQ(SpeedOf(output_state[0]), SpeedOf(input_state[0]));
  EXPECT_FLOAT_EQ(output_state[0].rcs, input_state[0].rcs);
}

TEST(SignalPipelineTest, DegradesTrackWhenDetectionMarginIsTooLow) {
  config::EnvironmentModelConfig env_config;
  env_config.jammer_sources.push_back(
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 40.0f));
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  const session::RadarSceneTargetList input_state{
      session::RadarSceneTarget(800.0f, 0.0f, 0.0f, 2.5f)};

  const auto output_state = CloneSceneTargets(
      RunPipelineCycle(&signal_pipeline, input_state, &environment_service).updated_scene_targets);

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_LE(SpeedOf(output_state[0]), SpeedOf(input_state[0]));
  EXPECT_LE(output_state[0].rcs, input_state[0].rcs);
}

TEST(SignalPipelineTest,
     RcsPhysicsOverrideChangesMarginalHeuristicDetectionWhileDisabledPathStaysSame) {
  config::RadarSessionConfig baseline_config;
  baseline_config.hardware.enable_physics_detection = false;
  ApplyDetectionIntentProfile(&baseline_config,
                              config::profiles::DetectionIntentProfile::kBalanced);
  ApplyHardwareProfile(&baseline_config,
                       config::profiles::RadarHardwareProfile::kLongRangeHighPower);

  config::EnvironmentModelConfig env_config;

  const session::RadarSceneTargetList input_state{BuildPhysicsTarget(4500.0f, 0.2f)};

  environment::EnvironmentService baseline_environment(env_config);
  signal::pipeline::SignalPipeline baseline_pipeline(baseline_config);
  const session::SignalCycleResult baseline_result =
      RunPipelineCycle(&baseline_pipeline, input_state, &baseline_environment);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_result.updated_scene_targets.size(), 1u);
  EXPECT_FALSE(baseline_measurements.empty());

  config::RadarSessionConfig disabled_override_config = baseline_config;
  ApplyRcsFusionProfile(&disabled_override_config, config::profiles::RcsFusionProfile::kDisabled);

  environment::EnvironmentService disabled_environment(env_config);
  signal::pipeline::SignalPipeline disabled_pipeline(disabled_override_config);
  const session::SignalCycleResult disabled_result =
      RunPipelineCycle(&disabled_pipeline, input_state, &disabled_environment);
  const auto disabled_measurements = disabled_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(disabled_result.updated_scene_targets.size(), 1u);
  EXPECT_FALSE(disabled_measurements.empty());
  EXPECT_FLOAT_EQ(disabled_result.updated_scene_targets[0].rcs,
                  baseline_result.updated_scene_targets[0].rcs);

  config::RadarSessionConfig enabled_override_config = disabled_override_config;
  ApplyRcsFusionProfile(&enabled_override_config, config::profiles::RcsFusionProfile::kEnhanced);

  environment::EnvironmentService enabled_environment(env_config);
  signal::pipeline::SignalPipeline enabled_pipeline(enabled_override_config);
  RunPipelineCycle(&enabled_pipeline, input_state, &enabled_environment);
  const auto enabled_measurements = enabled_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(enabled_measurements.size(), 1u);
  EXPECT_GT(enabled_measurements[0].raw_measurement.detection_margin_db, -2.0f);
}

TEST(SignalPipelineTest, ExposesPublicPlatformAttitudeUpdateApi) {
  signal::pipeline::SignalPipeline signal_pipeline;
  model::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 12.0f;
  platform_attitude_deg.pitch_deg = -3.0f;
  platform_attitude_deg.roll_deg = 1.5f;

  signal_pipeline.UpdatePlatformAttitude(platform_attitude_deg);

  const model::PlatformAttitudeDeg cached_platform_attitude = signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 12.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -3.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 1.5f);
}

TEST(SignalPipelineTest, ExposesPublicPlatformAltitudeUpdateApi) {
  signal::pipeline::SignalPipeline signal_pipeline;

  signal_pipeline.UpdatePlatformAltitudeM(1200.0f);

  EXPECT_FLOAT_EQ(signal_pipeline.GetPlatformAltitudeM(), 1200.0f);
}

TEST(SignalPipelineTest, UsesEnvironmentCycleIndexForExecutionContract) {
  signal::pipeline::SignalPipeline signal_pipeline;
  environment::EnvironmentService environment_service;
  session::RadarSceneTarget target = BuildPhysicsTarget(1500.0f, 4.0f);
  target.external_target_id = 42U;

  const session::SignalCycleResult result = RunPipelineCycle(
      &signal_pipeline, session::RadarSceneTargetList{target}, &environment_service, 77U);

  ASSERT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.decision_frame.cycle_index, 77U);
}

TEST(SignalPipelineTest, RestoreRuntimeStatePreservesLifecycleTracks) {
  signal::pipeline::SignalPipeline signal_pipeline;
  environment::EnvironmentService environment_service;
  session::RadarSceneTarget target = BuildPhysicsTarget(1500.0f, 4.0f);
  target.external_target_id = 43U;

  const session::SignalCycleResult baseline = RunPipelineCycle(
      &signal_pipeline, session::RadarSceneTargetList{target}, &environment_service, 1U);
  ASSERT_TRUE(baseline.executed_this_cycle);
  ASSERT_FALSE(baseline.decision_frame.tracks.empty());

  const extension::SignalPipelineRuntimeState snapshot = signal_pipeline.CaptureRuntimeState();

  const session::SignalCycleResult missed_once =
      RunPipelineCycle(&signal_pipeline, session::RadarSceneTargetList(), &environment_service, 2U);
  ASSERT_TRUE(missed_once.executed_this_cycle);
  ASSERT_FALSE(missed_once.decision_frame.tracks.empty());
  EXPECT_EQ(missed_once.decision_frame.tracks[0].miss_count, 1U);

  signal_pipeline.RestoreRuntimeState(snapshot);

  const session::SignalCycleResult missed_after_restore =
      RunPipelineCycle(&signal_pipeline, session::RadarSceneTargetList(), &environment_service, 2U);
  ASSERT_TRUE(missed_after_restore.executed_this_cycle);
  ASSERT_FALSE(missed_after_restore.decision_frame.tracks.empty());
  EXPECT_EQ(missed_after_restore.decision_frame.tracks[0].miss_count, 1U);
}

TEST(SignalPipelineTest, AutoLifecycleManagerBuildsWithDefaultInternalImmConfig) {
  config::RadarSessionConfig session_runtime_config;
  session_runtime_config.policy.lifecycle.enable_imm_lifecycle = true;
  const ExecutionConfig exec_config =
      config::mapping::MapSessionToExecution(session_runtime_config);

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);

  const signal::tracking::CycleContext cycle = MakeLifecycleCycle(1u, 7u);
  const std::vector<signal::tracking::TrackMeasurement> measurements;
  lifecycle_manager->Update(cycle, measurements);

  model::DecisionInputFrame decision_frame(lifecycle_manager->BuildTrackStateSnapshots());
  decision_frame.cycle_index = 1u;
  decision_frame.batch_id = 7u;
  decision_frame.environment_jamming_detected = false;
  EXPECT_EQ(decision_frame.cycle_index, 1u);
  EXPECT_EQ(decision_frame.batch_id, 7u);
}

TEST(SignalPipelineTest, AutoLifecycleManagerCreationFailsWhenImmAssemblyIsInvalid) {
  config::RadarSessionConfig session_runtime_config;
  session_runtime_config.policy.tracking.enable_kalman_filter = true;
  session_runtime_config.policy.lifecycle.enable_imm_lifecycle = true;
  ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_runtime_config);
  exec_config.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};
  exec_config.lifecycle.imm_initial_weights = {1.0f};

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);

  EXPECT_EQ(lifecycle_manager, nullptr);
}

TEST(SignalPipelineTest, ControlProfilePowerReductionLowersPhysicalDetectionMargin) {
  config::RadarSessionConfig session_config;
  session_config.hardware.enable_physics_detection = true;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);

  environment::EnvironmentService environment_service;

  const session::RadarSceneTargetList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile reduced_power_profile;
  reduced_power_profile.enable_lpi_power_control = true;
  reduced_power_profile.lpi_power_scale = 0.20f;
  signal::pipeline::SignalPipeline reduced_pipeline(session_config);
  reduced_pipeline.SetControlProfile(reduced_power_profile);
  RunPipelineCycle(&reduced_pipeline, input_state, &environment_service);
  const auto reduced_measurements = reduced_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(reduced_measurements.size(), 1u);
  EXPECT_LT(reduced_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, AdaptiveBeamformingProfileTightensMeasurementCovariance) {
  config::RadarSessionConfig session_config;
  session_config.hardware.enable_physics_detection = true;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);
  session_config.hardware.antenna.pattern.model_type =
      config::AntennaPatternModelType::kParabolicMainLobe;
  session_config.hardware.antenna.pattern.max_sidelobe_level_db = -18.0f;
  session_config.hardware.antenna.pattern.max_scan_loss_db = 8.0f;

  environment::EnvironmentService environment_service;

  const session::RadarSceneTargetList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile adaptive_profile;
  adaptive_profile.enable_adaptive_beamforming = true;
  signal::pipeline::SignalPipeline adaptive_pipeline(session_config);
  adaptive_pipeline.SetControlProfile(adaptive_profile);
  RunPipelineCycle(&adaptive_pipeline, input_state, &environment_service);
  const auto adaptive_measurements = adaptive_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(adaptive_measurements.size(), 1u);
  EXPECT_LT(adaptive_measurements[0].raw_measurement.measurement_covariance(1, 1),
            baseline_measurements[0].raw_measurement.measurement_covariance(1, 1));
  EXPECT_LT(adaptive_measurements[0].raw_measurement.measurement_covariance(2, 2),
            baseline_measurements[0].raw_measurement.measurement_covariance(2, 2));
}

TEST(SignalPipelineTest, EccmProfileRelaxesHeuristicAssociationGateForSeededTracks) {
  config::RadarSessionConfig session_config;
  ApplyTrackingPolicyProfile(&session_config,
                             config::profiles::TrackingPolicyProfile::kFastAssociation);

  environment::EnvironmentService environment_service;

  session::RadarSceneTarget target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);

  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const session::RadarSceneTargetList input_state{target};

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 7u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f::Zero();
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Zero();

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  baseline_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline(session_config);
  protected_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  protected_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&protected_pipeline, input_state, &environment_service);
  const auto protected_measurements = protected_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(protected_measurements.size(), 1u);
  EXPECT_TRUE(baseline_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(protected_measurements[0].raw_measurement.matched_existing_track);
}

TEST(SignalPipelineTest, AssociationQualityMetricsExposeTypeSpecificStressSummary) {
  config::RadarSessionConfig session_config;
  ApplyTrackingPolicyProfile(&session_config,
                             config::profiles::TrackingPolicyProfile::kFastAssociation);

  session::RadarSceneTarget target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);

  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const session::RadarSceneTargetList input_state{target};

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 11u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f::Zero();
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Zero();

  config::EnvironmentModelConfig noise_env_config;
  config::JammerEmitterState noise_source;
  noise_source.technique = config::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 8.0f;
  noise_source.js_db = 8.0f;
  noise_source.has_direction_deg = true;
  noise_source.azimuth_deg = 24.0f;
  noise_source.elevation_deg = 7.0f;
  noise_source.angular_span_deg = 30.0f;
  noise_source.confidence = 1.0f;
  noise_env_config.jammer_sources.push_back(noise_source);
  environment::EnvironmentService noise_environment(noise_env_config);

  config::EnvironmentModelConfig deception_env_config;
  config::JammerEmitterState deception_source;
  deception_source.technique = config::JammingTechnique::kDeception;
  deception_source.power_db = 8.0f;
  deception_source.js_db = 8.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 2.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 1.0f;
  deception_env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService deception_environment(deception_env_config);

  signal::pipeline::SignalPipeline noise_pipeline(session_config);
  noise_pipeline.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  RunPipelineCycle(&noise_pipeline, input_state, &noise_environment);
  const session::AssociationQualityMetrics noise_metrics =
      noise_pipeline.GetLastAssociationQualityMetrics();

  signal::pipeline::SignalPipeline deception_pipeline(session_config);
  deception_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  RunPipelineCycle(&deception_pipeline, input_state, &deception_environment);
  const session::AssociationQualityMetrics deception_metrics =
      deception_pipeline.GetLastAssociationQualityMetrics();

  EXPECT_EQ(noise_metrics.dominant_jamming_semantic, model::JammingSemantic::kNoiseSuppression);
  EXPECT_EQ(deception_metrics.dominant_jamming_semantic, model::JammingSemantic::kDeception);
  EXPECT_GT(deception_metrics.jamming_severity, noise_metrics.jamming_severity);
  EXPECT_GT(noise_metrics.association_stress, 0.0f);
  EXPECT_GT(deception_metrics.association_stress, 0.0f);
}

TEST(SignalPipelineTest, DominantJammingSemanticReturnsMixedWhenSecondScoreCloseToNoiseBest) {
  extension::control::RadarControlProfile control_profile;
  session::EnvironmentSnapshot snapshot;
  snapshot.jamming_detected = true;

  session::JammerSourceFact noise_source;
  noise_source.technique = config::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 8.0f;
  noise_source.js_db = 8.0f;
  noise_source.in_sidelobe = false;
  noise_source.confidence = 1.0f;
  snapshot.jammer_sources.push_back(noise_source);

  session::JammerSourceFact deception_source;
  deception_source.technique = config::JammingTechnique::kDeception;
  deception_source.power_db = 8.0f;
  deception_source.js_db = 8.0f;
  deception_source.frequency_overlap_ratio = 0.10f;
  deception_source.prf_lock_risk = 0.10f;
  deception_source.confidence = 1.0f;
  snapshot.jammer_sources.push_back(deception_source);

  EXPECT_EQ(signal::pipeline::ResolveDominantJammingSemantic(control_profile, snapshot),
            model::JammingSemantic::kMixed);
}

TEST(SignalPipelineTest, DominantJammingSemanticStaysNoiseWhenSecondScoreBelowThreshold) {
  extension::control::RadarControlProfile control_profile;
  session::EnvironmentSnapshot snapshot;
  snapshot.jamming_detected = true;

  session::JammerSourceFact noise_source;
  noise_source.technique = config::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 8.0f;
  noise_source.js_db = 8.0f;
  noise_source.confidence = 1.0f;
  snapshot.jammer_sources.push_back(noise_source);

  session::JammerSourceFact deception_source;
  deception_source.technique = config::JammingTechnique::kDeception;
  deception_source.power_db = 0.0f;
  deception_source.js_db = 0.0f;
  deception_source.frequency_overlap_ratio = 0.0f;
  deception_source.prf_lock_risk = 0.0f;
  deception_source.confidence = 0.25f;
  snapshot.jammer_sources.push_back(deception_source);

  EXPECT_EQ(signal::pipeline::ResolveDominantJammingSemantic(control_profile, snapshot),
            model::JammingSemantic::kNoiseSuppression);
}

TEST(SignalPipelineTest, MatchedEccmLowersAssociationStressForDeceptionJamming) {
  config::RadarSessionConfig session_config;
  ApplyTrackingPolicyProfile(&session_config,
                             config::profiles::TrackingPolicyProfile::kFastAssociation);

  session::RadarSceneTarget target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);

  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const session::RadarSceneTargetList input_state{target};

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 12u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f::Zero();
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Zero();

  config::EnvironmentModelConfig deception_env_config;
  config::JammerEmitterState deception_source;
  deception_source.technique = config::JammingTechnique::kDeception;
  deception_source.power_db = 8.0f;
  deception_source.js_db = 8.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 3.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 1.0f;
  deception_env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService deception_environment(deception_env_config);

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  baseline_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  RunPipelineCycle(&baseline_pipeline, input_state, &deception_environment);
  const session::AssociationQualityMetrics baseline_metrics =
      baseline_pipeline.GetLastAssociationQualityMetrics();

  extension::control::RadarControlProfile protected_profile;
  protected_profile.enable_agility_frequency = true;
  protected_profile.enable_eccm_rejitter = true;
  signal::pipeline::SignalPipeline protected_pipeline(session_config);
  protected_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  protected_pipeline.SetControlProfile(protected_profile);
  RunPipelineCycle(&protected_pipeline, input_state, &deception_environment);
  const session::AssociationQualityMetrics protected_metrics =
      protected_pipeline.GetLastAssociationQualityMetrics();

  EXPECT_EQ(baseline_metrics.dominant_jamming_semantic, model::JammingSemantic::kDeception);
  EXPECT_EQ(protected_metrics.dominant_jamming_semantic, model::JammingSemantic::kDeception);
  EXPECT_LT(protected_metrics.jamming_severity, baseline_metrics.jamming_severity);
  EXPECT_LT(protected_metrics.association_stress, baseline_metrics.association_stress);
}

TEST(SignalPipelineTest, EccmProfileMitigatesJammingPenaltyInPhysicalDetection) {
  config::RadarSessionConfig session_config;
  session_config.hardware.enable_physics_detection = true;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);
  session_config.mission.orientation.scan_center_deg.az_deg = 0.0f;
  session_config.mission.orientation.scan_center_deg.el_deg = 0.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.az_min_deg = 0.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.az_max_deg = 0.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  session_config.mission.orientation.electronic_scan_limits_deg =
      session_config.mission.orientation.mechanical_scan_limits_deg;

  config::EnvironmentModelConfig env_config;
  env_config.jammer_sources.push_back(
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 12.0f));
  environment::EnvironmentService environment_service(env_config);

  const session::RadarSceneTargetList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline eccm_pipeline(session_config);
  eccm_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&eccm_pipeline, input_state, &environment_service);
  const auto eccm_measurements = eccm_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(eccm_measurements.size(), 1u);
  EXPECT_GT(eccm_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, DetailedJammingFactsModulatePhysicalEccmBenefit) {
  config::RadarSessionConfig session_config;
  session_config.hardware.enable_physics_detection = true;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);
  session_config.mission.orientation.scan_center_deg.az_deg = 0.0f;
  session_config.mission.orientation.scan_center_deg.el_deg = 0.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.az_min_deg = 0.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.az_max_deg = 0.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  session_config.mission.orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  session_config.mission.orientation.electronic_scan_limits_deg =
      session_config.mission.orientation.mechanical_scan_limits_deg;

  config::JammerEmitterState favorable_source =
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 12.0f);
  favorable_source.has_direction_deg = true;
  favorable_source.azimuth_deg = 28.0f;
  favorable_source.elevation_deg = 9.0f;
  favorable_source.angular_span_deg = 30.0f;
  environment::EnvironmentService favorable_environment(
      MakeEnvironmentConfigWithJammers({favorable_source}));

  config::JammerEmitterState unfavorable_source =
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 12.0f);
  unfavorable_source.has_direction_deg = true;
  unfavorable_source.azimuth_deg = 0.0f;
  unfavorable_source.elevation_deg = 0.0f;
  unfavorable_source.angular_span_deg = 5.0f;
  environment::EnvironmentService unfavorable_environment(
      MakeEnvironmentConfigWithJammers({unfavorable_source}));

  const session::RadarSceneTargetList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_adaptive_beamforming = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;

  signal::pipeline::SignalPipeline favorable_pipeline(session_config);
  favorable_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&favorable_pipeline, input_state, &favorable_environment);
  const auto favorable_measurements = favorable_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline unfavorable_pipeline(session_config);
  unfavorable_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&unfavorable_pipeline, input_state, &unfavorable_environment);
  const auto unfavorable_measurements = unfavorable_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(favorable_measurements.size(), 1u);
  ASSERT_EQ(unfavorable_measurements.size(), 1u);
  EXPECT_GT(favorable_measurements[0].raw_measurement.detection_margin_db,
            unfavorable_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, DeceptionJammingFactsShrinkPhysicalCovarianceWhenMatchedEccmEnabled) {
  config::RadarSessionConfig session_config;
  session_config.hardware.enable_physics_detection = true;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);

  config::EnvironmentModelConfig env_config;
  config::JammerEmitterState deception_source;
  deception_source.technique = config::JammingTechnique::kDeception;
  deception_source.power_db = -20.0f;
  deception_source.js_db = 8.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 3.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 1.0f;
  env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService environment_service(env_config);

  const session::RadarSceneTargetList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile protected_profile;
  protected_profile.enable_agility_frequency = true;
  protected_profile.enable_eccm_rejitter = true;
  signal::pipeline::SignalPipeline protected_pipeline(session_config);
  protected_pipeline.SetControlProfile(protected_profile);
  RunPipelineCycle(&protected_pipeline, input_state, &environment_service);
  const auto protected_measurements = protected_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(protected_measurements.size(), 1u);
  EXPECT_LT(protected_measurements[0].raw_measurement.measurement_covariance(0, 0),
            baseline_measurements[0].raw_measurement.measurement_covariance(0, 0));
  EXPECT_LT(protected_measurements[0].raw_measurement.measurement_covariance(1, 1),
            baseline_measurements[0].raw_measurement.measurement_covariance(1, 1));
}

TEST(SignalPipelineTest, AgilityFrequencyHopPhaseControlsFrequencyDirection) {
  config::RadarSessionConfig phase_zero_config;
  ApplyHardwareProfile(&phase_zero_config,
                       config::profiles::RadarHardwareProfile::kGenericAirborneXBand);
  config::RadarSessionConfig phase_one_config = phase_zero_config;
  ExecutionConfig phase_zero_exec = config::mapping::MapSessionToExecution(phase_zero_config);
  ExecutionConfig phase_one_exec = config::mapping::MapSessionToExecution(phase_one_config);
  phase_zero_exec.detection.engineering.transmitter.frequency_hz = 1.0e9f;
  phase_one_exec.detection.engineering.transmitter.frequency_hz = 1.0e9f;

  extension::control::RadarControlProfile profile;
  profile.enable_agility_frequency = true;
  profile.agility_frequency_hop_phase = 0U;
  signal::pipeline::ApplyControlProfileToConfig(profile, &phase_zero_exec);
  profile.agility_frequency_hop_phase = 1U;
  signal::pipeline::ApplyControlProfileToConfig(profile, &phase_one_exec);

  EXPECT_FLOAT_EQ(phase_zero_exec.detection.engineering.transmitter.frequency_hz, 1.015e9f);
  EXPECT_FLOAT_EQ(phase_one_exec.detection.engineering.transmitter.frequency_hz, 0.985e9f);
}

TEST(SignalPipelineTest, EccmProfileReducesHeuristicTrackingLossDecay) {
  config::EnvironmentModelConfig env_config;
  env_config.jammer_sources.push_back(
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 45.0f));
  environment::EnvironmentService environment_service(env_config);

  session::RadarSceneTarget target(800.0f, 0.0f, 0.0f, 2.5f);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const session::RadarSceneTargetList input_state{target};

  signal::pipeline::SignalPipeline baseline_pipeline;
  const auto baseline_output =
      CloneSceneTargets(RunPipelineCycle(&baseline_pipeline, input_state, &environment_service)
                            .updated_scene_targets);

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline;
  protected_pipeline.SetControlProfile(eccm_profile);
  const auto protected_output =
      CloneSceneTargets(RunPipelineCycle(&protected_pipeline, input_state, &environment_service)
                            .updated_scene_targets);

  ASSERT_EQ(baseline_output.size(), 1u);
  ASSERT_EQ(protected_output.size(), 1u);
  EXPECT_GE(SpeedOf(protected_output[0]), SpeedOf(baseline_output[0]));
  EXPECT_GE(protected_output[0].rcs, baseline_output[0].rcs);
}

TEST(SignalPipelineTest, DetailedJammingFactsModulateHeuristicEccmRelief) {
  config::RadarSessionConfig session_config;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);

  config::JammerEmitterState favorable_source =
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 12.0f);
  favorable_source.has_direction_deg = true;
  favorable_source.azimuth_deg = 30.0f;
  favorable_source.elevation_deg = 10.0f;
  favorable_source.angular_span_deg = 32.0f;

  config::EnvironmentModelConfig favorable_env_config =
      MakeEnvironmentConfigWithJammers({favorable_source});
  environment::EnvironmentService favorable_environment(favorable_env_config);

  config::JammerEmitterState unfavorable_source =
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 12.0f);
  unfavorable_source.has_direction_deg = true;
  unfavorable_source.azimuth_deg = 1.0f;
  unfavorable_source.elevation_deg = 0.0f;
  unfavorable_source.angular_span_deg = 6.0f;
  config::EnvironmentModelConfig unfavorable_env_config =
      MakeEnvironmentConfigWithJammers({unfavorable_source});
  environment::EnvironmentService unfavorable_environment(unfavorable_env_config);

  session::RadarSceneTarget target(800.0f, 0.0f, 0.0f, 2.5f, 1.0f, 0.0f, 0.0f);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const session::RadarSceneTargetList input_state{target};

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_adaptive_beamforming = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;

  signal::pipeline::SignalPipeline favorable_pipeline(session_config);
  favorable_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&favorable_pipeline, input_state, &favorable_environment);
  const auto favorable_measurements = favorable_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline unfavorable_pipeline(session_config);
  unfavorable_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&unfavorable_pipeline, input_state, &unfavorable_environment);
  const auto unfavorable_measurements = unfavorable_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(favorable_measurements.size(), 1u);
  ASSERT_EQ(unfavorable_measurements.size(), 1u);
  EXPECT_GT(favorable_measurements[0].raw_measurement.detection_margin_db,
            unfavorable_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, AutoLifecycleAssemblyUsesControlProfileAdjustedKalmanUpdater) {
  config::RadarSessionConfig session_config;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);
  ApplyLifecyclePolicyProfile(&session_config,
                              config::profiles::LifecyclePolicyProfile::kFastConfirm);
  session_config.policy.tracking.enable_kalman_filter = true;

  environment::EnvironmentService environment_service;

  session::RadarSceneTarget target(1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const session::RadarSceneTargetList cycle_1_input{target};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> baseline_manager =
      baseline_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(baseline_manager != nullptr);
  RunPipelineCycle(&baseline_pipeline, cycle_1_input, &environment_service, 1u);
  baseline_manager->Update(MakeLifecycleCycle(1u, 1u),
                           baseline_pipeline.GetLastTrackMeasurements());

  target.position_x = 101.0f;
  target.range_m = 101.0f;
  const session::RadarSceneTargetList cycle_2_input{target};
  baseline_pipeline.SetAssociationSeeds(baseline_manager->BuildAssociationSeeds());
  RunPipelineCycle(&baseline_pipeline, cycle_2_input, &environment_service, 2u);
  baseline_manager->Update(MakeLifecycleCycle(2u, 2u),
                           baseline_pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      baseline_manager->BuildAssociationSeeds();

  extension::control::RadarControlProfile adaptive_profile;
  adaptive_profile.enable_adaptive_beamforming = true;
  signal::pipeline::SignalPipeline adaptive_pipeline(session_config);
  adaptive_pipeline.SetControlProfile(adaptive_profile);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> adaptive_manager =
      adaptive_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(adaptive_manager != nullptr);
  RunPipelineCycle(&adaptive_pipeline, cycle_1_input, &environment_service, 1u);
  adaptive_manager->Update(MakeLifecycleCycle(1u, 1u),
                           adaptive_pipeline.GetLastTrackMeasurements());
  adaptive_pipeline.SetAssociationSeeds(adaptive_manager->BuildAssociationSeeds());
  RunPipelineCycle(&adaptive_pipeline, cycle_2_input, &environment_service, 2u);
  adaptive_manager->Update(MakeLifecycleCycle(2u, 2u),
                           adaptive_pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> adaptive_seeds =
      adaptive_manager->BuildAssociationSeeds();

  ASSERT_EQ(baseline_seeds.size(), 1u);
  ASSERT_EQ(adaptive_seeds.size(), 1u);
  EXPECT_LT(adaptive_seeds[0].gaussian_state.covariance(0, 0),
            baseline_seeds[0].gaussian_state.covariance(0, 0));
}

TEST(SignalPipelineTest, AutoLifecycleManagerSyncsRuntimeTuningAcrossCycles) {
  config::RadarSessionConfig session_config;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);
  ApplyLifecyclePolicyProfile(&session_config,
                              config::profiles::LifecyclePolicyProfile::kFastConfirm);
  session_config.policy.tracking.enable_kalman_filter = true;

  const ExecutionConfig base_exec_config = config::mapping::MapSessionToExecution(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> unsynced_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(base_exec_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> synced_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(base_exec_config);
  ASSERT_TRUE(unsynced_manager != nullptr);
  ASSERT_TRUE(synced_manager != nullptr);

  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline measurement_source_pipeline(session_config);
  const session::RadarSceneTargetList cycle_1_input{BuildPhysicsTarget(100.0f, 4.0f)};
  RunPipelineCycle(&measurement_source_pipeline, cycle_1_input, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> cycle_1_measurements =
      measurement_source_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_1_measurements.size(), 1u);

  unsynced_manager->Update(MakeLifecycleCycle(1u, 1u), cycle_1_measurements);
  synced_manager->Update(MakeLifecycleCycle(1u, 1u), cycle_1_measurements);

  extension::control::RadarControlProfile agile_profile;
  agile_profile.enable_agility_frequency = true;
  const signal::pipeline::ResolvedRuntimePipelineConfig agile_runtime_config =
      signal::pipeline::ResolveRuntimePipelineConfig(base_exec_config, agile_profile);
  signal::pipeline::SyncAutoLifecycleManagerForResolvedRuntimeConfig(agile_runtime_config,
                                                                     synced_manager.get());

  unsynced_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  synced_manager->Update(MakeLifecycleCycle(2u, 2u), {});

  const std::vector<signal::tracking::AssociationTrackSeed> unsynced_seeds =
      unsynced_manager->BuildAssociationSeeds();
  const std::vector<signal::tracking::AssociationTrackSeed> synced_seeds =
      synced_manager->BuildAssociationSeeds();
  ASSERT_EQ(unsynced_seeds.size(), 1u);
  ASSERT_EQ(synced_seeds.size(), 1u);
  EXPECT_GT(synced_seeds[0].gaussian_state.covariance(0, 0),
            unsynced_seeds[0].gaussian_state.covariance(0, 0));
}

TEST(SignalPipelineTest, InvalidTopologyRebuildKeepsPreviousLifecycleAssemblyOperational) {
  config::RadarSessionConfig session_config;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);
  ApplyLifecyclePolicyProfile(&session_config,
                              config::profiles::LifecyclePolicyProfile::kFastConfirm);
  session_config.policy.tracking.enable_kalman_filter = true;

  const ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::CreateAutoLifecycleManagerForRuntimeConfig(exec_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);

  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline measurement_source_pipeline(session_config);
  const session::RadarSceneTargetList cycle_1_input{BuildPhysicsTarget(100.0f, 4.0f)};
  RunPipelineCycle(&measurement_source_pipeline, cycle_1_input, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> cycle_1_measurements =
      measurement_source_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_1_measurements.size(), 1u);

  lifecycle_manager->Update(MakeLifecycleCycle(1u, 1u), cycle_1_measurements);
  const std::vector<signal::tracking::AssociationTrackSeed> previous_seeds =
      lifecycle_manager->BuildAssociationSeeds();
  ASSERT_EQ(previous_seeds.size(), 1u);

  ExecutionConfig invalid_exec = exec_config;
  invalid_exec.lifecycle.policy.enable_imm_lifecycle = true;
  invalid_exec.lifecycle.engineering.enable_imm_lifecycle = true;
  invalid_exec.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};
  invalid_exec.lifecycle.imm_initial_weights = {1.0f};

  signal::pipeline::ResolvedRuntimePipelineConfig invalid_runtime_config;
  invalid_runtime_config.config = invalid_exec;

  const bool sync_succeeded = signal::pipeline::SyncAutoLifecycleManagerForResolvedRuntimeConfig(
      invalid_runtime_config, lifecycle_manager.get());
  EXPECT_FALSE(sync_succeeded);

  lifecycle_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  const std::vector<signal::tracking::AssociationTrackSeed> retained_seeds =
      lifecycle_manager->BuildAssociationSeeds();
  ASSERT_EQ(retained_seeds.size(), 1u);
  EXPECT_EQ(retained_seeds[0].association_key, previous_seeds[0].association_key);
}

TEST(SignalPipelineTest, DeceptionJammingInflatesPhysicalCovarianceMoreThanNoiseSuppression) {
  config::RadarSessionConfig session_config;
  session_config.hardware.enable_physics_detection = true;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);

  config::EnvironmentModelConfig noise_env_config;
  config::JammerEmitterState noise_source;
  noise_source.technique = config::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = -20.0f;
  noise_source.js_db = 8.0f;
  noise_source.has_direction_deg = true;
  noise_source.azimuth_deg = 20.0f;
  noise_source.elevation_deg = 7.0f;
  noise_source.angular_span_deg = 30.0f;
  noise_source.confidence = 1.0f;
  noise_env_config.jammer_sources.push_back(noise_source);
  environment::EnvironmentService noise_environment(noise_env_config);

  config::EnvironmentModelConfig deception_env_config;
  config::JammerEmitterState deception_source;
  deception_source.technique = config::JammingTechnique::kDeception;
  deception_source.power_db = -20.0f;
  deception_source.js_db = 8.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 3.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 1.0f;
  deception_env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService deception_environment(deception_env_config);

  const session::RadarSceneTargetList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline noise_pipeline(session_config);
  RunPipelineCycle(&noise_pipeline, input_state, &noise_environment);
  const auto noise_measurements = noise_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline deception_pipeline(session_config);
  RunPipelineCycle(&deception_pipeline, input_state, &deception_environment);
  const auto deception_measurements = deception_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(noise_measurements.size(), 1u);
  ASSERT_EQ(deception_measurements.size(), 1u);
  EXPECT_GT(deception_measurements[0].raw_measurement.measurement_covariance(0, 0),
            noise_measurements[0].raw_measurement.measurement_covariance(0, 0));
}

TEST(SignalPipelineTest, AutoImmLifecycleAssemblyUsesControlProfileAdjustedImmParameters) {
  config::RadarSessionConfig session_config;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);
  session_config.policy.lifecycle.enable_imm_lifecycle = true;
  ApplyLifecyclePolicyProfile(&session_config,
                              config::profiles::LifecyclePolicyProfile::kFastConfirm);

  environment::EnvironmentService environment_service;

  session::RadarSceneTarget target = BuildPhysicsTarget(120.0f, 4.0f);
  const session::RadarSceneTargetList cycle_1_input{target};

  signal::pipeline::SignalPipeline baseline_pipeline(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> baseline_manager =
      baseline_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(baseline_manager != nullptr);
  RunPipelineCycle(&baseline_pipeline, cycle_1_input, &environment_service, 1u);
  baseline_manager->Update(MakeLifecycleCycle(1u, 1u),
                           baseline_pipeline.GetLastTrackMeasurements());
  baseline_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      baseline_manager->BuildAssociationSeeds();

  extension::control::RadarControlProfile protected_profile;
  protected_profile.enable_eccm_rejitter = true;
  protected_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline(session_config);
  protected_pipeline.SetControlProfile(protected_profile);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> protected_manager =
      protected_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(protected_manager != nullptr);
  RunPipelineCycle(&protected_pipeline, cycle_1_input, &environment_service, 1u);
  protected_manager->Update(MakeLifecycleCycle(1u, 1u),
                            protected_pipeline.GetLastTrackMeasurements());
  protected_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  const std::vector<signal::tracking::AssociationTrackSeed> protected_seeds =
      protected_manager->BuildAssociationSeeds();

  ASSERT_EQ(baseline_seeds.size(), 1u);
  ASSERT_EQ(protected_seeds.size(), 1u);
  EXPECT_GT(protected_seeds[0].gaussian_state.covariance(0, 0),
            baseline_seeds[0].gaussian_state.covariance(0, 0));
}

TEST(SignalPipelineTest, ExposesStructuredTrackMeasurements) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  session::RadarSceneTarget first(100.0f, 0.0f, 0.0f, 2.0f);

  first.position_x = 10.0f;
  first.range_m = 10.0f;
  session::RadarSceneTarget second(220.0f, 0.0f, 0.0f, 5.0f);

  second.position_x = 100.0f;
  second.range_m = 100.0f;
  const session::RadarSceneTargetList cycle_1{first, second};

  RunPipelineCycle(&signal_pipeline, cycle_1, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(first_measurements.size(), 2u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_FALSE(first_measurements[1].raw_measurement.matched_existing_track);
  EXPECT_EQ(first_measurements[0].raw_measurement.source_index, 0u);
  EXPECT_EQ(first_measurements[1].raw_measurement.source_index, 1u);
  EXPECT_GT(first_measurements[0].filtered_feature.observed_speed, 0.0f);
  EXPECT_GT(first_measurements[0].raw_measurement.detection_margin_db, -2.0f);
  EXPECT_TRUE(first_measurements[0].raw_measurement.used_position_association);

  first.velocity_x = 101.0f;
  first.velocity_y = 0.0f;
  first.velocity_z = 0.0f;
  first.rcs = 2.1f;
  first.position_x = 11.0f;
  first.range_m = 11.0f;
  second.velocity_x = 219.5f;
  second.velocity_y = 0.0f;
  second.velocity_z = 0.0f;
  second.rcs = 4.9f;
  second.position_x = 101.0f;
  second.range_m = 101.0f;
  signal::tracking::AssociationTrackSeed first_seed;
  first_seed.association_key = first_measurements[0].raw_measurement.association_key;
  first_seed.has_position = true;
  first_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  first_seed.has_gaussian_state = true;
  first_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  first_seed.gaussian_state.mean(0) = 10.0f;
  first_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;

  signal::tracking::AssociationTrackSeed second_seed;
  second_seed.association_key = first_measurements[1].raw_measurement.association_key;
  second_seed.has_position = true;
  second_seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  second_seed.has_gaussian_state = true;
  second_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  second_seed.gaussian_state.mean(0) = 100.0f;
  second_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;

  signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>{first_seed, second_seed});
  const session::RadarSceneTargetList cycle_2{first, second};
  RunPipelineCycle(&signal_pipeline, cycle_2, &environment_service, 2u);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(second_measurements.size(), 2u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(second_measurements[1].raw_measurement.matched_existing_track);
  EXPECT_GT(second_measurements[0].raw_measurement.association_cost, 0.0f);
  EXPECT_GT(second_measurements[1].raw_measurement.association_cost, 0.0f);
  EXPECT_TRUE(second_measurements[0].raw_measurement.used_position_association);
}

TEST(SignalPipelineTest, CompletesWithoutCrashWhenDetectedTargetUsesDefaultPosition) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  const session::RadarSceneTargetList input_state{
      session::RadarSceneTarget(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f)};

  RunPipelineCycle(&signal_pipeline, input_state, &environment_service);
}

TEST(SignalPipelineTest, UsesPositionAssociationByDefaultWhenCartesianPositionExists) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  session::RadarSceneTarget first(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  first.position_x = 10.0f;
  first.position_y = 0.0f;
  first.position_z = 0.0f;
  first.range_m = 10.0f;

  session::RadarSceneTarget second(220.0f, 0.0f, 0.0f, 5.0f, 3.0f, 0.0f, 0.0f);

  second.position_x = 100.0f;
  second.position_y = 0.0f;
  second.position_z = 0.0f;
  second.range_m = 100.0f;

  RunPipelineCycle(&signal_pipeline, session::RadarSceneTargetList{first, second},
                   &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(first_measurements.size(), 2u);
  EXPECT_TRUE(first_measurements[0].raw_measurement.used_position_association);
  EXPECT_TRUE(first_measurements[1].raw_measurement.used_position_association);

  signal::tracking::AssociationTrackSeed first_seed;
  first_seed.association_key = first_measurements[0].raw_measurement.association_key;
  first_seed.has_position = true;
  first_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  first_seed.has_gaussian_state = true;
  first_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  first_seed.gaussian_state.mean(0) = 10.0f;
  first_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;

  signal::tracking::AssociationTrackSeed second_seed;
  second_seed.association_key = first_measurements[1].raw_measurement.association_key;
  second_seed.has_position = true;
  second_seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  second_seed.has_gaussian_state = true;
  second_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  second_seed.gaussian_state.mean(0) = 100.0f;
  second_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;

  signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>{first_seed, second_seed});

  first.position_x = 11.0f;
  second.position_x = 101.0f;
  RunPipelineCycle(&signal_pipeline, session::RadarSceneTargetList{second, first},
                   &environment_service, 2u);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(second_measurements.size(), 2u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.used_position_association);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(second_measurements[1].raw_measurement.matched_existing_track);
}

TEST(SignalPipelineTest, UsesLifecycleAssociationSeedsByDefault) {
  environment::EnvironmentService environment_service;
  environment_service.BeginCycle(MakeEnvironmentCycle(1u));

  signal::pipeline::SignalPipeline signal_pipeline;
  session::RadarSceneTarget target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  RunPipelineCycle(&signal_pipeline, session::RadarSceneTargetList{target}, &environment_service,
                   1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);

  target.position_x = 10.2f;
  target.range_m = 10.2f;
  RunPipelineCycle(&signal_pipeline, session::RadarSceneTargetList{target}, &environment_service,
                   2u);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_EQ(second_measurements[0].raw_measurement.association_key, first_key);
}

TEST(SignalPipelineTest, ClearManualAssociationSeedsKeepsLifecycleSeedsActive) {
  environment::EnvironmentService environment_service;
  environment_service.BeginCycle(MakeEnvironmentCycle(1u));

  signal::pipeline::SignalPipeline signal_pipeline;

  signal::tracking::AssociationTrackSeed side_channel_seed;
  side_channel_seed.association_key = 777u;
  side_channel_seed.has_position = true;
  side_channel_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  side_channel_seed.has_gaussian_state = true;
  side_channel_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  side_channel_seed.gaussian_state.mean(0) = 10.0f;
  side_channel_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;
  signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, side_channel_seed));
  signal_pipeline.ClearManualAssociationSeeds();

  session::RadarSceneTarget target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  RunPipelineCycle(&signal_pipeline, session::RadarSceneTargetList{target}, &environment_service,
                   1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);

  target.position_x = 10.1f;
  target.range_m = 10.1f;
  RunPipelineCycle(&signal_pipeline, session::RadarSceneTargetList{target}, &environment_service,
                   2u);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_EQ(second_measurements[0].raw_measurement.association_key, first_key);
}

TEST(SignalPipelineTest, InvalidManualAssociationSeedsDoNotDisableLifecycleSeeds) {
  environment::EnvironmentService environment_service;
  environment_service.BeginCycle(MakeEnvironmentCycle(1u));

  signal::pipeline::SignalPipeline signal_pipeline;

  signal::tracking::AssociationTrackSeed invalid_seed;
  invalid_seed.association_key = 0u;
  invalid_seed.has_position = true;
  invalid_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  invalid_seed.has_gaussian_state = true;
  invalid_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  invalid_seed.gaussian_state.mean(0) = 10.0f;
  invalid_seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;
  signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, invalid_seed));

  session::RadarSceneTarget target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  const session::SignalCycleResult first_result = RunPipelineCycle(
      &signal_pipeline, session::RadarSceneTargetList{target}, &environment_service, 1u);
  EXPECT_TRUE(first_result.executed_this_cycle);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);

  target.position_x = 10.1f;
  target.range_m = 10.1f;
  const session::SignalCycleResult second_result = RunPipelineCycle(
      &signal_pipeline, session::RadarSceneTargetList{target}, &environment_service, 2u);
  EXPECT_TRUE(second_result.executed_this_cycle);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_EQ(second_measurements[0].raw_measurement.association_key, first_key);
}

TEST(SignalPipelineTest, InvalidEnvironmentCycleAbortsAndClearsLastCycleCache) {
  signal::pipeline::SignalPipeline signal_pipeline;
  environment::EnvironmentService valid_environment;
  environment::EnvironmentService invalid_environment;

  session::RadarSceneTarget target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

  target.position_x = 10.0f;
  target.range_m = 10.0f;

  const session::SignalCycleResult valid_result = RunPipelineCycle(
      &signal_pipeline, session::RadarSceneTargetList{target}, &valid_environment, 1u);
  EXPECT_TRUE(valid_result.executed_this_cycle);
  EXPECT_EQ(valid_result.abort_reason, session::SignalCycleAbortReason::kNone);
  ASSERT_EQ(valid_result.updated_scene_targets.size(), 1u);
  ASSERT_EQ(signal_pipeline.GetLastTrackMeasurements().size(), 1u);

  const session::SignalCycleResult invalid_result = signal_pipeline.RunCycle(
      ToSceneTargets(session::RadarSceneTargetList{target}), invalid_environment);
  EXPECT_FALSE(invalid_result.executed_this_cycle);
  EXPECT_EQ(invalid_result.abort_reason,
            session::SignalCycleAbortReason::kInvalidEnvironmentCycle);
  EXPECT_TRUE(invalid_result.updated_scene_targets.empty());
  EXPECT_TRUE(invalid_result.decision_frame.tracks.empty());
  EXPECT_TRUE(signal_pipeline.GetLastTrackMeasurements().empty());
  EXPECT_EQ(signal_pipeline.GetLastAssociationQualityMetrics().detection_count, 0u);
}

// ============================================================================
// PropagationModel — 边界条件补充
// ============================================================================

/// @brief 杂波功率由内部模型统一给出，不再由外部场景直填。

TEST(SignalPipelineTest, SameInstanceControlProfileSwitchAcrossCyclesSyncsLifecycleCovariance) {
  config::RadarSessionConfig session_config;
  ApplyDetectionIntentProfile(&session_config,
                              config::profiles::DetectionIntentProfile::kDetectionPriority);
  ApplyLifecyclePolicyProfile(&session_config,
                              config::profiles::LifecyclePolicyProfile::kFastConfirm);
  session_config.policy.tracking.enable_kalman_filter = true;

  environment::EnvironmentService environment_service;

  session::RadarSceneTarget target(1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const session::RadarSceneTargetList input{target};

  signal::pipeline::SignalPipeline pipeline(session_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> manager =
      pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(manager != nullptr);

  RunPipelineCycle(&pipeline, input, &environment_service, 1u);
  manager->Update(MakeLifecycleCycle(1u, 1u), pipeline.GetLastTrackMeasurements());

  extension::control::RadarControlProfile baseline_profile;
  pipeline.SetControlProfile(baseline_profile);

  target.position_x = 101.0f;
  target.range_m = 101.0f;
  const session::RadarSceneTargetList input2{target};
  pipeline.SetAssociationSeeds(manager->BuildAssociationSeeds());
  RunPipelineCycle(&pipeline, input2, &environment_service, 2u);
  manager->Update(MakeLifecycleCycle(2u, 2u), pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      manager->BuildAssociationSeeds();
  ASSERT_EQ(baseline_seeds.size(), 1u);
  const float baseline_cov = baseline_seeds[0].gaussian_state.covariance(0, 0);

  extension::control::RadarControlProfile agile_profile;
  agile_profile.enable_agility_frequency = true;
  pipeline.SetControlProfile(agile_profile);

  target.position_x = 102.0f;
  target.range_m = 102.0f;
  const session::RadarSceneTargetList input3{target};
  pipeline.SetAssociationSeeds(manager->BuildAssociationSeeds());
  RunPipelineCycle(&pipeline, input3, &environment_service, 3u);
  manager->Update(MakeLifecycleCycle(3u, 3u), pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> agile_seeds =
      manager->BuildAssociationSeeds();
  ASSERT_EQ(agile_seeds.size(), 1u);

  EXPECT_GT(std::fabs(agile_seeds[0].gaussian_state.covariance(0, 0) - baseline_cov), 1.0e-4f);
}

TEST(SignalPipelineInternalConfigTest, DetectionBuilderProfileMapsToBaselineProfile) {
  const ExecutionConfig exec_config =
      config::mapping::MapSessionToExecution(MakeDetectionFocusedConfig());

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(SignalPipelineInternalConfigTest, TrackingBuilderProfileMapsToTrackingProfile) {
  const ExecutionConfig exec_config =
      config::mapping::MapSessionToExecution(MakeTrackingFocusedConfig());

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 1.0f);
}

TEST(SignalPipelineInternalConfigTest, RobustBuilderProfileMapsToRobustProfile) {
  const ExecutionConfig exec_config =
      config::mapping::MapSessionToExecution(MakeRobustTrackingConfig());

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 12.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(SignalPipelineInternalConfigTest,
     NonDefaultRcsPhysicsBreaksTrackingProfileSignatureAndFallsBackToBaseline) {
  config::RadarSessionConfig session_config = MakeTrackingFocusedConfig();
  ApplyRcsFusionProfile(&session_config, config::profiles::RcsFusionProfile::kEnhanced);
  const ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_config);

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 1.0f);
}

TEST(SignalPipelineInternalConfigTest, CustomConfigStaysBaselineEvenWithHighLifecycleThresholds) {
  config::RadarSessionConfig session_config = MakeDetectionFocusedConfig();
  ApplyDetectionIntentProfile(&session_config, config::profiles::DetectionIntentProfile::kBalanced);
  ApplyLifecyclePolicyProfile(&session_config, config::profiles::LifecyclePolicyProfile::kBalanced);
  ApplyTrackingPolicyProfile(&session_config, config::profiles::TrackingPolicyProfile::kBalanced);
  const ExecutionConfig exec_config = config::mapping::MapSessionToExecution(session_config);

  EXPECT_FLOAT_EQ(exec_config.association.policy.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(exec_config.tracking.engineering.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(SignalPipelineInternalConfigTest, ImmToggleOnlyControlsImmInternalDefaults) {
  config::RadarSessionConfig session_config = MakeTrackingFocusedConfig();
  const ExecutionConfig imm_disabled_exec = config::mapping::MapSessionToExecution(session_config);
  EXPECT_TRUE(imm_disabled_exec.lifecycle.imm_model_noise_diff_coeffs.empty());
  EXPECT_FLOAT_EQ(imm_disabled_exec.tracking.engineering.speed_decay_ratio_on_loss, 1.0f);
  EXPECT_FLOAT_EQ(imm_disabled_exec.tracking.engineering.rcs_decay_ratio_on_loss, 1.0f);

  session_config.policy.lifecycle.enable_imm_lifecycle = true;
  const ExecutionConfig imm_enabled_exec = config::mapping::MapSessionToExecution(session_config);
  ASSERT_EQ(imm_enabled_exec.lifecycle.imm_model_noise_diff_coeffs.size(), 2U);
  EXPECT_FLOAT_EQ(imm_enabled_exec.lifecycle.imm_model_noise_diff_coeffs[0], 1.0f);
  EXPECT_FLOAT_EQ(imm_enabled_exec.lifecycle.imm_model_noise_diff_coeffs[1], 10.0f);
  EXPECT_FLOAT_EQ(imm_enabled_exec.tracking.engineering.speed_decay_ratio_on_loss, 1.0f);
  EXPECT_FLOAT_EQ(imm_enabled_exec.tracking.engineering.rcs_decay_ratio_on_loss, 1.0f);
}

}  // namespace tests
}  // namespace airborne_radar
