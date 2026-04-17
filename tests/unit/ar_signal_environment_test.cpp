// Copyright 2026. All Rights Reserved.
//
// @file signal_environment_test.cpp
// @brief 验证真实环境建模与信号处理实现的基础行为。

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <vector>

#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/environment/SceneManager.h"
#include "airborne_radar/environment/PropagationModel.h"
#include "airborne_radar/signal/pipeline/effects/ControlProfileEffects.h"
#include "airborne_radar/signal/pipeline/effects/JammingEffects.h"
#include "airborne_radar/signal/pipeline/core/SignalPipeline.h"
#include "airborne_radar/signal/pipeline/assembly/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace tests {

namespace {

model::TargetFeature BuildPhysicsTarget(float range_m, float rcs) {
  model::TargetFeature target(220.0f, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = range_m;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = range_m;
  return target;
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

environment::EnvironmentCycleContext MakeEnvironmentCycle(std::uint32_t cycle_index) {
  environment::EnvironmentCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.dt_sec = 1.0f;
  return cycle;
}

template <typename PipelineType>
extension::SignalCycleResult RunPipelineCycle(PipelineType* pipeline,
                                              const model::TargetFeatureList& input_state,
                                              environment::EnvironmentService* environment_service,
                                              std::uint32_t cycle_index = 1u) {
  environment_service->BeginCycle(MakeEnvironmentCycle(cycle_index));
  return pipeline->RunCycle(input_state, *environment_service);
}

environment::JammerEmitterState MakeJammerEmitter(environment::JammingTechnique technique,
                                                  float power_db) {
  environment::JammerEmitterState jammer;
  jammer.technique = technique;
  jammer.power_db = power_db;
  jammer.confidence = 1.0f;
  return jammer;
}

environment::EnvironmentModelConfig MakeEnvironmentConfigWithJammers(
    std::initializer_list<environment::JammerEmitterState> jammer_sources) {
  environment::EnvironmentModelConfig config;
  config.jammer_sources.insert(config.jammer_sources.end(), jammer_sources.begin(),
                               jammer_sources.end());
  return config;
}

}  // namespace

TEST(EnvironmentServiceTest, DetectsJammingByConfiguredThreshold) {
  environment::JammerEmitterState jammer_source =
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 7.0f);

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({jammer_source}));
  service.SetJammingDetectionThresholdDb(6.0f);

  const auto snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jamming_detected);
  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 7.0f);
  EXPECT_GE(snapshot.jammer_sources[0].frequency_overlap_ratio, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].frequency_overlap_ratio, 1.0f);
  EXPECT_GE(snapshot.jammer_sources[0].prf_lock_risk, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].prf_lock_risk, 1.0f);
}

TEST(EnvironmentServiceTest, KeepsDirectionUnknownWhenExternalDirectionIsMissing) {
  environment::JammerEmitterState jammer_source =
      MakeJammerEmitter(environment::JammingTechnique::kDeception, 10.0f);
  jammer_source.js_db = 6.0f;
  jammer_source.angular_span_deg = 15.0f;
  jammer_source.has_direction_deg = false;

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({jammer_source}));
  const auto snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FALSE(snapshot.jammer_sources[0].has_direction_deg);
  EXPECT_GE(snapshot.jammer_sources[0].frequency_overlap_ratio, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].frequency_overlap_ratio, 1.0f);
  EXPECT_GE(snapshot.jammer_sources[0].prf_lock_risk, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].prf_lock_risk, 1.0f);
}

TEST(EnvironmentServiceTest, ModelConfigAtmosphericPhysicsAffectsDefaultSnapshot) {
  environment::EnvironmentModelConfig config;
  config.atmospheric_physics.enable_physical_model = true;
  config.atmospheric_physics.relative_humidity = 0.7f;

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_GT(snapshot.propagation_loss_db, 6.5f);
}

TEST(EnvironmentServiceTest, FreezesSnapshotUntilNextCycle) {
  environment::EnvironmentService service;

  environment::JammerEmitterState emitter;
  emitter.technique = environment::JammingTechnique::kNoiseSuppression;
  emitter.power_db = 8.0f;
  emitter.confidence = 1.0f;
  emitter.has_direction_deg = true;
  emitter.azimuth_deg = 18.0f;
  emitter.elevation_deg = 1.0f;
  emitter.angular_span_deg = 10.0f;
  const environment::EnvironmentSceneState scene_state = environment::EnvironmentSceneBuilder()
                                                             .AddJammer(emitter)
                                                             .Build();

  service.UpdateSceneState(scene_state);

  const environment::EnvironmentSnapshot pending_snapshot = service.SampleEnvironment();
  EXPECT_FALSE(pending_snapshot.jamming_detected);
  EXPECT_NEAR(pending_snapshot.propagation_loss_db, 6.5f, 1e-6f);

  {
    environment::EnvironmentCycleContext ctx;
    ctx.cycle_index = 1U;
    ctx.dt_sec = 1.0f;
    service.BeginCycle(ctx);
  }
  const environment::EnvironmentSnapshot cycle_snapshot = service.SampleEnvironment();
  const environment::EnvironmentSnapshot repeated_snapshot = service.SampleEnvironment();

  EXPECT_TRUE(cycle_snapshot.jamming_detected);
  EXPECT_FLOAT_EQ(cycle_snapshot.propagation_loss_db, 6.5f);
  EXPECT_FLOAT_EQ(cycle_snapshot.clutter_power_db, 3.0f);
  EXPECT_EQ(cycle_snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(cycle_snapshot.jammer_sources.size(), repeated_snapshot.jammer_sources.size());
  EXPECT_FLOAT_EQ(repeated_snapshot.jammer_sources[0].power_db,
                  cycle_snapshot.jammer_sources[0].power_db);
  EXPECT_EQ(repeated_snapshot.jamming_detected, cycle_snapshot.jamming_detected);
}

TEST(EnvironmentServiceTest, SupportsMultipleJammerSourcesInSnapshot) {
  environment::EnvironmentModelConfig config;

  environment::JammerEmitterState noise_source;
  noise_source.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 9.0f;
  noise_source.js_db = 7.0f;
  noise_source.has_direction_deg = true;
  noise_source.azimuth_deg = 24.0f;
  noise_source.elevation_deg = 7.0f;
  noise_source.angular_span_deg = 30.0f;
  noise_source.confidence = 0.9f;

  environment::JammerEmitterState deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 4.0f;
  deception_source.js_db = 5.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 2.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 0.8f;

  config.jammer_sources.push_back(noise_source);
  config.jammer_sources.push_back(deception_source);

  environment::EnvironmentService service(config);
  service.SetJammingDetectionThresholdDb(6.0f);

  const auto snapshot = service.SampleEnvironment();
  ASSERT_EQ(snapshot.jammer_sources.size(), 2u);
  EXPECT_TRUE(snapshot.jamming_detected);
  EXPECT_EQ(snapshot.jammer_sources[0].technique, environment::JammingTechnique::kNoiseSuppression);
  EXPECT_EQ(snapshot.jammer_sources[1].technique, environment::JammingTechnique::kDeception);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 9.0f);
  EXPECT_TRUE(snapshot.jammer_sources[0].in_sidelobe);
}

TEST(EnvironmentServiceTest, AppliesPendingSceneJammerOnNextCycleOnly) {
  environment::EnvironmentService service;

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);

  service.UpdateSceneState(
      environment::EnvironmentSceneBuilder()
          .AddJammer(MakeJammerEmitter(environment::JammingTechnique::kUnknown, 7.0f))
          .Build());

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);

  environment::EnvironmentCycleContext cycle_3;
  cycle_3.cycle_index = 3U;
  cycle_3.dt_sec = 1.0f;
  service.BeginCycle(cycle_3);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jamming_detected);
  ASSERT_EQ(snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(snapshot.jammer_sources[0].technique, environment::JammingTechnique::kUnknown);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 7.0f);
}

TEST(SceneManagerTest, CommitsPendingSceneOnlyWhenBeginCycleArrives) {
  environment::EnvironmentSceneState initial_scene;
  initial_scene.jammer_emitters.push_back(
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 2.0f));

  environment::SceneManager scene_manager(initial_scene);

  environment::EnvironmentSceneState pending_scene = initial_scene;
  pending_scene.jammer_emitters[0].power_db = 15.0f;
  scene_manager.UpdatePendingScene(pending_scene);

  ASSERT_EQ(scene_manager.GetActiveScene().jammer_emitters.size(), 1U);
  ASSERT_EQ(scene_manager.GetPendingScene().jammer_emitters.size(), 1U);
  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().jammer_emitters[0].power_db, 2.0f);
  EXPECT_FLOAT_EQ(scene_manager.GetPendingScene().jammer_emitters[0].power_db, 15.0f);

  environment::EnvironmentCycleContext cycle_9;
  cycle_9.cycle_index = 9U;
  cycle_9.dt_sec = 1.0f;
  scene_manager.CommitPendingScene(cycle_9);

  ASSERT_EQ(scene_manager.GetActiveScene().jammer_emitters.size(), 1U);
  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().jammer_emitters[0].power_db, 15.0f);
  EXPECT_EQ(scene_manager.GetActiveCycleContext().cycle_index, 9U);
}

TEST(PropagationModelTest, NegativeTerrainReflectionYieldsNetGainPassesThroughUnchanged) {
  environment::EnvironmentSceneState scene_state;

  environment::PropagationModel propagation_model;
  const environment::PropagationResult result = propagation_model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 6.5f);
  EXPECT_FLOAT_EQ(result.clutter_power_db, 3.0f);
}

TEST(PropagationModelTest, OptionalAtmosphericPhysicsAddsExtraLossWhenEnabled) {
  environment::EnvironmentSceneState baseline_scene;

  environment::EnvironmentSceneState physics_scene = baseline_scene;
  physics_scene.atmospheric_physics.enable_physical_model = true;
  physics_scene.atmospheric_physics.relative_humidity = 0.75f;

  environment::PropagationModel propagation_model;
  const environment::PropagationResult baseline_result =
      propagation_model.Evaluate(baseline_scene);
  const environment::PropagationResult physics_result =
      propagation_model.Evaluate(physics_scene);

  EXPECT_GT(physics_result.propagation_loss_db, baseline_result.propagation_loss_db);
}

TEST(PropagationModelTest, OptionalVegetationScatterPhysicsRaisesClutterWhenEnabled) {
  environment::EnvironmentSceneState baseline_scene;

  environment::EnvironmentSceneState physics_scene = baseline_scene;
  physics_scene.vegetation_scatter_physics.enable_physical_model = true;
  physics_scene.vegetation_scatter_physics.cover_profile =
      environment::VegetationCoverProfile::kTropicalDense;

  environment::PropagationModel propagation_model;
  const environment::PropagationResult baseline_result =
      propagation_model.Evaluate(baseline_scene);
  const environment::PropagationResult physics_result =
      propagation_model.Evaluate(physics_scene);

  EXPECT_FLOAT_EQ(physics_result.propagation_loss_db, baseline_result.propagation_loss_db);
  EXPECT_GT(physics_result.clutter_power_db, baseline_result.clutter_power_db);
}

TEST(EnvironmentServiceTest, ModelConfigVegetationScatterAffectsDefaultSnapshotClutter) {
  environment::EnvironmentModelConfig baseline_config;

  environment::EnvironmentModelConfig physics_config = baseline_config;
  physics_config.vegetation_scatter_physics.enable_physical_model = true;
  physics_config.vegetation_scatter_physics.cover_profile =
      environment::VegetationCoverProfile::kConiferousForest;

  environment::EnvironmentService baseline_service(baseline_config);
  environment::EnvironmentService physics_service(physics_config);

  const environment::EnvironmentSnapshot baseline_snapshot = baseline_service.SampleEnvironment();
  const environment::EnvironmentSnapshot physics_snapshot = physics_service.SampleEnvironment();
  EXPECT_GT(physics_snapshot.clutter_power_db, baseline_snapshot.clutter_power_db);
}

TEST(SignalPipelineTest, KeepsTrackStableWhenDetectionMarginIsEnough) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  model::TargetFeature target(800.0f, 0.0f, 0.0f, 2.5f, 0.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const model::TargetFeatureList input_state{target};

  const auto output_state =
      RunPipelineCycle(&signal_pipeline, input_state, &environment_service).updated_features;

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_FLOAT_EQ(output_state[0].current_track_speed, input_state[0].current_track_speed);
  EXPECT_FLOAT_EQ(output_state[0].current_track_rcs, input_state[0].current_track_rcs);
}

TEST(SignalPipelineTest, DegradesTrackWhenDetectionMarginIsTooLow) {
  environment::EnvironmentModelConfig env_config;
  env_config.jammer_sources.push_back(
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 40.0f));
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  const model::TargetFeatureList input_state{
      model::TargetFeature(800.0f, 0.0f, 0.0f, 2.5f)};

  const auto output_state =
      RunPipelineCycle(&signal_pipeline, input_state, &environment_service).updated_features;

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_LE(output_state[0].current_track_speed, input_state[0].current_track_speed);
  EXPECT_LE(output_state[0].current_track_rcs, input_state[0].current_track_rcs);
}

TEST(SignalPipelineTest,
     RcsPhysicsOverrideChangesMarginalHeuristicDetectionWhileDisabledPathStaysSame) {
  config::SignalPipelineConfig baseline_config;
  baseline_config.detection.enable_physics_detection = false;
  baseline_config.detection.intent_profile = config::DetectionIntentProfile::kBalanced;
  baseline_config.detection.hardware_profile = config::RadarHardwareProfile::kLongRangeHighPower;

  environment::EnvironmentModelConfig env_config;

  const model::TargetFeatureList input_state{BuildPhysicsTarget(4500.0f, 0.2f)};

  environment::EnvironmentService baseline_environment(env_config);
  signal::pipeline::SignalPipeline baseline_pipeline(baseline_config);
  const extension::SignalCycleResult baseline_result =
      RunPipelineCycle(&baseline_pipeline, input_state, &baseline_environment);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_result.updated_features.size(), 1u);
  EXPECT_FALSE(baseline_measurements.empty());

  config::SignalPipelineConfig disabled_override_config = baseline_config;
  disabled_override_config.detection.rcs_fusion_profile = config::RcsFusionProfile::kDisabled;

  environment::EnvironmentService disabled_environment(env_config);
  signal::pipeline::SignalPipeline disabled_pipeline(disabled_override_config);
  const extension::SignalCycleResult disabled_result =
      RunPipelineCycle(&disabled_pipeline, input_state, &disabled_environment);
  const auto disabled_measurements = disabled_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(disabled_result.updated_features.size(), 1u);
  EXPECT_FALSE(disabled_measurements.empty());
  EXPECT_FLOAT_EQ(disabled_result.updated_features[0].current_track_rcs,
                  baseline_result.updated_features[0].current_track_rcs);

  config::SignalPipelineConfig enabled_override_config = disabled_override_config;
  enabled_override_config.detection.rcs_fusion_profile = config::RcsFusionProfile::kEnhanced;

  environment::EnvironmentService enabled_environment(env_config);
  signal::pipeline::SignalPipeline enabled_pipeline(enabled_override_config);
  RunPipelineCycle(&enabled_pipeline, input_state, &enabled_environment);
  const auto enabled_measurements = enabled_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(enabled_measurements.size(), 1u);
  EXPECT_GT(enabled_measurements[0].raw_measurement.detection_margin_db,
            -2.0f);
}

TEST(SignalPipelineTest, ExposesPublicPlatformAttitudeUpdateApi) {
  signal::pipeline::SignalPipeline signal_pipeline;
  model::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 12.0f;
  platform_attitude_deg.pitch_deg = -3.0f;
  platform_attitude_deg.roll_deg = 1.5f;

  signal_pipeline.UpdatePlatformAttitude(platform_attitude_deg);

  const model::PlatformAttitudeDeg cached_platform_attitude =
      signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 12.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -3.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 1.5f);
}

TEST(SignalPipelineTest, AutoLifecycleManagerBuildsWithDefaultInternalImmConfig) {
  config::SignalPipelineConfig runtime_config;
  runtime_config.lifecycle.enable_imm_fusion = true;

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::assembly::internal::CreateAutoLifecycleManagerForRuntimeConfig(
          runtime_config,
          signal::pipeline::internal::BuildInternalSignalPipelineConfig(runtime_config));
  ASSERT_TRUE(lifecycle_manager != nullptr);

  const signal::tracking::CycleContext cycle = MakeLifecycleCycle(1u, 7u);
  const std::vector<signal::tracking::TrackMeasurement> measurements;
  lifecycle_manager->Update(cycle, measurements);

  const model::DecisionInputFrame decision_frame =
      lifecycle_manager->BuildDecisionFrame(1u, 7u, false);
  EXPECT_EQ(decision_frame.cycle_index, 1u);
  EXPECT_EQ(decision_frame.batch_id, 7u);
}

TEST(SignalPipelineTest, AutoLifecycleManagerCreationFailsWhenImmAssemblyIsInvalid) {
  config::SignalPipelineConfig runtime_config;
  runtime_config.tracking.enable_tracking_filter = true;
  runtime_config.lifecycle.enable_imm_fusion = true;

  signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(runtime_config);
  internal_config.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};
  internal_config.lifecycle.imm_initial_weights = {1.0f};

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::assembly::internal::CreateAutoLifecycleManagerForRuntimeConfig(
          runtime_config, internal_config);

  EXPECT_EQ(lifecycle_manager, nullptr);
}

TEST(SignalPipelineInternalConfigTest, DetectionPresetMapsToBaselineProfile) {
  const config::SignalPipelineConfig config =
      config::MakeDetectionMissionSignalPipelineConfig();
  const signal::pipeline::SignalPipelineConfig pipeline_config = config;

  const signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(pipeline_config);

  EXPECT_FLOAT_EQ(internal_config.association.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(internal_config.tracking.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(SignalPipelineInternalConfigTest, TrackingPresetMapsToTrackingProfile) {
  const config::SignalPipelineConfig config =
      config::MakeTrackingMissionSignalPipelineConfig();
  const signal::pipeline::SignalPipelineConfig pipeline_config = config;

  const signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(pipeline_config);

  EXPECT_FLOAT_EQ(internal_config.association.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.speed_decay_ratio_on_loss, 0.90f);
  EXPECT_FLOAT_EQ(internal_config.tracking.rcs_decay_ratio_on_loss, 0.85f);
}

TEST(SignalPipelineInternalConfigTest, RobustPresetMapsToRobustProfile) {
  const config::SignalPipelineConfig config =
      config::MakeHighRobustnessSignalPipelineConfig();
  const signal::pipeline::SignalPipelineConfig pipeline_config = config;

  const signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(pipeline_config);

  EXPECT_FLOAT_EQ(internal_config.association.unassigned_cost, 12.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(internal_config.tracking.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(SignalPipelineInternalConfigTest,
     NonDefaultRcsPhysicsBreaksTrackingPresetSignatureAndFallsBackToBaseline) {
  config::SignalPipelineConfig config = config::MakeTrackingMissionSignalPipelineConfig();
  config.detection.rcs_fusion_profile = config::RcsFusionProfile::kEnhanced;

  const signal::pipeline::SignalPipelineConfig pipeline_config = config;
  const signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(pipeline_config);

  EXPECT_FLOAT_EQ(internal_config.association.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.speed_decay_ratio_on_loss, 0.90f);
  EXPECT_FLOAT_EQ(internal_config.tracking.rcs_decay_ratio_on_loss, 0.85f);
}

TEST(SignalPipelineInternalConfigTest, CustomConfigStaysBaselineEvenWithHighLifecycleThresholds) {
  config::SignalPipelineConfig config = config::MakeDetectionMissionSignalPipelineConfig();
  config.detection.intent_profile = config::DetectionIntentProfile::kBalanced;
  config.lifecycle.policy_profile = config::LifecyclePolicyProfile::kBalanced;
  config.tracking.policy_profile = config::TrackingPolicyProfile::kBalanced;

  const signal::pipeline::SignalPipelineConfig pipeline_config = config;
  const signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(pipeline_config);

  EXPECT_FLOAT_EQ(internal_config.association.unassigned_cost, 9.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.kalman_noise_diff_coeff, 1.0f);
  EXPECT_FLOAT_EQ(internal_config.tracking.speed_decay_ratio_on_loss, 0.90f);
  EXPECT_FLOAT_EQ(internal_config.tracking.rcs_decay_ratio_on_loss, 0.85f);
}

TEST(SignalPipelineInternalConfigTest, ImmToggleOnlyControlsImmInternalDefaults) {
  config::SignalPipelineConfig config = config::MakeTrackingMissionSignalPipelineConfig();
  const signal::pipeline::SignalPipelineConfig pipeline_config = config;

  const signal::pipeline::internal::InternalSignalPipelineConfig imm_disabled_internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(pipeline_config);
  EXPECT_TRUE(imm_disabled_internal_config.lifecycle.imm_model_noise_diff_coeffs.empty());
  EXPECT_FLOAT_EQ(imm_disabled_internal_config.tracking.speed_decay_ratio_on_loss, 0.90f);
  EXPECT_FLOAT_EQ(imm_disabled_internal_config.tracking.rcs_decay_ratio_on_loss, 0.85f);

  config.lifecycle.enable_imm_fusion = true;
  const signal::pipeline::SignalPipelineConfig imm_enabled_pipeline_config = config;
  const signal::pipeline::internal::InternalSignalPipelineConfig imm_enabled_internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(imm_enabled_pipeline_config);
  ASSERT_EQ(imm_enabled_internal_config.lifecycle.imm_model_noise_diff_coeffs.size(), 2U);
  EXPECT_FLOAT_EQ(imm_enabled_internal_config.lifecycle.imm_model_noise_diff_coeffs[0], 0.5f);
  EXPECT_FLOAT_EQ(imm_enabled_internal_config.lifecycle.imm_model_noise_diff_coeffs[1], 4.0f);
  EXPECT_FLOAT_EQ(imm_enabled_internal_config.tracking.speed_decay_ratio_on_loss, 0.90f);
  EXPECT_FLOAT_EQ(imm_enabled_internal_config.tracking.rcs_decay_ratio_on_loss, 0.85f);
}

TEST(SignalPipelineTest, ControlProfilePowerReductionLowersPhysicalDetectionMargin) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;

  environment::EnvironmentService environment_service;

  const model::TargetFeatureList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile reduced_power_profile;
  reduced_power_profile.enable_lpi_power_control = true;
  reduced_power_profile.lpi_power_scale = 0.20f;
  signal::pipeline::SignalPipeline reduced_pipeline(pipeline_config);
  reduced_pipeline.SetControlProfile(reduced_power_profile);
  RunPipelineCycle(&reduced_pipeline, input_state, &environment_service);
  const auto reduced_measurements = reduced_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(reduced_measurements.size(), 1u);
  EXPECT_LT(reduced_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, AdaptiveBeamformingProfileTightensMeasurementCovariance) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;
  pipeline_config.detection.antenna_pattern.profile = config::AntennaPatternProfile::kWideCoverage;

  environment::EnvironmentService environment_service;

  const model::TargetFeatureList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile adaptive_profile;
  adaptive_profile.enable_adaptive_beamforming = true;
  signal::pipeline::SignalPipeline adaptive_pipeline(pipeline_config);
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
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.tracking.policy_profile = config::TrackingPolicyProfile::kFastAssociation;

  environment::EnvironmentService environment_service;

  model::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const model::TargetFeatureList input_state{target};

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 7u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f::Zero();
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Zero();

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
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
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.tracking.policy_profile = config::TrackingPolicyProfile::kFastAssociation;

  model::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const model::TargetFeatureList input_state{target};

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 11u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f::Zero();
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Zero();

  environment::EnvironmentModelConfig noise_env_config;
  environment::JammerEmitterState noise_source;
  noise_source.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 8.0f;
  noise_source.js_db = 8.0f;
  noise_source.has_direction_deg = true;
  noise_source.azimuth_deg = 24.0f;
  noise_source.elevation_deg = 7.0f;
  noise_source.angular_span_deg = 30.0f;
  noise_source.confidence = 1.0f;
  noise_env_config.jammer_sources.push_back(noise_source);
  environment::EnvironmentService noise_environment(noise_env_config);

  environment::EnvironmentModelConfig deception_env_config;
  environment::JammerEmitterState deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 8.0f;
  deception_source.js_db = 8.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 2.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 1.0f;
  deception_env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService deception_environment(deception_env_config);

  signal::pipeline::SignalPipeline noise_pipeline(pipeline_config);
  noise_pipeline.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  RunPipelineCycle(&noise_pipeline, input_state, &noise_environment);
  const extension::AssociationQualityMetrics noise_metrics =
      noise_pipeline.GetLastAssociationQualityMetrics();

  signal::pipeline::SignalPipeline deception_pipeline(pipeline_config);
  deception_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  RunPipelineCycle(&deception_pipeline, input_state, &deception_environment);
  const extension::AssociationQualityMetrics deception_metrics =
      deception_pipeline.GetLastAssociationQualityMetrics();

  EXPECT_EQ(noise_metrics.dominant_jamming_semantic,
            model::JammingSemantic::kNoiseSuppression);
  EXPECT_EQ(deception_metrics.dominant_jamming_semantic,
            model::JammingSemantic::kDeception);
  EXPECT_GT(deception_metrics.jamming_severity, noise_metrics.jamming_severity);
  EXPECT_GT(noise_metrics.association_stress, 0.0f);
  EXPECT_GT(deception_metrics.association_stress, 0.0f);
}

TEST(SignalPipelineTest, DominantJammingSemanticReturnsMixedWhenSecondScoreCloseToNoiseBest) {
  extension::control::RadarControlProfile control_profile;
  environment::EnvironmentSnapshot snapshot;
  snapshot.jamming_detected = true;

  environment::JammerSourceFact noise_source;
  noise_source.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 8.0f;
  noise_source.js_db = 8.0f;
  noise_source.in_sidelobe = false;
  noise_source.confidence = 1.0f;
  snapshot.jammer_sources.push_back(noise_source);

  environment::JammerSourceFact deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 8.0f;
  deception_source.js_db = 8.0f;
  deception_source.frequency_overlap_ratio = 0.10f;
  deception_source.prf_lock_risk = 0.10f;
  deception_source.confidence = 1.0f;
  snapshot.jammer_sources.push_back(deception_source);

  EXPECT_EQ(signal::pipeline::internal::ResolveDominantJammingSemantic(control_profile, snapshot),
            model::JammingSemantic::kMixed);
}

TEST(SignalPipelineTest, DominantJammingSemanticStaysNoiseWhenSecondScoreBelowThreshold) {
  extension::control::RadarControlProfile control_profile;
  environment::EnvironmentSnapshot snapshot;
  snapshot.jamming_detected = true;

  environment::JammerSourceFact noise_source;
  noise_source.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 8.0f;
  noise_source.js_db = 8.0f;
  noise_source.confidence = 1.0f;
  snapshot.jammer_sources.push_back(noise_source);

  environment::JammerSourceFact deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 0.0f;
  deception_source.js_db = 0.0f;
  deception_source.frequency_overlap_ratio = 0.0f;
  deception_source.prf_lock_risk = 0.0f;
  deception_source.confidence = 0.25f;
  snapshot.jammer_sources.push_back(deception_source);

  EXPECT_EQ(signal::pipeline::internal::ResolveDominantJammingSemantic(control_profile, snapshot),
            model::JammingSemantic::kNoiseSuppression);
}

TEST(SignalPipelineTest, MatchedEccmLowersAssociationStressForDeceptionJamming) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.tracking.policy_profile = config::TrackingPolicyProfile::kFastAssociation;

  model::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const model::TargetFeatureList input_state{target};

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 12u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f::Zero();
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Zero();

  environment::EnvironmentModelConfig deception_env_config;
  environment::JammerEmitterState deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 8.0f;
  deception_source.js_db = 8.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 3.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 1.0f;
  deception_env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService deception_environment(deception_env_config);

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  RunPipelineCycle(&baseline_pipeline, input_state, &deception_environment);
  const extension::AssociationQualityMetrics baseline_metrics =
      baseline_pipeline.GetLastAssociationQualityMetrics();

  extension::control::RadarControlProfile protected_profile;
  protected_profile.enable_agility_frequency = true;
  protected_profile.enable_eccm_rejitter = true;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
  protected_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  protected_pipeline.SetControlProfile(protected_profile);
  RunPipelineCycle(&protected_pipeline, input_state, &deception_environment);
  const extension::AssociationQualityMetrics protected_metrics =
      protected_pipeline.GetLastAssociationQualityMetrics();

  EXPECT_EQ(baseline_metrics.dominant_jamming_semantic, model::JammingSemantic::kDeception);
  EXPECT_EQ(protected_metrics.dominant_jamming_semantic,
            model::JammingSemantic::kDeception);
  EXPECT_LT(protected_metrics.jamming_severity, baseline_metrics.jamming_severity);
  EXPECT_LT(protected_metrics.association_stress, baseline_metrics.association_stress);
}

TEST(SignalPipelineTest, EccmProfileMitigatesJammingPenaltyInPhysicalDetection) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;
  pipeline_config.beam_control.radar_orientation.scan_center_deg.az_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.scan_center_deg.el_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.az_min_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.az_max_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.electronic_scan_limits_deg =
      pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg;

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_sources.push_back(
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 12.0f));
  environment::EnvironmentService environment_service(env_config);

  const model::TargetFeatureList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline eccm_pipeline(pipeline_config);
  eccm_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&eccm_pipeline, input_state, &environment_service);
  const auto eccm_measurements = eccm_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(eccm_measurements.size(), 1u);
  EXPECT_GT(eccm_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, DetailedJammingFactsModulatePhysicalEccmBenefit) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;
  pipeline_config.beam_control.radar_orientation.scan_center_deg.az_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.scan_center_deg.el_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.az_min_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.az_max_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.electronic_scan_limits_deg =
      pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg;

  environment::JammerEmitterState favorable_source =
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 12.0f);
  favorable_source.has_direction_deg = true;
  favorable_source.azimuth_deg = 28.0f;
  favorable_source.elevation_deg = 9.0f;
  favorable_source.angular_span_deg = 30.0f;
  environment::EnvironmentService favorable_environment(
      MakeEnvironmentConfigWithJammers({favorable_source}));

  environment::JammerEmitterState unfavorable_source =
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 12.0f);
  unfavorable_source.has_direction_deg = true;
  unfavorable_source.azimuth_deg = 0.0f;
  unfavorable_source.elevation_deg = 0.0f;
  unfavorable_source.angular_span_deg = 5.0f;
  environment::EnvironmentService unfavorable_environment(
      MakeEnvironmentConfigWithJammers({unfavorable_source}));

  const model::TargetFeatureList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_adaptive_beamforming = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;

  signal::pipeline::SignalPipeline favorable_pipeline(pipeline_config);
  favorable_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&favorable_pipeline, input_state, &favorable_environment);
  const auto favorable_measurements = favorable_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline unfavorable_pipeline(pipeline_config);
  unfavorable_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&unfavorable_pipeline, input_state, &unfavorable_environment);
  const auto unfavorable_measurements = unfavorable_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(favorable_measurements.size(), 1u);
  ASSERT_EQ(unfavorable_measurements.size(), 1u);
  EXPECT_GT(favorable_measurements[0].raw_measurement.detection_margin_db,
            unfavorable_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, DeceptionJammingFactsShrinkPhysicalCovarianceWhenMatchedEccmEnabled) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;

  environment::EnvironmentModelConfig env_config;
  environment::JammerEmitterState deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = -20.0f;
  deception_source.js_db = 8.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 3.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 1.0f;
  env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService environment_service(env_config);

  const model::TargetFeatureList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  RunPipelineCycle(&baseline_pipeline, input_state, &environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  extension::control::RadarControlProfile protected_profile;
  protected_profile.enable_agility_frequency = true;
  protected_profile.enable_eccm_rejitter = true;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
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
  config::SignalPipelineConfig phase_zero_config;
  phase_zero_config.detection.hardware_profile = config::RadarHardwareProfile::kGenericAirborneXBand;
  config::SignalPipelineConfig phase_one_config = phase_zero_config;
  signal::pipeline::SignalPipelineConfig phase_zero_pipeline_config = phase_zero_config;
  signal::pipeline::SignalPipelineConfig phase_one_pipeline_config = phase_one_config;

  extension::control::RadarControlProfile profile;
  profile.enable_agility_frequency = true;
  signal::pipeline::internal::InternalSignalPipelineConfig phase_zero_internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(phase_zero_pipeline_config);
  signal::pipeline::internal::InternalSignalPipelineConfig phase_one_internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(phase_one_pipeline_config);
  phase_zero_internal_config.detection.engineering.transmitter.frequency_hz = 1.0e9f;
  phase_one_internal_config.detection.engineering.transmitter.frequency_hz = 1.0e9f;

  profile.agility_frequency_hop_phase = 0U;
  signal::pipeline::internal::ApplyControlProfileToConfig(profile, &phase_zero_pipeline_config,
                                                          &phase_zero_internal_config);
  profile.agility_frequency_hop_phase = 1U;
  signal::pipeline::internal::ApplyControlProfileToConfig(profile, &phase_one_pipeline_config,
                                                          &phase_one_internal_config);

  EXPECT_FLOAT_EQ(phase_zero_internal_config.detection.engineering.transmitter.frequency_hz, 1.015e9f);
  EXPECT_FLOAT_EQ(phase_one_internal_config.detection.engineering.transmitter.frequency_hz, 0.985e9f);
}

TEST(SignalPipelineTest, EccmProfileReducesHeuristicTrackingLossDecay) {
  environment::EnvironmentModelConfig env_config;
  env_config.jammer_sources.push_back(
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 45.0f));
  environment::EnvironmentService environment_service(env_config);

  model::TargetFeature target(800.0f, 0.0f, 0.0f, 2.5f);
  target.has_cartesian_position = true;
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const model::TargetFeatureList input_state{target};

  signal::pipeline::SignalPipeline baseline_pipeline;
  const auto baseline_output =
      RunPipelineCycle(&baseline_pipeline, input_state, &environment_service).updated_features;

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline;
  protected_pipeline.SetControlProfile(eccm_profile);
  const auto protected_output =
      RunPipelineCycle(&protected_pipeline, input_state, &environment_service).updated_features;

  ASSERT_EQ(baseline_output.size(), 1u);
  ASSERT_EQ(protected_output.size(), 1u);
  EXPECT_GE(protected_output[0].current_track_speed, baseline_output[0].current_track_speed);
  EXPECT_GE(protected_output[0].current_track_rcs, baseline_output[0].current_track_rcs);
}

TEST(SignalPipelineTest, DetailedJammingFactsModulateHeuristicEccmRelief) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;

  environment::JammerEmitterState favorable_source =
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 12.0f);
  favorable_source.has_direction_deg = true;
  favorable_source.azimuth_deg = 30.0f;
  favorable_source.elevation_deg = 10.0f;
  favorable_source.angular_span_deg = 32.0f;

  environment::EnvironmentModelConfig favorable_env_config =
      MakeEnvironmentConfigWithJammers({favorable_source});
  environment::EnvironmentService favorable_environment(favorable_env_config);

  environment::JammerEmitterState unfavorable_source =
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 12.0f);
  unfavorable_source.has_direction_deg = true;
  unfavorable_source.azimuth_deg = 1.0f;
  unfavorable_source.elevation_deg = 0.0f;
  unfavorable_source.angular_span_deg = 6.0f;
  environment::EnvironmentModelConfig unfavorable_env_config =
      MakeEnvironmentConfigWithJammers({unfavorable_source});
  environment::EnvironmentService unfavorable_environment(unfavorable_env_config);

  model::TargetFeature target(800.0f, 0.0f, 0.0f, 2.5f, 1.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const model::TargetFeatureList input_state{target};

  extension::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_adaptive_beamforming = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;

  signal::pipeline::SignalPipeline favorable_pipeline(pipeline_config);
  favorable_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&favorable_pipeline, input_state, &favorable_environment);
  const auto favorable_measurements = favorable_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline unfavorable_pipeline(pipeline_config);
  unfavorable_pipeline.SetControlProfile(eccm_profile);
  RunPipelineCycle(&unfavorable_pipeline, input_state, &unfavorable_environment);
  const auto unfavorable_measurements = unfavorable_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(favorable_measurements.size(), 1u);
  ASSERT_EQ(unfavorable_measurements.size(), 1u);
  EXPECT_GT(favorable_measurements[0].raw_measurement.detection_margin_db,
            unfavorable_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, AutoLifecycleAssemblyUsesControlProfileAdjustedKalmanUpdater) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;
  pipeline_config.lifecycle.policy_profile = config::LifecyclePolicyProfile::kFastConfirm;
  pipeline_config.tracking.enable_tracking_filter = true;

  environment::EnvironmentService environment_service;

  model::TargetFeature target(1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const model::TargetFeatureList cycle_1_input{target};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> baseline_manager =
      baseline_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(baseline_manager != nullptr);
  RunPipelineCycle(&baseline_pipeline, cycle_1_input, &environment_service, 1u);
  baseline_manager->Update(MakeLifecycleCycle(1u, 1u),
                           baseline_pipeline.GetLastTrackMeasurements());

  target.position_x = 101.0f;
  target.range_m = 101.0f;
  const model::TargetFeatureList cycle_2_input{target};
  baseline_pipeline.SetAssociationSeeds(baseline_manager->BuildAssociationSeeds());
  RunPipelineCycle(&baseline_pipeline, cycle_2_input, &environment_service, 2u);
  baseline_manager->Update(MakeLifecycleCycle(2u, 2u),
                           baseline_pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      baseline_manager->BuildAssociationSeeds();

  extension::control::RadarControlProfile adaptive_profile;
  adaptive_profile.enable_adaptive_beamforming = true;
  signal::pipeline::SignalPipeline adaptive_pipeline(pipeline_config);
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
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;
  pipeline_config.lifecycle.policy_profile = config::LifecyclePolicyProfile::kFastConfirm;
  pipeline_config.tracking.enable_tracking_filter = true;

  const signal::pipeline::internal::InternalSignalPipelineConfig base_internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(pipeline_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> unsynced_manager =
      signal::pipeline::assembly::internal::CreateAutoLifecycleManagerForRuntimeConfig(pipeline_config,
                                                                            base_internal_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> synced_manager =
      signal::pipeline::assembly::internal::CreateAutoLifecycleManagerForRuntimeConfig(pipeline_config,
                                                                            base_internal_config);
  ASSERT_TRUE(unsynced_manager != nullptr);
  ASSERT_TRUE(synced_manager != nullptr);

  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline measurement_source_pipeline(pipeline_config);
  const model::TargetFeatureList cycle_1_input{BuildPhysicsTarget(100.0f, 4.0f)};
  RunPipelineCycle(&measurement_source_pipeline, cycle_1_input, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> cycle_1_measurements =
      measurement_source_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_1_measurements.size(), 1u);

  unsynced_manager->Update(MakeLifecycleCycle(1u, 1u), cycle_1_measurements);
  synced_manager->Update(MakeLifecycleCycle(1u, 1u), cycle_1_measurements);

  extension::control::RadarControlProfile agile_profile;
  agile_profile.enable_agility_frequency = true;
  const signal::pipeline::assembly::internal::ResolvedRuntimeSignalPipelineConfig agile_runtime_config =
      signal::pipeline::assembly::internal::ResolveRuntimeSignalPipelineConfig(
          pipeline_config, base_internal_config, agile_profile);
  signal::pipeline::assembly::internal::SyncAutoLifecycleManagerForResolvedRuntimeConfig(
      agile_runtime_config,
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
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;
  pipeline_config.lifecycle.policy_profile = config::LifecyclePolicyProfile::kFastConfirm;
  pipeline_config.tracking.enable_tracking_filter = true;

  const signal::pipeline::internal::InternalSignalPipelineConfig base_internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(pipeline_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::pipeline::assembly::internal::CreateAutoLifecycleManagerForRuntimeConfig(
          pipeline_config, base_internal_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);

  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline measurement_source_pipeline(pipeline_config);
  const model::TargetFeatureList cycle_1_input{BuildPhysicsTarget(100.0f, 4.0f)};
  RunPipelineCycle(&measurement_source_pipeline, cycle_1_input, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> cycle_1_measurements =
      measurement_source_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_1_measurements.size(), 1u);

  lifecycle_manager->Update(MakeLifecycleCycle(1u, 1u), cycle_1_measurements);
  const std::vector<signal::tracking::AssociationTrackSeed> previous_seeds =
      lifecycle_manager->BuildAssociationSeeds();
  ASSERT_EQ(previous_seeds.size(), 1u);

  signal::pipeline::assembly::internal::ResolvedRuntimeSignalPipelineConfig invalid_runtime_config;
  invalid_runtime_config.public_config = pipeline_config;
  invalid_runtime_config.public_config.lifecycle.enable_imm_fusion = true;
  invalid_runtime_config.internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(
          invalid_runtime_config.public_config);
  invalid_runtime_config.internal_config.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};
  invalid_runtime_config.internal_config.lifecycle.imm_initial_weights = {1.0f};

  const bool sync_succeeded =
      signal::pipeline::assembly::internal::SyncAutoLifecycleManagerForResolvedRuntimeConfig(
          invalid_runtime_config, lifecycle_manager.get());
  EXPECT_FALSE(sync_succeeded);

  lifecycle_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  const std::vector<signal::tracking::AssociationTrackSeed> retained_seeds =
      lifecycle_manager->BuildAssociationSeeds();
  ASSERT_EQ(retained_seeds.size(), 1u);
  EXPECT_EQ(retained_seeds[0].association_key, previous_seeds[0].association_key);
}

TEST(SignalPipelineTest, DeceptionJammingInflatesPhysicalCovarianceMoreThanNoiseSuppression) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;

  environment::EnvironmentModelConfig noise_env_config;
  environment::JammerEmitterState noise_source;
  noise_source.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = -20.0f;
  noise_source.js_db = 8.0f;
  noise_source.has_direction_deg = true;
  noise_source.azimuth_deg = 20.0f;
  noise_source.elevation_deg = 7.0f;
  noise_source.angular_span_deg = 30.0f;
  noise_source.confidence = 1.0f;
  noise_env_config.jammer_sources.push_back(noise_source);
  environment::EnvironmentService noise_environment(noise_env_config);

  environment::EnvironmentModelConfig deception_env_config;
  environment::JammerEmitterState deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = -20.0f;
  deception_source.js_db = 8.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 3.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 1.0f;
  deception_env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService deception_environment(deception_env_config);

  const model::TargetFeatureList input_state{BuildPhysicsTarget(200.0f, 10000.0f)};

  signal::pipeline::SignalPipeline noise_pipeline(pipeline_config);
  RunPipelineCycle(&noise_pipeline, input_state, &noise_environment);
  const auto noise_measurements = noise_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline deception_pipeline(pipeline_config);
  RunPipelineCycle(&deception_pipeline, input_state, &deception_environment);
  const auto deception_measurements = deception_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(noise_measurements.size(), 1u);
  ASSERT_EQ(deception_measurements.size(), 1u);
  EXPECT_GT(deception_measurements[0].raw_measurement.measurement_covariance(0, 0),
            noise_measurements[0].raw_measurement.measurement_covariance(0, 0));
  EXPECT_GT(deception_measurements[0].raw_measurement.measurement_covariance(1, 1),
            noise_measurements[0].raw_measurement.measurement_covariance(1, 1));
}

TEST(SignalPipelineTest, AutoImmLifecycleAssemblyUsesControlProfileAdjustedImmParameters) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;
  pipeline_config.lifecycle.enable_imm_fusion = true;
  pipeline_config.lifecycle.policy_profile = config::LifecyclePolicyProfile::kFastConfirm;

  environment::EnvironmentService environment_service;

  model::TargetFeature target = BuildPhysicsTarget(120.0f, 4.0f);
  const model::TargetFeatureList cycle_1_input{target};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
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
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
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

TEST(TrackFilterTest, KeepsStateWhenDetectionIsStable) {
  signal::tracking::TrackFilter filter;
  const model::TargetFeature input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = true;
  context.jamming_detected = false;
  context.detection_margin_db = 0.0f;

  const model::TargetFeature output = filter.Filter(input, context);

  EXPECT_FLOAT_EQ(output.current_track_speed, input.current_track_speed);
  EXPECT_FLOAT_EQ(output.current_track_rcs, input.current_track_rcs);
}

TEST(TrackFilterTest, AppliesLossDecayAndJammingPenalty) {
  signal::tracking::TrackFilter filter;
  const model::TargetFeature input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = false;
  context.jamming_detected = true;
  context.detection_margin_db = -10.0f;

  const model::TargetFeature output = filter.Filter(input, context);

  EXPECT_LT(output.current_track_speed, input.current_track_speed);
  EXPECT_LT(output.current_track_rcs, input.current_track_rcs);
}

TEST(TrackFilterTest, DeceptionJammingRetainsMoreTrackEnergyThanNoiseSuppression) {
  signal::tracking::TrackFilter filter;
  const model::TargetFeature input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext noise_context;
  noise_context.detection_succeeded = false;
  noise_context.jamming_detected = true;
  noise_context.dominant_jamming_semantic = model::JammingSemantic::kNoiseSuppression;
  noise_context.jamming_severity = 0.8f;
  noise_context.detection_margin_db = -10.0f;

  signal::tracking::TrackFilterContext deception_context = noise_context;
  deception_context.dominant_jamming_semantic = model::JammingSemantic::kDeception;

  const model::TargetFeature noise_output = filter.Filter(input, noise_context);
  const model::TargetFeature deception_output = filter.Filter(input, deception_context);

  EXPECT_GT(deception_output.current_track_speed, noise_output.current_track_speed);
  EXPECT_GT(deception_output.current_track_rcs, noise_output.current_track_rcs);
}

TEST(SignalPipelineTest, ExposesStructuredTrackMeasurements) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  model::TargetFeature first(100.0f, 0.0f, 0.0f, 2.0f);
  first.has_cartesian_position = true;
  first.position_x = 10.0f;
  first.range_m = 10.0f;
  model::TargetFeature second(220.0f, 0.0f, 0.0f, 5.0f);
  second.has_cartesian_position = true;
  second.position_x = 100.0f;
  second.range_m = 100.0f;
  const model::TargetFeatureList cycle_1{first, second};

  RunPipelineCycle(&signal_pipeline, cycle_1, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(first_measurements.size(), 2u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_FALSE(first_measurements[1].raw_measurement.matched_existing_track);
  EXPECT_EQ(first_measurements[0].raw_measurement.source_index, 0u);
  EXPECT_EQ(first_measurements[1].raw_measurement.source_index, 1u);
  EXPECT_TRUE(first_measurements[0].raw_measurement.has_cartesian_position);
  EXPECT_GT(first_measurements[0].filtered_feature.observed_speed, 0.0f);
  EXPECT_GT(first_measurements[0].raw_measurement.detection_margin_db, -2.0f);
  EXPECT_TRUE(first_measurements[0].raw_measurement.used_position_association);

  first.current_track_speed = 101.0f;
  first.current_track_rcs = 2.1f;
  first.position_x = 11.0f;
  first.range_m = 11.0f;
  second.current_track_speed = 219.5f;
  second.current_track_rcs = 4.9f;
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
  const model::TargetFeatureList cycle_2{first, second};
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

TEST(SignalPipelineTest, CompletesWithoutCrashWhenDetectedTargetLacksCartesianPosition) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  const model::TargetFeatureList input_state{
      model::TargetFeature(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f)};

  // Contract violation is logged and skipped; cycle completes without aborting
  RunPipelineCycle(&signal_pipeline, input_state, &environment_service);
}

TEST(SignalPipelineTest, UsesPositionAssociationByDefaultWhenCartesianPositionExists) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  model::TargetFeature first(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);
  first.has_cartesian_position = true;
  first.position_x = 10.0f;
  first.position_y = 0.0f;
  first.position_z = 0.0f;
  first.range_m = 10.0f;

  model::TargetFeature second(220.0f, 0.0f, 0.0f, 5.0f, 3.0f, 0.0f, 0.0f);
  second.has_cartesian_position = true;
  second.position_x = 100.0f;
  second.position_y = 0.0f;
  second.position_z = 0.0f;
  second.range_m = 100.0f;

  RunPipelineCycle(&signal_pipeline, model::TargetFeatureList{first, second}, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(first_measurements.size(), 2u);
  EXPECT_TRUE(first_measurements[0].raw_measurement.used_position_association);
  EXPECT_TRUE(first_measurements[0].raw_measurement.has_cartesian_position);
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
  RunPipelineCycle(&signal_pipeline, model::TargetFeatureList{second, first}, &environment_service, 2u);
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
  model::TargetFeature target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  RunPipelineCycle(&signal_pipeline, model::TargetFeatureList{target}, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);

  target.position_x = 10.2f;
  target.range_m = 10.2f;
  RunPipelineCycle(&signal_pipeline, model::TargetFeatureList{target}, &environment_service, 2u);
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

  model::TargetFeature target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  RunPipelineCycle(&signal_pipeline, model::TargetFeatureList{target}, &environment_service, 1u);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);

  target.position_x = 10.1f;
  target.range_m = 10.1f;
  RunPipelineCycle(&signal_pipeline, model::TargetFeatureList{target}, &environment_service, 2u);
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
  invalid_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;
  signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, invalid_seed));

  model::TargetFeature target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  const extension::SignalCycleResult first_result =
      RunPipelineCycle(&signal_pipeline, model::TargetFeatureList{target}, &environment_service, 1u);
  EXPECT_TRUE(first_result.executed_this_cycle);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);

  target.position_x = 10.1f;
  target.range_m = 10.1f;
  const extension::SignalCycleResult second_result =
      RunPipelineCycle(&signal_pipeline, model::TargetFeatureList{target}, &environment_service, 2u);
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

  model::TargetFeature target(0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 10.0f;
  target.range_m = 10.0f;

  const extension::SignalCycleResult valid_result =
      RunPipelineCycle(&signal_pipeline, model::TargetFeatureList{target}, &valid_environment, 1u);
  EXPECT_TRUE(valid_result.executed_this_cycle);
  EXPECT_EQ(valid_result.abort_reason, extension::SignalCycleAbortReason::kNone);
  ASSERT_EQ(valid_result.updated_features.size(), 1u);
  ASSERT_EQ(signal_pipeline.GetLastTrackMeasurements().size(), 1u);

  const extension::SignalCycleResult invalid_result =
      signal_pipeline.RunCycle(model::TargetFeatureList{target}, invalid_environment);
  EXPECT_FALSE(invalid_result.executed_this_cycle);
  EXPECT_EQ(invalid_result.abort_reason,
            extension::SignalCycleAbortReason::kInvalidEnvironmentCycle);
  EXPECT_TRUE(invalid_result.updated_features.empty());
  EXPECT_TRUE(invalid_result.decision_frame.tracks.empty());
  EXPECT_TRUE(signal_pipeline.GetLastTrackMeasurements().empty());
  EXPECT_EQ(signal_pipeline.GetLastAssociationQualityMetrics().detection_count, 0u);
}

// ============================================================================
// PropagationModel — 边界条件补充
// ============================================================================

/// @brief 杂波功率由内部模型统一给出，不再由外部场景直填。
TEST(PropagationModelTest, ClutterPowerUsesInternalBaselineWhenVegetationModelDisabled) {
  environment::EnvironmentSceneState scene_state;

  environment::PropagationModel model;
  const environment::PropagationResult result = model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 6.5f);
  EXPECT_FLOAT_EQ(result.clutter_power_db, 3.0f);
}

/// @brief 关闭植被物理模型时，杂波保持内部默认值。
TEST(PropagationModelTest, BaselineClutterRemainsStableAcrossDefaultScenes) {
  environment::EnvironmentSceneState scene_state;

  environment::PropagationModel model;
  EXPECT_FLOAT_EQ(model.Evaluate(scene_state).clutter_power_db, 3.0f);
}

/// @brief 启用植被散射物理模型后，杂波高于内部基线。
TEST(PropagationModelTest, VegetationScatterRaisesClutterFromInternalBaseline) {
  environment::EnvironmentSceneState scene_state;
  scene_state.vegetation_scatter_physics.enable_physical_model = true;
  scene_state.vegetation_scatter_physics.cover_profile =
      environment::VegetationCoverProfile::kSparseWoodland;

  environment::PropagationModel model;
  EXPECT_GT(model.Evaluate(scene_state).clutter_power_db, 3.0f);
}

/// @brief 传播损耗由内部基线组成。
TEST(PropagationModelTest, PositivePropagationLossIsRetained) {
  environment::EnvironmentSceneState scene_state;

  environment::PropagationModel model;
  const environment::PropagationResult result = model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 6.5f);
}

/// @brief 默认场景下，传播损耗与杂波均回到内部基线。
TEST(PropagationModelTest, ZeroAtmosphericAttenuationKeepsInternalPropagationBaseline) {
  environment::EnvironmentSceneState scene_state;

  environment::PropagationModel model;
  const environment::PropagationResult result = model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 6.5f);
  EXPECT_FLOAT_EQ(result.clutter_power_db, 3.0f);
}

// ============================================================================
// EnvironmentService — 结构化输入与规范化测试
// ============================================================================

/// @brief 默认配置下不生成干扰源，也不会误判为探测到干扰。
TEST(EnvironmentServiceTest, EmptyJammerSourcesProduceNoJammingFacts) {
  environment::EnvironmentService service;
  service.SetJammingDetectionThresholdDb(0.001f);

  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jammer_sources.empty());
  EXPECT_FALSE(snapshot.jamming_detected);
}

/// @brief 仅旁瓣属性为真且功率为零的结构化输入应被保留，但不应触发干扰探测。
TEST(EnvironmentServiceTest, StructuredSidelobeFactIsPreservedWithoutDetection) {
  environment::JammerEmitterState source =
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 0.0f);
  source.has_direction_deg = true;
  source.azimuth_deg = 35.0f;
  source.elevation_deg = 8.0f;
  source.angular_span_deg = 30.0f;

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({source}));
  service.SetJammingDetectionThresholdDb(0.001f);

  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_TRUE(snapshot.jammer_sources[0].in_sidelobe);
  EXPECT_FALSE(snapshot.jamming_detected);
}

/// @brief NormalizeEmitterState：负值 power_db 钳位到 0。
TEST(EnvironmentServiceTest, NegativeEmitterPowerIsClampedToZero) {
  environment::EnvironmentModelConfig config;
  environment::JammerEmitterState source;
  source.technique = environment::JammingTechnique::kNoiseSuppression;
  source.power_db = -10.0f;  // 负值应被钳位
  source.js_db = 3.0f;
  source.confidence = 1.0f;
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 0.0f);
}

/// @brief NormalizeEmitterState：负值 js_db 与 angular_span_deg 钳位到 0。
TEST(EnvironmentServiceTest, NegativeEmitterJsAndAngularSpanAreClampedToZero) {
  environment::EnvironmentModelConfig config;
  environment::JammerEmitterState source;
  source.technique = environment::JammingTechnique::kNoiseSuppression;
  source.power_db = 3.0f;
  source.js_db = -2.0f;
  source.angular_span_deg = -15.0f;
  source.confidence = 0.7f;
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].js_db, 0.0f);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].angular_span_deg, 0.0f);
}

/// @brief NormalizeEmitterState：派生 frequency_overlap_ratio 超过上限时钳位到 1.0。
TEST(EnvironmentServiceTest, EmitterOverlapRatioAboveOneIsClampedToOne) {
  environment::EnvironmentModelConfig config;
  environment::JammerEmitterState source;
  source.technique = environment::JammingTechnique::kDeception;
  source.power_db = 5.0f;
  source.js_db = 12.0f;
  source.has_direction_deg = true;
  source.azimuth_deg = 0.0f;
  source.elevation_deg = 0.0f;
  source.angular_span_deg = 0.0f;
  source.confidence = 0.8f;
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].frequency_overlap_ratio, 1.0f);
}

/// @brief NormalizeEmitterState：confidence > 1.0 钳位到 1.0。
TEST(EnvironmentServiceTest, EmitterConfidenceAboveOneIsClampedToOne) {
  environment::EnvironmentModelConfig config;
  environment::JammerEmitterState source;
  source.power_db = 5.0f;
  source.confidence = 2.5f;  // 超出范围
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].confidence, 1.0f);
}

/// @brief 干扰功率恰好等于检测门限时，应判定为探测到干扰（>= 门限）。
TEST(EnvironmentServiceTest, JammingDetectedWhenPowerEqualsThreshold) {
  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers(
      {MakeJammerEmitter(environment::JammingTechnique::kUnknown, 5.0f)}));
  service.SetJammingDetectionThresholdDb(5.0f);  // 恰好等于

  EXPECT_TRUE(service.SampleEnvironment().jamming_detected);
}

/// @brief 干扰功率低于检测门限时，不判定为探测到干扰。
TEST(EnvironmentServiceTest, JammingNotDetectedWhenPowerBelowThreshold) {
  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers(
      {MakeJammerEmitter(environment::JammingTechnique::kUnknown, 4.9f)}));
  service.SetJammingDetectionThresholdDb(5.0f);

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);
}

/// @brief 多干扰源输入应完整保留到快照中。
TEST(EnvironmentServiceTest, KeepsAllJammerSourcesInSnapshot) {
  environment::EnvironmentModelConfig config;

  environment::JammerEmitterState low;
  low.power_db = 3.0f;
  low.technique = environment::JammingTechnique::kNoiseSuppression;
  low.confidence = 1.0f;

  environment::JammerEmitterState high;
  high.power_db = 12.0f;
  high.technique = environment::JammingTechnique::kDeception;
  high.confidence = 1.0f;

  config.jammer_sources.push_back(low);
  config.jammer_sources.push_back(high);

  environment::EnvironmentService service(config);
  service.SetJammingDetectionThresholdDb(2.0f);

  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  ASSERT_EQ(snapshot.jammer_sources.size(), 2u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 3.0f);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[1].power_db, 12.0f);
  EXPECT_TRUE(snapshot.jamming_detected);
}

// ============================================================================
// TrackFilter — 衰减语义正确性测试
// ============================================================================

/// @brief 检测成功时，速度和 RCS 保持不变。
TEST(TrackFilterTest, DetectionSuccessPreservesSpeedAndRcs) {
  signal::tracking::TrackFilter filter;

  model::TargetFeature input(300.0f, 0.0f, 0.0f, 2.0f);
  input.has_cartesian_position = true;
  input.position_x = 1000.0f;
  input.range_m = 1000.0f;

  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = true;

  const model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(output.current_track_speed, 300.0f);
  EXPECT_FLOAT_EQ(output.current_track_rcs, 2.0f);
}

/// @brief 检测失配时，速度按配置系数衰减（speed = input * ratio）。
TEST(TrackFilterTest, DetectionMissDecaysSpeedByConfiguredRatio) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.80f;
  cfg.rcs_decay_ratio_on_loss = 1.0f;  // RCS 不衰减，隔离速度分支
  signal::tracking::TrackFilter filter(cfg);

  model::TargetFeature input(500.0f, 0.0f, 0.0f, 2.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(output.current_track_speed, 400.0f);  // 500 * 0.8
}

/// @brief 检测失配时，RCS 按配置系数衰减，且不低于 0.05。
TEST(TrackFilterTest, DetectionMissDecaysRcsByConfiguredRatio) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 0.70f;
  signal::tracking::TrackFilter filter(cfg);

  model::TargetFeature input(100.0f, 0.0f, 0.0f, 2.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_NEAR(output.current_track_rcs, 1.40f, 1e-4f);  // 2.0 * 0.7
}

/// @brief 连续多次失配时，速度单调递减。
TEST(TrackFilterTest, ConsecutiveMissesMonotonicallyReduceSpeed) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.90f;
  cfg.rcs_decay_ratio_on_loss = 1.0f;
  signal::tracking::TrackFilter filter(cfg);

  signal::tracking::TrackFilterContext miss_ctx;
  miss_ctx.detection_succeeded = false;

  model::TargetFeature state(500.0f, 0.0f, 0.0f, 1.0f);
  float prev_speed = 500.0f;
  for (int i = 0; i < 5; ++i) {
    state = filter.Filter(state, miss_ctx);
    EXPECT_LT(state.current_track_speed, prev_speed)
        << "Speed should decrease at miss #" << (i + 1);
    prev_speed = state.current_track_speed;
  }
}

/// @brief 衰减系数为 1.0 时，失配不改变速度。
TEST(TrackFilterTest, DecayRatioOnePreservesSpeedOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 1.0f;
  signal::tracking::TrackFilter filter(cfg);

  model::TargetFeature input(400.0f, 0.0f, 0.0f, 1.5f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(output.current_track_speed, 400.0f);
}

/// @brief 速度不会因失配变为负数（钳位到 0）。
TEST(TrackFilterTest, SpeedNeverGoesNegativeOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.0f;  // 衰减至 0
  cfg.rcs_decay_ratio_on_loss = 1.0f;
  signal::tracking::TrackFilter filter(cfg);

  model::TargetFeature input(300.0f, 0.0f, 0.0f, 1.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_GE(output.current_track_speed, 0.0f);
}

/// @brief RCS 不会因失配低于最小值 0.05。
TEST(TrackFilterTest, RcsNeverGoesBelowMinimumOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 0.0f;  // 衰减至 0
  signal::tracking::TrackFilter filter(cfg);

  model::TargetFeature input(100.0f, 0.0f, 0.0f, 0.01f);  // 极小 RCS
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_GE(output.current_track_rcs, 0.05f);
}

TEST(SignalPipelineTest, SameInstanceControlProfileSwitchAcrossCyclesSyncsLifecycleCovariance) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;
  pipeline_config.lifecycle.policy_profile = config::LifecyclePolicyProfile::kFastConfirm;
  pipeline_config.tracking.enable_tracking_filter = true;

  environment::EnvironmentService environment_service;

  model::TargetFeature target(1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const model::TargetFeatureList input{target};

  signal::pipeline::SignalPipeline pipeline(pipeline_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> manager =
      pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(manager != nullptr);

  RunPipelineCycle(&pipeline, input, &environment_service, 1u);
  manager->Update(MakeLifecycleCycle(1u, 1u), pipeline.GetLastTrackMeasurements());

  extension::control::RadarControlProfile baseline_profile;
  pipeline.SetControlProfile(baseline_profile);

  target.position_x = 101.0f;
  target.range_m = 101.0f;
  const model::TargetFeatureList input2{target};
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
  const model::TargetFeatureList input3{target};
  pipeline.SetAssociationSeeds(manager->BuildAssociationSeeds());
  RunPipelineCycle(&pipeline, input3, &environment_service, 3u);
  manager->Update(MakeLifecycleCycle(3u, 3u), pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> agile_seeds =
      manager->BuildAssociationSeeds();
  ASSERT_EQ(agile_seeds.size(), 1u);

  EXPECT_GT(std::fabs(agile_seeds[0].gaussian_state.covariance(0, 0) - baseline_cov), 1.0e-4f);
}

}  // namespace tests
}  // namespace airborne_radar
