// Copyright 2026. All Rights Reserved.
//
// @file signal_environment_test.cpp
// @brief 验证真实环境建模与信号处理实现的基础行为。

#include <gtest/gtest.h>

#include <initializer_list>
#include <vector>

#include "1q/airborne_radar/common/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/environment/scene/SceneManager.h"
#include "airborne_radar/environment/simulation/PropagationModel.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/signal/runtime/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "environment_test_fixture.h"

namespace airborne_radar {
namespace tests {

namespace {

common::model::TargetFeature BuildPhysicsTarget(float range_m, float rcs) {
  common::model::TargetFeature target(220.0f, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f);
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

environment::JammerSourceFact MakeJammerSource(environment::JammingTechnique technique,
                                               float power_db) {
  return environment_test::MakeJammerEmitter(technique, power_db);
}

environment::EnvironmentModelConfig MakeEnvironmentConfigWithJammers(
    std::initializer_list<environment::JammerSourceFact> jammer_sources) {
  environment::EnvironmentModelConfig config;
  config.jammer_sources.insert(config.jammer_sources.end(), jammer_sources.begin(),
                               jammer_sources.end());
  return config;
}

}  // namespace

TEST(EnvironmentServiceTest, DetectsJammingByConfiguredThreshold) {
  environment::JammerSourceFact jammer_source =
      MakeJammerSource(environment::JammingTechnique::kUnknown, 7.0f);
  jammer_source.frequency_overlap_ratio = 0.75f;
  jammer_source.prf_lock_risk = 0.60f;
  jammer_source.in_sidelobe = true;

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({jammer_source}));
  service.SetJammingDetectionThresholdDb(6.0f);

  const auto snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jamming_detected);
  EXPECT_FLOAT_EQ(snapshot.jammer_power_db, 7.0f);
  EXPECT_FLOAT_EQ(snapshot.jammer_frequency_overlap_ratio, 0.75f);
  EXPECT_FLOAT_EQ(snapshot.jammer_prf_lock_risk, 0.60f);
  EXPECT_TRUE(snapshot.jammer_in_sidelobe);
  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 7.0f);
}

TEST(EnvironmentServiceTest, FreezesSnapshotUntilNextCycle) {
  environment::EnvironmentService service;

  environment::JammerEmitterState emitter =
      environment_test::MakeJammerEmitter(environment::JammingTechnique::kNoiseSuppression, 8.0f);
  emitter.frequency_overlap_ratio = 0.4f;
  emitter.prf_lock_risk = 0.3f;
  emitter.in_sidelobe = true;
  const environment::EnvironmentSceneState scene_state = environment_test::MemorySceneBuilder()
                                                             .WithPropagation(12.0f, 5.0f, 3.0f)
                                                             .WithClutter(9.0f)
                                                             .AddJammer(emitter)
                                                             .BuildSceneState();

  service.UpdateSceneState(scene_state);

  const environment::EnvironmentSnapshot pending_snapshot = service.SampleEnvironment();
  EXPECT_FALSE(pending_snapshot.jamming_detected);
  EXPECT_NEAR(pending_snapshot.propagation_loss_db, 6.5f, 1e-6f);

  service.BeginCycle(environment_test::MakeEnvironmentCycle(1U));
  const environment::EnvironmentSnapshot cycle_snapshot = service.SampleEnvironment();
  const environment::EnvironmentSnapshot repeated_snapshot = service.SampleEnvironment();

  EXPECT_TRUE(cycle_snapshot.jamming_detected);
  EXPECT_FLOAT_EQ(cycle_snapshot.propagation_loss_db, 20.0f);
  EXPECT_FLOAT_EQ(cycle_snapshot.clutter_power_db, 9.0f);
  EXPECT_FLOAT_EQ(cycle_snapshot.jammer_power_db, 8.0f);
  EXPECT_EQ(cycle_snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(cycle_snapshot.jammer_sources.size(), repeated_snapshot.jammer_sources.size());
  EXPECT_FLOAT_EQ(repeated_snapshot.jammer_power_db, cycle_snapshot.jammer_power_db);
  EXPECT_EQ(repeated_snapshot.jamming_detected, cycle_snapshot.jamming_detected);
}

TEST(EnvironmentServiceTest, SupportsMultipleJammerSourcesInSnapshot) {
  environment::EnvironmentModelConfig config;

  environment::JammerSourceFact noise_source;
  noise_source.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 9.0f;
  noise_source.js_db = 7.0f;
  noise_source.frequency_overlap_ratio = 0.2f;
  noise_source.prf_lock_risk = 0.1f;
  noise_source.in_sidelobe = true;
  noise_source.confidence = 0.9f;

  environment::JammerSourceFact deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 4.0f;
  deception_source.js_db = 5.0f;
  deception_source.frequency_overlap_ratio = 0.85f;
  deception_source.prf_lock_risk = 0.80f;
  deception_source.in_sidelobe = false;
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
  EXPECT_FLOAT_EQ(snapshot.jammer_power_db, 9.0f);
  EXPECT_TRUE(snapshot.jammer_in_sidelobe);
}

TEST(EnvironmentServiceTest, AppliesPendingSceneJammerOnNextCycleOnly) {
  environment::EnvironmentService service;

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);

  service.UpdateSceneState(
      environment_test::MemorySceneBuilder()
          .AddJammer(MakeJammerSource(environment::JammingTechnique::kUnknown, 7.0f))
          .BuildSceneState());

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);

  service.BeginCycle(environment_test::MakeEnvironmentCycle(3U));
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jamming_detected);
  ASSERT_EQ(snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(snapshot.jammer_sources[0].technique, environment::JammingTechnique::kUnknown);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 7.0f);
}

TEST(SceneManagerTest, CommitsPendingSceneOnlyWhenBeginCycleArrives) {
  environment::EnvironmentSceneState initial_scene;
  initial_scene.base_propagation_loss_db = 1.0f;

  environment::scene::SceneManager scene_manager(initial_scene);

  environment::EnvironmentSceneState pending_scene = initial_scene;
  pending_scene.base_propagation_loss_db = 15.0f;
  scene_manager.UpdatePendingScene(pending_scene);

  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().base_propagation_loss_db, 1.0f);
  EXPECT_FLOAT_EQ(scene_manager.GetPendingScene().base_propagation_loss_db, 15.0f);

  scene_manager.CommitPendingScene(environment_test::MakeEnvironmentCycle(9U));

  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().base_propagation_loss_db, 15.0f);
  EXPECT_EQ(scene_manager.GetActiveCycleContext().cycle_index, 9U);
}

TEST(PropagationModelTest, NegativeTerrainReflectionYieldsNetGainPassesThroughUnchanged) {
  environment::EnvironmentSceneState scene_state;
  scene_state.base_propagation_loss_db = -10.0f;
  scene_state.atmospheric_attenuation_db = 2.0f;
  // terrain_reflection_db < 0 represents multipath gain; must not be clamped.
  scene_state.terrain_reflection_db = -4.0f;
  // Negative clutter_power_db (e.g. -3 dBW ≈ 0.5 W) is physically valid for
  // low-clutter environments and must not be clamped.
  scene_state.clutter_power_db = -3.0f;

  environment::simulation::PropagationModel propagation_model;
  const environment::simulation::PropagationResult result = propagation_model.Evaluate(scene_state);

  // No clamp: -10 + 2 + (-4) = -12 dB (net multipath gain is physically valid).
  EXPECT_FLOAT_EQ(result.propagation_loss_db, -12.0f);
  // Clutter power passes through unchanged so callers can model near-zero
  // clutter by setting large negative dBW values.
  EXPECT_FLOAT_EQ(result.clutter_power_db, -3.0f);
}

TEST(SignalPipelineTest, KeepsTrackStableWhenDetectionMarginIsEnough) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  common::model::TargetFeature target(800.0f, 0.0f, 0.0f, 2.5f, 0.0f, 0.0f, 0.0f);
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const common::model::TargetFeatureList input_state{target};

  const auto output_state =
      signal_pipeline.RunCycle(input_state, environment_service).updated_features;

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_FLOAT_EQ(output_state[0].current_track_speed, input_state[0].current_track_speed);
  EXPECT_FLOAT_EQ(output_state[0].current_track_rcs, input_state[0].current_track_rcs);
}

TEST(SignalPipelineTest, DegradesTrackWhenDetectionMarginIsTooLow) {
  environment::EnvironmentModelConfig env_config;
  env_config.base_propagation_loss_db = 60.0f;
  env_config.atmospheric_attenuation_db = 25.0f;
  env_config.terrain_reflection_db = 15.0f;
  env_config.clutter_power_db = 20.0f;
  env_config.jammer_sources.push_back(
      MakeJammerSource(environment::JammingTechnique::kUnknown, 12.0f));
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  const common::model::TargetFeatureList input_state{
      common::model::TargetFeature(800.0f, 0.0f, 0.0f, 2.5f)};

  const auto output_state =
      signal_pipeline.RunCycle(input_state, environment_service).updated_features;

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_LT(output_state[0].current_track_speed, input_state[0].current_track_speed);
  EXPECT_LT(output_state[0].current_track_rcs, input_state[0].current_track_rcs);
}

TEST(SignalPipelineTest, ExposesPublicPlatformAttitudeUpdateApi) {
  signal::pipeline::SignalPipeline signal_pipeline;
  common::config::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 12.0f;
  platform_attitude_deg.pitch_deg = -3.0f;
  platform_attitude_deg.roll_deg = 1.5f;

  signal_pipeline.UpdatePlatformAttitude(platform_attitude_deg);

  const common::config::PlatformAttitudeDeg cached_platform_attitude =
      signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 12.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -3.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 1.5f);
}

TEST(SignalPipelineTest, AutoLifecycleManagerFallsBackWhenImmConfigInvalid) {
  signal::pipeline::SignalPipelineConfig runtime_config;
  runtime_config.lifecycle.enable_auto_lifecycle_manager = true;
  runtime_config.lifecycle.enable_imm_lifecycle = true;
  runtime_config.lifecycle.imm_model_noise_diff_coeffs = std::vector<float>();

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal::runtime::internal::CreateAutoLifecycleManagerForRuntimeConfig(runtime_config);
  ASSERT_TRUE(lifecycle_manager != nullptr);

  const signal::tracking::CycleContext cycle = MakeLifecycleCycle(1u, 7u);
  const std::vector<signal::tracking::TrackMeasurement> measurements;
  lifecycle_manager->Update(cycle, measurements);

  const common::model::DecisionInputFrame decision_frame =
      lifecycle_manager->BuildDecisionFrame(1u, 7u, false);
  EXPECT_EQ(decision_frame.cycle_index, 1u);
  EXPECT_EQ(decision_frame.batch_id, 7u);
}

TEST(SignalPipelineTest, ControlProfilePowerReductionLowersPhysicalDetectionMargin) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;

  environment::EnvironmentService environment_service;

  const common::model::TargetFeatureList input_state{BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.RunCycle(input_state, environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  common::control::RadarControlProfile reduced_power_profile;
  reduced_power_profile.enable_lpi_power_control = true;
  reduced_power_profile.lpi_power_scale = 0.20f;
  signal::pipeline::SignalPipeline reduced_pipeline(pipeline_config);
  reduced_pipeline.SetControlProfile(reduced_power_profile);
  reduced_pipeline.RunCycle(input_state, environment_service);
  const auto reduced_measurements = reduced_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(reduced_measurements.size(), 1u);
  EXPECT_LT(reduced_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, AdaptiveBeamformingProfileTightensMeasurementCovariance) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;
  pipeline_config.detection.radar_system.antenna.nominal_az_beamwidth_deg = 8.0f;
  pipeline_config.detection.radar_system.antenna.nominal_el_beamwidth_deg = 8.0f;

  environment::EnvironmentService environment_service;

  const common::model::TargetFeatureList input_state{BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.RunCycle(input_state, environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  common::control::RadarControlProfile adaptive_profile;
  adaptive_profile.enable_adaptive_beamforming = true;
  signal::pipeline::SignalPipeline adaptive_pipeline(pipeline_config);
  adaptive_pipeline.SetControlProfile(adaptive_profile);
  adaptive_pipeline.RunCycle(input_state, environment_service);
  const auto adaptive_measurements = adaptive_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(adaptive_measurements.size(), 1u);
  EXPECT_LT(adaptive_measurements[0].raw_measurement.measurement_covariance(1, 1),
            baseline_measurements[0].raw_measurement.measurement_covariance(1, 1));
  EXPECT_LT(adaptive_measurements[0].raw_measurement.measurement_covariance(2, 2),
            baseline_measurements[0].raw_measurement.measurement_covariance(2, 2));
}

TEST(SignalPipelineTest, EccmProfileRelaxesHeuristicAssociationGateForSeededTracks) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.tracking.kalman_measurement_noise_std = 1.0f;

  environment::EnvironmentService environment_service;

  common::model::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const common::model::TargetFeatureList input_state{target};

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
  baseline_pipeline.RunCycle(input_state, environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  common::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
  protected_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  protected_pipeline.SetControlProfile(eccm_profile);
  protected_pipeline.RunCycle(input_state, environment_service);
  const auto protected_measurements = protected_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(protected_measurements.size(), 1u);
  EXPECT_FALSE(baseline_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(protected_measurements[0].raw_measurement.matched_existing_track);
}

TEST(SignalPipelineTest, AssociationQualityMetricsExposeTypeSpecificStressSummary) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.tracking.kalman_measurement_noise_std = 1.0f;

  common::model::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const common::model::TargetFeatureList input_state{target};

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 11u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f::Zero();
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Zero();

  environment::EnvironmentModelConfig noise_env_config;
  environment::JammerSourceFact noise_source;
  noise_source.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 8.0f;
  noise_source.js_db = 8.0f;
  noise_source.in_sidelobe = true;
  noise_source.confidence = 1.0f;
  noise_env_config.jammer_sources.push_back(noise_source);
  environment::EnvironmentService noise_environment(noise_env_config);

  environment::EnvironmentModelConfig deception_env_config;
  environment::JammerSourceFact deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 8.0f;
  deception_source.js_db = 8.0f;
  deception_source.frequency_overlap_ratio = 0.9f;
  deception_source.prf_lock_risk = 0.9f;
  deception_source.confidence = 1.0f;
  deception_env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService deception_environment(deception_env_config);

  signal::pipeline::SignalPipeline noise_pipeline(pipeline_config);
  noise_pipeline.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  noise_pipeline.RunCycle(input_state, noise_environment);
  const signal::pipeline::AssociationQualityMetrics noise_metrics =
      noise_pipeline.GetLastAssociationQualityMetrics();

  signal::pipeline::SignalPipeline deception_pipeline(pipeline_config);
  deception_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  deception_pipeline.RunCycle(input_state, deception_environment);
  const signal::pipeline::AssociationQualityMetrics deception_metrics =
      deception_pipeline.GetLastAssociationQualityMetrics();

  EXPECT_EQ(noise_metrics.dominant_jamming_semantic,
            common::utils::JammingSemantic::kNoiseSuppression);
  EXPECT_EQ(deception_metrics.dominant_jamming_semantic,
            common::utils::JammingSemantic::kDeception);
  EXPECT_GT(deception_metrics.jamming_severity, noise_metrics.jamming_severity);
  EXPECT_GT(noise_metrics.association_stress, 0.0f);
  EXPECT_GT(deception_metrics.association_stress, 0.0f);
}

TEST(SignalPipelineTest, MatchedEccmLowersAssociationStressForDeceptionJamming) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.tracking.kalman_measurement_noise_std = 1.0f;

  common::model::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const common::model::TargetFeatureList input_state{target};

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 12u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f::Zero();
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Zero();

  environment::EnvironmentModelConfig deception_env_config;
  environment::JammerSourceFact deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 8.0f;
  deception_source.js_db = 8.0f;
  deception_source.frequency_overlap_ratio = 0.9f;
  deception_source.prf_lock_risk = 0.9f;
  deception_source.confidence = 1.0f;
  deception_env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService deception_environment(deception_env_config);

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  baseline_pipeline.RunCycle(input_state, deception_environment);
  const signal::pipeline::AssociationQualityMetrics baseline_metrics =
      baseline_pipeline.GetLastAssociationQualityMetrics();

  common::control::RadarControlProfile protected_profile;
  protected_profile.enable_agility_frequency = true;
  protected_profile.enable_eccm_rejitter = true;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
  protected_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  protected_pipeline.SetControlProfile(protected_profile);
  protected_pipeline.RunCycle(input_state, deception_environment);
  const signal::pipeline::AssociationQualityMetrics protected_metrics =
      protected_pipeline.GetLastAssociationQualityMetrics();

  EXPECT_EQ(baseline_metrics.dominant_jamming_semantic, common::utils::JammingSemantic::kDeception);
  EXPECT_EQ(protected_metrics.dominant_jamming_semantic,
            common::utils::JammingSemantic::kDeception);
  EXPECT_LT(protected_metrics.jamming_severity, baseline_metrics.jamming_severity);
  EXPECT_LT(protected_metrics.association_stress, baseline_metrics.association_stress);
}

TEST(SignalPipelineTest, EccmProfileMitigatesJammingPenaltyInPhysicalDetection) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;
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
      MakeJammerSource(environment::JammingTechnique::kUnknown, 12.0f));
  environment::EnvironmentService environment_service(env_config);

  const common::model::TargetFeatureList input_state{BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.RunCycle(input_state, environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  common::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline eccm_pipeline(pipeline_config);
  eccm_pipeline.SetControlProfile(eccm_profile);
  eccm_pipeline.RunCycle(input_state, environment_service);
  const auto eccm_measurements = eccm_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(eccm_measurements.size(), 1u);
  EXPECT_GT(eccm_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, DetailedJammingFactsModulatePhysicalEccmBenefit) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;
  pipeline_config.beam_control.radar_orientation.scan_center_deg.az_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.scan_center_deg.el_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.az_min_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.az_max_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.el_min_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg.el_max_deg = 0.0f;
  pipeline_config.beam_control.radar_orientation.electronic_scan_limits_deg =
      pipeline_config.beam_control.radar_orientation.mechanical_scan_limits_deg;

  environment::JammerSourceFact favorable_source =
      MakeJammerSource(environment::JammingTechnique::kUnknown, 12.0f);
  favorable_source.frequency_overlap_ratio = 0.9f;
  favorable_source.prf_lock_risk = 0.9f;
  favorable_source.in_sidelobe = true;
  environment::EnvironmentService favorable_environment(
      MakeEnvironmentConfigWithJammers({favorable_source}));

  environment::JammerSourceFact unfavorable_source =
      MakeJammerSource(environment::JammingTechnique::kUnknown, 12.0f);
  environment::EnvironmentService unfavorable_environment(
      MakeEnvironmentConfigWithJammers({unfavorable_source}));

  const common::model::TargetFeatureList input_state{BuildPhysicsTarget(1000.0f, 1000.0f)};

  common::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_adaptive_beamforming = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;

  signal::pipeline::SignalPipeline favorable_pipeline(pipeline_config);
  favorable_pipeline.SetControlProfile(eccm_profile);
  favorable_pipeline.RunCycle(input_state, favorable_environment);
  const auto favorable_measurements = favorable_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline unfavorable_pipeline(pipeline_config);
  unfavorable_pipeline.SetControlProfile(eccm_profile);
  unfavorable_pipeline.RunCycle(input_state, unfavorable_environment);
  const auto unfavorable_measurements = unfavorable_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(favorable_measurements.size(), 1u);
  ASSERT_EQ(unfavorable_measurements.size(), 1u);
  EXPECT_GT(favorable_measurements[0].raw_measurement.detection_margin_db,
            unfavorable_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, DeceptionJammingFactsShrinkPhysicalCovarianceWhenMatchedEccmEnabled) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;

  environment::EnvironmentModelConfig env_config;
  environment::JammerSourceFact deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = -20.0f;
  deception_source.js_db = 8.0f;
  deception_source.frequency_overlap_ratio = 0.9f;
  deception_source.prf_lock_risk = 0.9f;
  deception_source.confidence = 1.0f;
  env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService environment_service(env_config);

  const common::model::TargetFeatureList input_state{BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.RunCycle(input_state, environment_service);
  const auto baseline_measurements = baseline_pipeline.GetLastTrackMeasurements();

  common::control::RadarControlProfile protected_profile;
  protected_profile.enable_agility_frequency = true;
  protected_profile.enable_eccm_rejitter = true;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
  protected_pipeline.SetControlProfile(protected_profile);
  protected_pipeline.RunCycle(input_state, environment_service);
  const auto protected_measurements = protected_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(protected_measurements.size(), 1u);
  EXPECT_LT(protected_measurements[0].raw_measurement.measurement_covariance(0, 0),
            baseline_measurements[0].raw_measurement.measurement_covariance(0, 0));
  EXPECT_LT(protected_measurements[0].raw_measurement.measurement_covariance(1, 1),
            baseline_measurements[0].raw_measurement.measurement_covariance(1, 1));
}

TEST(SignalPipelineTest, EccmProfileReducesHeuristicTrackingLossDecay) {
  environment::EnvironmentModelConfig env_config;
  env_config.base_propagation_loss_db = 80.0f;
  env_config.atmospheric_attenuation_db = 25.0f;
  env_config.terrain_reflection_db = 15.0f;
  env_config.clutter_power_db = 40.0f;
  environment::EnvironmentService environment_service(env_config);

  common::model::TargetFeature target(800.0f, 0.0f, 0.0f, 2.5f);
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const common::model::TargetFeatureList input_state{target};

  signal::pipeline::SignalPipeline baseline_pipeline;
  const auto baseline_output =
      baseline_pipeline.RunCycle(input_state, environment_service).updated_features;

  common::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline;
  protected_pipeline.SetControlProfile(eccm_profile);
  const auto protected_output =
      protected_pipeline.RunCycle(input_state, environment_service).updated_features;

  ASSERT_EQ(baseline_output.size(), 1u);
  ASSERT_EQ(protected_output.size(), 1u);
  EXPECT_GT(protected_output[0].current_track_speed, baseline_output[0].current_track_speed);
  EXPECT_GT(protected_output[0].current_track_rcs, baseline_output[0].current_track_rcs);
}

TEST(SignalPipelineTest, DetailedJammingFactsModulateHeuristicEccmRelief) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -6.0f;

  environment::JammerSourceFact favorable_source =
      MakeJammerSource(environment::JammingTechnique::kUnknown, 12.0f);
  favorable_source.frequency_overlap_ratio = 0.9f;
  favorable_source.prf_lock_risk = 0.9f;
  favorable_source.in_sidelobe = true;

  environment::EnvironmentModelConfig favorable_env_config =
      MakeEnvironmentConfigWithJammers({favorable_source});
  favorable_env_config.base_propagation_loss_db = 30.0f;
  favorable_env_config.atmospheric_attenuation_db = 10.0f;
  favorable_env_config.terrain_reflection_db = 5.0f;
  favorable_env_config.clutter_power_db = 12.0f;
  environment::EnvironmentService favorable_environment(favorable_env_config);

  environment::JammerSourceFact unfavorable_source =
      MakeJammerSource(environment::JammingTechnique::kUnknown, 12.0f);
  environment::EnvironmentModelConfig unfavorable_env_config =
      MakeEnvironmentConfigWithJammers({unfavorable_source});
  unfavorable_env_config.base_propagation_loss_db = favorable_env_config.base_propagation_loss_db;
  unfavorable_env_config.atmospheric_attenuation_db =
      favorable_env_config.atmospheric_attenuation_db;
  unfavorable_env_config.terrain_reflection_db = favorable_env_config.terrain_reflection_db;
  unfavorable_env_config.clutter_power_db = favorable_env_config.clutter_power_db;
  environment::EnvironmentService unfavorable_environment(unfavorable_env_config);

  common::model::TargetFeature target(800.0f, 0.0f, 0.0f, 2.5f, 1.0f, 0.0f, 0.0f);
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const common::model::TargetFeatureList input_state{target};

  common::control::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_adaptive_beamforming = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;

  signal::pipeline::SignalPipeline favorable_pipeline(pipeline_config);
  favorable_pipeline.SetControlProfile(eccm_profile);
  favorable_pipeline.RunCycle(input_state, favorable_environment);
  const auto favorable_measurements = favorable_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline unfavorable_pipeline(pipeline_config);
  unfavorable_pipeline.SetControlProfile(eccm_profile);
  unfavorable_pipeline.RunCycle(input_state, unfavorable_environment);
  const auto unfavorable_measurements = unfavorable_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(favorable_measurements.size(), 1u);
  ASSERT_EQ(unfavorable_measurements.size(), 1u);
  EXPECT_GT(favorable_measurements[0].raw_measurement.detection_margin_db,
            unfavorable_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest, AutoLifecycleAssemblyUsesControlProfileAdjustedKalmanUpdater) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -100.0f;
  pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
  pipeline_config.lifecycle.lifecycle_config.confirm_hits = 1;
  pipeline_config.tracking.enable_kalman_filter = true;
  pipeline_config.tracking.kalman_measurement_noise_std = 4.0f;

  environment::EnvironmentService environment_service;

  common::model::TargetFeature target(1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f);
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const common::model::TargetFeatureList cycle_1_input{target};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> baseline_manager =
      baseline_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(baseline_manager != nullptr);
  baseline_pipeline.RunCycle(cycle_1_input, environment_service);
  baseline_manager->Update(MakeLifecycleCycle(1u, 1u),
                           baseline_pipeline.GetLastTrackMeasurements());

  target.position_x = 101.0f;
  target.range_m = 101.0f;
  const common::model::TargetFeatureList cycle_2_input{target};
  baseline_pipeline.SetAssociationSeeds(baseline_manager->BuildAssociationSeeds());
  baseline_pipeline.RunCycle(cycle_2_input, environment_service);
  baseline_manager->Update(MakeLifecycleCycle(2u, 2u),
                           baseline_pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      baseline_manager->BuildAssociationSeeds();

  common::control::RadarControlProfile adaptive_profile;
  adaptive_profile.enable_adaptive_beamforming = true;
  signal::pipeline::SignalPipeline adaptive_pipeline(pipeline_config);
  adaptive_pipeline.SetControlProfile(adaptive_profile);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> adaptive_manager =
      adaptive_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(adaptive_manager != nullptr);
  adaptive_pipeline.RunCycle(cycle_1_input, environment_service);
  adaptive_manager->Update(MakeLifecycleCycle(1u, 1u),
                           adaptive_pipeline.GetLastTrackMeasurements());
  adaptive_pipeline.SetAssociationSeeds(adaptive_manager->BuildAssociationSeeds());
  adaptive_pipeline.RunCycle(cycle_2_input, environment_service);
  adaptive_manager->Update(MakeLifecycleCycle(2u, 2u),
                           adaptive_pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> adaptive_seeds =
      adaptive_manager->BuildAssociationSeeds();

  ASSERT_EQ(baseline_seeds.size(), 1u);
  ASSERT_EQ(adaptive_seeds.size(), 1u);
  EXPECT_LT(adaptive_seeds[0].gaussian_state.covariance(0, 0),
            baseline_seeds[0].gaussian_state.covariance(0, 0));
}

TEST(SignalPipelineTest, DeceptionJammingInflatesPhysicalCovarianceMoreThanNoiseSuppression) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;

  environment::EnvironmentModelConfig noise_env_config;
  environment::JammerSourceFact noise_source;
  noise_source.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = -20.0f;
  noise_source.js_db = 8.0f;
  noise_source.in_sidelobe = true;
  noise_source.confidence = 1.0f;
  noise_env_config.jammer_sources.push_back(noise_source);
  environment::EnvironmentService noise_environment(noise_env_config);

  environment::EnvironmentModelConfig deception_env_config;
  environment::JammerSourceFact deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = -20.0f;
  deception_source.js_db = 8.0f;
  deception_source.frequency_overlap_ratio = 0.9f;
  deception_source.prf_lock_risk = 0.9f;
  deception_source.confidence = 1.0f;
  deception_env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService deception_environment(deception_env_config);

  const common::model::TargetFeatureList input_state{BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline noise_pipeline(pipeline_config);
  noise_pipeline.RunCycle(input_state, noise_environment);
  const auto noise_measurements = noise_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline deception_pipeline(pipeline_config);
  deception_pipeline.RunCycle(input_state, deception_environment);
  const auto deception_measurements = deception_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(noise_measurements.size(), 1u);
  ASSERT_EQ(deception_measurements.size(), 1u);
  EXPECT_GT(deception_measurements[0].raw_measurement.measurement_covariance(0, 0),
            noise_measurements[0].raw_measurement.measurement_covariance(0, 0));
  EXPECT_GT(deception_measurements[0].raw_measurement.measurement_covariance(1, 1),
            noise_measurements[0].raw_measurement.measurement_covariance(1, 1));
}

TEST(SignalPipelineTest, AutoImmLifecycleAssemblyUsesControlProfileAdjustedImmParameters) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -100.0f;
  pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
  pipeline_config.lifecycle.enable_imm_lifecycle = true;
  pipeline_config.lifecycle.lifecycle_config.confirm_hits = 1;
  pipeline_config.lifecycle.lifecycle_config.imm_activation_policy =
      signal::pipeline::ImmActivationPolicy::kAllTracks;
  pipeline_config.lifecycle.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 4.0f};
  pipeline_config.lifecycle.imm_initial_weights = std::vector<float>{0.8f, 0.2f};

  environment::EnvironmentService environment_service;

  common::model::TargetFeature target = BuildPhysicsTarget(120.0f, 4.0f);
  const common::model::TargetFeatureList cycle_1_input{target};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> baseline_manager =
      baseline_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(baseline_manager != nullptr);
  baseline_pipeline.RunCycle(cycle_1_input, environment_service);
  baseline_manager->Update(MakeLifecycleCycle(1u, 1u),
                           baseline_pipeline.GetLastTrackMeasurements());
  baseline_manager->Update(MakeLifecycleCycle(2u, 2u), {});
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      baseline_manager->BuildAssociationSeeds();

  common::control::RadarControlProfile protected_profile;
  protected_profile.enable_eccm_rejitter = true;
  protected_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
  protected_pipeline.SetControlProfile(protected_profile);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> protected_manager =
      protected_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(protected_manager != nullptr);
  protected_pipeline.RunCycle(cycle_1_input, environment_service);
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
  const common::model::TargetFeature input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = true;
  context.jamming_detected = false;
  context.detection_margin_db = 0.0f;

  const common::model::TargetFeature output = filter.Filter(input, context);

  EXPECT_FLOAT_EQ(output.current_track_speed, input.current_track_speed);
  EXPECT_FLOAT_EQ(output.current_track_rcs, input.current_track_rcs);
}

TEST(TrackFilterTest, AppliesLossDecayAndJammingPenalty) {
  signal::tracking::TrackFilter filter;
  const common::model::TargetFeature input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = false;
  context.jamming_detected = true;
  context.detection_margin_db = -10.0f;

  const common::model::TargetFeature output = filter.Filter(input, context);

  EXPECT_LT(output.current_track_speed, input.current_track_speed);
  EXPECT_LT(output.current_track_rcs, input.current_track_rcs);
}

TEST(TrackFilterTest, DeceptionJammingRetainsMoreTrackEnergyThanNoiseSuppression) {
  signal::tracking::TrackFilter filter;
  const common::model::TargetFeature input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext noise_context;
  noise_context.detection_succeeded = false;
  noise_context.jamming_detected = true;
  noise_context.dominant_jamming_semantic = common::utils::JammingSemantic::kNoiseSuppression;
  noise_context.jamming_severity = 0.8f;
  noise_context.detection_margin_db = -10.0f;

  signal::tracking::TrackFilterContext deception_context = noise_context;
  deception_context.dominant_jamming_semantic = common::utils::JammingSemantic::kDeception;

  const common::model::TargetFeature noise_output = filter.Filter(input, noise_context);
  const common::model::TargetFeature deception_output = filter.Filter(input, deception_context);

  EXPECT_GT(deception_output.current_track_speed, noise_output.current_track_speed);
  EXPECT_GT(deception_output.current_track_rcs, noise_output.current_track_rcs);
}

TEST(SignalPipelineTest, ExposesStructuredTrackMeasurements) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  common::model::TargetFeature first(100.0f, 0.0f, 0.0f, 2.0f);
  first.position_x = 10.0f;
  first.range_m = 10.0f;
  common::model::TargetFeature second(220.0f, 0.0f, 0.0f, 5.0f);
  second.position_x = 100.0f;
  second.range_m = 100.0f;
  const common::model::TargetFeatureList cycle_1{first, second};

  signal_pipeline.RunCycle(cycle_1, environment_service);
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
  const common::model::TargetFeatureList cycle_2{first, second};
  signal_pipeline.RunCycle(cycle_2, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(second_measurements.size(), 2u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(second_measurements[1].raw_measurement.matched_existing_track);
  EXPECT_GT(second_measurements[0].raw_measurement.association_cost, 0.0f);
  EXPECT_GT(second_measurements[1].raw_measurement.association_cost, 0.0f);
  EXPECT_TRUE(second_measurements[0].raw_measurement.used_position_association);
}

TEST(SignalPipelineTest, FailsFastWhenDetectedTargetLacksCartesianPosition) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  const common::model::TargetFeatureList input_state{
      common::model::TargetFeature(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f)};

  EXPECT_DEATH_IF_SUPPORTED(signal_pipeline.RunCycle(input_state, environment_service),
                            "missing cartesian position");
}

TEST(SignalPipelineTest, UsesPositionAssociationByDefaultWhenCartesianPositionExists) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  common::model::TargetFeature first(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);
  first.position_x = 10.0f;
  first.position_y = 0.0f;
  first.position_z = 0.0f;
  first.range_m = 10.0f;

  common::model::TargetFeature second(220.0f, 0.0f, 0.0f, 5.0f, 3.0f, 0.0f, 0.0f);
  second.position_x = 100.0f;
  second.position_y = 0.0f;
  second.position_z = 0.0f;
  second.range_m = 100.0f;

  signal_pipeline.RunCycle(common::model::TargetFeatureList{first, second}, environment_service);
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
  signal_pipeline.RunCycle(common::model::TargetFeatureList{second, first}, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(second_measurements.size(), 2u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.used_position_association);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(second_measurements[1].raw_measurement.matched_existing_track);
}

TEST(SignalPipelineTest, UsesStatelessAssociationByDefaultWithoutLifecycleSeeds) {
  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  common::model::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);
  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  signal_pipeline.RunCycle(common::model::TargetFeatureList{target}, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);

  target.position_x = 10.2f;
  target.range_m = 10.2f;
  signal_pipeline.RunCycle(common::model::TargetFeatureList{target}, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_FALSE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_NE(second_measurements[0].raw_measurement.association_key, first_key);
}

TEST(SignalPipelineTest, ResetAssociationSeedModeToStatelessClearsSideChannelSeeds) {
  environment::EnvironmentService environment_service;

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
  signal_pipeline.ResetAssociationSeedModeToStateless();

  common::model::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);
  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  signal_pipeline.RunCycle(common::model::TargetFeatureList{target}, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  const std::uint64_t first_key = first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);

  target.position_x = 10.1f;
  target.range_m = 10.1f;
  signal_pipeline.RunCycle(common::model::TargetFeatureList{target}, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_FALSE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_NE(second_measurements[0].raw_measurement.association_key, first_key);
}

// ============================================================================
// PropagationModel — 边界条件补充
// ============================================================================

/// @brief 大负值杂波功率（低杂波场景，-200 dBW）透传不被钳位。
TEST(PropagationModelTest, LargeNegativeClutterPassesThroughUnchanged) {
  environment::EnvironmentSceneState scene_state;
  scene_state.base_propagation_loss_db = 5.0f;
  scene_state.atmospheric_attenuation_db = 0.0f;
  scene_state.terrain_reflection_db = 0.0f;
  scene_state.clutter_power_db = -200.0f;

  environment::simulation::PropagationModel model;
  const environment::simulation::PropagationResult result = model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 5.0f);
  EXPECT_FLOAT_EQ(result.clutter_power_db, -200.0f);
}

/// @brief 零杂波功率透传不被修改。
TEST(PropagationModelTest, ZeroClutterPassesThroughUnchanged) {
  environment::EnvironmentSceneState scene_state;
  scene_state.clutter_power_db = 0.0f;

  environment::simulation::PropagationModel model;
  EXPECT_FLOAT_EQ(model.Evaluate(scene_state).clutter_power_db, 0.0f);
}

/// @brief 正值杂波功率透传不被修改。
TEST(PropagationModelTest, PositiveClutterPassesThroughUnchanged) {
  environment::EnvironmentSceneState scene_state;
  scene_state.clutter_power_db = 15.0f;

  environment::simulation::PropagationModel model;
  EXPECT_FLOAT_EQ(model.Evaluate(scene_state).clutter_power_db, 15.0f);
}

/// @brief 各组件均为正值时，传播损耗等于三者之和。
TEST(PropagationModelTest, PositivePropagationLossIsRetained) {
  environment::EnvironmentSceneState scene_state;
  scene_state.base_propagation_loss_db = 10.0f;
  scene_state.atmospheric_attenuation_db = 3.0f;
  scene_state.terrain_reflection_db = 2.0f;
  scene_state.clutter_power_db = 0.0f;

  environment::simulation::PropagationModel model;
  const environment::simulation::PropagationResult result = model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 15.0f);
}

/// @brief 各分量均为零时，损耗和杂波均为零。
TEST(PropagationModelTest, AllZeroComponentsProduceZeroResults) {
  environment::EnvironmentSceneState scene_state;
  scene_state.base_propagation_loss_db = 0.0f;
  scene_state.atmospheric_attenuation_db = 0.0f;
  scene_state.terrain_reflection_db = 0.0f;
  scene_state.clutter_power_db = 0.0f;

  environment::simulation::PropagationModel model;
  const environment::simulation::PropagationResult result = model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 0.0f);
  EXPECT_FLOAT_EQ(result.clutter_power_db, 0.0f);
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
  environment::JammerSourceFact source =
      MakeJammerSource(environment::JammingTechnique::kUnknown, 0.0f);
  source.in_sidelobe = true;

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
  environment::JammerSourceFact source;
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

/// @brief NormalizeEmitterState：frequency_overlap_ratio > 1.0 钳位到 1.0。
TEST(EnvironmentServiceTest, EmitterOverlapRatioAboveOneIsClampedToOne) {
  environment::EnvironmentModelConfig config;
  environment::JammerSourceFact source;
  source.technique = environment::JammingTechnique::kDeception;
  source.power_db = 5.0f;
  source.frequency_overlap_ratio = 1.5f;  // 超出范围
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
  environment::JammerSourceFact source;
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
      {MakeJammerSource(environment::JammingTechnique::kUnknown, 5.0f)}));
  service.SetJammingDetectionThresholdDb(5.0f);  // 恰好等于

  EXPECT_TRUE(service.SampleEnvironment().jamming_detected);
}

/// @brief 干扰功率低于检测门限时，不判定为探测到干扰。
TEST(EnvironmentServiceTest, JammingNotDetectedWhenPowerBelowThreshold) {
  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers(
      {MakeJammerSource(environment::JammingTechnique::kUnknown, 4.9f)}));
  service.SetJammingDetectionThresholdDb(5.0f);

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);
}

/// @brief 多干扰源时，primary jammer 选取功率最大的来源。
TEST(EnvironmentServiceTest, PrimaryJammerIsTheHighestPowerSource) {
  environment::EnvironmentModelConfig config;

  environment::JammerSourceFact low;
  low.power_db = 3.0f;
  low.technique = environment::JammingTechnique::kNoiseSuppression;
  low.confidence = 1.0f;

  environment::JammerSourceFact high;
  high.power_db = 12.0f;
  high.technique = environment::JammingTechnique::kDeception;
  high.confidence = 1.0f;

  config.jammer_sources.push_back(low);
  config.jammer_sources.push_back(high);

  environment::EnvironmentService service(config);
  service.SetJammingDetectionThresholdDb(2.0f);

  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_FLOAT_EQ(snapshot.jammer_power_db, 12.0f);
  EXPECT_TRUE(snapshot.jamming_detected);
}

// ============================================================================
// TrackFilter — 衰减语义正确性测试
// ============================================================================

/// @brief 检测成功时，速度和 RCS 保持不变。
TEST(TrackFilterTest, DetectionSuccessPreservesSpeedAndRcs) {
  signal::tracking::TrackFilter filter;

  common::model::TargetFeature input(300.0f, 0.0f, 0.0f, 2.0f);
  input.position_x = 1000.0f;
  input.range_m = 1000.0f;

  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = true;

  const common::model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(output.current_track_speed, 300.0f);
  EXPECT_FLOAT_EQ(output.current_track_rcs, 2.0f);
}

/// @brief 检测失配时，速度按配置系数衰减（speed = input * ratio）。
TEST(TrackFilterTest, DetectionMissDecaysSpeedByConfiguredRatio) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.80f;
  cfg.rcs_decay_ratio_on_loss = 1.0f;  // RCS 不衰减，隔离速度分支
  signal::tracking::TrackFilter filter(cfg);

  common::model::TargetFeature input(500.0f, 0.0f, 0.0f, 2.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const common::model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(output.current_track_speed, 400.0f);  // 500 * 0.8
}

/// @brief 检测失配时，RCS 按配置系数衰减，且不低于 0.05。
TEST(TrackFilterTest, DetectionMissDecaysRcsByConfiguredRatio) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 0.70f;
  signal::tracking::TrackFilter filter(cfg);

  common::model::TargetFeature input(100.0f, 0.0f, 0.0f, 2.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const common::model::TargetFeature output = filter.Filter(input, ctx);

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

  common::model::TargetFeature state(500.0f, 0.0f, 0.0f, 1.0f);
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

  common::model::TargetFeature input(400.0f, 0.0f, 0.0f, 1.5f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const common::model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(output.current_track_speed, 400.0f);
}

/// @brief 速度不会因失配变为负数（钳位到 0）。
TEST(TrackFilterTest, SpeedNeverGoesNegativeOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.0f;  // 衰减至 0
  cfg.rcs_decay_ratio_on_loss = 1.0f;
  signal::tracking::TrackFilter filter(cfg);

  common::model::TargetFeature input(300.0f, 0.0f, 0.0f, 1.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const common::model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_GE(output.current_track_speed, 0.0f);
}

/// @brief RCS 不会因失配低于最小值 0.05。
TEST(TrackFilterTest, RcsNeverGoesBelowMinimumOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 0.0f;  // 衰减至 0
  signal::tracking::TrackFilter filter(cfg);

  common::model::TargetFeature input(100.0f, 0.0f, 0.0f, 0.01f);  // 极小 RCS
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const common::model::TargetFeature output = filter.Filter(input, ctx);

  EXPECT_GE(output.current_track_rcs, 0.05f);
}

}  // namespace tests
}  // namespace airborne_radar
