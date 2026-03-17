// Copyright 2026. All Rights Reserved.
//
// Description: 验证真实环境建模与信号处理实现的基础行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipeline.h"
#include "1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "1q/airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"

namespace airborne_radar { namespace tests {

namespace {

common::TargetFeature BuildPhysicsTarget(float range_m, float rcs) {
  common::TargetFeature target(220.0f, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f);
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

} // namespace

TEST(EnvironmentServiceTest, DetectsJammingByConfiguredThreshold) {
  environment::EnvironmentModelConfig config;
  config.jammer_power_db = 7.0f;
  config.jammer_frequency_overlap_ratio = 0.75f;
  config.jammer_prf_lock_risk = 0.60f;
  config.jammer_in_sidelobe = true;

  environment::EnvironmentService service(config);
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
  EXPECT_EQ(snapshot.jammer_sources[0].technique,
            environment::JammingTechnique::kNoiseSuppression);
  EXPECT_EQ(snapshot.jammer_sources[1].technique,
            environment::JammingTechnique::kDeception);
  EXPECT_FLOAT_EQ(snapshot.jammer_power_db, 9.0f);
  EXPECT_TRUE(snapshot.jammer_in_sidelobe);
}

TEST(SignalPipelineTest, KeepsTrackStableWhenDetectionMarginIsEnough) {
  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  common::TargetFeature target(800.0f, 0.0f, 0.0f, 2.5f, 0.0f, 0.0f,
                               0.0f);
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const common::TargetFeatureList input_state{target};

  const auto output_state =
      signal_pipeline.RunCycle(input_state, environment_service);

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_FLOAT_EQ(output_state[0].current_track_speed,
                  input_state[0].current_track_speed);
  EXPECT_FLOAT_EQ(output_state[0].current_track_rcs,
                  input_state[0].current_track_rcs);
}

TEST(SignalPipelineTest, DegradesTrackWhenDetectionMarginIsTooLow) {
  environment::EnvironmentModelConfig env_config;
  env_config.base_propagation_loss_db = 60.0f;
  env_config.atmospheric_attenuation_db = 25.0f;
  env_config.terrain_reflection_db = 15.0f;
  env_config.clutter_power_db = 20.0f;
  env_config.jammer_power_db = 12.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
    const common::TargetFeatureList input_state{common::TargetFeature(
      800.0f, 0.0f, 0.0f, 2.5f, 1.0f, 0.0f, 0.0f)};

  const auto output_state =
      signal_pipeline.RunCycle(input_state, environment_service);

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_LT(output_state[0].current_track_speed, input_state[0].current_track_speed);
  EXPECT_LT(output_state[0].current_track_rcs, input_state[0].current_track_rcs);
  EXPECT_LT(output_state[0].current_track_acceleration,
            input_state[0].current_track_acceleration);
}

TEST(SignalPipelineTest, ExposesPublicPlatformAttitudeUpdateApi) {
  signal::pipeline::SignalPipeline signal_pipeline;
  common::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 12.0f;
  platform_attitude_deg.pitch_deg = -3.0f;
  platform_attitude_deg.roll_deg = 1.5f;

  signal_pipeline.UpdatePlatformAttitude(platform_attitude_deg);

  const common::PlatformAttitudeDeg cached_platform_attitude =
      signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 12.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -3.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 1.5f);
}

TEST(SignalPipelineTest,
     ControlProfilePowerReductionLowersPhysicalDetectionMargin) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  const common::TargetFeatureList input_state{
      BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.RunCycle(input_state, environment_service);
  const auto baseline_measurements =
      baseline_pipeline.GetLastTrackMeasurements();

  common::RadarControlProfile reduced_power_profile;
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

TEST(SignalPipelineTest,
     AdaptiveBeamformingProfileTightensMeasurementCovariance) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;
  pipeline_config.detection.radar_system.antenna.nominal_az_beamwidth_deg =
      8.0f;
  pipeline_config.detection.radar_system.antenna.nominal_el_beamwidth_deg =
      8.0f;

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  const common::TargetFeatureList input_state{
      BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.RunCycle(input_state, environment_service);
  const auto baseline_measurements =
      baseline_pipeline.GetLastTrackMeasurements();

  common::RadarControlProfile adaptive_profile;
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

TEST(SignalPipelineTest,
     EccmProfileRelaxesHeuristicAssociationGateForSeededTracks) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.tracking.kalman_measurement_noise_std = 1.0f;

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  common::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const common::TargetFeatureList input_state{target};

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

  common::RadarControlProfile eccm_profile;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
  protected_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  protected_pipeline.SetControlProfile(eccm_profile);
  protected_pipeline.RunCycle(input_state, environment_service);
  const auto protected_measurements =
      protected_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(baseline_measurements.size(), 1u);
  ASSERT_EQ(protected_measurements.size(), 1u);
  EXPECT_FALSE(baseline_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(protected_measurements[0].raw_measurement.matched_existing_track);
}

TEST(SignalPipelineTest,
     AssociationQualityMetricsExposeTypeSpecificStressSummary) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.tracking.kalman_measurement_noise_std = 1.0f;

  common::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const common::TargetFeatureList input_state{target};

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
  noise_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
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
            common::JammingSemantic::kNoiseSuppression);
  EXPECT_EQ(deception_metrics.dominant_jamming_semantic,
            common::JammingSemantic::kDeception);
  EXPECT_GT(deception_metrics.jamming_severity, noise_metrics.jamming_severity);
  EXPECT_GT(noise_metrics.association_stress, 0.0f);
  EXPECT_GT(deception_metrics.association_stress, 0.0f);
}

TEST(SignalPipelineTest,
     MatchedEccmLowersAssociationStressForDeceptionJamming) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.tracking.kalman_measurement_noise_std = 1.0f;

  common::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  target.position_x = 4.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 4.0f;
  const common::TargetFeatureList input_state{target};

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

  common::RadarControlProfile protected_profile;
  protected_profile.enable_agility_frequency = true;
  protected_profile.enable_eccm_rejitter = true;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
  protected_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  protected_pipeline.SetControlProfile(protected_profile);
  protected_pipeline.RunCycle(input_state, deception_environment);
  const signal::pipeline::AssociationQualityMetrics protected_metrics =
      protected_pipeline.GetLastAssociationQualityMetrics();

  EXPECT_EQ(baseline_metrics.dominant_jamming_semantic,
            common::JammingSemantic::kDeception);
  EXPECT_EQ(protected_metrics.dominant_jamming_semantic,
            common::JammingSemantic::kDeception);
  EXPECT_LT(protected_metrics.jamming_severity, baseline_metrics.jamming_severity);
  EXPECT_LT(protected_metrics.association_stress,
            baseline_metrics.association_stress);
}

TEST(SignalPipelineTest, EccmProfileMitigatesJammingPenaltyInPhysicalDetection) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 12.0f;
  environment::EnvironmentService environment_service(env_config);

  const common::TargetFeatureList input_state{
      BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.RunCycle(input_state, environment_service);
  const auto baseline_measurements =
      baseline_pipeline.GetLastTrackMeasurements();

  common::RadarControlProfile eccm_profile;
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

TEST(SignalPipelineTest,
     DetailedJammingFactsModulatePhysicalEccmBenefit) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.enable_physics_detection = true;
  pipeline_config.detection.pulse_count = 64;
  pipeline_config.detection.radar_system.detection.cfar_pfa = 0.5f;
  pipeline_config.detection.radar_system.detection.min_snr_db = -50.0f;

  environment::EnvironmentModelConfig favorable_env_config;
  favorable_env_config.jammer_power_db = 12.0f;
  favorable_env_config.jammer_frequency_overlap_ratio = 0.9f;
  favorable_env_config.jammer_prf_lock_risk = 0.9f;
  favorable_env_config.jammer_in_sidelobe = true;
  environment::EnvironmentService favorable_environment(favorable_env_config);

  environment::EnvironmentModelConfig unfavorable_env_config;
  unfavorable_env_config.jammer_power_db = 12.0f;
  unfavorable_env_config.jammer_frequency_overlap_ratio = 0.0f;
  unfavorable_env_config.jammer_prf_lock_risk = 0.0f;
  unfavorable_env_config.jammer_in_sidelobe = false;
  environment::EnvironmentService unfavorable_environment(
      unfavorable_env_config);

  const common::TargetFeatureList input_state{
      BuildPhysicsTarget(1000.0f, 1000.0f)};

  common::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_adaptive_beamforming = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;

  signal::pipeline::SignalPipeline favorable_pipeline(pipeline_config);
  favorable_pipeline.SetControlProfile(eccm_profile);
  favorable_pipeline.RunCycle(input_state, favorable_environment);
  const auto favorable_measurements =
      favorable_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline unfavorable_pipeline(pipeline_config);
  unfavorable_pipeline.SetControlProfile(eccm_profile);
  unfavorable_pipeline.RunCycle(input_state, unfavorable_environment);
  const auto unfavorable_measurements =
      unfavorable_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(favorable_measurements.size(), 1u);
  ASSERT_EQ(unfavorable_measurements.size(), 1u);
  EXPECT_GT(favorable_measurements[0].raw_measurement.detection_margin_db,
            unfavorable_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest,
     DeceptionJammingFactsShrinkPhysicalCovarianceWhenMatchedEccmEnabled) {
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

  const common::TargetFeatureList input_state{
      BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  baseline_pipeline.RunCycle(input_state, environment_service);
  const auto baseline_measurements =
      baseline_pipeline.GetLastTrackMeasurements();

  common::RadarControlProfile protected_profile;
  protected_profile.enable_agility_frequency = true;
  protected_profile.enable_eccm_rejitter = true;
  signal::pipeline::SignalPipeline protected_pipeline(pipeline_config);
  protected_pipeline.SetControlProfile(protected_profile);
  protected_pipeline.RunCycle(input_state, environment_service);
  const auto protected_measurements =
      protected_pipeline.GetLastTrackMeasurements();

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
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  common::TargetFeature target(800.0f, 0.0f, 0.0f, 2.5f, 1.0f, 0.0f, 0.0f);
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const common::TargetFeatureList input_state{target};

  signal::pipeline::SignalPipeline baseline_pipeline;
  const auto baseline_output =
      baseline_pipeline.RunCycle(input_state, environment_service);

  common::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;
  signal::pipeline::SignalPipeline protected_pipeline;
  protected_pipeline.SetControlProfile(eccm_profile);
  const auto protected_output =
      protected_pipeline.RunCycle(input_state, environment_service);

  ASSERT_EQ(baseline_output.size(), 1u);
  ASSERT_EQ(protected_output.size(), 1u);
  EXPECT_GT(protected_output[0].current_track_speed,
            baseline_output[0].current_track_speed);
  EXPECT_GT(protected_output[0].current_track_rcs,
            baseline_output[0].current_track_rcs);
  EXPECT_GT(protected_output[0].current_track_acceleration,
            baseline_output[0].current_track_acceleration);
}

TEST(SignalPipelineTest, DetailedJammingFactsModulateHeuristicEccmRelief) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -6.0f;

  environment::EnvironmentModelConfig favorable_env_config;
  favorable_env_config.base_propagation_loss_db = 30.0f;
  favorable_env_config.atmospheric_attenuation_db = 10.0f;
  favorable_env_config.terrain_reflection_db = 5.0f;
  favorable_env_config.clutter_power_db = 12.0f;
  favorable_env_config.jammer_power_db = 12.0f;
  favorable_env_config.jammer_frequency_overlap_ratio = 0.9f;
  favorable_env_config.jammer_prf_lock_risk = 0.9f;
  favorable_env_config.jammer_in_sidelobe = true;
  environment::EnvironmentService favorable_environment(favorable_env_config);

  environment::EnvironmentModelConfig unfavorable_env_config =
      favorable_env_config;
  unfavorable_env_config.jammer_frequency_overlap_ratio = 0.0f;
  unfavorable_env_config.jammer_prf_lock_risk = 0.0f;
  unfavorable_env_config.jammer_in_sidelobe = false;
  environment::EnvironmentService unfavorable_environment(
      unfavorable_env_config);

  common::TargetFeature target(800.0f, 0.0f, 0.0f, 2.5f, 1.0f, 0.0f, 0.0f);
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const common::TargetFeatureList input_state{target};

  common::RadarControlProfile eccm_profile;
  eccm_profile.enable_sidelobe_canceller = true;
  eccm_profile.enable_adaptive_beamforming = true;
  eccm_profile.enable_agility_frequency = true;
  eccm_profile.enable_eccm_rejitter = true;
  eccm_profile.eccm_burnthrough_gain = 1.5f;

  signal::pipeline::SignalPipeline favorable_pipeline(pipeline_config);
  favorable_pipeline.SetControlProfile(eccm_profile);
  favorable_pipeline.RunCycle(input_state, favorable_environment);
  const auto favorable_measurements =
      favorable_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline unfavorable_pipeline(pipeline_config);
  unfavorable_pipeline.SetControlProfile(eccm_profile);
  unfavorable_pipeline.RunCycle(input_state, unfavorable_environment);
  const auto unfavorable_measurements =
      unfavorable_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(favorable_measurements.size(), 1u);
  ASSERT_EQ(unfavorable_measurements.size(), 1u);
  EXPECT_GT(favorable_measurements[0].raw_measurement.detection_margin_db,
            unfavorable_measurements[0].raw_measurement.detection_margin_db);
}

TEST(SignalPipelineTest,
     AutoLifecycleAssemblyUsesControlProfileAdjustedKalmanUpdater) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -100.0f;
  pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
  pipeline_config.lifecycle.lifecycle_config.confirm_hits = 1;
  pipeline_config.tracking.enable_kalman_filter = true;
  pipeline_config.tracking.kalman_measurement_noise_std = 4.0f;

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  common::TargetFeature target(1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f);
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  const common::TargetFeatureList cycle_1_input{target};

  signal::pipeline::SignalPipeline baseline_pipeline(pipeline_config);
  std::unique_ptr<signal::tracking::ITrackLifecycleManager> baseline_manager =
      baseline_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(baseline_manager != nullptr);
  baseline_pipeline.RunCycle(cycle_1_input, environment_service);
  baseline_manager->Update(MakeLifecycleCycle(1u, 1u),
                           baseline_pipeline.GetLastTrackMeasurements());

  target.position_x = 101.0f;
  target.range_m = 101.0f;
  const common::TargetFeatureList cycle_2_input{target};
  baseline_pipeline.SetAssociationSeeds(baseline_manager->BuildAssociationSeeds());
  baseline_pipeline.RunCycle(cycle_2_input, environment_service);
  baseline_manager->Update(MakeLifecycleCycle(2u, 2u),
                           baseline_pipeline.GetLastTrackMeasurements());
  const std::vector<signal::tracking::AssociationTrackSeed> baseline_seeds =
      baseline_manager->BuildAssociationSeeds();

  common::RadarControlProfile adaptive_profile;
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

TEST(SignalPipelineTest,
     DeceptionJammingInflatesPhysicalCovarianceMoreThanNoiseSuppression) {
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

  const common::TargetFeatureList input_state{
      BuildPhysicsTarget(1000.0f, 1000.0f)};

  signal::pipeline::SignalPipeline noise_pipeline(pipeline_config);
  noise_pipeline.RunCycle(input_state, noise_environment);
  const auto noise_measurements = noise_pipeline.GetLastTrackMeasurements();

  signal::pipeline::SignalPipeline deception_pipeline(pipeline_config);
  deception_pipeline.RunCycle(input_state, deception_environment);
  const auto deception_measurements =
      deception_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(noise_measurements.size(), 1u);
  ASSERT_EQ(deception_measurements.size(), 1u);
  EXPECT_GT(deception_measurements[0].raw_measurement.measurement_covariance(0, 0),
            noise_measurements[0].raw_measurement.measurement_covariance(0, 0));
  EXPECT_GT(deception_measurements[0].raw_measurement.measurement_covariance(1, 1),
            noise_measurements[0].raw_measurement.measurement_covariance(1, 1));
}

TEST(SignalPipelineTest,
     AutoImmLifecycleAssemblyUsesControlProfileAdjustedImmParameters) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -100.0f;
  pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
  pipeline_config.lifecycle.enable_imm_lifecycle = true;
  pipeline_config.lifecycle.lifecycle_config.confirm_hits = 1;
  pipeline_config.lifecycle.lifecycle_config.imm_activation_policy =
      signal::tracking::ImmActivationPolicy::kAllTracks;
  pipeline_config.lifecycle.imm_model_noise_diff_coeffs =
      std::vector<float>{0.5f, 4.0f};
  pipeline_config.lifecycle.imm_initial_weights = std::vector<float>{0.8f, 0.2f};

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  common::TargetFeature target = BuildPhysicsTarget(120.0f, 4.0f);
  const common::TargetFeatureList cycle_1_input{target};

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

  common::RadarControlProfile protected_profile;
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
  const common::TargetFeature input(800.0f, 0.0f, 0.0f, 2.5f, 0.0f,
                                    0.0f, 0.0f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = true;
  context.jamming_detected = false;
  context.detection_margin_db = 0.0f;

  const common::TargetFeature output = filter.Filter(input, context);

  EXPECT_FLOAT_EQ(output.current_track_speed, input.current_track_speed);
  EXPECT_FLOAT_EQ(output.current_track_rcs, input.current_track_rcs);
  EXPECT_FLOAT_EQ(output.current_track_acceleration,
                  input.current_track_acceleration);
}

TEST(TrackFilterTest, AppliesLossDecayAndJammingPenalty) {
  signal::tracking::TrackFilter filter;
  const common::TargetFeature input(800.0f, 0.0f, 0.0f, 2.5f, 1.0f,
                                    0.0f, 0.0f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = false;
  context.jamming_detected = true;
  context.detection_margin_db = -10.0f;

  const common::TargetFeature output = filter.Filter(input, context);

  EXPECT_LT(output.current_track_speed, input.current_track_speed);
  EXPECT_LT(output.current_track_rcs, input.current_track_rcs);
  EXPECT_LT(output.current_track_acceleration,
            input.current_track_acceleration);
}

TEST(TrackFilterTest, DeceptionJammingRetainsMoreTrackEnergyThanNoiseSuppression) {
  signal::tracking::TrackFilter filter;
  const common::TargetFeature input(800.0f, 0.0f, 0.0f, 2.5f, 1.0f,
                                    0.0f, 0.0f);

  signal::tracking::TrackFilterContext noise_context;
  noise_context.detection_succeeded = false;
  noise_context.jamming_detected = true;
  noise_context.dominant_jamming_semantic =
      common::JammingSemantic::kNoiseSuppression;
  noise_context.jamming_severity = 0.8f;
  noise_context.detection_margin_db = -10.0f;

  signal::tracking::TrackFilterContext deception_context = noise_context;
  deception_context.dominant_jamming_semantic =
      common::JammingSemantic::kDeception;

  const common::TargetFeature noise_output = filter.Filter(input, noise_context);
  const common::TargetFeature deception_output =
      filter.Filter(input, deception_context);

  EXPECT_GT(deception_output.current_track_speed, noise_output.current_track_speed);
  EXPECT_GT(deception_output.current_track_rcs, noise_output.current_track_rcs);
  EXPECT_GT(deception_output.current_track_acceleration,
            noise_output.current_track_acceleration);
}

TEST(SignalPipelineTest, ExposesStructuredTrackMeasurements) {
  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  common::TargetFeature first(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f,
                              0.0f);
  first.position_x = 10.0f;
  first.range_m = 10.0f;
  common::TargetFeature second(220.0f, 0.0f, 0.0f, 5.0f, 3.0f, 0.0f,
                               0.0f);
  second.position_x = 100.0f;
  second.range_m = 100.0f;
  const common::TargetFeatureList cycle_1{first, second};

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
    first.current_track_acceleration = 1.1f;
    first.position_x = 11.0f;
    first.range_m = 11.0f;
    second.current_track_speed = 219.5f;
    second.current_track_rcs = 4.9f;
    second.current_track_acceleration = 2.9f;
    second.position_x = 101.0f;
    second.range_m = 101.0f;
    signal::tracking::AssociationTrackSeed first_seed;
    first_seed.association_key =
        first_measurements[0].raw_measurement.association_key;
    first_seed.has_position = true;
    first_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
    first_seed.has_gaussian_state = true;
    first_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
    first_seed.gaussian_state.mean(0) = 10.0f;
    first_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;

    signal::tracking::AssociationTrackSeed second_seed;
    second_seed.association_key =
        first_measurements[1].raw_measurement.association_key;
    second_seed.has_position = true;
    second_seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
    second_seed.has_gaussian_state = true;
    second_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
    second_seed.gaussian_state.mean(0) = 100.0f;
    second_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;

    signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>{first_seed,
                                 second_seed});
    const common::TargetFeatureList cycle_2{first, second};
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
  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
    const common::TargetFeatureList input_state{common::TargetFeature(
      100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f)};

  EXPECT_DEATH_IF_SUPPORTED(signal_pipeline.RunCycle(input_state, environment_service),
                            "missing cartesian position");
}

TEST(SignalPipelineTest,
     UsesPositionAssociationByDefaultWhenCartesianPositionExists) {
  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  common::TargetFeature first(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f,
                              0.0f);
  first.position_x = 10.0f;
  first.position_y = 0.0f;
  first.position_z = 0.0f;
  first.range_m = 10.0f;

  common::TargetFeature second(220.0f, 0.0f, 0.0f, 5.0f, 3.0f, 0.0f,
                               0.0f);
  second.position_x = 100.0f;
  second.position_y = 0.0f;
  second.position_z = 0.0f;
  second.range_m = 100.0f;

  signal_pipeline.RunCycle(common::TargetFeatureList{first, second},
                           environment_service);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(first_measurements.size(), 2u);
  EXPECT_TRUE(first_measurements[0].raw_measurement.used_position_association);
  EXPECT_TRUE(first_measurements[0].raw_measurement.has_cartesian_position);
  EXPECT_TRUE(first_measurements[1].raw_measurement.used_position_association);

    signal::tracking::AssociationTrackSeed first_seed;
    first_seed.association_key =
        first_measurements[0].raw_measurement.association_key;
    first_seed.has_position = true;
    first_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
    first_seed.has_gaussian_state = true;
    first_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
    first_seed.gaussian_state.mean(0) = 10.0f;
    first_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;

    signal::tracking::AssociationTrackSeed second_seed;
    second_seed.association_key =
        first_measurements[1].raw_measurement.association_key;
    second_seed.has_position = true;
    second_seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
    second_seed.has_gaussian_state = true;
    second_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
    second_seed.gaussian_state.mean(0) = 100.0f;
    second_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;

    signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>{first_seed,
                                 second_seed});

  first.position_x = 11.0f;
  second.position_x = 101.0f;
  signal_pipeline.RunCycle(common::TargetFeatureList{second, first},
                           environment_service);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();

  ASSERT_EQ(second_measurements.size(), 2u);
  EXPECT_TRUE(second_measurements[0].raw_measurement.used_position_association);
  EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(second_measurements[1].raw_measurement.matched_existing_track);
}

TEST(SignalPipelineTest,
     UsesStatelessAssociationByDefaultWithoutLifecycleSeeds) {
  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  common::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f,
                               0.0f);
  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  signal_pipeline.RunCycle(common::TargetFeatureList{target},
                           environment_service);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);
  const std::uint64_t first_key =
      first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);

  target.position_x = 10.2f;
  target.range_m = 10.2f;
  signal_pipeline.RunCycle(common::TargetFeatureList{target},
                           environment_service);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_FALSE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_NE(second_measurements[0].raw_measurement.association_key, first_key);
}

TEST(SignalPipelineTest,
   ResetAssociationSeedModeToStatelessClearsSideChannelSeeds) {
  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

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

  common::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f,
                               0.0f);
  target.position_x = 10.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 10.0f;

  signal_pipeline.RunCycle(common::TargetFeatureList{target},
                           environment_service);
  const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(first_measurements.size(), 1u);
  const std::uint64_t first_key =
      first_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);

  target.position_x = 10.1f;
  target.range_m = 10.1f;
  signal_pipeline.RunCycle(common::TargetFeatureList{target},
                           environment_service);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_FALSE(second_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_NE(second_measurements[0].raw_measurement.association_key, first_key);
}

} } // namespace airborne_radar::tests
