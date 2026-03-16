// Copyright 2026. All Rights Reserved.
//
// Description: 验证真实环境建模与信号处理实现的基础行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"

namespace airborne_radar { namespace tests {

TEST(EnvironmentServiceTest, DetectsJammingByConfiguredThreshold) {
  environment::EnvironmentModelConfig config;
  config.jammer_power_db = 7.0f;

  environment::EnvironmentService service(config);
  service.SetJammingDetectionThresholdDb(6.0f);

  const auto snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jamming_detected);
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
  EXPECT_FALSE(first_measurements[0].matched_existing_track);
  EXPECT_FALSE(first_measurements[1].matched_existing_track);
  EXPECT_EQ(first_measurements[0].source_index, 0u);
  EXPECT_EQ(first_measurements[1].source_index, 1u);
    EXPECT_TRUE(first_measurements[0].has_cartesian_position);
  EXPECT_GT(first_measurements[0].observed_speed, 0.0f);
  EXPECT_GT(first_measurements[0].detection_margin_db, -2.0f);
    EXPECT_TRUE(first_measurements[0].used_position_association);

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
    first_seed.association_key = first_measurements[0].association_key;
    first_seed.has_position = true;
    first_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
    first_seed.has_gaussian_state = true;
    first_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
    first_seed.gaussian_state.mean(0) = 10.0f;
    first_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;

    signal::tracking::AssociationTrackSeed second_seed;
    second_seed.association_key = first_measurements[1].association_key;
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
  EXPECT_TRUE(second_measurements[0].matched_existing_track);
  EXPECT_TRUE(second_measurements[1].matched_existing_track);
  EXPECT_GT(second_measurements[0].association_cost, 0.0f);
  EXPECT_GT(second_measurements[1].association_cost, 0.0f);
  EXPECT_TRUE(second_measurements[0].used_position_association);
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
  EXPECT_TRUE(first_measurements[0].used_position_association);
  EXPECT_TRUE(first_measurements[0].has_cartesian_position);
  EXPECT_TRUE(first_measurements[1].used_position_association);

    signal::tracking::AssociationTrackSeed first_seed;
    first_seed.association_key = first_measurements[0].association_key;
    first_seed.has_position = true;
    first_seed.position = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
    first_seed.has_gaussian_state = true;
    first_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
    first_seed.gaussian_state.mean(0) = 10.0f;
    first_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;

    signal::tracking::AssociationTrackSeed second_seed;
    second_seed.association_key = first_measurements[1].association_key;
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
  EXPECT_TRUE(second_measurements[0].used_position_association);
  EXPECT_TRUE(second_measurements[0].matched_existing_track);
  EXPECT_TRUE(second_measurements[1].matched_existing_track);
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
  EXPECT_FALSE(first_measurements[0].matched_existing_track);
  const std::uint64_t first_key = first_measurements[0].association_key;
  EXPECT_NE(first_key, 0u);

  target.position_x = 10.2f;
  target.range_m = 10.2f;
  signal_pipeline.RunCycle(common::TargetFeatureList{target},
                           environment_service);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_FALSE(second_measurements[0].matched_existing_track);
  EXPECT_NE(second_measurements[0].association_key, first_key);
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
  const std::uint64_t first_key = first_measurements[0].association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(first_measurements[0].matched_existing_track);

  target.position_x = 10.1f;
  target.range_m = 10.1f;
  signal_pipeline.RunCycle(common::TargetFeatureList{target},
                           environment_service);
  const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(second_measurements.size(), 1u);
  EXPECT_FALSE(second_measurements[0].matched_existing_track);
  EXPECT_NE(second_measurements[0].association_key, first_key);
}

} } // namespace airborne_radar::tests
