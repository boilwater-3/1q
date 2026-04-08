// Copyright 2026. All Rights Reserved.
//
// @file signal_association_test.cpp
// @brief 验证数据关联模块的关键行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/model/TargetFeature.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/association/DistanceMetric.h"
#include "airborne_radar/signal/association/Gater.h"
#include "airborne_radar/signal/association/Hypothesiser.h"

namespace airborne_radar {
namespace tests {

namespace {

model::TargetFeature MakeTarget(float speed, float rcs) {
  return model::TargetFeature(speed, 0.0f, 0.0f, rcs);
}

model::TargetFeature MakePositionTarget(float x, float y, float z) {
  model::TargetFeature target(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);
  target.has_cartesian_position = true;
  target.position_x = x;
  target.position_y = y;
  target.position_z = z;
  return target;
}

signal::tracking::AssociationTrackSeed MakeExternalSeed(std::uint64_t key,
                                                        const Eigen::Vector3f& position) {
  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = key;
  seed.has_position = true;
  seed.position = position;
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.mean(0) = position(0);
  seed.gaussian_state.mean(2) = position(1);
  seed.gaussian_state.mean(4) = position(2);
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;
  return seed;
}

}  // namespace

TEST(DataAssociationEngineTest, KeepsStableAssociationAcrossCycles) {
  signal::association::DataAssociationEngine engine;

  const model::TargetFeatureList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f),
                                          MakePositionTarget(100.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 2u);
  EXPECT_NE(keys_1[0], 0u);
  EXPECT_NE(keys_1[1], 0u);
  EXPECT_NE(keys_1[0], keys_1[1]);

  const std::vector<signal::tracking::AssociationTrackSeed> seeds{
      MakeExternalSeed(keys_1[0], Eigen::Vector3f(10.0f, 0.0f, 0.0f)),
      MakeExternalSeed(keys_1[1], Eigen::Vector3f(100.0f, 0.0f, 0.0f))};
  engine.SetAssociationSeeds(seeds);

  const model::TargetFeatureList cycle_2{MakePositionTarget(11.0f, 0.0f, 0.0f),
                                          MakePositionTarget(101.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U, 1U};

  const std::vector<std::uint64_t> keys_2 = engine.Associate(cycle_2, detected_2);
  ASSERT_EQ(keys_2.size(), 2u);
  EXPECT_EQ(keys_2[0], keys_1[0]);
  EXPECT_EQ(keys_2[1], keys_1[1]);
}

TEST(DataAssociationEngineTest, StatelessWithoutSeedsMakesAssociationAcrossCyclesIndependent) {
  signal::association::DataAssociationEngine engine;

  const model::TargetFeatureList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};

  const signal::association::AssociationResult first_result =
      engine.AssociateDetections(cycle_1, detected_1);
  ASSERT_EQ(first_result.target_keys.size(), 1u);
  ASSERT_NE(first_result.target_keys[0], 0u);

  const model::TargetFeatureList cycle_2{MakePositionTarget(10.5f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U};
  const signal::association::AssociationResult second_result =
      engine.AssociateDetections(cycle_2, detected_2);

  ASSERT_EQ(second_result.target_keys.size(), 1u);
  EXPECT_NE(second_result.target_keys[0], 0u);
  EXPECT_NE(second_result.target_keys[0], first_result.target_keys[0]);
  EXPECT_TRUE(second_result.matches.empty());
  ASSERT_EQ(second_result.unassociated_target_indices.size(), 1u);
  EXPECT_EQ(second_result.unassociated_target_indices[0], 0u);
}

TEST(DataAssociationEngineTest, StatelessModeDoesNotReusePreviousCycleAssociations) {
  signal::association::DataAssociationEngine engine;

  const model::TargetFeatureList warmup_cycle{MakePositionTarget(50.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected{1U};
  const signal::association::AssociationResult first_result =
      engine.AssociateDetections(warmup_cycle, detected);
  ASSERT_EQ(first_result.target_keys.size(), 1u);
  ASSERT_NE(first_result.target_keys[0], 0u);

  const signal::association::AssociationResult second_result =
      engine.AssociateDetections(warmup_cycle, detected);
  ASSERT_EQ(second_result.target_keys.size(), 1u);
  ASSERT_NE(second_result.target_keys[0], 0u);
  EXPECT_NE(second_result.target_keys[0], first_result.target_keys[0]);
  EXPECT_TRUE(second_result.matches.empty());
}

TEST(DataAssociationEngineTest, ExternalSeedsStillDriveAssociationWhenProvided) {
  signal::association::DataAssociationEngine engine;

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 88u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f(20.0f, 0.0f, 0.0f);
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.mean(0) = 20.0f;
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 4.0f;
  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed));

  const model::TargetFeatureList targets{MakePositionTarget(20.5f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected{1U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(targets, detected);

  ASSERT_EQ(result.target_keys.size(), 1u);
  EXPECT_EQ(result.target_keys[0], 88u);
  EXPECT_TRUE(result.used_external_association_seeds);
  ASSERT_EQ(result.matches.size(), 1u);
  EXPECT_EQ(result.matches[0].association_key, 88u);
}

TEST(DataAssociationEngineTest, ExternalAssociationSeedMissingGaussianStateIsSkipped) {
  signal::association::DataAssociationEngine engine;

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 77u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f(20.0f, 0.0f, 0.0f);
  seed.has_gaussian_state = false;

  // Invalid seed is silently skipped (contract violation logged, not aborted)
  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed));

  // Subsequent association should work without the invalid seed
  const model::TargetFeatureList targets{MakePositionTarget(20.5f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected{1U};
  const signal::association::AssociationResult result = engine.AssociateDetections(targets, detected);
  EXPECT_FALSE(result.used_external_association_seeds);
}

TEST(DataAssociationEngineTest, ExternalAssociationSeedWithReservedKeyIsRejected) {
  signal::association::DataAssociationEngine engine;

  signal::tracking::AssociationTrackSeed seed = MakeExternalSeed(0u, Eigen::Vector3f(20.0f, 0.0f, 0.0f));

  EXPECT_FALSE(engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed)));

  const model::TargetFeatureList targets{MakePositionTarget(20.5f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected{1U};
  const signal::association::AssociationResult result = engine.AssociateDetections(targets, detected);
  ASSERT_EQ(result.target_keys.size(), 1u);
  EXPECT_FALSE(result.used_external_association_seeds);
  EXPECT_TRUE(result.matches.empty());
  EXPECT_NE(result.target_keys[0], 0u);
}

TEST(DataAssociationEngineTest, HandlesCrossedMeasurementsByCostMinimization) {
  signal::association::DataAssociationEngine engine;

  const model::TargetFeatureList cycle_1{MakePositionTarget(20.0f, 0.0f, 0.0f),
                                          MakePositionTarget(200.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 2u);

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>{
      MakeExternalSeed(keys_1[0], Eigen::Vector3f(20.0f, 0.0f, 0.0f)),
      MakeExternalSeed(keys_1[1], Eigen::Vector3f(200.0f, 0.0f, 0.0f))});

  // The measurement order is swapped, but position-space cost should preserve identity.
  const model::TargetFeatureList cycle_2{MakePositionTarget(201.0f, 0.0f, 0.0f),
                                          MakePositionTarget(19.5f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U, 1U};

  const std::vector<std::uint64_t> keys_2 = engine.Associate(cycle_2, detected_2);
  ASSERT_EQ(keys_2.size(), 2u);
  EXPECT_EQ(keys_2[0], keys_1[1]);
  EXPECT_EQ(keys_2[1], keys_1[0]);
}

TEST(DataAssociationEngineTest, AssignsNewKeysForNewMeasurements) {
  signal::association::DataAssociationConfig config;
  config.unassigned_cost = 9.0f;
  signal::association::DataAssociationEngine engine(config);

  const model::TargetFeatureList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 1u);
  ASSERT_NE(keys_1[0], 0u);

  // Very far position should exceed gating threshold and receive a new key.
  const model::TargetFeatureList cycle_2{MakePositionTarget(200.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U};
  const std::vector<std::uint64_t> keys_2 = engine.Associate(cycle_2, detected_2);
  ASSERT_EQ(keys_2.size(), 1u);
  EXPECT_NE(keys_2[0], 0u);
  EXPECT_NE(keys_2[0], keys_1[0]);
}

TEST(DataAssociationEngineTest, ReportsMatchesMissesAndUnassociatedTargets) {
  signal::association::DataAssociationEngine engine;

  const model::TargetFeatureList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 1u);
  ASSERT_NE(keys_1[0], 0u);

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>{
      MakeExternalSeed(keys_1[0], Eigen::Vector3f(10.0f, 0.0f, 0.0f))});

  const model::TargetFeatureList cycle_2{MakePositionTarget(11.0f, 0.0f, 0.0f),
                                          MakePositionTarget(200.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U, 1U};

  const signal::association::AssociationResult result =
      engine.AssociateDetections(cycle_2, detected_2);

  ASSERT_EQ(result.target_keys.size(), 2u);
  ASSERT_EQ(result.matches.size(), 1u);
  EXPECT_EQ(result.matches[0].association_key, keys_1[0]);
  EXPECT_EQ(result.matches[0].target_index, 0u);
  EXPECT_TRUE(result.missed_track_keys.empty());
  ASSERT_EQ(result.unassociated_target_indices.size(), 1u);
  EXPECT_EQ(result.unassociated_target_indices[0], 1u);
  EXPECT_EQ(result.target_keys[0], keys_1[0]);
  EXPECT_NE(result.target_keys[1], 0u);
  EXPECT_NE(result.target_keys[1], keys_1[0]);

  EXPECT_EQ(result.quality_metrics.prior_track_count, 1u);
  EXPECT_EQ(result.quality_metrics.detection_count, 2u);
  EXPECT_EQ(result.quality_metrics.matched_count, 1u);
  EXPECT_EQ(result.quality_metrics.new_track_count, 1u);
  EXPECT_EQ(result.quality_metrics.missed_track_count, 0u);
  EXPECT_FLOAT_EQ(result.quality_metrics.match_rate, 0.5f);
  EXPECT_FLOAT_EQ(result.quality_metrics.new_track_rate, 0.5f);
  EXPECT_FLOAT_EQ(result.quality_metrics.missed_track_rate, 0.0f);
  EXPECT_GT(result.quality_metrics.mean_match_cost, 0.0f);
  EXPECT_FLOAT_EQ(result.quality_metrics.p95_match_cost, result.matches[0].cost);
}

TEST(DataAssociationEngineTest, ReportsMissedTrackKeysWhenNoDetectionArrives) {
  signal::association::DataAssociationEngine engine;

  const model::TargetFeatureList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 1u);

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>{
      MakeExternalSeed(keys_1[0], Eigen::Vector3f(10.0f, 0.0f, 0.0f))});

  const model::TargetFeatureList cycle_2{MakePositionTarget(10.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{0U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(cycle_2, detected_2);

  EXPECT_TRUE(result.matches.empty());
  EXPECT_TRUE(result.unassociated_target_indices.empty());
  ASSERT_EQ(result.missed_track_keys.size(), 1u);
  EXPECT_EQ(result.missed_track_keys[0], keys_1[0]);
  ASSERT_EQ(result.target_keys.size(), 1u);
  EXPECT_EQ(result.target_keys[0], 0u);

  EXPECT_EQ(result.quality_metrics.prior_track_count, 1u);
  EXPECT_EQ(result.quality_metrics.detection_count, 0u);
  EXPECT_EQ(result.quality_metrics.matched_count, 0u);
  EXPECT_EQ(result.quality_metrics.new_track_count, 0u);
  EXPECT_EQ(result.quality_metrics.missed_track_count, 1u);
  EXPECT_FLOAT_EQ(result.quality_metrics.match_rate, 0.0f);
  EXPECT_FLOAT_EQ(result.quality_metrics.new_track_rate, 0.0f);
  EXPECT_FLOAT_EQ(result.quality_metrics.missed_track_rate, 1.0f);
  EXPECT_FLOAT_EQ(result.quality_metrics.mean_match_cost, 0.0f);
  EXPECT_FLOAT_EQ(result.quality_metrics.p95_match_cost, 0.0f);
}

TEST(DataAssociationEngineTest, UsesCartesianPositionByDefaultWhenPositionAvailable) {
  signal::association::DataAssociationConfig config;
  config.kalman_measurement_noise_std = 2.0f;
  signal::association::DataAssociationEngine engine(config);

  const model::TargetFeatureList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f),
                                          MakePositionTarget(100.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

  const signal::association::AssociationResult first_result =
      engine.AssociateDetections(cycle_1, detected_1);
  const std::vector<std::uint64_t>& keys_1 = first_result.target_keys;
  ASSERT_EQ(keys_1.size(), 2u);
  ASSERT_NE(keys_1[0], 0u);
  ASSERT_NE(keys_1[1], 0u);
  EXPECT_TRUE(first_result.used_position_association);

  const model::TargetFeatureList cycle_2{MakePositionTarget(101.0f, 0.0f, 0.0f),
                                          MakePositionTarget(11.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U, 1U};

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>{
      MakeExternalSeed(keys_1[0], Eigen::Vector3f(10.0f, 0.0f, 0.0f)),
      MakeExternalSeed(keys_1[1], Eigen::Vector3f(100.0f, 0.0f, 0.0f))});

  const signal::association::AssociationResult second_result =
      engine.AssociateDetections(cycle_2, detected_2);
  const std::vector<std::uint64_t>& keys_2 = second_result.target_keys;
  ASSERT_EQ(keys_2.size(), 2u);
  EXPECT_EQ(keys_2[0], keys_1[1]);
  EXPECT_EQ(keys_2[1], keys_1[0]);
  EXPECT_TRUE(second_result.used_position_association);
  EXPECT_TRUE(second_result.used_external_association_seeds);
}

TEST(DataAssociationEngineTest, DetectedTargetMissingPositionCompletesGracefully) {
  signal::association::DataAssociationEngine engine;

  const model::TargetFeatureList cycle_1{MakeTarget(100.0f, 2.0f), MakeTarget(220.0f, 5.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

  // Contract violation is logged and dropped before position association.
  const signal::association::AssociationResult result = engine.AssociateDetections(cycle_1, detected_1);
  EXPECT_EQ(result.target_keys.size(), 2u);
  EXPECT_EQ(result.target_keys[0], 0u);
  EXPECT_EQ(result.target_keys[1], 0u);
  EXPECT_TRUE(result.matches.empty());
  EXPECT_TRUE(result.unassociated_target_indices.empty());
}

TEST(DataAssociationEngineTest, DetectedTargetWithCoordinatesButMissingPositionFlagCompletesGracefully) {
  signal::association::DataAssociationEngine engine;

  model::TargetFeature target = MakeTarget(100.0f, 2.0f);
  target.position_x = 20.0f;
  target.position_y = 1.0f;
  target.position_z = 0.0f;
  target.has_cartesian_position = false;

  const model::TargetFeatureList cycle_1{target};
  const std::vector<std::uint8_t> detected_1{1U};
  // Contract violation is logged and dropped before position association.
  const signal::association::AssociationResult result = engine.AssociateDetections(cycle_1, detected_1);
  EXPECT_EQ(result.target_keys.size(), 1u);
  EXPECT_EQ(result.target_keys[0], 0u);
  EXPECT_TRUE(result.matches.empty());
  EXPECT_TRUE(result.unassociated_target_indices.empty());
}

TEST(DataAssociationEngineTest, DropsOnlyInvalidDetectedTargetsWithoutPosition) {
  signal::association::DataAssociationEngine engine;

  model::TargetFeature invalid_target = MakeTarget(100.0f, 2.0f);
  model::TargetFeature valid_target = MakePositionTarget(30.0f, 0.0f, 0.0f);

  const model::TargetFeatureList cycle_1{invalid_target, valid_target};
  const std::vector<std::uint8_t> detected_1{1U, 1U};
  const signal::association::AssociationResult result = engine.AssociateDetections(cycle_1, detected_1);
  ASSERT_EQ(result.target_keys.size(), 2u);
  EXPECT_EQ(result.target_keys[0], 0u);
  EXPECT_NE(result.target_keys[1], 0u);
  ASSERT_EQ(result.unassociated_target_indices.size(), 1u);
  EXPECT_EQ(result.unassociated_target_indices[0], 1u);
}

TEST(DataAssociationEngineTest, DetectedOriginWithPositionFlagPassesValidation) {
  signal::association::DataAssociationEngine engine;

  model::TargetFeature target = MakeTarget(120.0f, 1.5f);
  target.has_cartesian_position = true;
  target.position_x = 0.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;

  const model::TargetFeatureList cycle_1{target};
  const std::vector<std::uint8_t> detected_1{1U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(cycle_1, detected_1);
  ASSERT_EQ(result.target_keys.size(), 1u);
  EXPECT_NE(result.target_keys[0], 0u);
}

TEST(DataAssociationEngineTest, DynamicMeasurementCovarianceChangesPositionAssociationGate) {
  signal::association::DataAssociationConfig config;
  config.unassigned_cost = 9.0f;

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 7u;
  seed.has_position = true;
  seed.position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Zero();

  const model::TargetFeatureList targets{MakePositionTarget(4.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected{1U};

  signal::association::DataAssociationEngine tight_engine(config);
  tight_engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  const std::vector<signal::tracking::MeasurementCovariance> tight_covariances(
      1, Eigen::Matrix3f::Identity());
  const signal::association::AssociationResult tight_result =
      tight_engine.AssociateDetections(targets, detected, tight_covariances);
  ASSERT_EQ(tight_result.target_keys.size(), 1u);
  EXPECT_NE(tight_result.target_keys[0], 7u);
  EXPECT_TRUE(tight_result.matches.empty());

  signal::association::DataAssociationEngine loose_engine(config);
  loose_engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed));
  const std::vector<signal::tracking::MeasurementCovariance> loose_covariances(
      1, Eigen::Matrix3f::Identity() * 16.0f);
  const signal::association::AssociationResult loose_result =
      loose_engine.AssociateDetections(targets, detected, loose_covariances);
  ASSERT_EQ(loose_result.target_keys.size(), 1u);
  EXPECT_EQ(loose_result.target_keys[0], 7u);
  ASSERT_EQ(loose_result.matches.size(), 1u);
  EXPECT_EQ(loose_result.matches[0].association_key, 7u);
}

TEST(DataAssociationEngineTest, ExternalAssociationSeedsAffectOnlyCurrentCycle) {
  signal::association::DataAssociationEngine engine;

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 42;
  seed.has_position = true;
  seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean(0) = 100.0f;
  seed.gaussian_state.mean(2) = 0.0f;
  seed.gaussian_state.mean(4) = 0.0f;
  seed.gaussian_state.covariance = signal::tracking::StateCovariance::Identity() * 25.0f;

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed));

  const model::TargetFeatureList cycle_1{MakePositionTarget(101.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const signal::association::AssociationResult first_result =
      engine.AssociateDetections(cycle_1, detected_1);

  ASSERT_EQ(first_result.target_keys.size(), 1u);
  EXPECT_EQ(first_result.target_keys[0], 42u);
  EXPECT_TRUE(first_result.used_position_association);
  EXPECT_TRUE(first_result.used_external_association_seeds);

  const model::TargetFeatureList cycle_2{MakePositionTarget(102.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U};
  const signal::association::AssociationResult second_result =
      engine.AssociateDetections(cycle_2, detected_2);

  ASSERT_EQ(second_result.target_keys.size(), 1u);
  EXPECT_NE(second_result.target_keys[0], 42u);
  EXPECT_TRUE(second_result.matches.empty());
  ASSERT_EQ(second_result.unassociated_target_indices.size(), 1u);
  EXPECT_EQ(second_result.unassociated_target_indices[0], 0u);
  EXPECT_FALSE(second_result.used_external_association_seeds);
}

TEST(DataAssociationEngineTest, EmptyExternalSeedsKeepsAssociationStateless) {
  signal::association::DataAssociationEngine engine;

  const model::TargetFeatureList warmup_cycle{MakePositionTarget(50.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> warmup_detected{1U};
  const signal::association::AssociationResult warmup_result =
      engine.AssociateDetections(warmup_cycle, warmup_detected);

  ASSERT_EQ(warmup_result.target_keys.size(), 1u);
  const std::uint64_t warmup_key = warmup_result.target_keys[0];
  ASSERT_NE(warmup_key, 0u);

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>());

  const model::TargetFeatureList cycle_2{MakePositionTarget(51.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(cycle_2, detected_2);

  ASSERT_EQ(result.target_keys.size(), 1u);
  EXPECT_FALSE(result.used_external_association_seeds);
  EXPECT_NE(result.target_keys[0], warmup_key);
  EXPECT_TRUE(result.matches.empty());
  ASSERT_EQ(result.unassociated_target_indices.size(), 1u);
  EXPECT_EQ(result.unassociated_target_indices[0], 0u);
}

TEST(CostThresholdGaterTest, RejectsHypothesesOutsideThreshold) {
  const signal::association::CostThresholdGater gater(9.0f);

  EXPECT_TRUE(gater.Accept(8.5f));
  EXPECT_TRUE(gater.Accept(9.0f));
  EXPECT_FALSE(gater.Accept(9.1f));
}

TEST(DenseCostHypothesiserTest, NullDependenciesFailFast) {
  const signal::association::CostThresholdGater gater(9.0f);
  const signal::association::MahalanobisDistanceMetric metric(40.0f, 8.0f, 10.0f);

  EXPECT_DEATH_IF_SUPPORTED(
      signal::association::DenseCostHypothesiser(
          static_cast<signal::association::IDistanceMetric*>(nullptr), &gater),
      "requires distance metric and gater");
  EXPECT_DEATH_IF_SUPPORTED(
      signal::association::DenseCostHypothesiser(
          &metric, static_cast<const signal::association::IGater*>(nullptr)),
      "requires distance metric and gater");
}

TEST(DenseCostHypothesiserTest, GeneratesOnlyGatedHypotheses) {
  const signal::association::MahalanobisDistanceMetric metric(40.0f, 8.0f, 10.0f);
  const signal::association::CostThresholdGater gater(9.0f);
  const signal::association::DenseCostHypothesiser hypothesiser(&metric, &gater);

  const signal::association::FeatureVectorList predicted_tracks{
      Eigen::Vector3f(100.0f, 2.0f, 1.0f)};
  const signal::association::FeatureVectorList measurements{Eigen::Vector3f(101.0f, 2.1f, 1.1f),
                                                            Eigen::Vector3f(700.0f, 35.0f, 20.0f)};

  const std::vector<signal::association::AssociationHypothesis> hypotheses =
      hypothesiser.Generate(predicted_tracks, measurements);

  ASSERT_EQ(hypotheses.size(), 1u);
  EXPECT_EQ(hypotheses[0].track_index, 0u);
  EXPECT_EQ(hypotheses[0].measurement_index, 0u);
  EXPECT_LT(hypotheses[0].cost, 9.0f);
}

TEST(DenseCostHypothesiserTest, UsesTrackWiseInnovationCovarianceForFullMahalanobis) {
  signal::association::FullMahalanobisDistanceMetric metric(Eigen::Matrix3f::Identity());
  const signal::association::CostThresholdGater gater(1.5f);
  const signal::association::DenseCostHypothesiser hypothesiser(&metric, &gater);

  const signal::association::FeatureVectorList predicted_tracks{Eigen::Vector3f(0.0f, 0.0f, 0.0f)};
  const signal::association::FeatureVectorList measurements{Eigen::Vector3f(2.0f, 0.0f, 0.0f)};

  const std::vector<signal::association::AssociationHypothesis> no_match = hypothesiser.Generate(
      predicted_tracks, measurements, std::vector<Eigen::Matrix3f>(1, Eigen::Matrix3f::Identity()));
  EXPECT_TRUE(no_match.empty());

  Eigen::Matrix3f wide_covariance = Eigen::Matrix3f::Identity() * 4.0f;
  const std::vector<signal::association::AssociationHypothesis> accepted = hypothesiser.Generate(
      predicted_tracks, measurements, std::vector<Eigen::Matrix3f>(1, wide_covariance));
  ASSERT_EQ(accepted.size(), 1u);
  EXPECT_NEAR(accepted[0].cost, 1.0f, 1e-3f);
}

TEST(DenseCostHypothesiserTest, MismatchedTrackWiseInnovationCovarianceFailsFast) {
  signal::association::FullMahalanobisDistanceMetric metric(Eigen::Matrix3f::Identity());
  const signal::association::CostThresholdGater gater(1.5f);
  const signal::association::DenseCostHypothesiser hypothesiser(&metric, &gater);

  const signal::association::FeatureVectorList predicted_tracks{
      Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f)};
  const signal::association::FeatureVectorList measurements{Eigen::Vector3f(2.0f, 0.0f, 0.0f)};

  EXPECT_DEATH_IF_SUPPORTED(
      hypothesiser.Generate(predicted_tracks, measurements,
                            std::vector<Eigen::Matrix3f>(1, Eigen::Matrix3f::Identity())),
      "innovation_covariances size must match predicted_tracks size");
}

TEST(DenseCostHypothesiserTest, ConstMetricRejectsDynamicInnovationCovariance) {
  const signal::association::FullMahalanobisDistanceMetric metric(Eigen::Matrix3f::Identity());
  const signal::association::CostThresholdGater gater(1.5f);
  const signal::association::DenseCostHypothesiser hypothesiser(&metric, &gater);

  const signal::association::FeatureVectorList predicted_tracks{Eigen::Vector3f(0.0f, 0.0f, 0.0f)};
  const signal::association::FeatureVectorList measurements{Eigen::Vector3f(2.0f, 0.0f, 0.0f)};

  EXPECT_DEATH_IF_SUPPORTED(
      hypothesiser.Generate(predicted_tracks, measurements,
                            std::vector<Eigen::Matrix3f>(1, Eigen::Matrix3f::Identity())),
      "requires FullMahalanobisDistanceMetric");
}

TEST(LapjvSolverTest, InvalidMatrixFailsFast) {
  const signal::association::LapjvSolver solver;

  Eigen::MatrixXf empty_matrix;
  EXPECT_DEATH_IF_SUPPORTED(solver.Solve(empty_matrix), "requires a non-empty square cost matrix");

  Eigen::MatrixXf non_square(1, 2);
  non_square << 1.0f, 2.0f;
  EXPECT_DEATH_IF_SUPPORTED(solver.Solve(non_square),
                            "requires a non-empty square cost matrix");
}

}  // namespace tests
}  // namespace airborne_radar
