// Copyright 2026. All Rights Reserved.
//
// @file signal_association_test.cpp
// @brief 验证数据关联模块的关键行为。

#include <gtest/gtest.h>

#include <cmath>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/association/DistanceMetric.h"
#include "airborne_radar/signal/association/Hypothesiser.h"
#include "airborne_radar/signal/detection/ArDeceptionMeasurementCandidate.h"

namespace airborne_radar {
namespace tests {

namespace {

session::ArSceneTarget MakeTarget(float speed, float rcs) {
  return session::ArSceneTarget(speed, 0.0f, 0.0f, rcs);
}

session::ArSceneTarget MakePositionTarget(float x, float y, float z) {
  session::ArSceneTarget target(100.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.0f);

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

  const session::ArSceneTargetList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f),
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

  const session::ArSceneTargetList cycle_2{MakePositionTarget(11.0f, 0.0f, 0.0f),
                                              MakePositionTarget(101.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U, 1U};

  const std::vector<std::uint64_t> keys_2 = engine.Associate(cycle_2, detected_2);
  ASSERT_EQ(keys_2.size(), 2u);
  EXPECT_EQ(keys_2[0], keys_1[0]);
  EXPECT_EQ(keys_2[1], keys_1[1]);
}

TEST(DataAssociationEngineTest, StatelessWithoutSeedsMakesAssociationAcrossCyclesIndependent) {
  signal::association::DataAssociationEngine engine;

  const session::ArSceneTargetList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};

  const signal::association::AssociationResult first_result =
      engine.AssociateDetections(cycle_1, detected_1);
  ASSERT_EQ(first_result.target_keys.size(), 1u);
  ASSERT_NE(first_result.target_keys[0], 0u);

  const session::ArSceneTargetList cycle_2{MakePositionTarget(10.5f, 0.0f, 0.0f)};
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

  const session::ArSceneTargetList warmup_cycle{MakePositionTarget(50.0f, 0.0f, 0.0f)};
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

  const session::ArSceneTargetList targets{MakePositionTarget(20.5f, 0.0f, 0.0f)};
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
  const session::ArSceneTargetList targets{MakePositionTarget(20.5f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected{1U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(targets, detected);
  EXPECT_FALSE(result.used_external_association_seeds);
}

TEST(DataAssociationEngineTest, ExternalAssociationSeedWithReservedKeyIsRejected) {
  signal::association::DataAssociationEngine engine;

  signal::tracking::AssociationTrackSeed seed =
      MakeExternalSeed(0u, Eigen::Vector3f(20.0f, 0.0f, 0.0f));

  EXPECT_FALSE(
      engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed)));

  const session::ArSceneTargetList targets{MakePositionTarget(20.5f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected{1U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(targets, detected);
  ASSERT_EQ(result.target_keys.size(), 1u);
  EXPECT_FALSE(result.used_external_association_seeds);
  EXPECT_TRUE(result.matches.empty());
  EXPECT_NE(result.target_keys[0], 0u);
}

TEST(DataAssociationEngineTest, DuplicateExternalAssociationSeedKeysAreRejected) {
  signal::association::DataAssociationEngine engine;

  const signal::tracking::AssociationTrackSeed first =
      MakeExternalSeed(91u, Eigen::Vector3f(20.0f, 0.0f, 0.0f));
  const signal::tracking::AssociationTrackSeed second =
      MakeExternalSeed(91u, Eigen::Vector3f(30.0f, 0.0f, 0.0f));

  EXPECT_FALSE(engine.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>{first, second}));

  const session::ArSceneTargetList targets{MakePositionTarget(20.5f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected{1U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(targets, detected);
  ASSERT_EQ(result.target_keys.size(), 1u);
  EXPECT_FALSE(result.used_external_association_seeds);
  EXPECT_TRUE(result.matches.empty());
  EXPECT_NE(result.target_keys[0], 0u);
}

TEST(DataAssociationEngineTest, HandlesCrossedMeasurementsByCostMinimization) {
  signal::association::DataAssociationEngine engine;

  const session::ArSceneTargetList cycle_1{MakePositionTarget(20.0f, 0.0f, 0.0f),
                                              MakePositionTarget(200.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 2u);

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>{
      MakeExternalSeed(keys_1[0], Eigen::Vector3f(20.0f, 0.0f, 0.0f)),
      MakeExternalSeed(keys_1[1], Eigen::Vector3f(200.0f, 0.0f, 0.0f))});

  // The measurement order is swapped, but position-space cost should preserve identity.
  const session::ArSceneTargetList cycle_2{MakePositionTarget(201.0f, 0.0f, 0.0f),
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

  const session::ArSceneTargetList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 1u);
  ASSERT_NE(keys_1[0], 0u);

  // Very far position should exceed gating threshold and receive a new key.
  const session::ArSceneTargetList cycle_2{MakePositionTarget(200.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U};
  const std::vector<std::uint64_t> keys_2 = engine.Associate(cycle_2, detected_2);
  ASSERT_EQ(keys_2.size(), 1u);
  EXPECT_NE(keys_2[0], 0u);
  EXPECT_NE(keys_2[0], keys_1[0]);
}

TEST(DataAssociationEngineTest, ReportsMatchesMissesAndUnassociatedTargets) {
  signal::association::DataAssociationEngine engine;

  const session::ArSceneTargetList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 1u);
  ASSERT_NE(keys_1[0], 0u);

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>{
      MakeExternalSeed(keys_1[0], Eigen::Vector3f(10.0f, 0.0f, 0.0f))});

  const session::ArSceneTargetList cycle_2{MakePositionTarget(11.0f, 0.0f, 0.0f),
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

  const session::ArSceneTargetList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 1u);

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>{
      MakeExternalSeed(keys_1[0], Eigen::Vector3f(10.0f, 0.0f, 0.0f))});

  const session::ArSceneTargetList cycle_2{MakePositionTarget(10.0f, 0.0f, 0.0f)};
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

  const session::ArSceneTargetList cycle_1{MakePositionTarget(10.0f, 0.0f, 0.0f),
                                              MakePositionTarget(100.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

  const signal::association::AssociationResult first_result =
      engine.AssociateDetections(cycle_1, detected_1);
  const std::vector<std::uint64_t>& keys_1 = first_result.target_keys;
  ASSERT_EQ(keys_1.size(), 2u);
  ASSERT_NE(keys_1[0], 0u);
  ASSERT_NE(keys_1[1], 0u);
  EXPECT_TRUE(first_result.used_position_association);

  const session::ArSceneTargetList cycle_2{MakePositionTarget(101.0f, 0.0f, 0.0f),
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

TEST(DataAssociationEngineTest, DetectedTargetWithoutExplicitPositionStillAssociates) {
  signal::association::DataAssociationEngine engine;

  const session::ArSceneTargetList cycle_1{MakeTarget(100.0f, 2.0f), MakeTarget(220.0f, 5.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

  const signal::association::AssociationResult result =
      engine.AssociateDetections(cycle_1, detected_1);
  EXPECT_EQ(result.target_keys.size(), 2u);
  EXPECT_NE(result.target_keys[0], 0u);
  EXPECT_NE(result.target_keys[1], 0u);
  EXPECT_TRUE(result.matches.empty());
  ASSERT_EQ(result.unassociated_target_indices.size(), 2u);
  EXPECT_EQ(result.unassociated_target_indices[0], 0u);
  EXPECT_EQ(result.unassociated_target_indices[1], 1u);
}

TEST(DataAssociationEngineTest, DetectedTargetWithCoordinatesAssociatesNormally) {
  signal::association::DataAssociationEngine engine;

  session::ArSceneTarget target = MakeTarget(100.0f, 2.0f);
  target.position_x = 20.0f;
  target.position_y = 1.0f;
  target.position_z = 0.0f;

  const session::ArSceneTargetList cycle_1{target};
  const std::vector<std::uint8_t> detected_1{1U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(cycle_1, detected_1);
  EXPECT_EQ(result.target_keys.size(), 1u);
  EXPECT_NE(result.target_keys[0], 0u);
  EXPECT_TRUE(result.matches.empty());
  ASSERT_EQ(result.unassociated_target_indices.size(), 1u);
  EXPECT_EQ(result.unassociated_target_indices[0], 0u);
}

TEST(DataAssociationEngineTest, TargetsWithoutExplicitPositionParticipateInAssociation) {
  signal::association::DataAssociationEngine engine;

  session::ArSceneTarget invalid_target = MakeTarget(100.0f, 2.0f);
  session::ArSceneTarget valid_target = MakePositionTarget(30.0f, 0.0f, 0.0f);

  const session::ArSceneTargetList cycle_1{invalid_target, valid_target};
  const std::vector<std::uint8_t> detected_1{1U, 1U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(cycle_1, detected_1);
  ASSERT_EQ(result.target_keys.size(), 2u);
  EXPECT_NE(result.target_keys[0], 0u);
  EXPECT_NE(result.target_keys[1], 0u);
  ASSERT_EQ(result.unassociated_target_indices.size(), 2u);
  EXPECT_EQ(result.unassociated_target_indices[0], 0u);
  EXPECT_EQ(result.unassociated_target_indices[1], 1u);
}

TEST(DataAssociationEngineTest, DetectedOriginWithPositionFlagPassesValidation) {
  signal::association::DataAssociationEngine engine;

  session::ArSceneTarget target = MakeTarget(120.0f, 1.5f);

  target.position_x = 0.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;

  const session::ArSceneTargetList cycle_1{target};
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

  const session::ArSceneTargetList targets{MakePositionTarget(4.0f, 0.0f, 0.0f)};
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

  const session::ArSceneTargetList cycle_1{MakePositionTarget(101.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const signal::association::AssociationResult first_result =
      engine.AssociateDetections(cycle_1, detected_1);

  ASSERT_EQ(first_result.target_keys.size(), 1u);
  EXPECT_EQ(first_result.target_keys[0], 42u);
  EXPECT_TRUE(first_result.used_position_association);
  EXPECT_TRUE(first_result.used_external_association_seeds);

  const session::ArSceneTargetList cycle_2{MakePositionTarget(102.0f, 0.0f, 0.0f)};
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

  const session::ArSceneTargetList warmup_cycle{MakePositionTarget(50.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> warmup_detected{1U};
  const signal::association::AssociationResult warmup_result =
      engine.AssociateDetections(warmup_cycle, warmup_detected);

  ASSERT_EQ(warmup_result.target_keys.size(), 1u);
  const std::uint64_t warmup_key = warmup_result.target_keys[0];
  ASSERT_NE(warmup_key, 0u);

  engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>());

  const session::ArSceneTargetList cycle_2{MakePositionTarget(51.0f, 0.0f, 0.0f)};
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

TEST(DenseCostHypothesiserTest, NullDependenciesReturnNoHypotheses) {
  const float max_cost = 9.0f;

  const signal::association::DenseCostHypothesiser hypothesiser(
      static_cast<const signal::association::MahalanobisDistanceMetric*>(nullptr), max_cost);
  const std::vector<signal::association::AssociationHypothesis> hypotheses = hypothesiser.Generate(
      signal::association::FeatureVectorList{Eigen::Vector3f(0.0f, 0.0f, 0.0f)},
      signal::association::FeatureVectorList{Eigen::Vector3f(1.0f, 0.0f, 0.0f)});

  EXPECT_TRUE(hypotheses.empty());
}

TEST(DenseCostHypothesiserTest, GeneratesOnlyGatedHypotheses) {
  const signal::association::MahalanobisDistanceMetric metric(40.0f, 8.0f, 10.0f);
  const float max_cost = 9.0f;
  const signal::association::DenseCostHypothesiser hypothesiser(&metric, max_cost);

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
  const float max_cost = 1.5f;
  const signal::association::DenseCostHypothesiser hypothesiser(&metric, max_cost);

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

TEST(DenseCostHypothesiserTest, MismatchedTrackWiseInnovationCovarianceReturnsNoHypotheses) {
  signal::association::FullMahalanobisDistanceMetric metric(Eigen::Matrix3f::Identity());
  const float max_cost = 1.5f;
  const signal::association::DenseCostHypothesiser hypothesiser(&metric, max_cost);

  const signal::association::FeatureVectorList predicted_tracks{Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                                                Eigen::Vector3f(1.0f, 0.0f, 0.0f)};
  const signal::association::FeatureVectorList measurements{Eigen::Vector3f(2.0f, 0.0f, 0.0f)};

  const std::vector<signal::association::AssociationHypothesis> hypotheses = hypothesiser.Generate(
      predicted_tracks, measurements, std::vector<Eigen::Matrix3f>(1, Eigen::Matrix3f::Identity()));

  EXPECT_TRUE(hypotheses.empty());
}

TEST(DenseCostHypothesiserTest, ConstMetricDynamicInnovationCovarianceReturnsNoHypotheses) {
  const signal::association::FullMahalanobisDistanceMetric metric(Eigen::Matrix3f::Identity());
  const float max_cost = 1.5f;
  const signal::association::DenseCostHypothesiser hypothesiser(&metric, max_cost);

  const signal::association::FeatureVectorList predicted_tracks{Eigen::Vector3f(0.0f, 0.0f, 0.0f)};
  const signal::association::FeatureVectorList measurements{Eigen::Vector3f(2.0f, 0.0f, 0.0f)};

  const std::vector<signal::association::AssociationHypothesis> hypotheses = hypothesiser.Generate(
      predicted_tracks, measurements, std::vector<Eigen::Matrix3f>(1, Eigen::Matrix3f::Identity()));

  EXPECT_TRUE(hypotheses.empty());
}

TEST(LapjvSolverTest, InvalidMatrixReturnsNoAssignments) {
  const signal::association::LapjvSolver solver;

  Eigen::MatrixXf empty_matrix;
  EXPECT_TRUE(solver.Solve(empty_matrix).empty());

  Eigen::MatrixXf non_square(1, 2);
  non_square << 1.0f, 2.0f;
  EXPECT_TRUE(solver.Solve(non_square).empty());
}

// ---------------------------------------------------------------------------
// AssociateDeceptionCandidates -- 欺骗候选量测关联测试
// ---------------------------------------------------------------------------

signal::detection::ArDeceptionMeasurementCandidate MakeTestCandidate(float x, float y, float z) {
  signal::detection::ArDeceptionMeasurementCandidate c;
  c.source_observation_id = 1U;
  c.position = Eigen::Vector3f(x, y, z);
  c.velocity = Eigen::Vector3f::Zero();
  c.measurement_covariance = Eigen::Matrix3f::Identity() * 100.0f;
  return c;
}

TEST(DataAssociationEngineTest, AssociateDeceptionCandidatesEmptyInput) {
  signal::association::DataAssociationEngine engine;
  signal::detection::ArDeceptionMeasurementCandidateList empty;
  std::vector<std::uint64_t> keys = engine.AssociateDeceptionCandidates(empty);
  EXPECT_TRUE(keys.empty());
}

TEST(DataAssociationEngineTest, AssociateDeceptionCandidatesSkipsZeroPosition) {
  signal::association::DataAssociationEngine engine;
  signal::detection::ArDeceptionMeasurementCandidateList candidates = {MakeTestCandidate(0.0f, 0.0f, 0.0f)};
  std::vector<std::uint64_t> keys = engine.AssociateDeceptionCandidates(candidates);
  ASSERT_EQ(keys.size(), 1U);
  EXPECT_EQ(keys[0], 0U);
}

TEST(DataAssociationEngineTest, AssociateDeceptionCandidatesGetsSequentialKeys) {
  signal::association::DataAssociationEngine engine;
  signal::detection::ArDeceptionMeasurementCandidateList candidates = {
      MakeTestCandidate(100.0f, 0.0f, 0.0f),
      MakeTestCandidate(200.0f, 0.0f, 0.0f)};
  std::vector<std::uint64_t> keys = engine.AssociateDeceptionCandidates(candidates);
  ASSERT_EQ(keys.size(), 2U);
  EXPECT_NE(keys[0], 0U);
  EXPECT_NE(keys[1], 0U);
  EXPECT_NE(keys[0], keys[1]);
}

TEST(DataAssociationEngineTest, AssociateDeceptionCandidatesKeyIncrementsNextKey) {
  signal::association::DataAssociationEngine engine;
  // 先分配一个真实目标键。
  session::ArSceneTargetList warmup = {MakePositionTarget(50.0f, 0.0f, 0.0f)};
  std::vector<std::uint8_t> detected = {1U};
  std::vector<std::uint64_t> real_keys = engine.Associate(warmup, detected);
  ASSERT_EQ(real_keys.size(), 1U);
  ASSERT_NE(real_keys[0], 0U);

  // candidate 应获得下一个新键（无 external seeds）。
  signal::detection::ArDeceptionMeasurementCandidateList candidates = {MakeTestCandidate(100.0f, 0.0f, 0.0f)};
  std::vector<std::uint64_t> candidate_keys = engine.AssociateDeceptionCandidates(candidates);
  ASSERT_EQ(candidate_keys.size(), 1U);
  EXPECT_NE(candidate_keys[0], 0U);
  EXPECT_NE(candidate_keys[0], real_keys[0]);
}

TEST(DataAssociationEngineTest, AssociateDeceptionCandidatesMatchesExistingTrack) {
  signal::association::DataAssociationEngine engine;
  // 建立一条轨迹并获取键。
  session::ArSceneTargetList cycle_1 = {MakePositionTarget(100.0f, 0.0f, 0.0f)};
  std::vector<std::uint8_t> detected = {1U};
  std::vector<std::uint64_t> real_keys = engine.Associate(cycle_1, detected);
  ASSERT_EQ(real_keys.size(), 1U);
  const std::uint64_t track_key = real_keys[0];

  // 注入该轨迹的关联种子。
  std::vector<signal::tracking::AssociationTrackSeed> seeds = {
      MakeExternalSeed(track_key, Eigen::Vector3f(100.0f, 0.0f, 0.0f))};
  engine.SetAssociationSeeds(seeds);

  // candidate 紧邻预测位置，应匹配到已有键。
  signal::detection::ArDeceptionMeasurementCandidateList candidates = {
      MakeTestCandidate(101.0f, 0.0f, 0.0f)};
  std::vector<std::uint64_t> candidate_keys = engine.AssociateDeceptionCandidates(candidates);
  ASSERT_EQ(candidate_keys.size(), 1U);
  EXPECT_EQ(candidate_keys[0], track_key);
}

TEST(DataAssociationEngineTest, AssociateDeceptionCandidatesFarFromTrackGetsNewKey) {
  signal::association::DataAssociationEngine engine;
  // 建立一条轨迹。
  session::ArSceneTargetList cycle_1 = {MakePositionTarget(100.0f, 0.0f, 0.0f)};
  std::vector<std::uint8_t> detected = {1U};
  std::vector<std::uint64_t> real_keys = engine.Associate(cycle_1, detected);
  const std::uint64_t track_key = real_keys[0];

  // 注入种子。
  std::vector<signal::tracking::AssociationTrackSeed> seeds = {
      MakeExternalSeed(track_key, Eigen::Vector3f(100.0f, 0.0f, 0.0f))};
  engine.SetAssociationSeeds(seeds);

  // candidate 远离预测位置，应获得新键而非匹配。
  signal::detection::ArDeceptionMeasurementCandidateList candidates = {
      MakeTestCandidate(100000.0f, 0.0f, 0.0f)};
  std::vector<std::uint64_t> candidate_keys = engine.AssociateDeceptionCandidates(candidates);
  ASSERT_EQ(candidate_keys.size(), 1U);
  EXPECT_NE(candidate_keys[0], 0U);
  EXPECT_NE(candidate_keys[0], track_key);
}

TEST(DataAssociationEngineTest, AssociateDeceptionCandidatesDoesNotConsumeSeeds) {
  signal::association::DataAssociationEngine engine;
  // 注入种子。
  std::vector<signal::tracking::AssociationTrackSeed> seeds = {
      MakeExternalSeed(42U, Eigen::Vector3f(100.0f, 0.0f, 0.0f))};
  engine.SetAssociationSeeds(seeds);

  // 先用 candidate 关联（不应消耗种子）。
  signal::detection::ArDeceptionMeasurementCandidateList candidates = {
      MakeTestCandidate(101.0f, 0.0f, 0.0f)};
  std::vector<std::uint64_t> candidate_keys = engine.AssociateDeceptionCandidates(candidates);
  ASSERT_EQ(candidate_keys.size(), 1U);
  EXPECT_EQ(candidate_keys[0], 42U);

  // 再走真实关联：种子应仍然可用。
  session::ArSceneTargetList real_targets = {MakePositionTarget(101.0f, 0.0f, 0.0f)};
  std::vector<std::uint8_t> detected = {1U};
  std::vector<std::uint64_t> real_keys = engine.Associate(real_targets, detected);
  ASSERT_EQ(real_keys.size(), 1U);
  EXPECT_EQ(real_keys[0], 42U);
}

}  // namespace tests
}  // namespace airborne_radar
