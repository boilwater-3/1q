// Copyright 2026. All Rights Reserved.
//
// Description: 验证数据关联模块的关键行为（稳定匹配/交叉匹配/新生目标）。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/association/DistanceMetric.h"
#include "airborne_radar/signal/association/Gater.h"
#include "airborne_radar/signal/association/Hypothesiser.h"

namespace airborne_radar { namespace tests {

namespace {

common::TargetFeature MakeTarget(float speed, float rcs, float acceleration) {
  return common::TargetFeature(speed, rcs, false, acceleration);
}

common::TargetFeature MakePositionTarget(float x, float y, float z) {
  common::TargetFeature target(100.0f, 2.0f, false, 1.0f);
  target.position_x = x;
  target.position_y = y;
  target.position_z = z;
  return target;
}

} // namespace

TEST(DataAssociationEngineTest, KeepsStableAssociationAcrossCycles) {
  signal::association::DataAssociationEngine engine;

  const common::TargetFeatureList cycle_1{
      MakeTarget(100.0f, 2.0f, 1.0f),
      MakeTarget(220.0f, 5.0f, 3.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 2u);
  EXPECT_NE(keys_1[0], 0u);
  EXPECT_NE(keys_1[1], 0u);
  EXPECT_NE(keys_1[0], keys_1[1]);

  const common::TargetFeatureList cycle_2{
      MakeTarget(101.0f, 2.1f, 1.1f),
      MakeTarget(219.5f, 4.9f, 2.9f)};
  const std::vector<std::uint8_t> detected_2{1U, 1U};

  const std::vector<std::uint64_t> keys_2 = engine.Associate(cycle_2, detected_2);
  ASSERT_EQ(keys_2.size(), 2u);
  EXPECT_EQ(keys_2[0], keys_1[0]);
  EXPECT_EQ(keys_2[1], keys_1[1]);
}

TEST(DataAssociationEngineTest, HandlesCrossedMeasurementsByCostMinimization) {
  signal::association::DataAssociationEngine engine;

  const common::TargetFeatureList cycle_1{
      MakeTarget(90.0f, 1.5f, 0.8f),
      MakeTarget(260.0f, 7.0f, 4.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 2u);

  // The measurement order is swapped, but feature similarity should preserve identity.
  const common::TargetFeatureList cycle_2{
      MakeTarget(261.0f, 7.1f, 4.1f),
      MakeTarget(89.5f, 1.4f, 0.7f)};
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

  const common::TargetFeatureList cycle_1{MakeTarget(120.0f, 2.0f, 1.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 1u);
  ASSERT_NE(keys_1[0], 0u);

  // Very far feature should exceed unassigned cost and receive a new key.
  const common::TargetFeatureList cycle_2{MakeTarget(700.0f, 35.0f, 20.0f)};
  const std::vector<std::uint8_t> detected_2{1U};
  const std::vector<std::uint64_t> keys_2 = engine.Associate(cycle_2, detected_2);
  ASSERT_EQ(keys_2.size(), 1u);
  EXPECT_NE(keys_2[0], 0u);
  EXPECT_NE(keys_2[0], keys_1[0]);
}

TEST(DataAssociationEngineTest, ReportsMatchesMissesAndUnassociatedTargets) {
  signal::association::DataAssociationEngine engine;

  const common::TargetFeatureList cycle_1{MakeTarget(120.0f, 2.0f, 1.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 1u);
  ASSERT_NE(keys_1[0], 0u);

  const common::TargetFeatureList cycle_2{
      MakeTarget(121.0f, 2.1f, 1.1f),
      MakeTarget(700.0f, 35.0f, 20.0f)};
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
}

TEST(DataAssociationEngineTest, ReportsMissedTrackKeysWhenNoDetectionArrives) {
  signal::association::DataAssociationEngine engine;

  const common::TargetFeatureList cycle_1{MakeTarget(120.0f, 2.0f, 1.0f)};
  const std::vector<std::uint8_t> detected_1{1U};
  const std::vector<std::uint64_t> keys_1 = engine.Associate(cycle_1, detected_1);
  ASSERT_EQ(keys_1.size(), 1u);

  const common::TargetFeatureList cycle_2{MakeTarget(120.0f, 2.0f, 1.0f)};
  const std::vector<std::uint8_t> detected_2{0U};
  const signal::association::AssociationResult result =
      engine.AssociateDetections(cycle_2, detected_2);

  EXPECT_TRUE(result.matches.empty());
  EXPECT_TRUE(result.unassociated_target_indices.empty());
  ASSERT_EQ(result.missed_track_keys.size(), 1u);
  EXPECT_EQ(result.missed_track_keys[0], keys_1[0]);
  ASSERT_EQ(result.target_keys.size(), 1u);
  EXPECT_EQ(result.target_keys[0], 0u);
}

  TEST(DataAssociationEngineTest, UsesCartesianPositionByDefaultWhenPositionAvailable) {
    signal::association::DataAssociationConfig config;
    config.kalman_measurement_noise_std = 2.0f;
    signal::association::DataAssociationEngine engine(config);

  const common::TargetFeatureList cycle_1{
      MakePositionTarget(10.0f, 0.0f, 0.0f),
      MakePositionTarget(100.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_1{1U, 1U};

    const signal::association::AssociationResult first_result =
      engine.AssociateDetections(cycle_1, detected_1);
    const std::vector<std::uint64_t> &keys_1 = first_result.target_keys;
    ASSERT_EQ(keys_1.size(), 2u);
  ASSERT_NE(keys_1[0], 0u);
  ASSERT_NE(keys_1[1], 0u);
    EXPECT_TRUE(first_result.used_position_association);
    EXPECT_FALSE(first_result.fell_back_to_feature_association);

  const common::TargetFeatureList cycle_2{
      MakePositionTarget(101.0f, 0.0f, 0.0f),
      MakePositionTarget(11.0f, 0.0f, 0.0f)};
  const std::vector<std::uint8_t> detected_2{1U, 1U};

    const signal::association::AssociationResult second_result =
      engine.AssociateDetections(cycle_2, detected_2);
    const std::vector<std::uint64_t> &keys_2 = second_result.target_keys;
  ASSERT_EQ(keys_2.size(), 2u);
  EXPECT_EQ(keys_2[0], keys_1[1]);
  EXPECT_EQ(keys_2[1], keys_1[0]);
    EXPECT_TRUE(second_result.used_position_association);
    EXPECT_FALSE(second_result.fell_back_to_feature_association);
  }

  TEST(DataAssociationEngineTest,
     FallsBackToFeatureAssociationWhenDetectedTargetLacksPosition) {
    signal::association::DataAssociationEngine engine;

    const common::TargetFeatureList cycle_1{
      MakeTarget(100.0f, 2.0f, 1.0f),
      MakeTarget(220.0f, 5.0f, 3.0f)};
    const std::vector<std::uint8_t> detected_1{1U, 1U};

    const signal::association::AssociationResult first_result =
      engine.AssociateDetections(cycle_1, detected_1);
    ASSERT_EQ(first_result.target_keys.size(), 2u);
    EXPECT_FALSE(first_result.used_position_association);
    EXPECT_TRUE(first_result.fell_back_to_feature_association);

    common::TargetFeature positioned = MakePositionTarget(500.0f, 0.0f, 0.0f);
    positioned.current_track_speed = 220.5f;
    positioned.current_track_rcs = 5.1f;
    positioned.current_track_acceleration = 3.1f;

    const common::TargetFeatureList cycle_2{
      MakeTarget(99.5f, 2.1f, 1.1f),
      positioned};
    const std::vector<std::uint8_t> detected_2{1U, 1U};

    const signal::association::AssociationResult second_result =
      engine.AssociateDetections(cycle_2, detected_2);
    ASSERT_EQ(second_result.target_keys.size(), 2u);
    EXPECT_FALSE(second_result.used_position_association);
    EXPECT_TRUE(second_result.fell_back_to_feature_association);
    EXPECT_EQ(second_result.target_keys[0], first_result.target_keys[0]);
    EXPECT_EQ(second_result.target_keys[1], first_result.target_keys[1]);
}

  TEST(DataAssociationEngineTest,
       ExternalAssociationSeedsAffectOnlyCurrentCycle) {
    signal::association::DataAssociationEngine engine;

    signal::tracking::AssociationTrackSeed seed;
    seed.association_key = 42;
    seed.legacy_feature = Eigen::Vector3f(800.0f, 20.0f, 5.0f);
    seed.has_position = true;
    seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
    seed.has_gaussian_state = true;
    seed.gaussian_state.mean(0) = 100.0f;
    seed.gaussian_state.mean(2) = 0.0f;
    seed.gaussian_state.mean(4) = 0.0f;
    seed.gaussian_state.covariance =
        signal::tracking::StateCovariance::Identity() * 25.0f;

    engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>(1, seed));

    const common::TargetFeatureList cycle_1{MakePositionTarget(101.0f, 0.0f, 0.0f)};
    const std::vector<std::uint8_t> detected_1{1U};
    const signal::association::AssociationResult first_result =
        engine.AssociateDetections(cycle_1, detected_1);

    ASSERT_EQ(first_result.target_keys.size(), 1u);
    EXPECT_EQ(first_result.target_keys[0], 42u);
    EXPECT_TRUE(first_result.used_position_association);
    EXPECT_FALSE(first_result.fell_back_to_feature_association);
      EXPECT_TRUE(first_result.used_external_association_seeds);

    const common::TargetFeatureList cycle_2{MakePositionTarget(102.0f, 0.0f, 0.0f)};
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

  TEST(DataAssociationEngineTest,
       EmptyExternalSeedsSuppressInternalHistoryFallback) {
    signal::association::DataAssociationEngine engine;

    const common::TargetFeatureList warmup_cycle{MakePositionTarget(50.0f, 0.0f, 0.0f)};
    const std::vector<std::uint8_t> warmup_detected{1U};
    const signal::association::AssociationResult warmup_result =
        engine.AssociateDetections(warmup_cycle, warmup_detected);

    ASSERT_EQ(warmup_result.target_keys.size(), 1u);
    const std::uint64_t warmup_key = warmup_result.target_keys[0];
    ASSERT_NE(warmup_key, 0u);

    engine.SetAssociationSeeds(std::vector<signal::tracking::AssociationTrackSeed>());

    const common::TargetFeatureList cycle_2{MakePositionTarget(51.0f, 0.0f, 0.0f)};
    const std::vector<std::uint8_t> detected_2{1U};
    const signal::association::AssociationResult result =
        engine.AssociateDetections(cycle_2, detected_2);

    ASSERT_EQ(result.target_keys.size(), 1u);
    EXPECT_TRUE(result.used_external_association_seeds);
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

TEST(DenseCostHypothesiserTest, GeneratesOnlyGatedHypotheses) {
  const signal::association::MahalanobisDistanceMetric metric(40.0f, 8.0f, 10.0f);
  const signal::association::CostThresholdGater gater(9.0f);
  const signal::association::DenseCostHypothesiser hypothesiser(&metric, &gater);

  const signal::association::FeatureVectorList predicted_tracks{
      Eigen::Vector3f(100.0f, 2.0f, 1.0f)};
  const signal::association::FeatureVectorList measurements{
      Eigen::Vector3f(101.0f, 2.1f, 1.1f),
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

  const signal::association::FeatureVectorList predicted_tracks{
      Eigen::Vector3f(0.0f, 0.0f, 0.0f)};
  const signal::association::FeatureVectorList measurements{
      Eigen::Vector3f(2.0f, 0.0f, 0.0f)};

  const std::vector<signal::association::AssociationHypothesis> no_match =
      hypothesiser.Generate(predicted_tracks, measurements,
                            std::vector<Eigen::Matrix3f>(1, Eigen::Matrix3f::Identity()));
  EXPECT_TRUE(no_match.empty());

  Eigen::Matrix3f wide_covariance = Eigen::Matrix3f::Identity() * 4.0f;
  const std::vector<signal::association::AssociationHypothesis> accepted =
      hypothesiser.Generate(predicted_tracks, measurements,
                            std::vector<Eigen::Matrix3f>(1, wide_covariance));
  ASSERT_EQ(accepted.size(), 1u);
  EXPECT_NEAR(accepted[0].cost, 1.0f, 1e-3f);
}

} } // namespace airborne_radar::tests
