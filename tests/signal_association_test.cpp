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

} } // namespace airborne_radar::tests
