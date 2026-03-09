// Copyright 2026. All Rights Reserved.
//
// Description: 验证数据关联模块的关键行为（稳定匹配/交叉匹配/新生目标）。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "airborne_radar/signal/association/DataAssociation.h"

namespace airborne_radar::tests {

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

} // namespace airborne_radar::tests
