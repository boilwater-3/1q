/**
 * @file common_vegetation_clutter_model_test.cpp
 * @brief 验证 common 植被散射杂波与最小传播损耗组合模型。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "common/radar/VegetationClutterModel.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

VegetationScatterPhysicsConfig MakeConfig(VegetationCoverProfile profile,
                                          bool enable_physical) {
  VegetationScatterPhysicsConfig config;
  config.cover_profile = profile;
  config.enable_physical_model = enable_physical;
  return config;
}

TEST(CommonVegetationClutterModelTest, DisabledVegetationUsesBaselines) {
  const auto result = EvaluatePropagationClutter(
      MakeConfig(VegetationCoverProfile::kDisabled, false));
  EXPECT_NEAR(result.propagation_loss_db, 6.5f, 1.0e-4f);
  EXPECT_NEAR(result.clutter_power_db, 3.0f, 1.0e-4f);
}

TEST(CommonVegetationClutterModelTest, EnabledVegetationRaisesClutterAboveBaseline) {
  const VegetationCoverProfile profiles[] = {
      VegetationCoverProfile::kOpenGrassland, VegetationCoverProfile::kSparseWoodland,
      VegetationCoverProfile::kDeciduousForest, VegetationCoverProfile::kConiferousForest,
      VegetationCoverProfile::kTropicalDense};
  for (const auto profile : profiles) {
    const auto result = EvaluatePropagationClutter(MakeConfig(profile, true));
    EXPECT_GE(result.clutter_power_db, 3.0f) << "profile=" << static_cast<int>(profile);
    EXPECT_NEAR(result.propagation_loss_db, 6.5f, 1.0e-4f);
  }
}

TEST(CommonVegetationClutterModelTest, DeterministicAcrossCalls) {
  const auto config = MakeConfig(VegetationCoverProfile::kDeciduousForest, true);
  const auto first = EvaluatePropagationClutter(config);
  const auto second = EvaluatePropagationClutter(config);
  EXPECT_FLOAT_EQ(first.propagation_loss_db, second.propagation_loss_db);
  EXPECT_FLOAT_EQ(first.clutter_power_db, second.clutter_power_db);
}

TEST(CommonVegetationClutterModelTest, DenseProfileExceedsGrassland) {
  const auto grass = EvaluatePropagationClutter(
      MakeConfig(VegetationCoverProfile::kOpenGrassland, true));
  const auto dense = EvaluatePropagationClutter(
      MakeConfig(VegetationCoverProfile::kTropicalDense, true));
  EXPECT_GT(dense.clutter_power_db, grass.clutter_power_db);
}

}  // namespace
}  // namespace radar
}  // namespace common
}  // namespace oneq
