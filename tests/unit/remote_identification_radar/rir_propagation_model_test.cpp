// Copyright 2026. All Rights Reserved.
//
// @file rir_propagation_model_test.cpp
// @brief 验证 RIR 环境层传播损耗模型（杂波已迁至 rir_surface_clutter_model_test）。

#include <gtest/gtest.h>

#include <cmath>

#include "remote_identification_radar/internal/RirPropagationModel.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirVegetationCoverProfile;
using internal::RirEnvironmentSceneState;
using internal::RirPropagationModel;

RirEnvironmentSceneState MakeScene(RirVegetationCoverProfile profile) {
  RirEnvironmentSceneState scene;
  scene.vegetation_cover_profile = profile;
  return scene;
}

/// @brief 植被关闭：传播损耗 = 4+1.5+1 = 6.5 dB（基线，无杂波输出）。
TEST(RirPropagationModelTest, DisabledVegetation_UsesBaselineLoss) {
  const RirPropagationModel model;
  const auto result = model.Evaluate(MakeScene(RirVegetationCoverProfile::kDisabled));
  EXPECT_NEAR(result.propagation_loss_db, 6.5f, 1.0e-4f);
}

/// @brief 启用植被档：传播损耗保持恒定基线（损耗链物理化为后续冻结项）。
TEST(RirPropagationModelTest, EnabledVegetation_KeepsBaselineLoss) {
  const RirPropagationModel model;
  const RirVegetationCoverProfile profiles[] = {
      RirVegetationCoverProfile::kOpenGrassland, RirVegetationCoverProfile::kSparseWoodland,
      RirVegetationCoverProfile::kDeciduousForest,
      RirVegetationCoverProfile::kConiferousForest, RirVegetationCoverProfile::kTropicalDense};
  for (auto profile : profiles) {
    const auto result = model.Evaluate(MakeScene(profile));
    EXPECT_NEAR(result.propagation_loss_db, 6.5f, 1.0e-4f)
        << "profile=" << static_cast<int>(profile);
  }
}

/// @brief 同输入确定性输出。
TEST(RirPropagationModelTest, DeterministicAcrossCalls) {
  const RirPropagationModel model;
  const auto scene = MakeScene(RirVegetationCoverProfile::kDeciduousForest);
  const auto first = model.Evaluate(scene);
  const auto second = model.Evaluate(scene);
  EXPECT_FLOAT_EQ(first.propagation_loss_db, second.propagation_loss_db);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
