// Copyright 2026. All Rights Reserved.
//
// @file rir_propagation_model_test.cpp
// @brief 验证 RIR 环境层传播/杂波模型（副本改写自 AR PropagationModel 语义；
//        阶段 2-M M5）。

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

/// @brief 植被关闭：传播损耗 = 4+1.5+1 = 6.5 dB，杂波 = 基线 3 dB（物理路径未启用）。
TEST(RirPropagationModelTest, DisabledVegetation_UsesBaselines) {
  const RirPropagationModel model;
  const auto result = model.Evaluate(MakeScene(RirVegetationCoverProfile::kDisabled));
  EXPECT_NEAR(result.propagation_loss_db, 6.5f, 1.0e-4f);
  EXPECT_NEAR(result.clutter_power_db, 3.0f, 1.0e-4f);
}

/// @brief 启用植被物理：各档位杂波 ≥ 基线（混合比例 0.7，乘子 ≥ 1）。
TEST(RirPropagationModelTest, EnabledVegetation_RaisesClutterAboveBaseline) {
  const RirPropagationModel model;
  const RirVegetationCoverProfile profiles[] = {
      RirVegetationCoverProfile::kOpenGrassland, RirVegetationCoverProfile::kSparseWoodland,
      RirVegetationCoverProfile::kDeciduousForest,
      RirVegetationCoverProfile::kConiferousForest, RirVegetationCoverProfile::kTropicalDense};
  for (auto profile : profiles) {
    const auto result = model.Evaluate(MakeScene(profile));
    EXPECT_GE(result.clutter_power_db, 3.0f) << "profile=" << static_cast<int>(profile);
    EXPECT_NEAR(result.propagation_loss_db, 6.5f, 1.0e-4f);
  }
}

/// @brief 同输入确定性输出（植被物理路径含随机树散射初始化，须可复现）。
TEST(RirPropagationModelTest, DeterministicAcrossCalls) {
  const RirPropagationModel model;
  const auto scene = MakeScene(RirVegetationCoverProfile::kDeciduousForest);
  const auto first = model.Evaluate(scene);
  const auto second = model.Evaluate(scene);
  EXPECT_FLOAT_EQ(first.propagation_loss_db, second.propagation_loss_db);
  EXPECT_FLOAT_EQ(first.clutter_power_db, second.clutter_power_db);
}

/// @brief 密集植被档杂波高于开阔草地档（密度项单调性抽样）。
TEST(RirPropagationModelTest, DenseProfileExceedsGrassland) {
  const RirPropagationModel model;
  const auto grass = model.Evaluate(MakeScene(RirVegetationCoverProfile::kOpenGrassland));
  const auto dense = model.Evaluate(MakeScene(RirVegetationCoverProfile::kTropicalDense));
  EXPECT_GT(dense.clutter_power_db, grass.clutter_power_db);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
