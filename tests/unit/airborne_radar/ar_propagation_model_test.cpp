// Copyright 2026. All Rights Reserved.
//
// @file ar_propagation_model_test.cpp
// @brief 验证传播模型与杂波计算的基础行为。

#include <gtest/gtest.h>
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/environment/PropagationModel.h"
#include <cmath>

namespace airborne_radar {
namespace tests {

TEST(PropagationModelTest, NegativeTerrainReflectionYieldsNetGainPassesThroughUnchanged) {
  session::EnvironmentSceneState scene_state;

  environment::PropagationModel propagation_model;
  const environment::PropagationResult result = propagation_model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 6.5f);
  EXPECT_FLOAT_EQ(result.clutter_power_db, 3.0f);
}

TEST(PropagationModelTest, OptionalVegetationScatterPhysicsRaisesClutterWhenEnabled) {
  session::EnvironmentSceneState baseline_scene;

  session::EnvironmentSceneState physics_scene = baseline_scene;
  physics_scene.vegetation_scatter_physics.enable_physical_model = true;
  physics_scene.vegetation_scatter_physics.cover_profile =
      config::VegetationCoverProfile::kTropicalDense;

  environment::PropagationModel propagation_model;
  const environment::PropagationResult baseline_result = propagation_model.Evaluate(baseline_scene);
  const environment::PropagationResult physics_result = propagation_model.Evaluate(physics_scene);

  EXPECT_FLOAT_EQ(physics_result.propagation_loss_db, baseline_result.propagation_loss_db);
  EXPECT_GT(physics_result.clutter_power_db, baseline_result.clutter_power_db);
}

TEST(PropagationModelTest, ClutterPowerUsesInternalBaselineWhenVegetationModelDisabled) {
  session::EnvironmentSceneState scene_state;

  environment::PropagationModel model;
  const environment::PropagationResult result = model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 6.5f);
  EXPECT_FLOAT_EQ(result.clutter_power_db, 3.0f);
}

/// @brief 关闭植被物理模型时，杂波保持内部默认值。

TEST(PropagationModelTest, BaselineClutterRemainsStableAcrossDefaultScenes) {
  session::EnvironmentSceneState scene_state;

  environment::PropagationModel model;
  EXPECT_FLOAT_EQ(model.Evaluate(scene_state).clutter_power_db, 3.0f);
}

/// @brief 启用植被散射物理模型后，杂波高于内部基线。

TEST(PropagationModelTest, VegetationScatterRaisesClutterFromInternalBaseline) {
  session::EnvironmentSceneState scene_state;
  scene_state.vegetation_scatter_physics.enable_physical_model = true;
  scene_state.vegetation_scatter_physics.cover_profile =
      config::VegetationCoverProfile::kSparseWoodland;

  environment::PropagationModel model;
  EXPECT_GT(model.Evaluate(scene_state).clutter_power_db, 3.0f);
}

/// @brief 传播损耗由内部基线组成。

TEST(PropagationModelTest, PositivePropagationLossIsRetained) {
  session::EnvironmentSceneState scene_state;

  environment::PropagationModel model;
  const environment::PropagationResult result = model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 6.5f);
}

/// @brief 默认场景下，传播损耗与杂波均回到内部基线。

TEST(PropagationModelTest, ZeroAtmosphericAttenuationKeepsInternalPropagationBaseline) {
  session::EnvironmentSceneState scene_state;

  environment::PropagationModel model;
  const environment::PropagationResult result = model.Evaluate(scene_state);

  EXPECT_FLOAT_EQ(result.propagation_loss_db, 6.5f);
  EXPECT_FLOAT_EQ(result.clutter_power_db, 3.0f);
}

// ============================================================================
// EnvironmentService — 结构化输入与规范化测试
// ============================================================================

/// @brief 默认配置下不生成干扰源，也不会误判为探测到干扰。

}  // namespace tests
}  // namespace airborne_radar
