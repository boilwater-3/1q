// Copyright 2026. All Rights Reserved.
//
// @file SceneManager.h
// @brief 管理环境层待生效与已冻结场景状态。

#ifndef AIRBORNE_RADAR_ENVIRONMENT_SCENE_SCENE_MANAGER_H_
#define AIRBORNE_RADAR_ENVIRONMENT_SCENE_SCENE_MANAGER_H_

#include "1q/airborne_radar/environment/EnvironmentTypes.h"

namespace airborne_radar {
namespace environment {
namespace scene {

/// @brief SceneManager 负责管理环境场景的 pending/active 双态。
class SceneManager {
public:
  /// @brief 使用初始场景构造管理器。
  explicit SceneManager(EnvironmentSceneState initial_scene = {});

  /// @brief 更新待生效场景。
  void UpdatePendingScene(const EnvironmentSceneState& scene_state);

  /// @brief 获取待生效场景。
  const EnvironmentSceneState& GetPendingScene() const;

  /// @brief 获取可修改的待生效场景。
  EnvironmentSceneState* MutablePendingScene();

  /// @brief 提交待生效场景为当前周期冻结场景。
  void CommitPendingScene(const EnvironmentCycleContext& cycle_context);

  /// @brief 获取当前已冻结场景。
  const EnvironmentSceneState& GetActiveScene() const;

  /// @brief 获取当前已冻结周期上下文。
  const EnvironmentCycleContext& GetActiveCycleContext() const;

private:
  EnvironmentSceneState active_scene_{};
  EnvironmentSceneState pending_scene_{};
  EnvironmentCycleContext active_cycle_context_{};
};

} // namespace scene
} // namespace environment
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_ENVIRONMENT_SCENE_SCENE_MANAGER_H_
