/**
 * @file SceneManager.h
 * @brief 管理环境层待生效与已冻结场景状态。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_SCENE_SCENE_MANAGER_H_
#define AIRBORNE_RADAR_ENVIRONMENT_SCENE_SCENE_MANAGER_H_

#include "1q/airborne_radar/session/ArEnvironmentInput.h"

namespace airborne_radar {
namespace environment {

using session::EnvironmentCycleContext;
using session::EnvironmentSceneState;

/**
 * @brief 管理环境场景的 pending/active 双态。
 */
class SceneManager {
 public:
  /**
   * @brief 使用初始场景构造管理器。
   * @param[in] initial_scene 初始待生效场景。
   */
  explicit SceneManager(const EnvironmentSceneState& initial_scene = {});

  /**
   * @brief 更新待生效场景。
   * @param[in] scene_state 新的待生效场景。
   */
  void UpdatePendingScene(const EnvironmentSceneState& scene_state);

  /**
   * @brief 获取待生效场景。
   * @return 当前待生效场景的只读引用。
   */
  const EnvironmentSceneState& GetPendingScene() const;

  /**
   * @brief 获取可修改的待生效场景。
   * @return 当前待生效场景的可写指针。
   */
  EnvironmentSceneState* MutablePendingScene();

  /**
   * @brief 提交待生效场景为当前周期冻结场景。
   * @param[in] cycle_context 当前周期上下文。
   */
  void CommitPendingScene(const EnvironmentCycleContext& cycle_context);

  /**
   * @brief 获取当前已冻结场景。
   * @return 当前生效场景的只读引用。
   */
  const EnvironmentSceneState& GetActiveScene() const;

  /**
   * @brief 获取当前已冻结周期上下文。
   * @return 当前生效周期上下文的只读引用。
   */
  const EnvironmentCycleContext& GetActiveCycleContext() const;

  /**
   * @brief 恢复 active/pending 场景与 active 周期上下文。
   * @param[in] active_scene 目标 active 场景。
   * @param[in] pending_scene 目标 pending 场景。
   * @param[in] cycle_context 目标 active 周期上下文。
   */
  void RestoreState(const EnvironmentSceneState& active_scene,
                    const EnvironmentSceneState& pending_scene,
                    const EnvironmentCycleContext& cycle_context);

 private:
  EnvironmentSceneState active_scene_{};
  EnvironmentSceneState pending_scene_{};
  EnvironmentCycleContext active_cycle_context_{};
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_SCENE_SCENE_MANAGER_H_
