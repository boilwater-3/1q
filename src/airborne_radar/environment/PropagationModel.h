/**
 * @file PropagationModel.h
 * @brief 定义环境层最小传播与杂波组合模型。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_SIMULATION_PROPAGATION_MODEL_H_
#define AIRBORNE_RADAR_ENVIRONMENT_SIMULATION_PROPAGATION_MODEL_H_

#include "1q/airborne_radar/session/ArEnvironmentInput.h"

namespace airborne_radar {
namespace environment {

using session::EnvironmentSceneState;
/**
 * @brief PropagationResult 表示传播与杂波组合输出。
 */
struct PropagationResult {
  /**
   * @brief 传播损耗（单位：dB）。
   */
  float propagation_loss_db{0.0f};
  /**
   * @brief 传播损耗中的大气物理附加项（单位：dB）。
   */
  float atmospheric_physics_loss_db{0.0f};
  /**
   * @brief 杂波功率（单位：dB）。
   */
  float clutter_power_db{0.0f};
};
/**
 * @brief PropagationModel 提供组合式传播/杂波建模。
 */
class PropagationModel {
 public:
  /**
   * @brief 根据场景状态计算传播与杂波输出。
   * @param[in] scene_state 当前周期场景状态（含植被覆盖、地形反射等物理量）。
   * @return 包含传播损耗、大气物理附加损耗与杂波功率的 PropagationResult。
   */
  PropagationResult Evaluate(const EnvironmentSceneState& scene_state) const;
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_SIMULATION_PROPAGATION_MODEL_H_
