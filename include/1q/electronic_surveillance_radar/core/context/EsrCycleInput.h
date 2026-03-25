/**
 * @file EsrCycleInput.h
 * @brief 定义电子侦察模块单周期输入载荷。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CORE_CONTEXT_ESR_CYCLE_INPUT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CORE_CONTEXT_ESR_CYCLE_INPUT_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/common/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/common/EsrOrientationConfig.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"

namespace electronic_surveillance_radar {
namespace core {
namespace context {

/**
 * @brief EsrCycleInput 描述电子侦察单周期输入。
 */
struct ONEQ_API EsrCycleInput {
  std::uint32_t cycle_index{0U};                                   /**< 当前周期号 */
  float dt_sec{1.0f};                                              /**< 当前周期步长（单位：s） */
  common::EsrPoseState platform_pose{};                            /**< 侦察平台姿态与运动状态 */
  common::EmitterTruthStateList scene_emitters{};                  /**< 场景辐射源真值列表 */
  environment::EsrEnvironmentSceneState environment_scene_state{}; /**< 本周期待冻结环境场景 */
};

}  // namespace context
}  // namespace core
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CORE_CONTEXT_ESR_CYCLE_INPUT_H_
