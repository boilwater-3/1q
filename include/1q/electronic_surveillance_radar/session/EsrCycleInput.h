/**
 * @file EsrCycleInput.h
 * @brief 定义电子侦察模块单周期输入载荷。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/model/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/model/EsrOrientationConfig.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrCycleInput 描述电子侦察单周期输入。
 */
struct ONEQ_API EsrCycleInput {
  std::uint32_t cycle_index{0U};                                /**< 当前周期号 */
  float dt_sec{1.0f};                                           /**< 当前周期步长（单位：s） */
  model::EsrPoseState platform_pose{};                         /**< 侦察平台姿态与运动状态 */
  model::EmitterTruthStateList scene_emitters{};               /**< 场景辐射源真值列表 */
  environment::EsrEnvironmentObservation environment_observation{}; /**< 本周期环境高层观测输入 */
};

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_
