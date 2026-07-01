/**
 * @file ArCycleInput.h
 * @brief AR module primary cycle input type.
 *
 * Primary header for cycle input.
 * Include this for new code; RadarCycleInput.h is the deprecated compat wrapper.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/api.hpp"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief ArCycleInput 描述单周期输入的周期号、场景、平台位姿与步长。
 */
struct ONEQ_API ArCycleInput {
  std::uint32_t cycle_index{0U};               /**< 当前周期号 */
  float dt_sec{1.0f};                          /**< 当前周期步长（单位：秒） */
  float platform_altitude_m{0.0f};             /**< 当前雷达平台 WGS84 绝对海拔（单位：m） */
  oneq::foundation::PoseState platform_pose{}; /**< 当前周期平台局部位姿状态 */
  ArSceneTargetList scene{};                /**< 当前周期场景目标输入列表 */
  bool has_environment{false};                 /**< 是否提供当前周期环境高层观测快照 */
  ArEnvironmentInput environment{};         /**< 当前周期环境高层观测输入 */
};

// 兼容别名：旧名称在 wrapper 阶段保留。
using RadarCycleInput = ArCycleInput;

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_
