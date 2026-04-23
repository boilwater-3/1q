/**
 * @file RadarCycleInput.h
 * @brief 定义按处理周期向雷达链路注入的标准输入载荷。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_RADAR_CYCLE_INPUT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_RADAR_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/airborne_radar/session/RadarEnvironmentInput.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarCycleInput 描述单周期输入的场景、环境、平台位姿与步长。
 */
struct RadarCycleInput {
  std::uint32_t cycle_index{0U};            /**< 当前周期号 */
  float dt_sec{1.0f};                       /**< 当前周期步长（单位：秒） */
  oneq::foundation::PoseState platform_pose{}; /**< 当前周期平台位姿状态 */
  RadarSceneTargetList scene{};             /**< 当前周期场景目标输入列表 */
  RadarEnvironmentInput environment{};      /**< 当前周期环境输入 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_RADAR_CYCLE_INPUT_H_
