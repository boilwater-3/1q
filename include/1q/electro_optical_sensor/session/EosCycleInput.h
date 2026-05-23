/**
 * @file EosCycleInput.h
 * @brief 定义光学传感器组件单周期输入载荷。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"
#include "1q/foundation/pose_types.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosCycleInput 描述光学传感器单周期输入。
 */
struct ONEQ_API EosCycleInput {
  std::uint32_t cycle_index{0U};               /**< 当前周期号 */
  float dt_sec{1.0f};                          /**< 当前周期步长（单位：s） */
  float platform_altitude_m{0.0f};             /**< 平台 WGS84 绝对海拔（单位：m） */
  oneq::foundation::PoseState platform_pose{}; /**< 平台局部位姿状态 */
  EosSceneTargetList scene{};                  /**< 当前周期场景目标输入列表 */
  EosEnvironmentInput environment{};           /**< 当前周期环境事实输入 */
};

}  // namespace session

}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_H_
