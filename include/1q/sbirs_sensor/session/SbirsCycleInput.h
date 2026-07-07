/**
 * @file SbirsCycleInput.h
 * @brief 定义 SBIRS-inspired 单周期输入。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsEnvironmentInput.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 单周期输入，描述一个仿真步的平台几何、目标场景与环境快照。
 * @note 纯数据类型 (POD)。卫星位置用于地球遮挡门控与辐射/噪声几何；`dt_sec` 推进扫描相位。
 */
struct ONEQ_API SbirsCycleInput {
  std::uint32_t cycle_index{0U};        /**< 周期序号 */
  float dt_sec{1.0f};                   /**< 本周期步长，单位 s */
  bool has_satellite_position{false};   /**< 是否提供卫星位置 */
  SbirsVector3M satellite_position_ecef_m{}; /**< 卫星 ECEF 位置，单位 m */
  SbirsSceneTargetList scene{};         /**< 目标场景列表 */
  SbirsEnvironmentInput environment{};  /**< 周期环境输入 */
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_
