/**
 * @file EosCycleInput.h
 * @brief 定义光学传感器组件单周期输入载荷。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosCycleInput 描述光学传感器单周期输入。
 * @note 场景目标 `scene` 为平台锚点 radar-local ENU（见 EosSceneTarget）；库内由
 *       ENU 位置 + `platform_attitude_deg`（Body->ENU，deg）派生体系球坐标
 *       （斜距/方位/仰角）驱动探测链。锚点 ENU 由集成层以平台 ECEF 位置建立
 *       （公共 `TryEcefToLla` + `TryMakeEnuSceneState`）。
 */
struct ONEQ_API EosCycleInput {
  std::uint32_t cycle_index{0U};               /**< 当前周期号 */
  float dt_sec{1.0f};                          /**< 当前周期步长（单位：s） */
  float platform_altitude_m{0.0f};             /**< 平台 WGS84 绝对海拔（单位：m） */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{}; /**< 平台姿态角（Body->ENU，单位：deg） */
  EosSceneTargetList scene{};                  /**< 当前周期场景目标输入列表（平台锚点 ENU） */
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_H_
