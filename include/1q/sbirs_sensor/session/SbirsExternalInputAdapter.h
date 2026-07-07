/**
 * @file SbirsExternalInputAdapter.h
 * @brief 定义 SBIRS-inspired 外部输入适配占位入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_INPUT_ADAPTER_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 由最小输入集合构造单周期输入的便捷工厂。
 * @param[in] cycle_index 周期序号
 * @param[in] dt_sec 步长，单位 s
 * @param[in] satellite_position_ecef_m 卫星 ECEF 位置，单位 m
 * @param[in] scene 目标场景列表
 * @return 已设置平台几何与场景的 `SbirsCycleInput`
 */
ONEQ_API SbirsCycleInput MakeSbirsCycleInput(std::uint32_t cycle_index, float dt_sec,
                                             const SbirsVector3M& satellite_position_ecef_m,
                                             const SbirsSceneTargetList& scene);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_INPUT_ADAPTER_H_
