/**
 * @file EosCycleInputAdapter.h
 * @brief 一步法构建 EosCycleInput，封装锚点求解与逐目标 ENU 转换。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_ADAPTER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosCycleInput 一步坐标适配器。
 *
 * 封装平台锚点求解（TryEcefToLla）与逐目标 ENU 转换
 * （TryMakeEosSceneTargetFromExternalInput），调用方只需提供外部坐标系下的
 * 平台运动学和目标列表，即可获得可直接传入 EosSession::Step() 的 EosCycleInput
 * （场景目标为平台锚点 ENU，见 docs/common/contract.md ENU 契约）。
 */
struct ONEQ_API EosCycleInputAdapter {
  /**
   * @brief 从外部坐标系输入一步构建 EosCycleInput。
   * @param[in] platform 外部平台运动学输入（ECEF 位置决定 ENU 锚点）。
   * @param[in] targets 外部目标输入列表（世界 ECEF/LLA + ECEF 速度）。
   * @param[in] dt_sec 周期步长（单位：秒）。
   * @param[out] output 输出 EosCycleInput；可为 nullptr。
   * @param[out] status 可选状态输出，nullptr 表示不关心失败原因。
   * @return 所有转换成功返回 true。
   */
  static bool Build(const EosExternalPoseInput& platform,
                    const std::vector<EosExternalTargetInput>& targets, float dt_sec,
                    EosCycleInput* output, EosCoordinateStatus* status = nullptr);

 private:
  EosCycleInputAdapter() = delete;
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_ADAPTER_H_
