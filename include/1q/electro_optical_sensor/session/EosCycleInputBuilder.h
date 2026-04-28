/**
 * @file EosCycleInputBuilder.h
 * @brief 一步法构建 EosCycleInput，封装 ExternalInputAdapter 的两步调用。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_BUILDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_BUILDER_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosCycleInput 一步构建器。
 *
 * 封装 ExternalInputAdapter 的两步调用（TryMakeEosPose + TryMakeEosSceneTarget），
 * 调用方只需提供外部坐标系下的平台运动学和目标列表，即可获得可直接传入
 * EosSession::Step() 的 EosCycleInput。
 */
struct ONEQ_API EosCycleInputBuilder {
  /**
   * @brief 从外部坐标系输入一步构建 EosCycleInput。
   * @param[in] platform 外部平台运动学输入。
   * @param[in] targets 外部目标输入列表。
   * @param[in] dt_sec 周期步长（单位：秒）。
   * @param[out] output 输出 EosCycleInput；可为 nullptr。
   * @param[out] status 可选状态输出，nullptr 表示不关心失败原因。
   * @return 所有转换成功返回 true。
   */
  static bool Build(const EosExternalPoseInput& platform,
                    const std::vector<EosExternalTargetInput>& targets,
                    float dt_sec,
                    EosCycleInput* output,
                    EosCoordinateStatus* status = nullptr);

 private:
  EosCycleInputBuilder() = delete;
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_INPUT_BUILDER_H_
