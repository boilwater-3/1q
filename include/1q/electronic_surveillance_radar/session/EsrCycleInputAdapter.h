/**
 * @file EsrCycleInputAdapter.h
 * @brief 一步法构建 EsrCycleInput，封装 ExternalInputAdapter 的两步调用。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_ADAPTER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrCycleInput 一步坐标适配器。
 *
 * 封装 ExternalInputAdapter 的两步调用（TryMakeEsrPose + TryMakeEsrSceneEmitter），
 * 调用方只需提供外部坐标系下的平台运动学和辐射源列表，即可获得可直接传入
 * EsrSession::Step() 的 EsrCycleInput。
 */
struct ONEQ_API EsrCycleInputAdapter {
  /**
   * @brief 从外部坐标系输入一步构建 EsrCycleInput。
   * @param[in] platform 外部平台运动学输入。
   * @param[in] emitters 外部辐射源输入列表。
   * @param[in] dt_sec 周期步长（单位：秒）。
   * @param[out] output 输出 EsrCycleInput；可为 nullptr。
   * @param[out] status 可选状态输出，nullptr 表示不关心失败原因。
   * @return 所有转换成功返回 true。
   */
  static bool Build(const EsrExternalPoseInput& platform,
                    const std::vector<EsrExternalEmitterInput>& emitters, float dt_sec,
                    EsrCycleInput* output, EsrCoordinateStatus* status = nullptr);

  /**
   * @brief 从外部坐标系输入与完整环境快照一步构建 EsrCycleInput。
   * @param[in] platform 外部平台运动学输入。
   * @param[in] emitters 外部辐射源输入列表。
   * @param[in] dt_sec 周期步长（单位：秒）。
   * @param[in] environment 本周期完整环境事实快照。
   * @param[out] output 输出 EsrCycleInput；可为 nullptr。
   * @param[out] status 可选状态输出，nullptr 表示不关心失败原因。
   * @return 所有转换成功返回 true。
   */
  static bool Build(const EsrExternalPoseInput& platform,
                    const std::vector<EsrExternalEmitterInput>& emitters, float dt_sec,
                    const EsrEnvironmentInput& environment, EsrCycleInput* output,
                    EsrCoordinateStatus* status = nullptr);

 private:
  EsrCycleInputAdapter() = delete;
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_ADAPTER_H_
