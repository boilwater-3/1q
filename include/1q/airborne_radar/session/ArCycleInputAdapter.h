/**
 * @file ArCycleInputAdapter.h
 * @brief 机载雷达单步周期输入适配器。
 *
 * 周期输入坐标适配（一步构建 ArCycleInput）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_ADAPTER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArExternalInputAdapter.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief ArCycleInput 一步坐标适配器。
 *
 * 封装 ArExternalInputAdapter 的两步调用（TryMakeArPose + TryMakeTarget），
 * 调用方只需提供外部坐标系下的平台运动学和目标列表，即可获得可直接传入
 * ArSession::Step() 的 ArCycleInput。
 *
 * @note AR 输入以当前雷达为局部坐标原点，`output.platform_pose.position_m`
 *       因此固定为 `(0,0,0)`；平台 ECEF 位置只用于建立局部参考系并转换目标相对位置。
 */
struct ONEQ_API ArCycleInputAdapter {
  /**
   * @brief 从外部坐标系输入一步构建 ArCycleInput。
   * @param[in] platform 外部平台运动学输入。
   * @param[in] targets 外部目标输入列表。
   * @param[in] dt_sec 周期步长（单位：秒）。
   * @param[out] output 输出 ArCycleInput；可为 nullptr。
   * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
   * @return 所有转换成功返回 true。
   */
  static bool Build(const ArExternalPoseInput& platform,
                    const std::vector<ArExternalTargetInput>& targets, float dt_sec,
                    ArCycleInput* output, ArCoordinateStatus* status = nullptr);

  /**
   * @brief 从外部坐标系输入与完整环境快照一步构建 ArCycleInput。
   * @param[in] platform 外部平台运动学输入.
   * @param[in] targets 外部目标输入列表。
   * @param[in] dt_sec 周期步长（单位：秒）。
   * @param[in] environment 本周期完整环境事实快照。
   * @param[out] output 输出 ArCycleInput；可为 nullptr。
   * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
   * @return 所有转换成功返回 true。
   */
  static bool Build(const ArExternalPoseInput& platform,
                    const std::vector<ArExternalTargetInput>& targets, float dt_sec,
                    const ArEnvironmentInput& environment, ArCycleInput* output,
                    ArCoordinateStatus* status = nullptr);

 private:
  ArCycleInputAdapter() = delete;
};


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_ADAPTER_H_
