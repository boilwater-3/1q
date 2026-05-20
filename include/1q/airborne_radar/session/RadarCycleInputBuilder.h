/**
 * @file RadarCycleInputBuilder.h
 * @brief 一步法构建 RadarCycleInput，封装 ExternalInputAdapter 的两步调用。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_CYCLE_INPUT_BUILDER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_CYCLE_INPUT_BUILDER_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarCycleInput 一步构建器。
 *
 * 封装 ExternalInputAdapter 的两步调用（TryMakeRadarPose + TryMakeTarget），
 * 调用方只需提供外部坐标系下的平台运动学和目标列表，即可获得可直接传入
 * RadarSession::Step() 的 RadarCycleInput。
 *
 * @note AR 输入以当前雷达为局部坐标原点，`output.platform_pose.position_m`
 *       因此固定为 `(0,0,0)`；平台 ECEF 位置只用于建立局部参考系并转换目标相对位置。
 */
struct ONEQ_API RadarCycleInputBuilder {
  /**
   * @brief 从外部坐标系输入一步构建 RadarCycleInput。
   * @param[in] platform 外部平台运动学输入。
   * @param[in] targets 外部目标输入列表。
   * @param[in] dt_sec 周期步长（单位：秒）。
   * @param[out] output 输出 RadarCycleInput；可为 nullptr。
   * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
   * @return 所有转换成功返回 true。
   */
  static bool Build(const RadarExternalPoseInput& platform,
                    const std::vector<RadarExternalTargetInput>& targets, float dt_sec,
                    RadarCycleInput* output, RadarCoordinateStatus* status = nullptr);

  /**
   * @brief 从外部坐标系输入与完整环境快照一步构建 RadarCycleInput。
   * @param[in] platform 外部平台运动学输入.
   * @param[in] targets 外部目标输入列表。
   * @param[in] dt_sec 周期步长（单位：秒）。
   * @param[in] environment 本周期完整环境事实快照。
   * @param[out] output 输出 RadarCycleInput；可为 nullptr。
   * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
   * @return 所有转换成功返回 true。
   */
  static bool Build(const RadarExternalPoseInput& platform,
                    const std::vector<RadarExternalTargetInput>& targets, float dt_sec,
                    const RadarEnvironmentInput& environment, RadarCycleInput* output,
                    RadarCoordinateStatus* status = nullptr);

 private:
  RadarCycleInputBuilder() = delete;
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_CYCLE_INPUT_BUILDER_H_
