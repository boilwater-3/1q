/**
 * @file sensor_utils.h
 * @brief 自定义实体-组件示例：传感器组件共用的平台坐标工具。
 *
 * 平台 LLA/航向/速度 → ECEF（零姿态保持共享局部系）。三传感器会话输出
 * → 泛型融合探测记录的适配（Adapt* 系列）与源通道常量已上移
 * examples/common/sensor_adapt.h（与 behavior_layer 共用，消除双份维护）；
 * 本文件仅保留平台坐标转换。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SENSOR_UTILS_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SENSOR_UTILS_H_

#include <cmath>
#include <cstdint>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/coordinate/types.h"

namespace component_attachment {

/// 平台 LLA/航向/速度 → ECEF 位置与速度（三会话共用；零姿态保持共享局部系）。
inline void ResolvePlatformEcef(const oneq::coordinate::LlaPositionDegM& position,
                                double heading_deg, double speed_mps,
                                oneq::coordinate::EcefPositionM* ecef_position,
                                oneq::coordinate::EcefVelocityMps* ecef_velocity) {
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  oneq::coordinate::EcefPositionM ecef;
  if (oneq::coordinate::TryLlaToEcef(position, &ecef)) {
    *ecef_position = ecef;
  }
  const double heading_rad = heading_deg * kDegToRad;
  oneq::coordinate::EnuVelocityMps enu_velocity;
  enu_velocity.east_mps = speed_mps * std::sin(heading_rad);
  enu_velocity.north_mps = speed_mps * std::cos(heading_rad);
  enu_velocity.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps ecef_vel;
  if (oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, position, &ecef_vel)) {
    *ecef_velocity = ecef_vel;
  }
}

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SENSOR_UTILS_H_
