/**
 * @file sensor_utils.h
 * @brief 自定义实体-组件示例：传感器组件共用的平台坐标工具。
 *
 * 平台 LLA/航向/速度 → ECEF（零姿态保持共享局部系）。三传感器会话输出
 * → 泛型融合探测记录的适配（Adapt* 系列）与源通道常量已上移
 * examples/common/sensor_adapt.h（示例共享边界适配，消除重复维护）；
 * 本文件仅保留平台坐标薄包装（航向分解与 ECEF 投影为库内单函数）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_SENSOR_UTILS_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_SENSOR_UTILS_H_

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/coordinate/types.h"

namespace component_attachment {

/// 平台 LLA/航向/速度 → ECEF 位置与速度（三会话共用；零姿态保持共享局部系）。
inline void ResolvePlatformEcef(const oneq::coordinate::LlaPositionDegM& position,
                                double heading_deg, double speed_mps,
                                oneq::coordinate::EcefPositionM* ecef_position,
                                oneq::coordinate::EcefVelocityMps* ecef_velocity) {
  oneq::coordinate::EcefPositionM ecef;
  if (oneq::coordinate::TryLlaToEcef(position, &ecef)) {
    *ecef_position = ecef;
  }
  oneq::coordinate::EcefVelocityMps ecef_vel;
  if (oneq::coordinate::TryMakeEcefVelocityFromHeading(heading_deg, speed_mps, position,
                                                       &ecef_vel)) {
    *ecef_velocity = ecef_vel;
  }
}

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_SENSOR_UTILS_H_
