/**
 * @file PipelineTargetUtils.h
 * @brief 信号处理流水线内部目标特征工具函数。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_PIPELINE_TARGET_UTILS_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_PIPELINE_TARGET_UTILS_H_

#include <Eigen/Core>

#include "1q/airborne_radar/session/RadarSceneTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 从场景目标中解算速度标量。
 * @param target 场景目标数据。
 * @return 速度矢量模长。
 */
inline float ResolveSpeedMagnitude(const session::RadarSceneTarget& target) {
  const Eigen::Vector3f velocity(target.velocity_x, target.velocity_y, target.velocity_z);
  return velocity.norm();
}


}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_PIPELINE_TARGET_UTILS_H_
