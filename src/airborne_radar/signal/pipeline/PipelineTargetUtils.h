/**
 * @file PipelineTargetUtils.h
 * @brief 信号处理流水线内部目标特征工具函数。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_PIPELINE_TARGET_UTILS_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_PIPELINE_TARGET_UTILS_H_

#include <Eigen/Core>

#include "1q/airborne_radar/common/model/TargetFeature.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief 从目标特征中解算速度标量。
 * @param target 目标特征数据，包含三维速度分量和标量速度。
 * @return 速度矢量的模长；当三维分量全为零时回退到 current_track_speed 字段。
 */
inline float ResolveSpeedMagnitude(const common::model::TargetFeature& target) {
  const Eigen::Vector3f velocity(target.current_track_velocity_x, target.current_track_velocity_y,
                                 target.current_track_velocity_z);
  if (velocity.squaredNorm() > 0.0f) {
    return velocity.norm();
  }
  return target.current_track_speed;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_PIPELINE_TARGET_UTILS_H_
