// Copyright 2026. All Rights Reserved.
//
// Description: 定义轨迹生命周期管理的输入类型。

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_TYPES_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_TYPES_H_

#include <cstdint>

#include <Eigen/Core>

namespace airborne_radar {
namespace signal {
namespace tracking {

/// @brief CycleContext 描述一次处理周期的元信息。
struct CycleContext {
  /// @brief 当前周期号。
  std::uint32_t cycle_index{0};

  /// @brief 当前探测批号。
  std::uint64_t batch_id{0};
};

/// @brief TrackMeasurement 描述关联后量测输入。
struct TrackMeasurement {
  /// @brief 关联键（例如关联模块输出的轨迹键）。
  std::uint64_t association_key{0};

  /// @brief 当前量测位置向量（x, y, z）。
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};

  /// @brief 当前量测速度向量（vx, vy, vz）。
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};

  /// @brief 当前量测加速度向量（ax, ay, az）。
  Eigen::Vector3f acceleration{Eigen::Vector3f::Zero()};

  /// @brief 当前量测估计 RCS。
  float rcs{0.0f};

  /// @brief 当前量测是否检测到干扰。
  bool jamming_detected{false};
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_TYPES_H_
