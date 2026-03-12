// Copyright 2026. All Rights Reserved.
//
// Description: 定义轨迹生命周期管理的输入类型。

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_TYPES_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_TYPES_H_

#include <cstddef>
#include <cstdint>

#include <Eigen/Core>

#include "1q/airborne_radar/signal/tracking/GaussianTrackState.h"

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

/// @brief AssociationTrackSeed 描述关联阶段使用的上一周期轨迹种子。
struct AssociationTrackSeed {
  /// @brief 关联键。
  std::uint64_t association_key{0};

  /// @brief 是否具备有效的笛卡尔位置。
  bool has_position{false};

  /// @brief 笛卡尔位置种子。
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};

  /// @brief 是否具备有效的高斯状态种子。
  bool has_gaussian_state{false};

  /// @brief 用于位置空间预测的高斯状态。
  GaussianTrackState gaussian_state;
};

/// @brief TrackMeasurement 描述关联后量测输入。
struct TrackMeasurement {
  /// @brief 原始输入目标索引，用于调试和结果回溯。
  std::size_t source_index{0};

  /// @brief 关联键（例如关联模块输出的轨迹键）。
  std::uint64_t association_key{0};

  /// @brief 是否匹配到了上一周期已有轨迹。
  bool matched_existing_track{false};

  /// @brief 关联代价；未命中旧轨迹时为 0。
  float association_cost{0.0f};

  /// @brief 当前量测是否通过位置空间关联路径得到关联结果。
  bool used_position_association{false};

  /// @brief 当前量测关联时是否使用了外部 Lifecycle 轨迹种子。
  bool used_external_association_seeds{false};

  /// @brief 当前量测探测裕量（dB）。
  float detection_margin_db{0.0f};

  /// @brief 当前是否具备可信的笛卡尔位置量测。
  bool has_cartesian_position{false};

  /// @brief 当前量测位置向量（x, y, z）。
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};

  /// @brief 当前量测标量速度估计（m/s）。
  float observed_speed{0.0f};

  /// @brief 当前量测速度向量（vx, vy, vz）。
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};

  /// @brief 当前量测标量加速度估计（m/s^2）。
  float observed_acceleration{0.0f};

  /// @brief 当前量测加速度向量（ax, ay, az）。
  Eigen::Vector3f acceleration{Eigen::Vector3f::Zero()};

  /// @brief 当前量测估计 RCS。
  float rcs{0.0f};

  /// @brief 当前量测是否检测到干扰。
  bool jamming_detected{false};

  /// @brief 笛卡尔坐标系下的量测噪声协方差矩阵 R。
  /// 对于 3D 位置 [x,y,z]，这是一个 3x3 矩阵，由雷达方程由于信噪比推算出的物理方差经坐标转换后得到。
  Eigen::Matrix3f measurement_covariance{Eigen::Matrix3f::Zero()};
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_TYPES_H_
