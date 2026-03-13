// Copyright 2026. All Rights Reserved.
//
// Description: 定义跟踪对象池与生命周期管理使用的公共轨迹类型。

#ifndef AIRBORNE_RADAR_COMMON_TRACK_TYPES_H_
#define AIRBORNE_RADAR_COMMON_TRACK_TYPES_H_

#include <cstdint>

#include <Eigen/Core>

#include "1q/airborne_radar/signal/tracking/GaussianTrackState.h"

namespace airborne_radar {
namespace common {

/// @brief TrackStatus 表示轨迹对象在生命周期中的状态。
enum class TrackStatus {
  /// @brief 候选轨迹，尚未达到确认阈值。
  kTentative = 0,

  /// @brief 已确认轨迹，可参与稳定跟踪与下游输出。
  kConfirmed,

  /// @brief 暂时丢失轨迹，等待短时重捕获。
  kLost,

  /// @brief 已回收轨迹，返回对象池可复用。
  kRecycled
};

/// @brief TrackState 是对象池内复用的“重对象”，承载完整生命周期信息。
struct TrackState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief 全局唯一轨迹编号。
  std::uint64_t track_id{0};

  /// @brief 轨迹所属发现批号（首次建轨批次号）。
  std::uint64_t batch_id{0};

  /// @brief 外部输入原始目标标识符（0 表示未知/未提供）。
  std::uint64_t external_target_id{0};

  /// @brief 复用代次，用于识别已回收对象的旧引用。
  std::uint32_t generation{0};

  /// @brief 当前轨迹生命周期状态。
  TrackStatus status{TrackStatus::kTentative};

  /// @brief 首次创建所在周期编号。
  std::uint32_t first_cycle{0};

  /// @brief 最近一次命中更新的周期编号。
  std::uint32_t last_update_cycle{0};

  /// @brief 连续失配计数。
  std::uint32_t miss_count{0};

  /// @brief 命中累计计数。
  std::uint32_t hit_count{0};

  /// @brief 目标位置向量（x, y, z）。
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};

  /// @brief 目标速度向量（vx, vy, vz）。
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};

  /// @brief 目标加速度向量（ax, ay, az）。
  Eigen::Vector3f acceleration{Eigen::Vector3f::Zero()};

  /// @brief 目标估计 RCS（单位：平方米）。
  float rcs{0.0f};

  /// @brief 当前周期是否检测到与该目标相关的干扰信号。
  bool jamming_detected{false};

  /// @brief 高斯状态估计（位置+速度的均值和协方差）。
  /// @details 与 Kalman 滤波器的 GaussianTrackState 共享类型定义。
  signal::tracking::GaussianTrackState gaussian_state;
};

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_TRACK_TYPES_H_
