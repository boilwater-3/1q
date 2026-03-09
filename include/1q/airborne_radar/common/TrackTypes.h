// Copyright 2026. All Rights Reserved.
//
// Description: 定义跟踪对象池与生命周期管理使用的公共轨迹类型。

#ifndef AIRBORNE_RADAR_COMMON_TRACK_TYPES_H_
#define AIRBORNE_RADAR_COMMON_TRACK_TYPES_H_

#include <cstdint>
#include <vector>

#include <Eigen/Core>

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

  /// @brief 简化协方差存储，后续可替换为固定维度矩阵。
  std::vector<float> covariance;
};

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_TRACK_TYPES_H_
