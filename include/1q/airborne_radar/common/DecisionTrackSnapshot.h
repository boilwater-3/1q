// Copyright 2026. All Rights Reserved.
//
// @file DecisionTrackSnapshot.h
// @brief 定义供决策层消费的稳定轨迹快照。

#ifndef AIRBORNE_RADAR_COMMON_DECISION_TRACK_SNAPSHOT_H_
#define AIRBORNE_RADAR_COMMON_DECISION_TRACK_SNAPSHOT_H_

#include <cmath>
#include <cstdint>
#include <vector>

namespace airborne_radar {
namespace common {

/// @brief DecisionTrackStatus 表示决策层可见的轨迹生命周期状态。
enum class DecisionTrackStatus {
  /// @brief 候选轨迹。
  kTentative = 0,

  /// @brief 已确认轨迹。
  kConfirmed,

  /// @brief 丢失轨迹。
  kLost
};

/// @brief DecisionTrackStateSnapshot 表示跨周期稳定的轨迹状态快照。
struct DecisionTrackStateSnapshot {
  /// @brief 当前快照对应的关联键。
  std::uint64_t association_key{0};

  /// @brief 外部输入原始目标标识符（0 表示未知/未提供）。
  std::uint64_t external_target_id{0};

  /// @brief 轨迹生命周期状态。
  DecisionTrackStatus status{DecisionTrackStatus::kTentative};

  /// @brief 雷达局部笛卡尔坐标位置（单位：m）。
  float position_x{0.0f};
  float position_y{0.0f};
  float position_z{0.0f};

  /// @brief 目标速度向量（单位：m/s）。
  float velocity_x{0.0f};
  float velocity_y{0.0f};
  float velocity_z{0.0f};

  /// @brief 速度模长（单位：m/s）。
  float speed{0.0f};

  /// @brief 目标加速度向量（单位：m/s^2）。
  float acceleration_x{0.0f};
  float acceleration_y{0.0f};
  float acceleration_z{0.0f};

  /// @brief 加速度模长（单位：m/s^2）。
  float acceleration{0.0f};

  /// @brief 目标估计雷达散射截面积（单位：平方米）。
  float rcs{0.0f};

  /// @brief 该轨迹是否携带干扰观测标记。
  bool jamming_detected{false};

  /// @brief 命中累计计数。
  std::uint32_t hit_count{0};

  /// @brief 连续失配计数。
  std::uint32_t miss_count{0};
};

/// @brief DecisionMeasurementEvidence 表示本周期观测证据快照。
struct DecisionMeasurementEvidence {
  /// @brief 当前轨迹是否具备本周期量测证据。
  bool has_measurement_evidence{false};

  /// @brief 当前轨迹本周期是否被真实量测更新。
  bool updated_this_cycle{false};

  /// @brief 当前轨迹本周期是否仅通过预测维持。
  bool predicted_only_this_cycle{false};

  /// @brief 当前量测是否命中已有轨迹。
  bool matched_existing_track{false};

  /// @brief 当前量测关联代价。
  float association_cost{0.0f};

  /// @brief 当前量测探测裕量（dB）。
  float detection_margin_db{0.0f};

  /// @brief 当前量测是否走位置空间关联路径。
  bool used_position_association{false};

  /// @brief 当前量测是否使用了外部关联种子。
  bool used_external_association_seeds{false};
};

/// @brief DecisionTrackSnapshot 表示从内部轨迹运行态导出的决策快照。
struct DecisionTrackSnapshot {
  /// @brief 跨周期稳定状态快照。
  DecisionTrackStateSnapshot state{};

  /// @brief 本周期证据快照。
  DecisionMeasurementEvidence evidence{};

  /// @brief 默认构造。
  DecisionTrackSnapshot() = default;

  /// @brief 便捷构造函数。
  DecisionTrackSnapshot(float velocity_x_in, float velocity_y_in,
                        float velocity_z_in, float rcs_in,
                        float acceleration_x_in = 0.0f,
                        float acceleration_y_in = 0.0f,
                        float acceleration_z_in = 0.0f,
                        bool jamming_detected_in = false,
                        std::uint64_t external_target_id_in = 0,
                        std::uint64_t association_key_in = 0) {
    state.association_key = association_key_in;
    state.external_target_id = external_target_id_in;
    state.velocity_x = velocity_x_in;
    state.velocity_y = velocity_y_in;
    state.velocity_z = velocity_z_in;
    state.speed = std::sqrt(velocity_x_in * velocity_x_in +
                            velocity_y_in * velocity_y_in +
                            velocity_z_in * velocity_z_in);
    state.acceleration_x = acceleration_x_in;
    state.acceleration_y = acceleration_y_in;
    state.acceleration_z = acceleration_z_in;
    state.acceleration = std::sqrt(acceleration_x_in * acceleration_x_in +
                                   acceleration_y_in * acceleration_y_in +
                                   acceleration_z_in * acceleration_z_in);
    state.rcs = rcs_in;
    state.jamming_detected = jamming_detected_in;
  }
};

/// @brief DecisionTrackSnapshotList 表示供决策层消费的轨迹快照集合。
using DecisionTrackSnapshotList = std::vector<DecisionTrackSnapshot>;

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_DECISION_TRACK_SNAPSHOT_H_
