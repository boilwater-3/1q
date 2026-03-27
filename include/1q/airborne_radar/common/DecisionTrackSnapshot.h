/**
 * @file DecisionTrackSnapshot.h
 * @brief 定义供决策层消费的稳定轨迹快照。
 */

#ifndef AIRBORNE_RADAR_COMMON_DECISION_TRACK_SNAPSHOT_H_
#define AIRBORNE_RADAR_COMMON_DECISION_TRACK_SNAPSHOT_H_

#include <cstdint>
#include <vector>

namespace airborne_radar {
namespace common {

/**
 * @brief DecisionTrackStatus 表示决策层可见的轨迹生命周期状态。
 */
enum class DecisionTrackStatus {
  kTentative = 0, /**< 候选轨迹 */
  kConfirmed,     /**< 已确认轨迹 */
  kLost           /**< 丢失轨迹 */
};

/**
 * @brief DecisionTrackStateSnapshot 表示跨周期稳定的轨迹状态快照。
 */
struct DecisionTrackStateSnapshot {
  std::uint64_t association_key{0};    /**< 当前快照对应的关联键 */
  std::uint64_t external_target_id{0}; /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  DecisionTrackStatus status{DecisionTrackStatus::kTentative}; /**< 轨迹生命周期状态 */

  float position_x{0.0f}; /**< 雷达局部笛卡尔坐标 x（单位：m） */
  float position_y{0.0f}; /**< 雷达局部笛卡尔坐标 y（单位：m） */
  float position_z{0.0f}; /**< 雷达局部笛卡尔坐标 z（单位：m） */

  float velocity_x{0.0f}; /**< 目标速度向量 x 分量（单位：m/s） */
  float velocity_y{0.0f}; /**< 目标速度向量 y 分量（单位：m/s） */
  float velocity_z{0.0f}; /**< 目标速度向量 z 分量（单位：m/s） */

  float speed{0.0f}; /**< 速度模长（单位：m/s） */

  float acceleration_x{0.0f}; /**< 目标加速度向量 x 分量（单位：m/s^2） */
  float acceleration_y{0.0f}; /**< 目标加速度向量 y 分量（单位：m/s^2） */
  float acceleration_z{0.0f}; /**< 目标加速度向量 z 分量（单位：m/s^2） */

  float acceleration{0.0f}; /**< 加速度模长（单位：m/s^2） */

  float rcs{0.0f}; /**< 目标估计雷达散射截面积（单位：平方米） */

  bool jamming_detected{false}; /**< 该轨迹是否携带干扰观测标记 */
  std::uint32_t hit_count{0};   /**< 命中累计计数 */
  std::uint32_t miss_count{0};  /**< 连续失配计数 */
};

/**
 * @brief DecisionMeasurementEvidence 表示本周期观测证据快照。
 */
struct DecisionMeasurementEvidence {
  bool has_measurement_evidence{false};        /**< 当前轨迹是否具备本周期量测证据 */
  bool updated_this_cycle{false};              /**< 当前轨迹本周期是否被真实量测更新 */
  bool predicted_only_this_cycle{false};       /**< 当前轨迹本周期是否仅通过预测维持 */
  bool matched_existing_track{false};          /**< 当前量测是否命中已有轨迹 */
  float association_cost{0.0f};                /**< 当前量测关联代价 */
  float detection_margin_db{0.0f};             /**< 当前量测探测裕量（dB） */
  bool used_position_association{false};       /**< 当前量测是否走位置空间关联路径 */
  bool used_external_association_seeds{false}; /**< 当前量测是否使用了外部关联种子 */
};

/**
 * @brief DecisionTrackSnapshot 表示从内部轨迹运行态导出的决策快照。
 */
struct DecisionTrackSnapshot {
  DecisionTrackStateSnapshot state{};     /**< 跨周期稳定状态快照 */
  DecisionMeasurementEvidence evidence{}; /**< 本周期证据快照 */

  DecisionTrackSnapshot() = default; /**< 默认构造 */

  /** @brief 便捷构造函数 */
  DecisionTrackSnapshot(float velocity_x_in, float velocity_y_in, float velocity_z_in, float rcs_in,
                        float acceleration_x_in = 0.0f, float acceleration_y_in = 0.0f,
                        float acceleration_z_in = 0.0f, bool jamming_detected_in = false,
                        std::uint64_t external_target_id_in = 0,
                        std::uint64_t association_key_in = 0);
};

/** @brief DecisionTrackSnapshotList 表示供决策层消费的轨迹快照集合 */
using DecisionTrackSnapshotList = std::vector<DecisionTrackSnapshot>;

}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_DECISION_TRACK_SNAPSHOT_H_
