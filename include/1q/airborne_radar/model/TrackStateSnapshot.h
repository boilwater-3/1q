/**
 * @file TrackStateSnapshot.h
 * @brief 定义供决策层消费的稳定轨迹快照。
 */

#ifndef AIRBORNE_RADAR_COMMON_TRACK_STATE_SNAPSHOT_H_
#define AIRBORNE_RADAR_COMMON_TRACK_STATE_SNAPSHOT_H_

#include <cstdint>
#include <string>
#include <vector>

namespace airborne_radar {
namespace model {

/**
 * @brief TrackStatus 表示对外可见的轨迹生命周期状态。
 */
enum class TrackStatus {
  kTentative = 0, /**< 候选轨迹 */
  kConfirmed,     /**< 已确认轨迹 */
  kLost           /**< 丢失轨迹 */
};

/**
 * @brief TrackStateSnapshot 表示跨周期稳定的轨迹状态快照。
 */
struct TrackStateSnapshot {
  std::uint64_t association_key{0};    /**< 当前快照对应的关联键 */
  std::uint64_t external_target_id{0}; /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  TrackStatus status{TrackStatus::kTentative}; /**< 轨迹生命周期状态 */

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

  /** @brief 决策层填充的目标分类类型（"UNKNOWN"/"LOW_THREAT_TARGET"/"HIGH_THREAT_FIGHTER" 等） */
  std::string target_type{"UNKNOWN"};
  /** @brief 决策层填充的目标分类置信度，范围 [0, 1] */
  float target_probability{0.0f};
};

/** @brief TrackStateSnapshotList 表示供外部消费的轨迹状态快照集合 */
using TrackStateSnapshotList = std::vector<TrackStateSnapshot>;

}  // namespace model
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_TRACK_STATE_SNAPSHOT_H_
