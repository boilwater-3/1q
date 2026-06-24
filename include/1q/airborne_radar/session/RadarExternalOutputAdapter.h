/**
 * @file RadarExternalOutputAdapter.h
 * @brief 机载雷达外部输出适配统一入口。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_OUTPUT_ADAPTER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_OUTPUT_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"
#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 外部可消费的单目标轨迹运动学输出。
 * @note 内部 TrackStateSnapshot 使用雷达局部相对坐标；本结构把位置和速度转换回 ECEF。
 */
struct ONEQ_API RadarExternalTrackKinematics {
  std::uint64_t association_key{0};    /**< 当前快照对应的关联键 */
  std::uint64_t external_target_id{0}; /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  std::string target_name{};           /**< 可选目标名称，随 external_target_id 透传，仅用于人读与调试 */
  model::TrackStatus status{model::TrackStatus::kTentative}; /**< 轨迹生命周期状态 */

  oneq::coordinate::EcefPositionM target_position_ecef_m{}; /**< 目标 ECEF 位置（单位：m） */
  oneq::coordinate::EcefVelocityMps target_velocity_mps{};  /**< 目标 ECEF 绝对速度（单位：m/s） */

  float speed{0.0f};              /**< 目标 ECEF 绝对速度模长（单位：m/s） */
  float rcs{0.0f};                /**< 目标估计雷达散射截面积（单位：平方米） */
  bool jamming_detected{false};   /**< 该轨迹是否携带干扰观测标记 */
  std::uint32_t hit_count{0};     /**< 命中累计计数 */
  std::uint32_t miss_count{0};    /**< 连续失配计数 */
  float target_probability{0.0f}; /**< 决策层填充的目标分类置信度 */
};

/** @brief RadarExternalTrackKinematicsList 表示外部轨迹运动学输出集合。 */
using RadarExternalTrackKinematicsList = std::vector<RadarExternalTrackKinematics>;

/**
 * @brief 将内部雷达局部轨迹快照转换为外部 ECEF 运动学输出。
 * @param[in] snapshot 内部 TrackStateSnapshot，位置/速度为雷达局部相对坐标。
 * @param[in] reference 雷达局部参考系信息。
 * @param[in] radar_local_velocity_mps 雷达平台在雷达局部坐标系下的速度。
 * @param[out] output 输出外部轨迹运动学；可为 nullptr。
 * @return 成功返回 true。
 */
ONEQ_API bool TryMakeExternalTrackFromSnapshot(const model::TrackStateSnapshot& snapshot,
                                               const oneq::coordinate::LocalFrameReference& reference,
                                               oneq::foundation::Vector3f radar_local_velocity_mps,
                                               RadarExternalTrackKinematics* output);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_OUTPUT_ADAPTER_H_
