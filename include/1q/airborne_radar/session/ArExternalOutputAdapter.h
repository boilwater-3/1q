/**
 * @file ArExternalOutputAdapter.h
 * @brief 机载雷达外部输出适配类型集合。
 *
 * 外部输出适配（内部轨迹快照转 ECEF 运动学）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_EXTERNAL_OUTPUT_ADAPTER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_EXTERNAL_OUTPUT_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 外部可消费的单目标轨迹运动学输出。
 * @note 内部 TrackStateSnapshot 使用雷达局部相对坐标；本结构把位置和速度转换回 ECEF。
 */
struct ONEQ_API ArExternalTrackKinematics {
  std::uint64_t association_key{0};    /**< 当前快照对应的关联键 */
  std::uint64_t external_target_id{0}; /**< DEPRECATED（sim-only，注册遗留）：场景真值目标 ID（0 表示未知/未提供）；权威归属见 ArCycleResult.track_attributions */
  std::string target_name{};           /**< DEPRECATED（sim-only，注册遗留）：目标名称，仅用于人读与调试；权威归属见 ArCycleResult.track_attributions */
  session::TrackStatus status{session::TrackStatus::kTentative}; /**< 轨迹生命周期状态 */

  oneq::coordinate::EcefPositionM target_position_ecef_m{}; /**< 目标 ECEF 位置（单位：m） */
  oneq::coordinate::EcefVelocityMps target_velocity_mps{};  /**< 目标 ECEF 绝对速度（单位：m/s） */

  float speed{0.0f};              /**< 目标 ECEF 绝对速度模长（单位：m/s） */
  float rcs{0.0f};                /**< 目标估计雷达散射截面积（单位：平方米） */
  std::uint32_t hit_count{0};     /**< 命中累计计数 */
  std::uint32_t miss_count{0};    /**< 连续失配计数 */
  float target_probability{0.0f}; /**< 决策层填充的目标分类置信度 */
};

/** @brief ArExternalTrackKinematicsList 表示外部轨迹运动学输出集合。 */
using ArExternalTrackKinematicsList = std::vector<ArExternalTrackKinematics>;

/**
 * @brief 将内部雷达局部轨迹快照转换为外部 ECEF 运动学输出。
 * @param[in] snapshot 内部 TrackStateSnapshot，位置/速度为雷达局部相对坐标。
 * @param[in] reference 雷达局部参考系信息。
 * @param[in] radar_local_velocity_mps 雷达平台在雷达局部坐标系下的速度。
 * @param[out] output 输出外部轨迹运动学；可为 nullptr。
 * @return 成功返回 true。
 */
ONEQ_API bool TryMakeExternalTrackFromSnapshot(const session::TrackStateSnapshot& snapshot,
                                               const oneq::coordinate::LocalFrameReference& reference,
                                               oneq::coordinate::Vector3d radar_local_velocity_mps,
                                               ArExternalTrackKinematics* output);


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_EXTERNAL_OUTPUT_ADAPTER_H_
