/**
 * @file TrackOutputQueries.h
 * @brief 定义面向外部调用方的轨迹输出查询辅助函数。
 */

#ifndef AIRBORNE_RADAR_COMMON_OUTPUT_TRACK_OUTPUT_QUERIES_H_
#define AIRBORNE_RADAR_COMMON_OUTPUT_TRACK_OUTPUT_QUERIES_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>

#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace output {

/**
 * @brief 按外部目标 ID 构造轨迹映射。
 * @details 面向手工构造或历史兼容输出帧的宽容查询工具。会话/控制器主路径会对非零
 *          `external_target_id` 执行唯一性校验；若传入帧仍存在重复 ID，则后出现的轨迹覆盖先前值。
 *          `external_target_id == 0` 的 unknown 轨迹不会进入该映射。
 * @param[in] frame 待查询的输出帧。
 * @return `external_target_id -> track snapshot` 的拷贝映射。
 */
ONEQ_API std::unordered_map<std::uint64_t, model::TrackStateSnapshot>
BuildTrackMapByExternalTargetId(const TrackOutputFrame& frame);

/**
 * @brief 按关联键构造轨迹映射。
 * @details 适用于 association_key 唯一场景；后出现的相同键会覆盖先前值。
 * @param[in] frame 待查询的输出帧。
 * @return `association_key -> track snapshot` 的拷贝映射。
 */
ONEQ_API std::unordered_map<std::uint64_t, model::TrackStateSnapshot>
BuildTrackMapByAssociationKey(const TrackOutputFrame& frame);

/**
 * @brief 收集指定外部目标 ID 对应的全部轨迹。
 * @param[in] frame 待查询的输出帧。
 * @param[in] external_target_id 外部目标 ID。
 * @return 匹配到的轨迹快照拷贝列表。
 */
ONEQ_API model::TrackStateSnapshotList CollectTracksByExternalTargetId(
    const TrackOutputFrame& frame, std::uint64_t external_target_id);

/**
 * @brief 收集所有已确认轨迹。
 * @param[in] frame 待查询的输出帧。
 * @return `status == kConfirmed` 的轨迹快照拷贝列表。
 */
ONEQ_API model::TrackStateSnapshotList CollectConfirmedTracks(const TrackOutputFrame& frame);

/**
 * @brief 收集所有 lost 轨迹。
 * @param[in] frame 待查询的输出帧。
 * @return `status == kLost` 的轨迹快照拷贝列表。
 */
ONEQ_API model::TrackStateSnapshotList CollectLostTracks(const TrackOutputFrame& frame);

/**
 * @brief 收集所有带干扰标记的轨迹。
 * @param[in] frame 待查询的输出帧。
 * @return `state.jamming_detected == true` 的轨迹快照拷贝列表。
 */
ONEQ_API model::TrackStateSnapshotList CollectJammingTracks(const TrackOutputFrame& frame);

/**
 * @brief 判断输出帧中是否包含指定外部目标 ID。
 * @param[in] frame 待查询的输出帧。
 * @param[in] external_target_id 外部目标 ID。
 * @return 至少存在一条匹配轨迹时返回 true。
 */
ONEQ_API bool ContainsExternalTargetId(const TrackOutputFrame& frame,
                                       std::uint64_t external_target_id);

/**
 * @brief 统计携带干扰标记的轨迹数量。
 * @param[in] frame 待查询的输出帧。
 * @return `state.jamming_detected == true` 的轨迹数。
 */
ONEQ_API std::size_t CountJammingTracks(const TrackOutputFrame& frame);

/**
 * @brief 按生命周期状态统计轨迹数量。
 * @param[in] frame 待查询的输出帧。
 * @param[in] status 目标状态。
 * @return 匹配状态的轨迹数。
 */
ONEQ_API std::size_t CountTracksByStatus(const TrackOutputFrame& frame,
                                         model::TrackStatus status);

}  // namespace output
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_OUTPUT_TRACK_OUTPUT_QUERIES_H_
