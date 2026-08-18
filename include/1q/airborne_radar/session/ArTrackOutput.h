/**
 * @file ArTrackOutput.h
 * @brief 定义 AR 工程周期发布的稳定轨迹输出帧与只读查询函数。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_OUTPUT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_OUTPUT_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>

#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/** @brief 单个已完成 AR 周期发布的稳定轨迹输出帧。 */
struct ONEQ_API TrackOutputFrame {
  std::uint32_t cycle_index{0};             /**< 当前周期号。 */
  std::uint64_t batch_id{0};                /**< 当前批号。 */
  session::TrackStateSnapshotList tracks{}; /**< 当前周期发布的轨迹快照。 */
};

/** @brief 按非零外部目标 ID 构造轨迹映射。 */
ONEQ_API std::unordered_map<std::uint64_t, session::TrackStateSnapshot>
BuildTrackMapByExternalTargetId(const TrackOutputFrame& frame);

/** @brief 按关联键构造轨迹映射。 */
ONEQ_API std::unordered_map<std::uint64_t, session::TrackStateSnapshot>
BuildTrackMapByAssociationKey(const TrackOutputFrame& frame);

/** @brief 收集指定外部目标 ID 对应的全部轨迹。 */
ONEQ_API session::TrackStateSnapshotList CollectTracksByExternalTargetId(
    const TrackOutputFrame& frame, std::uint64_t external_target_id);

/** @brief 收集所有已确认轨迹。 */
ONEQ_API session::TrackStateSnapshotList CollectConfirmedTracks(const TrackOutputFrame& frame);

/** @brief 收集所有 lost 轨迹。 */
ONEQ_API session::TrackStateSnapshotList CollectLostTracks(const TrackOutputFrame& frame);

/** @brief 判断输出帧中是否包含指定外部目标 ID。 */
ONEQ_API bool ContainsExternalTargetId(const TrackOutputFrame& frame,
                                       std::uint64_t external_target_id);

/** @brief 按生命周期状态统计轨迹数量。 */
ONEQ_API std::size_t CountTracksByStatus(const TrackOutputFrame& frame,
                                        session::TrackStatus status);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_OUTPUT_H_
