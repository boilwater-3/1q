// Copyright 2026. All Rights Reserved.
//
// @file TrackOutputQueries.h
// @brief 定义面向外部调用方的轨迹输出查询辅助函数。

#ifndef AIRBORNE_RADAR_CORE_OUTPUT_TRACK_OUTPUT_QUERIES_H_
#define AIRBORNE_RADAR_CORE_OUTPUT_TRACK_OUTPUT_QUERIES_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>

#include "1q/airborne_radar/common/TrackOutputFrame.h"

namespace airborne_radar {
namespace core {
namespace output {

/// @brief 按外部目标 ID 构造轨迹映射。
/// @details 仅适用于外部 ID 唯一场景；若存在重复 ID，则后出现的轨迹覆盖先前值。
///          `external_target_id == 0` 的 unknown 轨迹不会进入该映射。
/// @param frame 待查询的输出帧。
/// @return `external_target_id -> track snapshot` 的拷贝映射。
std::unordered_map<std::uint64_t, common::DecisionTrackSnapshot>
BuildTrackMapByExternalTargetId(const common::TrackOutputFrame& frame);

/// @brief 收集指定外部目标 ID 对应的全部轨迹。
/// @param frame 待查询的输出帧。
/// @param external_target_id 外部目标 ID。
/// @return 匹配到的轨迹快照拷贝列表。
common::DecisionTrackSnapshotList CollectTracksByExternalTargetId(
    const common::TrackOutputFrame& frame, std::uint64_t external_target_id);

/// @brief 判断输出帧中是否包含指定外部目标 ID。
/// @param frame 待查询的输出帧。
/// @param external_target_id 外部目标 ID。
/// @return 至少存在一条匹配轨迹时返回 true。
bool ContainsExternalTargetId(const common::TrackOutputFrame& frame,
                              std::uint64_t external_target_id);

/// @brief 统计携带干扰标记的轨迹数量。
/// @param frame 待查询的输出帧。
/// @return `state.jamming_detected == true` 的轨迹数。
std::size_t CountJammingTracks(const common::TrackOutputFrame& frame);

} // namespace output
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_OUTPUT_TRACK_OUTPUT_QUERIES_H_
