// Copyright 2026. All Rights Reserved.
//
// @file TrackOutputQueries.cpp
// @brief 实现轨迹输出帧查询辅助函数。

#include "1q/airborne_radar/core/output/TrackOutputQueries.h"

namespace airborne_radar {
namespace core {
namespace output {

std::unordered_map<std::uint64_t, common::DecisionTrackSnapshot>
BuildTrackMapByExternalTargetId(const common::TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, common::DecisionTrackSnapshot> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const common::DecisionTrackSnapshot& track = frame.tracks[i];
    if (track.state.external_target_id == 0U) {
      continue;
    }
    track_map[track.state.external_target_id] = track;
  }
  return track_map;
}

common::DecisionTrackSnapshotList CollectTracksByExternalTargetId(
    const common::TrackOutputFrame& frame, std::uint64_t external_target_id) {
  common::DecisionTrackSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.external_target_id == external_target_id) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

bool ContainsExternalTargetId(const common::TrackOutputFrame& frame,
                              std::uint64_t external_target_id) {
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.external_target_id == external_target_id) {
      return true;
    }
  }
  return false;
}

std::size_t CountJammingTracks(const common::TrackOutputFrame& frame) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.jamming_detected) {
      ++count;
    }
  }
  return count;
}

} // namespace output
} // namespace core
} // namespace airborne_radar
