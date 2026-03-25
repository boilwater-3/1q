#include "1q/airborne_radar/core/output/TrackOutputQueries.h"

namespace airborne_radar {
namespace core {
namespace output {

namespace {

/**
 * @brief 收集满足给定状态的轨迹快照。
 * @param frame 待查询的输出帧。
 * @param status 目标状态。
 * @return 匹配状态的轨迹快照拷贝列表。
 */
common::DecisionTrackSnapshotList CollectTracksByStatus(const common::TrackOutputFrame& frame,
                                                        common::DecisionTrackStatus status) {
  common::DecisionTrackSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.status == status) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

}  // namespace

std::unordered_map<std::uint64_t, common::DecisionTrackSnapshot> BuildTrackMapByExternalTargetId(
    const common::TrackOutputFrame& frame) {
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

std::unordered_map<std::uint64_t, common::DecisionTrackSnapshot> BuildTrackMapByAssociationKey(
    const common::TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, common::DecisionTrackSnapshot> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    track_map[frame.tracks[i].state.association_key] = frame.tracks[i];
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

common::DecisionTrackSnapshotList CollectConfirmedTracks(const common::TrackOutputFrame& frame) {
  return CollectTracksByStatus(frame, common::DecisionTrackStatus::kConfirmed);
}

common::DecisionTrackSnapshotList CollectLostTracks(const common::TrackOutputFrame& frame) {
  return CollectTracksByStatus(frame, common::DecisionTrackStatus::kLost);
}

common::DecisionTrackSnapshotList CollectJammingTracks(const common::TrackOutputFrame& frame) {
  common::DecisionTrackSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.jamming_detected) {
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

std::size_t CountTracksByStatus(const common::TrackOutputFrame& frame,
                                common::DecisionTrackStatus status) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.status == status) {
      ++count;
    }
  }
  return count;
}

}  // namespace output
}  // namespace core
}  // namespace airborne_radar
