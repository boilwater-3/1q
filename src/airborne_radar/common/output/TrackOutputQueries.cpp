#include "1q/airborne_radar/common/output/TrackOutputQueries.h"

namespace airborne_radar {
namespace common {
namespace output {

namespace {

/**
 * @brief 收集满足给定状态的轨迹快照。
 * @param frame 待查询的输出帧。
 * @param status 目标状态。
 * @return 匹配状态的轨迹快照拷贝列表。
 */
DecisionTrackSnapshotList CollectTracksByStatus(const TrackOutputFrame& frame,
                                                DecisionTrackStatus status) {
  DecisionTrackSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.status == status) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

}  // namespace

std::unordered_map<std::uint64_t, DecisionTrackSnapshot> BuildTrackMapByExternalTargetId(
    const TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, DecisionTrackSnapshot> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const DecisionTrackSnapshot& track = frame.tracks[i];
    if (track.state.external_target_id == 0U) {
      continue;
    }
    track_map[track.state.external_target_id] = track;
  }
  return track_map;
}

std::unordered_map<std::uint64_t, DecisionTrackSnapshot> BuildTrackMapByAssociationKey(
    const TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, DecisionTrackSnapshot> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    track_map[frame.tracks[i].state.association_key] = frame.tracks[i];
  }
  return track_map;
}

DecisionTrackSnapshotList CollectTracksByExternalTargetId(const TrackOutputFrame& frame,
                                                          std::uint64_t external_target_id) {
  DecisionTrackSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.external_target_id == external_target_id) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

DecisionTrackSnapshotList CollectConfirmedTracks(const TrackOutputFrame& frame) {
  return CollectTracksByStatus(frame, DecisionTrackStatus::kConfirmed);
}

DecisionTrackSnapshotList CollectLostTracks(const TrackOutputFrame& frame) {
  return CollectTracksByStatus(frame, DecisionTrackStatus::kLost);
}

DecisionTrackSnapshotList CollectJammingTracks(const TrackOutputFrame& frame) {
  DecisionTrackSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.jamming_detected) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

bool ContainsExternalTargetId(const TrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.external_target_id == external_target_id) {
      return true;
    }
  }
  return false;
}

std::size_t CountJammingTracks(const TrackOutputFrame& frame) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.jamming_detected) {
      ++count;
    }
  }
  return count;
}

std::size_t CountTracksByStatus(const TrackOutputFrame& frame, DecisionTrackStatus status) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.status == status) {
      ++count;
    }
  }
  return count;
}

}  // namespace output
}  // namespace common
}  // namespace airborne_radar
