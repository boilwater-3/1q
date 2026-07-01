#include "1q/airborne_radar/session/ArCycleResult.h"

namespace airborne_radar {
namespace session {

namespace {

/**
 * @brief 收集满足给定状态的轨迹快照。
 * @param frame 待查询的输出帧。
 * @param status 目标状态。
 * @return 匹配状态的轨迹快照拷贝列表。
 */
session::TrackStateSnapshotList CollectTracksByStatus(const TrackOutputFrame& frame,
                                                       session::TrackStatus status) {
  session::TrackStateSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].status == status) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

}  // namespace

std::unordered_map<std::uint64_t, session::TrackStateSnapshot> BuildTrackMapByExternalTargetId(
    const TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, session::TrackStateSnapshot> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const session::TrackStateSnapshot& track = frame.tracks[i];
    if (track.external_target_id == 0U) {
      continue;
    }
    track_map[track.external_target_id] = track;
  }
  return track_map;
}

std::unordered_map<std::uint64_t, session::TrackStateSnapshot> BuildTrackMapByAssociationKey(
    const TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, session::TrackStateSnapshot> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    track_map[frame.tracks[i].association_key] = frame.tracks[i];
  }
  return track_map;
}

session::TrackStateSnapshotList CollectTracksByExternalTargetId(const TrackOutputFrame& frame,
                                                                 std::uint64_t external_target_id) {
  session::TrackStateSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].external_target_id == external_target_id) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

session::TrackStateSnapshotList CollectConfirmedTracks(const TrackOutputFrame& frame) {
  return CollectTracksByStatus(frame, session::TrackStatus::kConfirmed);
}

session::TrackStateSnapshotList CollectLostTracks(const TrackOutputFrame& frame) {
  return CollectTracksByStatus(frame, session::TrackStatus::kLost);
}

session::TrackStateSnapshotList CollectJammingTracks(const TrackOutputFrame& frame) {
  session::TrackStateSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].jamming_detected) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

bool ContainsExternalTargetId(const TrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].external_target_id == external_target_id) {
      return true;
    }
  }
  return false;
}

std::size_t CountJammingTracks(const TrackOutputFrame& frame) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].jamming_detected) {
      ++count;
    }
  }
  return count;
}

std::size_t CountTracksByStatus(const TrackOutputFrame& frame, session::TrackStatus status) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].status == status) {
      ++count;
    }
  }
  return count;
}

}  // namespace session
}  // namespace airborne_radar
