#include "1q/airborne_radar/output/TrackOutputQueries.h"

namespace airborne_radar {
namespace output {

namespace {

/**
 * @brief 收集满足给定状态的轨迹快照。
 * @param frame 待查询的输出帧。
 * @param status 目标状态。
 * @return 匹配状态的轨迹快照拷贝列表。
 */
model::TrackStateSnapshotList CollectTracksByStatus(const TrackOutputFrame& frame,
                                                       model::TrackStatus status) {
  model::TrackStateSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].status == status) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

}  // namespace

std::unordered_map<std::uint64_t, model::TrackStateSnapshot> BuildTrackMapByExternalTargetId(
    const TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, model::TrackStateSnapshot> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const model::TrackStateSnapshot& track = frame.tracks[i];
    if (track.external_target_id == 0U) {
      continue;
    }
    track_map[track.external_target_id] = track;
  }
  return track_map;
}

std::unordered_map<std::uint64_t, model::TrackStateSnapshot> BuildTrackMapByAssociationKey(
    const TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, model::TrackStateSnapshot> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    track_map[frame.tracks[i].association_key] = frame.tracks[i];
  }
  return track_map;
}

model::TrackStateSnapshotList CollectTracksByExternalTargetId(const TrackOutputFrame& frame,
                                                                 std::uint64_t external_target_id) {
  model::TrackStateSnapshotList tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].external_target_id == external_target_id) {
      tracks.push_back(frame.tracks[i]);
    }
  }
  return tracks;
}

model::TrackStateSnapshotList CollectConfirmedTracks(const TrackOutputFrame& frame) {
  return CollectTracksByStatus(frame, model::TrackStatus::kConfirmed);
}

model::TrackStateSnapshotList CollectLostTracks(const TrackOutputFrame& frame) {
  return CollectTracksByStatus(frame, model::TrackStatus::kLost);
}

model::TrackStateSnapshotList CollectJammingTracks(const TrackOutputFrame& frame) {
  model::TrackStateSnapshotList tracks;
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

std::size_t CountTracksByStatus(const TrackOutputFrame& frame, model::TrackStatus status) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].status == status) {
      ++count;
    }
  }
  return count;
}

}  // namespace output
}  // namespace airborne_radar
