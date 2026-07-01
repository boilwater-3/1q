/**
 * @file TrackStateSnapshotEmitter.cpp
 * @brief 轨迹快照导出器实现。
 */

#include "airborne_radar/signal/tracking/TrackStateSnapshotEmitter.h"

#include <cmath>

namespace airborne_radar {
namespace signal {
namespace tracking {

namespace {

/**
 * @brief 将内部轨迹状态映射为决策层轨迹状态。
 * @param status 内部轨迹状态。
 * @return 决策层可见的轨迹状态。
 */
session::TrackStatus ToTrackStatus(TrackStatus status) {
  switch (status) {
    case TrackStatus::kConfirmed:
      return session::TrackStatus::kConfirmed;
    case TrackStatus::kLost:
      return session::TrackStatus::kLost;
    case TrackStatus::kTentative:
    case TrackStatus::kRecycled:
    default:
      return session::TrackStatus::kTentative;
  }
}

}  // namespace

int TrackStateSnapshotEmitter::OutputPriority(const TrackState& track) {
  switch (track.status) {
    case TrackStatus::kConfirmed:
      return 3;
    case TrackStatus::kTentative:
      return 2;
    case TrackStatus::kLost:
      return 1;
    case TrackStatus::kRecycled:
    default:
      return 0;
  }
}

bool TrackStateSnapshotEmitter::IsBetterKnownExternalOutput(const ActiveTrackEntry& candidate,
                                                            const ActiveTrackEntry& current) {
  const TrackState& candidate_track = *candidate.track;
  const TrackState& current_track = *current.track;
  const int candidate_priority = OutputPriority(candidate_track);
  const int current_priority = OutputPriority(current_track);
  if (candidate_priority != current_priority) {
    return candidate_priority > current_priority;
  }
  if (candidate_track.last_update_cycle != current_track.last_update_cycle) {
    return candidate_track.last_update_cycle > current_track.last_update_cycle;
  }
  if (candidate_track.hit_count != current_track.hit_count) {
    return candidate_track.hit_count > current_track.hit_count;
  }
  if (candidate_track.miss_count != current_track.miss_count) {
    return candidate_track.miss_count < current_track.miss_count;
  }
  return candidate.key > current.key;
}

std::vector<TrackStateSnapshotEmitter::ActiveTrackEntry>
TrackStateSnapshotEmitter::SelectOutputTracks(const std::vector<ActiveTrackEntry>& active_tracks) {
  std::vector<ActiveTrackEntry> selected_tracks;
  selected_tracks.reserve(active_tracks.size());
  std::unordered_map<std::uint64_t, std::size_t> known_external_index;

  for (std::vector<ActiveTrackEntry>::const_iterator it = active_tracks.begin();
       it != active_tracks.end(); ++it) {
    if (it->track == nullptr) {
      continue;
    }
    const std::uint64_t external_target_id = it->track->external_target_id;
    if (external_target_id == 0U) {
      selected_tracks.push_back(*it);
      continue;
    }

    std::unordered_map<std::uint64_t, std::size_t>::const_iterator found =
        known_external_index.find(external_target_id);
    if (found == known_external_index.end()) {
      known_external_index[external_target_id] = selected_tracks.size();
      selected_tracks.push_back(*it);
      continue;
    }

    ActiveTrackEntry& current = selected_tracks[found->second];
    if (IsBetterKnownExternalOutput(*it, current)) {
      current = *it;
    }
  }

  return selected_tracks;
}

void TrackStateSnapshotEmitter::Refresh(
    const std::unordered_map<std::uint64_t, TrackState*>& tracks_by_key) {
  active_tracks_.clear();
  active_tracks_.reserve(tracks_by_key.size());
  for (std::unordered_map<std::uint64_t, TrackState*>::const_iterator it = tracks_by_key.begin();
       it != tracks_by_key.end(); ++it) {
    if (it->second->status != TrackStatus::kRecycled) {
      ActiveTrackEntry entry;
      entry.key = it->first;
      entry.track = it->second;
      active_tracks_.push_back(entry);
    }
  }
}

session::ArSceneTargetList TrackStateSnapshotEmitter::BuildSceneTargetSnapshot() const {
  session::ArSceneTargetList features;
  features.reserve(active_tracks_.size());
  for (std::vector<ActiveTrackEntry>::const_iterator it = active_tracks_.begin();
       it != active_tracks_.end(); ++it) {
    const TrackState& track = *it->track;
    session::ArSceneTarget feature(track.velocity(0), track.velocity(1), track.velocity(2),
                                      track.rcs);
    feature.position_x = track.position(0);
    feature.position_y = track.position(1);
    feature.position_z = track.position(2);
    feature.external_target_id = track.external_target_id;
    feature.target_name = track.target_name;
    features.push_back(feature);
  }
  return features;
}

session::TrackStateSnapshotList TrackStateSnapshotEmitter::BuildTrackStateSnapshots() const {
  const std::vector<ActiveTrackEntry> output_tracks = SelectOutputTracks(active_tracks_);
  session::TrackStateSnapshotList snapshots;
  snapshots.reserve(output_tracks.size());
  for (std::vector<ActiveTrackEntry>::const_iterator it = output_tracks.begin();
       it != output_tracks.end(); ++it) {
    const std::uint64_t key = it->key;
    const TrackState& track = *it->track;
    session::TrackStateSnapshot snapshot;
    snapshot.association_key = key;
    snapshot.external_target_id = track.external_target_id;
    snapshot.target_name = track.target_name;
    snapshot.velocity_x = track.velocity(0);
    snapshot.velocity_y = track.velocity(1);
    snapshot.velocity_z = track.velocity(2);
    snapshot.speed = std::sqrt(snapshot.velocity_x * snapshot.velocity_x +
                               snapshot.velocity_y * snapshot.velocity_y +
                               snapshot.velocity_z * snapshot.velocity_z);
    snapshot.acceleration_x = track.acceleration(0);
    snapshot.acceleration_y = track.acceleration(1);
    snapshot.acceleration_z = track.acceleration(2);
    snapshot.acceleration = std::sqrt(snapshot.acceleration_x * snapshot.acceleration_x +
                                      snapshot.acceleration_y * snapshot.acceleration_y +
                                      snapshot.acceleration_z * snapshot.acceleration_z);
    snapshot.rcs = track.rcs;
    snapshot.jamming_detected = track.jamming_detected;
    snapshot.status = ToTrackStatus(track.status);
    snapshot.position_x = track.position(0);
    snapshot.position_y = track.position(1);
    snapshot.position_z = track.position(2);
    snapshot.hit_count = track.hit_count;
    snapshot.miss_count = track.miss_count;

    snapshots.push_back(snapshot);
  }
  return snapshots;
}

std::vector<AssociationTrackSeed> TrackStateSnapshotEmitter::BuildAssociationSeeds() const {
  std::vector<AssociationTrackSeed> seeds;
  seeds.reserve(active_tracks_.size());
  for (std::vector<ActiveTrackEntry>::const_iterator it = active_tracks_.begin();
       it != active_tracks_.end(); ++it) {
    AssociationTrackSeed seed;
    seed.association_key = it->key;
    seed.has_position = true;
    seed.position = it->track->position;
    seed.has_gaussian_state = true;
    seed.gaussian_state = it->track->gaussian_state;
    seeds.push_back(seed);
  }
  return seeds;
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
