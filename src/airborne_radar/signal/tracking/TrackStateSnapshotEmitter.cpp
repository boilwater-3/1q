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
model::TrackStatus ToTrackStatus(TrackStatus status) {
  switch (status) {
    case TrackStatus::kConfirmed:
      return model::TrackStatus::kConfirmed;
    case TrackStatus::kLost:
      return model::TrackStatus::kLost;
    case TrackStatus::kTentative:
    case TrackStatus::kRecycled:
    default:
      return model::TrackStatus::kTentative;
  }
}

}  // namespace

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

session::RadarSceneTargetList TrackStateSnapshotEmitter::BuildSceneTargetSnapshot() const {
  session::RadarSceneTargetList features;
  features.reserve(active_tracks_.size());
  for (std::vector<ActiveTrackEntry>::const_iterator it = active_tracks_.begin();
       it != active_tracks_.end(); ++it) {
    const TrackState& track = *it->track;
    session::RadarSceneTarget feature(track.velocity(0), track.velocity(1), track.velocity(2),
                                      track.rcs);
    feature.position_x = track.position(0);
    feature.position_y = track.position(1);
    feature.position_z = track.position(2);
    feature.external_target_id = track.external_target_id;
    features.push_back(feature);
  }
  return features;
}

model::TrackStateSnapshotList TrackStateSnapshotEmitter::BuildTrackStateSnapshots() const {
  model::TrackStateSnapshotList snapshots;
  snapshots.reserve(active_tracks_.size());
  for (std::vector<ActiveTrackEntry>::const_iterator it = active_tracks_.begin();
       it != active_tracks_.end(); ++it) {
    const std::uint64_t key = it->key;
    const TrackState& track = *it->track;
    model::TrackStateSnapshot snapshot;
    snapshot.association_key = key;
    snapshot.external_target_id = track.external_target_id;
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

model::DecisionInputFrame TrackStateSnapshotEmitter::BuildDecisionFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id, bool environment_jamming_detected) const {
  model::DecisionInputFrame frame;
  frame.cycle_index = cycle_index;
  frame.batch_id = batch_id;
  frame.environment_jamming_detected = environment_jamming_detected;
  frame.tracks = BuildTrackStateSnapshots();
  return frame;
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
