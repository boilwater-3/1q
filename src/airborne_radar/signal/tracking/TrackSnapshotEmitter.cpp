/**
 * @file TrackSnapshotEmitter.cpp
 * @brief 轨迹快照导出器实现。
 */

#include "airborne_radar/signal/tracking/TrackSnapshotEmitter.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

namespace {

/**
 * @brief 将内部轨迹状态映射为决策层轨迹状态。
 * @param status 内部轨迹状态。
 * @return 决策层可见的轨迹状态。
 */
model::DecisionTrackStatus ToDecisionTrackStatus(TrackStatus status) {
  switch (status) {
    case TrackStatus::kConfirmed:
      return model::DecisionTrackStatus::kConfirmed;
    case TrackStatus::kLost:
      return model::DecisionTrackStatus::kLost;
    case TrackStatus::kTentative:
    case TrackStatus::kRecycled:
    default:
      return model::DecisionTrackStatus::kTentative;
  }
}

}  // namespace

void TrackSnapshotEmitter::Refresh(
    const std::unordered_map<std::uint64_t, TrackState*>& tracks_by_key,
    const std::unordered_map<std::uint64_t, model::DecisionMeasurementEvidence>&
        evidence_by_key,
    std::uint32_t last_cycle_index) {
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
  evidence_by_key_ = evidence_by_key;
  last_cycle_index_ = last_cycle_index;
}

model::TargetFeatureList TrackSnapshotEmitter::BuildFeatureSnapshot() const {
  model::TargetFeatureList features;
  features.reserve(active_tracks_.size());
  for (std::vector<ActiveTrackEntry>::const_iterator it = active_tracks_.begin();
       it != active_tracks_.end(); ++it) {
    const TrackState& track = *it->track;
    model::TargetFeature feature(track.velocity(0), track.velocity(1), track.velocity(2),
                                         track.rcs);
    feature.has_cartesian_position = true;
    feature.position_x = track.position(0);
    feature.position_y = track.position(1);
    feature.position_z = track.position(2);
    feature.external_target_id = track.external_target_id;
    features.push_back(feature);
  }
  return features;
}

model::DecisionTrackSnapshotList TrackSnapshotEmitter::BuildDecisionSnapshot() const {
  model::DecisionTrackSnapshotList snapshots;
  snapshots.reserve(active_tracks_.size());
  for (std::vector<ActiveTrackEntry>::const_iterator it = active_tracks_.begin();
       it != active_tracks_.end(); ++it) {
    const std::uint64_t key = it->key;
    const TrackState& track = *it->track;
    model::DecisionTrackSnapshot snapshot(
        track.velocity(0), track.velocity(1), track.velocity(2), track.rcs, track.acceleration(0),
        track.acceleration(1), track.acceleration(2), track.jamming_detected,
        track.external_target_id, key);
    snapshot.state.status = ToDecisionTrackStatus(track.status);
    snapshot.state.position_x = track.position(0);
    snapshot.state.position_y = track.position(1);
    snapshot.state.position_z = track.position(2);
    snapshot.state.hit_count = track.hit_count;
    snapshot.state.miss_count = track.miss_count;

    std::unordered_map<std::uint64_t, model::DecisionMeasurementEvidence>::const_iterator
        evidence_found = evidence_by_key_.find(key);
    if (evidence_found != evidence_by_key_.end()) {
      snapshot.evidence = evidence_found->second;
    } else {
      snapshot.evidence.has_measurement_evidence = false;
      snapshot.evidence.updated_this_cycle = false;
      snapshot.evidence.predicted_only_this_cycle = track.last_update_cycle != last_cycle_index_;
    }
    snapshots.push_back(snapshot);
  }
  return snapshots;
}

model::DecisionInputFrame TrackSnapshotEmitter::BuildDecisionFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id, bool environment_jamming_detected) const {
  model::DecisionInputFrame frame;
  frame.cycle_index = cycle_index;
  frame.batch_id = batch_id;
  frame.environment_jamming_detected = environment_jamming_detected;
  frame.tracks = BuildDecisionSnapshot();
  return frame;
}

std::vector<AssociationTrackSeed> TrackSnapshotEmitter::BuildAssociationSeeds() const {
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
