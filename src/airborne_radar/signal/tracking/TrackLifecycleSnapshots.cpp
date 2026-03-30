#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

namespace {
/**
 * @brief 将内部轨迹状态映射为决策层轨迹状态。
 * @param status 内部轨迹状态。
 * @return 决策层可见的轨迹状态。
 */
common::model::DecisionTrackStatus ToDecisionTrackStatus(TrackStatus status) {
  switch (status) {
    case TrackStatus::kConfirmed:
      return common::model::DecisionTrackStatus::kConfirmed;
    case TrackStatus::kLost:
      return common::model::DecisionTrackStatus::kLost;
    case TrackStatus::kTentative:
    case TrackStatus::kRecycled:
    default:
      return common::model::DecisionTrackStatus::kTentative;
  }
}
}  // namespace

std::vector<const TrackState*> TrackLifecycleManager::GetActiveTracks() const {
  std::vector<const TrackState*> result;
  result.reserve(tracks_by_key_.size());

  for (std::unordered_map<std::uint64_t, TrackState*>::const_iterator it = tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    const TrackState* track = it->second;
    if (track->status != TrackStatus::kRecycled) {
      result.push_back(track);
    }
  }

  return result;
}

common::model::TargetFeatureList TrackLifecycleManager::BuildFeatureSnapshot() const {
  common::model::TargetFeatureList features;
  features.reserve(tracks_by_key_.size());
  ForEachActiveTrack([&features](std::uint64_t /*key*/, const TrackState& track) {
    common::model::TargetFeature feature(track.velocity(0), track.velocity(1), track.velocity(2),
                                         track.rcs);
    feature.has_cartesian_position = true;
    feature.position_x = track.position(0);
    feature.position_y = track.position(1);
    feature.position_z = track.position(2);
    feature.external_target_id = track.external_target_id;
    features.push_back(feature);
  });
  return features;
}

common::model::DecisionTrackSnapshotList TrackLifecycleManager::BuildDecisionSnapshot() const {
  common::model::DecisionTrackSnapshotList snapshots;
  snapshots.reserve(tracks_by_key_.size());
  ForEachActiveTrack([&](std::uint64_t key, const TrackState& track) {
    common::model::DecisionTrackSnapshot snapshot(
        track.velocity(0), track.velocity(1), track.velocity(2), track.rcs, track.acceleration(0),
        track.acceleration(1), track.acceleration(2), track.jamming_detected,
        track.external_target_id, key);
    snapshot.state.status = ToDecisionTrackStatus(track.status);
    snapshot.state.position_x = track.position(0);
    snapshot.state.position_y = track.position(1);
    snapshot.state.position_z = track.position(2);
    snapshot.state.hit_count = track.hit_count;
    snapshot.state.miss_count = track.miss_count;

    std::unordered_map<std::uint64_t, common::model::DecisionMeasurementEvidence>::const_iterator
        evidence_found = latest_evidence_by_key_.find(key);
    if (evidence_found != latest_evidence_by_key_.end()) {
      snapshot.evidence = evidence_found->second;
    } else {
      snapshot.evidence.has_measurement_evidence = false;
      snapshot.evidence.updated_this_cycle = false;
      snapshot.evidence.predicted_only_this_cycle = track.last_update_cycle != last_cycle_index_;
    }
    snapshots.push_back(snapshot);
  });
  return snapshots;
}

common::model::DecisionInputFrame TrackLifecycleManager::BuildDecisionFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id, bool environment_jamming_detected) const {
  common::model::DecisionInputFrame frame;
  frame.cycle_index = cycle_index;
  frame.batch_id = batch_id;
  frame.environment_jamming_detected = environment_jamming_detected;
  frame.tracks = BuildDecisionSnapshot();
  return frame;
}

std::vector<AssociationTrackSeed> TrackLifecycleManager::BuildAssociationSeeds() const {
  std::vector<AssociationTrackSeed> seeds;
  seeds.reserve(tracks_by_key_.size());
  ForEachActiveTrack([&seeds](std::uint64_t key, const TrackState& track) {
    AssociationTrackSeed seed;
    seed.association_key = key;
    seed.has_position = true;
    seed.position = track.position;
    seed.has_gaussian_state = true;
    seed.gaussian_state = track.gaussian_state;
    seeds.push_back(seed);
  });
  return seeds;
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
