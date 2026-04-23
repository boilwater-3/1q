#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

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

model::TargetFeatureList TrackLifecycleManager::BuildFeatureSnapshot() const {
  return snapshot_emitter_.BuildFeatureSnapshot();
}

model::TrackStateSnapshotList TrackLifecycleManager::BuildTrackStateSnapshots() const {
  return snapshot_emitter_.BuildTrackStateSnapshots();
}

model::DecisionInputFrame TrackLifecycleManager::BuildDecisionFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id, bool environment_jamming_detected) const {
  return snapshot_emitter_.BuildDecisionFrame(cycle_index, batch_id, environment_jamming_detected);
}

std::vector<AssociationTrackSeed> TrackLifecycleManager::BuildAssociationSeeds() const {
  return snapshot_emitter_.BuildAssociationSeeds();
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
