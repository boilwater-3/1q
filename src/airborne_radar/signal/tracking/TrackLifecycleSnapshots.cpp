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

session::RadarSceneTargetList TrackLifecycleManager::BuildSceneTargetSnapshot() const {
  return snapshot_emitter_.BuildSceneTargetSnapshot();
}

model::TrackStateSnapshotList TrackLifecycleManager::BuildTrackStateSnapshots() const {
  return snapshot_emitter_.BuildTrackStateSnapshots();
}

std::vector<AssociationTrackSeed> TrackLifecycleManager::BuildAssociationSeeds() const {
  return snapshot_emitter_.BuildAssociationSeeds();
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
