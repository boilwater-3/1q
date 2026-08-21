#include "airborne_radar/signal/tracking/BoostTrackPool.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

BoostTrackPool::BoostTrackPool(std::size_t prewarm_count, std::size_t max_cached_objects)
    : pool_(prewarm_count, max_cached_objects, "BoostTrackPool") {}

TrackState* BoostTrackPool::Acquire() { return pool_.Acquire(); }

void BoostTrackPool::Release(TrackState* track) { pool_.Release(track); }

std::size_t BoostTrackPool::Capacity() const { return pool_.Capacity(); }

std::size_t BoostTrackPool::InUseCount() const { return pool_.InUseCount(); }

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
