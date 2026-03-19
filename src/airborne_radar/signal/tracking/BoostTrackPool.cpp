#include "airborne_radar/signal/tracking/BoostTrackPool.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

BoostTrackPool::BoostTrackPool(std::size_t prewarm_count,
                               std::size_t max_cached_objects)
    : pool_(),
      free_list_(),
      max_cached_objects_(max_cached_objects),
      in_use_count_(0) {
  free_list_.reserve(prewarm_count);
  for (std::size_t i = 0; i < prewarm_count; ++i) {
    common::TrackState *track = pool_.construct();
    if (track == nullptr) {
      break;
    }
    free_list_.push_back(track);
  }
}

common::TrackState *BoostTrackPool::Acquire() {
  common::TrackState *track = nullptr;
  if (!free_list_.empty()) {
    track = free_list_.back();
    free_list_.pop_back();
  } else {
    track = pool_.construct();
  }

  if (track != nullptr) {
    ++in_use_count_;
  }
  return track;
}

void BoostTrackPool::Release(common::TrackState *track) {
  if (track == nullptr) {
    return;
  }

  if (in_use_count_ > 0) {
    --in_use_count_;
  }

  if (free_list_.size() < max_cached_objects_) {
    free_list_.push_back(track);
    return;
  }

  // 超过空闲缓存上限时执行显式析构，避免空闲列表无限膨胀。
  pool_.destroy(track);
}

std::size_t BoostTrackPool::Capacity() const {
  return in_use_count_ + free_list_.size();
}

std::size_t BoostTrackPool::InUseCount() const {
  return in_use_count_;
}

} // namespace tracking
} // namespace signal
} // namespace airborne_radar
