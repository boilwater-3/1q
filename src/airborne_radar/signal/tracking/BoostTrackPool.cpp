#include "airborne_radar/signal/tracking/BoostTrackPool.h"

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

BoostTrackPool::BoostTrackPool(std::size_t prewarm_count, std::size_t max_cached_objects)
    : pool_(), free_list_(), max_cached_objects_(max_cached_objects) {
  free_list_.reserve(prewarm_count);
  for (std::size_t i = 0; i < prewarm_count; ++i) {
    TrackState* track = pool_.construct();
    if (track == nullptr) {
      PROJECT_LOG_ERROR(
          "[BoostTrackPool] prewarm construct failed: prepared={} target_prewarm={} in_use={} "
          "cached={}",
          free_list_.size(), prewarm_count, in_use_count_, free_list_.size());
      break;
    }
    free_list_.push_back(track);
  }
}

TrackState* BoostTrackPool::Acquire() {
  TrackState* track = nullptr;
  if (!free_list_.empty()) {
    track = free_list_.back();
    free_list_.pop_back();
  } else {
    track = pool_.construct();
    if (track == nullptr) {
      PROJECT_LOG_ERROR(
          "[BoostTrackPool] acquire construct failed: in_use={} cached={} capacity={}",
          in_use_count_, free_list_.size(), Capacity());
    }
  }

  if (track != nullptr) {
    const std::pair<std::unordered_set<TrackState*>::iterator, bool> inserted =
        in_use_tracks_.insert(track);
    if (!inserted.second) {
      PROJECT_LOG_ERROR("[BoostTrackPool] Acquire detected duplicate in-use pointer: {}",
                        static_cast<void*>(track));
      return nullptr;
    }
    ++in_use_count_;
  }
  return track;
}

void BoostTrackPool::Release(TrackState* track) {
  if (track == nullptr) {
    return;
  }

  if (in_use_tracks_.erase(track) == 0U) {
    PROJECT_LOG_ERROR("[BoostTrackPool] Release rejected unknown or double-released pointer: {}",
                      static_cast<void*>(track));
    return;
  }

  if (in_use_count_ == 0) {
    PROJECT_LOG_ERROR(
        "[BoostTrackPool] Release called with in_use_count_=0: "
        "internal state mismatch");
    in_use_tracks_.insert(track);
    return;
  }
  --in_use_count_;

  if (free_list_.size() < max_cached_objects_) {
    free_list_.push_back(track);
    return;
  }

  // 超过空闲缓存上限时执行显式析构，避免空闲列表无限膨胀。
  pool_.destroy(track);
}

std::size_t BoostTrackPool::Capacity() const { return in_use_count_ + free_list_.size(); }

std::size_t BoostTrackPool::InUseCount() const { return in_use_count_; }

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
