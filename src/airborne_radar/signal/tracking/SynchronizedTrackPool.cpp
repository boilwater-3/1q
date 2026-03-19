// Copyright 2026. All Rights Reserved.
//
// @file SynchronizedTrackPool.cpp
// @brief 实现 SynchronizedTrackPool 的线程安全对象池逻辑。

#include "airborne_radar/signal/tracking/SynchronizedTrackPool.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

SynchronizedTrackPool::SynchronizedTrackPool(ITrackPool &inner) : inner_(inner) {}

common::TrackState *SynchronizedTrackPool::Acquire() {
  std::lock_guard<std::mutex> lock(mutex_);
  return inner_.Acquire();
}

void SynchronizedTrackPool::Release(common::TrackState *track) {
  std::lock_guard<std::mutex> lock(mutex_);
  inner_.Release(track);
}

std::size_t SynchronizedTrackPool::Capacity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return inner_.Capacity();
}

std::size_t SynchronizedTrackPool::InUseCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return inner_.InUseCount();
}

} // namespace tracking
} // namespace signal
} // namespace airborne_radar
