// Copyright 2026. All Rights Reserved.
//
// Description: SynchronizedTrackPool 的实现。

#include "airborne_radar/signal/tracking/SynchronizedTrackPool.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/// @brief 构造线程安全对象池包装器。
/// @param inner 被包装的底层对象池实现。
SynchronizedTrackPool::SynchronizedTrackPool(ITrackPool &inner) : inner_(inner) {}

/// @brief 线程安全地申请轨迹对象。
/// @return 成功返回对象指针；失败返回 nullptr。
common::TrackState *SynchronizedTrackPool::Acquire() {
  std::lock_guard<std::mutex> lock(mutex_);
  return inner_.Acquire();
}

/// @brief 线程安全地归还轨迹对象。
/// @param track 待归还对象，可为空。
void SynchronizedTrackPool::Release(common::TrackState *track) {
  std::lock_guard<std::mutex> lock(mutex_);
  inner_.Release(track);
}

/// @brief 线程安全地读取对象池容量。
/// @return 对象池总容量。
std::size_t SynchronizedTrackPool::Capacity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return inner_.Capacity();
}

/// @brief 线程安全地读取在用对象数。
/// @return 当前在用对象总数。
std::size_t SynchronizedTrackPool::InUseCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return inner_.InUseCount();
}

} // namespace tracking
} // namespace signal
} // namespace airborne_radar
