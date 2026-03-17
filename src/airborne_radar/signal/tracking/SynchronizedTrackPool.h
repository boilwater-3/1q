// Copyright 2026. All Rights Reserved.
//
// Description: SynchronizedTrackPool 为任意对象池提供全局互斥包装。

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_SYNCHRONIZED_TRACK_POOL_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_SYNCHRONIZED_TRACK_POOL_H_

#include <mutex>

#include "airborne_radar/signal/tracking/ITrackPool.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/// @brief SynchronizedTrackPool 为任意对象池提供全局互斥包装。
class SynchronizedTrackPool : public ITrackPool {
public:
  /// @brief 构造函数。
  /// @param inner 被包装的底层对象池。
  explicit SynchronizedTrackPool(ITrackPool &inner);

  ~SynchronizedTrackPool() override = default;

  common::TrackState *Acquire() override;
  void Release(common::TrackState *track) override;
  std::size_t Capacity() const override;
  std::size_t InUseCount() const override;

private:
  ITrackPool &inner_;
  mutable std::mutex mutex_;
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_SYNCHRONIZED_TRACK_POOL_H_
