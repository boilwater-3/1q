/**
 * @file BoostTrackPool.h
 * @brief 基于 common ObjectPool 的轨迹对象池实现。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_BOOST_TRACK_POOL_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_BOOST_TRACK_POOL_H_

#include <cstddef>

#include "airborne_radar/signal/tracking/ITrackPool.h"
#include "common/tracking/ObjectPool.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief BoostTrackPool 提供轻量对象复用能力（common ObjectPool 薄适配）。
 */
class BoostTrackPool final : public ITrackPool {
 public:
  explicit BoostTrackPool(std::size_t prewarm_count = 128, std::size_t max_cached_objects = 4096);
  ~BoostTrackPool() override = default;

  TrackState* Acquire() override;
  void Release(TrackState* track) override;
  std::size_t Capacity() const override;
  std::size_t InUseCount() const override;

 private:
  oneq::common::tracking::ObjectPool<TrackState> pool_;
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_BOOST_TRACK_POOL_H_
