/**
 * @file BoostTrackPool.h
 * @brief 基于 Boost object_pool 的轨迹对象池实现。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_BOOST_TRACK_POOL_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_BOOST_TRACK_POOL_H_

#include <cstddef>
#include <vector>

#include <boost/pool/object_pool.hpp>

#include "airborne_radar/signal/tracking/ITrackPool.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief BoostTrackPool 提供轻量对象复用能力。
 * 当前实现默认单写场景，不内建并发锁。
 */
class BoostTrackPool final : public ITrackPool {
public:
/**
 * @brief 构造函数。
 * @param prewarm_count 预热对象数量。
 * @param max_cached_objects 空闲缓存上限。
 */
  explicit BoostTrackPool(std::size_t prewarm_count = 128,
                          std::size_t max_cached_objects = 4096);

  ~BoostTrackPool() override = default;
/**
 * @brief 申请轨迹对象。
 * @return 可写轨迹对象指针；失败返回 nullptr。
 */
  common::TrackState *Acquire() override;
/**
 * @brief 归还轨迹对象。
 * @param track 待归还轨迹对象。
 */
  void Release(common::TrackState *track) override;
/**
 * @brief 获取对象池容量估算值（在用 + 空闲）。
 * @return 对象池总容量。
 */
  std::size_t Capacity() const override;
/**
 * @brief 获取当前在用对象数量。
 * @return 当前在用对象总数。
 */
  std::size_t InUseCount() const override;

private:
  boost::object_pool<common::TrackState> pool_;  /**< Boost 对象池，负责对象内存管理。 */
  std::vector<common::TrackState *> free_list_;  /**< 空闲对象列表，用于快速复用。 */
  std::size_t max_cached_objects_{4096};         /**< 空闲缓存上限，超过后将销毁对象。 */
  std::size_t in_use_count_{0};                  /**< 当前在用对象数量统计。 */
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_BOOST_TRACK_POOL_H_
