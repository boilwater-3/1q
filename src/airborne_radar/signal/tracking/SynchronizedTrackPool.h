/**
 * @file SynchronizedTrackPool.h
 * @brief SynchronizedTrackPool 为任意对象池提供全局互斥包装。
 */

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

  /// @brief 申请一个可写轨迹对象（线程安全）。
  /// @return 成功返回轨迹对象指针；失败返回 nullptr。
  common::TrackState *Acquire() override;

  /// @brief 归还轨迹对象到对象池（线程安全）。
  /// @param track 待归还对象，可为空指针。
  void Release(common::TrackState *track) override;

  /// @brief 返回对象池当前可见容量（线程安全）。
  /// @return 对象池总容量。
  std::size_t Capacity() const override;

  /// @brief 返回对象池当前在用对象数量（线程安全）。
  /// @return 当前在用对象总数。
  std::size_t InUseCount() const override;

private:
  ITrackPool &inner_;
  mutable std::mutex mutex_;
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_SYNCHRONIZED_TRACK_POOL_H_
