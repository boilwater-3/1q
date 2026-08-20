/**
 * @file ITrackPool.h
 * @brief 定义轨迹对象池抽象接口。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_I_TRACK_POOL_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_I_TRACK_POOL_H_

#include <cstddef>

#include "airborne_radar/signal/tracking/TrackState.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief ITrackPool 提供轨迹对象的申请与归还能力。
 * 该接口仅承担内存复用职责，不包含业务状态机逻辑。
 */
class ITrackPool {
 public:
  virtual ~ITrackPool() = default;
  /**
   * @brief 申请一个可写轨迹对象。
   * @return 成功返回轨迹对象指针；失败返回 nullptr。
   */
  virtual TrackState* Acquire() = 0;
  /**
   * @brief 归还轨迹对象到对象池。
   * @param track 待归还对象，可为空指针。
   */
  virtual void Release(TrackState* track) = 0;
  /**
   * @brief 返回对象池当前可见容量（在用 + 空闲）。
   * @return 对象池总容量。
   */
  virtual std::size_t Capacity() const = 0;
  /**
   * @brief 返回对象池当前在用对象数量。
   * @return 当前在用对象总数。
   */
  virtual std::size_t InUseCount() const = 0;
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_I_TRACK_POOL_H_
