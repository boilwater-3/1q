/**
 * @file RirTrackPool.h
 * @brief RIR 航迹对象池（阶段 2-T N3）。
 *
 * 副本来源：`src/airborne_radar/signal/tracking/ITrackPool.h` /
 * `BoostTrackPool.*`（审计基线 96de367c）。RIR 仅有单一池实现，
 * 不保留接口抽象层；申请/归还/双重释放拒绝语义与 AR 逐行一致。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_POOL_H_
#define REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_POOL_H_

#include <boost/pool/object_pool.hpp>
#include <cstddef>
#include <unordered_set>
#include <vector>

#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tracking {

/**
 * @brief RirTrackPool 轻量航迹对象池（内存复用职责，不含状态机逻辑）。
 * @details 当前实现默认单写场景，不内建并发锁；空闲对象走自由表复用，
 *          在用指针集合守护重复归还。
 */
class RirTrackPool final {
 public:
  /**
   * @brief 构造对象池。
   * @param prewarm_count 预热对象数量。
   * @param max_cached_objects 空闲缓存上限（超过后归还即销毁）。
   */
  explicit RirTrackPool(std::size_t prewarm_count = 128, std::size_t max_cached_objects = 4096);

  /**
   * @brief 申请一个可写航迹对象。
   * @return 成功返回航迹对象指针；失败返回 nullptr。
   */
  RirTrackState* Acquire();

  /**
   * @brief 归还航迹对象到池；未知/重复释放被拒绝（记日志，对象不重入池）。
   * @param track 待归还对象，可为空指针。
   */
  void Release(RirTrackState* track);

  /** @brief 对象池当前可见容量估算（在用 + 空闲）。 */
  std::size_t Capacity() const;

  /** @brief 当前在用对象数量。 */
  std::size_t InUseCount() const { return in_use_count_; }

  /** @brief 当前空闲可复用对象数量。 */
  std::size_t FreeCount() const { return free_list_.size(); }

 private:
  boost::object_pool<RirTrackState> pool_;           /**< Boost 对象池，负责对象内存管理。 */
  std::vector<RirTrackState*> free_list_;            /**< 空闲对象列表，用于快速复用。 */
  std::unordered_set<RirTrackState*> in_use_tracks_; /**< 在用对象集合，防止重复归还。 */
  std::size_t max_cached_objects_{4096};             /**< 空闲缓存上限，超过后销毁对象。 */
  std::size_t in_use_count_{0};                      /**< 当前在用对象数量统计。 */
};

}  // namespace tracking
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_TRACK_POOL_H_
