/**
 * @file TrackLifecyclePromote.h
 * @brief 航迹命中/丢失计数与状态晋升 FSM（不含 ECCM 判别逻辑本体）。
 */

#ifndef COMMON_TRACKING_TRACK_LIFECYCLE_PROMOTE_H_
#define COMMON_TRACKING_TRACK_LIFECYCLE_PROMOTE_H_

#include <cstdint>

namespace oneq {
namespace common {
namespace tracking {

/** @brief 生命周期相位（与 AR/RIR 状态枚举值域对齐的通用表示）。 */
enum class TrackLifecyclePhase : std::uint8_t {
  kTentative = 0,
  kConfirmed = 1,
  kLost = 2,
  kRecycled = 3
};

/** @brief 生命周期计数与相位（调用方映射到模块航迹对象）。 */
struct TrackLifecycleCounters {
  TrackLifecyclePhase status{TrackLifecyclePhase::kTentative};
  std::uint32_t hit_count{0U};
  std::uint32_t miss_count{0U};
  std::uint32_t last_update_cycle{0U};
  std::uint32_t generation{0U};
};

/** @brief 生命周期策略标量；suppress_confirm / recycle_as_status 为模块钩子。 */
struct TrackLifecyclePromotePolicy {
  std::uint32_t confirm_hits{3U};
  std::uint32_t max_miss_before_lost{3U};
  std::uint32_t max_lost_cycles{5U};
  std::uint32_t extra_miss_tolerance{0U};
  /** AR 反欺骗：命中周期抑制 tentative→confirmed。 */
  bool suppress_confirm{false};
  /** AR：丢失超时写 kRecycled 并 ++generation；RIR：仅返回 should_recycle。 */
  bool recycle_as_status{false};
};

/**
 * @brief 推进航迹生命周期计数/相位。
 * @return true 表示应回收（RIR erase；AR 在 recycle_as_status 时已写 kRecycled）。
 */
inline bool PromoteTrackLifecycle(TrackLifecycleCounters* counters, std::uint32_t cycle_index,
                                  bool hit_this_cycle,
                                  const TrackLifecyclePromotePolicy& policy) {
  if (counters == nullptr) {
    return false;
  }

  if (hit_this_cycle) {
    if (!policy.suppress_confirm &&
        (counters->status == TrackLifecyclePhase::kLost ||
         (counters->status == TrackLifecyclePhase::kTentative &&
          counters->hit_count >= policy.confirm_hits))) {
      counters->status = TrackLifecyclePhase::kConfirmed;
    }
    return false;
  }

  counters->miss_count += 1U;
  if (counters->status == TrackLifecyclePhase::kTentative ||
      counters->status == TrackLifecyclePhase::kConfirmed) {
    const std::uint32_t max_miss =
        policy.max_miss_before_lost + policy.extra_miss_tolerance;
    if (counters->miss_count > max_miss) {
      counters->status = TrackLifecyclePhase::kLost;
    }
    return false;
  }

  if (counters->status == TrackLifecyclePhase::kLost) {
    const std::uint32_t lost_cycles = cycle_index >= counters->last_update_cycle
                                          ? (cycle_index - counters->last_update_cycle)
                                          : 0U;
    if (lost_cycles > policy.max_lost_cycles) {
      if (policy.recycle_as_status) {
        counters->status = TrackLifecyclePhase::kRecycled;
        counters->generation += 1U;
      }
      return true;
    }
  }
  return false;
}

}  // namespace tracking
}  // namespace common
}  // namespace oneq

#endif  // COMMON_TRACKING_TRACK_LIFECYCLE_PROMOTE_H_
