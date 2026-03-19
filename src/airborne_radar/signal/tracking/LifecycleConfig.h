/**
 * @file LifecycleConfig.h
 * @brief 定义轨迹生命周期配置类型。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_LIFECYCLE_CONFIG_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_LIFECYCLE_CONFIG_H_

#include <cstdint>

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief TrackPoolThreadSafetyMode 定义对象池线程安全策略。
 */
enum class TrackPoolThreadSafetyMode {
  kSingleThreadNoLock = 0,  /**< 默认无锁模式，适用于当前单线程生命周期更新。 */
  kMultiThreadGlobalLock  /**< 使用全局互斥保护对象池接口，为未来多线程模式做准备。 */
};
/**
 * @brief ImmActivationPolicy 定义 IMM 多模型路径的激活策略。
 */
enum class ImmActivationPolicy {
  kAllTracks = 0,  /**< 所有轨迹都按当前 IMM 语义创建和使用多模型路径。 */
  kConfirmedTracksOnly  /**< 仅已确认轨迹在再次命中时懒创建并启用 IMM。 */
};
/**
 * @brief LifecycleConfig 定义轨迹状态机阈值配置。
 */
struct LifecycleConfig {
  std::uint32_t confirm_hits{3};  /**< 候选轨迹转已确认所需最小命中次数。 */
  std::uint32_t max_miss_before_lost{2};  /**< 已确认轨迹转丢失前允许的最大连续失配次数。 */
  std::uint32_t max_lost_cycles{5};  /**< 丢失轨迹可保留的最大周期数，超出则回收。 */
  ImmActivationPolicy imm_activation_policy{
      ImmActivationPolicy::kConfirmedTracksOnly};  /**< IMM 激活策略。 */
  TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      TrackPoolThreadSafetyMode::kSingleThreadNoLock};  /**< 对象池线程安全策略。 */
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_LIFECYCLE_CONFIG_H_
