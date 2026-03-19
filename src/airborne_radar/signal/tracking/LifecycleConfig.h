// Copyright 2026. All Rights Reserved.

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
/**
 * @brief 默认无锁模式，适用于当前单线程生命周期更新。
 */
  kSingleThreadNoLock = 0,
/**
 * @brief 使用全局互斥保护对象池接口，为未来多线程模式做准备。
 */
  kMultiThreadGlobalLock
};
/**
 * @brief ImmActivationPolicy 定义 IMM 多模型路径的激活策略。
 */
enum class ImmActivationPolicy {
/**
 * @brief 所有轨迹都按当前 IMM 语义创建和使用多模型路径。
 */
  kAllTracks = 0,
/**
 * @brief 仅已确认轨迹在再次命中时懒创建并启用 IMM。
 */
  kConfirmedTracksOnly
};
/**
 * @brief LifecycleConfig 定义轨迹状态机阈值配置。
 */
struct LifecycleConfig {
/**
 * @brief 候选轨迹转已确认所需最小命中次数。
 */
  std::uint32_t confirm_hits{3};
/**
 * @brief 已确认轨迹转丢失前允许的最大连续失配次数。
 */
  std::uint32_t max_miss_before_lost{2};
/**
 * @brief 丢失轨迹可保留的最大周期数，超出则回收。
 */
  std::uint32_t max_lost_cycles{5};
/**
 * @brief IMM 激活策略。
 */
  ImmActivationPolicy imm_activation_policy{
      ImmActivationPolicy::kConfirmedTracksOnly};
/**
 * @brief 对象池线程安全策略。
 */
  TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      TrackPoolThreadSafetyMode::kSingleThreadNoLock};
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_LIFECYCLE_CONFIG_H_
