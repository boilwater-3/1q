/**
 * @file SignalLifecycleConfig.h
 * @brief 定义生命周期域关键配置（config 主入口）。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SIGNAL_LIFECYCLE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SIGNAL_LIFECYCLE_CONFIG_H_

#include <cstdint>

namespace airborne_radar {
namespace config {

/**
 * @brief LifecycleConfig 定义轨迹状态机阈值配置。
 */
struct LifecycleConfig {
  std::uint32_t confirm_hits{3};         /**< 候选轨迹转已确认所需最小命中次数 */
  std::uint32_t max_miss_before_lost{2}; /**< 已确认轨迹转丢失前允许的最大连续失配次数 */
  std::uint32_t max_lost_cycles{5};      /**< 丢失轨迹可保留的最大周期数，超出则回收 */
};

/**
 * @brief SignalLifecycleConfig 描述生命周期域配置。
 */
struct SignalLifecycleConfig {
  bool enable_auto_lifecycle_manager{false}; /**< 是否启用自动生命周期管理 */
  LifecycleConfig lifecycle_config{};        /**< 生命周期配置 */
  bool enable_imm_lifecycle{false};          /**< 是否启用 IMM 生命周期管理 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_LIFECYCLE_CONFIG_H_
