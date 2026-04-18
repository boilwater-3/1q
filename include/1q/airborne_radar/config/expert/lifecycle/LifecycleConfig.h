/**
 * @file LifecycleConfig.h
 * @brief 定义 expert 生命周期配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_LIFECYCLE_LIFECYCLE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_LIFECYCLE_LIFECYCLE_CONFIG_H_

#include <cstdint>

namespace airborne_radar {
namespace config {
namespace expert {
namespace lifecycle {

/**
 * @brief expert 航迹生命周期参数。
 */
struct LifecycleConfig {
  std::uint32_t confirm_hits{3U}; /**< 航迹确认所需命中数。 */
  std::uint32_t max_miss_before_lost{2U}; /**< 丢失前最大连续失配数。 */
  std::uint32_t max_lost_cycles{5U}; /**< 允许保留的最大 lost 周期数。 */
  bool enable_imm_lifecycle{false}; /**< 是否启用 IMM 生命周期路径。 */
};

}  // namespace lifecycle
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_LIFECYCLE_LIFECYCLE_CONFIG_H_
