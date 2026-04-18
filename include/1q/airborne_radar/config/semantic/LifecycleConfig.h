/**
 * @file LifecycleConfig.h
 * @brief 定义 semantic 航迹生命周期配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SEMANTIC_LIFECYCLE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SEMANTIC_LIFECYCLE_CONFIG_H_

#include "1q/airborne_radar/config/semantic/profiles/LifecycleProfiles.h"

namespace airborne_radar {
namespace config {
namespace semantic {

/**
 * @brief semantic 航迹生命周期配置。
 */
struct LifecycleConfig {
  LifecyclePolicyProfile policy_profile{LifecyclePolicyProfile::kBalanced}; /**< 生命周期策略语义档位。 */
  bool enable_imm_fusion{false}; /**< 是否启用 IMM 融合路径。 */
};

}  // namespace semantic
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SEMANTIC_LIFECYCLE_CONFIG_H_
