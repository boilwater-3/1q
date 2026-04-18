/**
 * @file TrackingConfig.h
 * @brief 定义 semantic 跟踪配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SEMANTIC_TRACKING_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SEMANTIC_TRACKING_CONFIG_H_

#include "1q/airborne_radar/config/semantic/profiles/TrackingProfiles.h"

namespace airborne_radar {
namespace config {
namespace semantic {

/**
 * @brief semantic 跟踪配置。
 */
struct TrackingConfig {
  bool enable_tracking_filter{true}; /**< 是否启用公开跟踪滤波链路。 */
  TrackingPolicyProfile policy_profile{TrackingPolicyProfile::kBalanced}; /**< 跟踪策略语义档位。 */
};

}  // namespace semantic
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SEMANTIC_TRACKING_CONFIG_H_
