/**
 * @file TrackingProfiles.h
 * @brief 定义 semantic 跟踪域使用的 profile 枚举。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SEMANTIC_TRACKING_PROFILES_H_
#define AIRBORNE_RADAR_CONFIG_SEMANTIC_TRACKING_PROFILES_H_

namespace airborne_radar {
namespace config {
namespace semantic {

/**
 * @brief 跟踪策略语义档位。
 */
enum class TrackingPolicyProfile {
  kBalanced = 0, /**< 关联速度与鲁棒性折中。 */
  kFastAssociation = 1, /**< 优先快速完成量测关联。 */
  kRobustAntiJamming = 2 /**< 优先在干扰环境下保持稳健。 */
};

}  // namespace semantic
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SEMANTIC_TRACKING_PROFILES_H_