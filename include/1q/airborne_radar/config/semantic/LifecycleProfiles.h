/**
 * @file LifecycleProfiles.h
 * @brief 定义 semantic 生命周期域使用的 profile 枚举。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SEMANTIC_LIFECYCLE_PROFILES_H_
#define AIRBORNE_RADAR_CONFIG_SEMANTIC_LIFECYCLE_PROFILES_H_

namespace airborne_radar {
namespace config {
namespace semantic {

/**
 * @brief 生命周期策略语义档位。
 */
enum class LifecyclePolicyProfile {
  kBalanced = 0, /**< 确认速度与保持能力折中。 */
  kFastConfirm = 1, /**< 优先快速确认目标。 */
  kHighPersistence = 2 /**< 优先保持弱目标或受干扰目标。 */
};

}  // namespace semantic
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SEMANTIC_LIFECYCLE_PROFILES_H_