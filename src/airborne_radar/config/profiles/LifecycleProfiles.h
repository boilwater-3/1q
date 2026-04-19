/**
 * @file LifecycleProfiles.h
 * @brief 定义生命周期域使用的 profile 枚举。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_PROFILES_LIFECYCLE_PROFILES_H_
#define AIRBORNE_RADAR_SRC_CONFIG_PROFILES_LIFECYCLE_PROFILES_H_

namespace airborne_radar {
namespace config {
namespace profiles {

enum class LifecyclePolicyProfile { kBalanced = 0, kFastConfirm = 1, kHighPersistence = 2 };

}  // namespace profiles
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_PROFILES_LIFECYCLE_PROFILES_H_
