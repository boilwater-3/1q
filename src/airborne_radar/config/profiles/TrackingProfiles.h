/**
 * @file TrackingProfiles.h
 * @brief 定义跟踪域使用的 profile 枚举。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_PROFILES_TRACKING_PROFILES_H_
#define AIRBORNE_RADAR_SRC_CONFIG_PROFILES_TRACKING_PROFILES_H_

namespace airborne_radar {
namespace config {
namespace profiles {

enum class TrackingPolicyProfile { kBalanced = 0, kFastAssociation = 1, kRobustAntiJamming = 2 };

}  // namespace profiles
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_PROFILES_TRACKING_PROFILES_H_
