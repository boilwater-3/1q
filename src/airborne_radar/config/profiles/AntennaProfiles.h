/**
 * @file AntennaProfiles.h
 * @brief 定义天线方向图域使用的 profile 枚举。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_PROFILES_ANTENNA_PROFILES_H_
#define AIRBORNE_RADAR_SRC_CONFIG_PROFILES_ANTENNA_PROFILES_H_

namespace airborne_radar {
namespace config {
namespace profiles {

enum class AntennaPatternProfile { kStandard = 0, kLowSidelobe = 1, kWideCoverage = 2 };

}  // namespace profiles
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_PROFILES_ANTENNA_PROFILES_H_
