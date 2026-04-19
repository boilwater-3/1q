/**
 * @file DetectionProfiles.h
 * @brief 定义探测域使用的 profile 枚举。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_PROFILES_DETECTION_PROFILES_H_
#define AIRBORNE_RADAR_SRC_CONFIG_PROFILES_DETECTION_PROFILES_H_

namespace airborne_radar {
namespace config {
namespace profiles {

enum class SwerlingModel {
  kSwerling0 = 0,
  kSwerling1 = 1,
  kSwerling2 = 2,
  kSwerling3 = 3,
  kSwerling4 = 4
};

enum class RadarHardwareProfile {
  kGenericAirborneXBand = 0,
  kLongRangeHighPower = 1,
  kLightweightLpi = 2
};

enum class DetectionIntentProfile {
  kBalanced = 0,
  kDetectionPriority = 1,
  kTrackStabilityPriority = 2
};

enum class RcsFusionProfile { kDisabled = 0, kConservative = 1, kEnhanced = 2 };

}  // namespace profiles
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_PROFILES_DETECTION_PROFILES_H_
