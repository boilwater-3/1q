/**
 * @file JammingThresholdUtils.h
 * @brief 干扰灵敏度档位与 dB 阈值的内部映射工具。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_JAMMING_THRESHOLD_UTILS_H_
#define AIRBORNE_RADAR_ENVIRONMENT_JAMMING_THRESHOLD_UTILS_H_

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"

namespace airborne_radar {
namespace environment {

using config::JammingSensitivityProfile;

inline float ResolveJammingDetectionThresholdDb(JammingSensitivityProfile profile) {
  switch (profile) {
    case JammingSensitivityProfile::kRelaxed:
      return 8.0f;
    case JammingSensitivityProfile::kStrict:
      return 4.0f;
    case JammingSensitivityProfile::kBalanced:
    default:
      return 6.0f;
  }
}

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_JAMMING_THRESHOLD_UTILS_H_
