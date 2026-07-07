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

/**
 * @brief 将干扰灵敏度语义档位解析为对应的干扰检测 dB 阈值。
 *
 * 档位越宽松（kRelaxed）阈值越高（8 dB），越不易触发干扰判定；
 * 档位越严格（kStrict）阈值越低（4 dB），越敏感。
 * @param[in] profile 干扰灵敏度语义档位。
 * @return 对应的干扰检测阈值（单位：dB），未知值按 kBalanced 处理。
 */
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
