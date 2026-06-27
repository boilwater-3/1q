/**
 * @file RadarSessionConfigValidation.h
 * @brief AR 会话配置校验工具。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

enum class ConfigValidationCode {
  kNone = 0,
  kCommandedBeamwidthAzNotPositive,
  kCommandedBeamwidthElNotPositive,
  kMechanicalScanLimitsSwappedAz,
  kMechanicalScanLimitsSwappedEl,
  kElectronicScanLimitsSwappedAz,
  kElectronicScanLimitsSwappedEl,
  kRobustTrackingWithoutImm
};

struct ConfigValidationIssue {
  ConfigValidationCode code{ConfigValidationCode::kNone};
  std::string field{};
  std::string message{};
};

using ValidationIssueList = std::vector<ConfigValidationIssue>;

ONEQ_API ValidationIssueList
ValidateRadarSessionConfig(const config::RadarSessionConfig& config) noexcept;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_VALIDATION_H_
