/**
 * @file ArSessionConfigValidation.h
 * @brief AR module primary session config validation entry points.
 *
 * Primary header for session configuration validation.
 * Include this for new code; RadarSessionConfigValidation.h is the deprecated compat wrapper.
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/airborne_radar/config/ArSessionConfig.h"
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
ValidateArSessionConfig(const config::ArSessionConfig& config) noexcept;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
