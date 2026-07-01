/**
 * @file ArSessionConfigValidation.h
 * @brief AR module primary validation entry points for session configuration.
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfigValidation.h"

namespace airborne_radar {
namespace config {

inline ValidationIssueList ValidateArSessionConfig(const ArSessionConfig& config) noexcept {
  return ValidateRadarSessionConfig(config);
}

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
