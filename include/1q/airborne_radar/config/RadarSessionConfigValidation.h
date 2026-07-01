/**
 * @file RadarSessionConfigValidation.h
 * @brief Deprecated compat wrapper — include ArSessionConfigValidation.h instead.
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_VALIDATION_H_

#include "1q/airborne_radar/config/ArSessionConfigValidation.h"

namespace airborne_radar {
namespace config {

inline ValidationIssueList ValidateRadarSessionConfig(const ArSessionConfig& config) noexcept {
  return ValidateArSessionConfig(config);
}

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_VALIDATION_H_
