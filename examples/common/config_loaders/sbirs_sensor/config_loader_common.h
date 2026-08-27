#ifndef EXAMPLES_SBIRS_CONFIG_LOADER_COMMON_H_
#define EXAMPLES_SBIRS_CONFIG_LOADER_COMMON_H_

#include <string>

#include "1q/sbirs_sensor/sbirs_sensor.hpp"
#include "json_reader.h"

namespace examples {

// -- enum helpers ------------------------------------------------------------

inline sbirs_sensor::config::SbirsWorkMode SbirsWorkModeFromString(
    const std::string& s) {
  if (s == "kStandby")
    return sbirs_sensor::config::SbirsWorkMode::kStandby;
  if (s == "kWideSearch")
    return sbirs_sensor::config::SbirsWorkMode::kWideSearch;
  if (s == "kSearchAndStare")
    return sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  return sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
}

inline sbirs_sensor::config::SbirsScanDirection SbirsScanDirectionFromString(
    const std::string& s) {
  if (s == "kIncreasingAzimuth")
    return sbirs_sensor::config::SbirsScanDirection::kIncreasingAzimuth;
  if (s == "kDecreasingAzimuth")
    return sbirs_sensor::config::SbirsScanDirection::kDecreasingAzimuth;
  return sbirs_sensor::config::SbirsScanDirection::kIncreasingAzimuth;
}

inline sbirs_sensor::config::SbirsTrackingMode SbirsTrackingModeFromString(
    const std::string& s) {
  if (s == "kEstimated")
    return sbirs_sensor::config::SbirsTrackingMode::kEstimated;
  if (s == "kStrictTruthAssisted")
    return sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;
  if (s == "kSensorLikeTruthAssisted")
    return sbirs_sensor::config::SbirsTrackingMode::kSensorLikeTruthAssisted;
  return sbirs_sensor::config::SbirsTrackingMode::kEstimated;
}

inline sbirs_sensor::config::SbirsEstimatedTrackingBackend
SbirsEstimatedTrackingBackendFromString(const std::string& s) {
  if (s == "kEkf")
    return sbirs_sensor::config::SbirsEstimatedTrackingBackend::kEkf;
  if (s == "kImm")
    return sbirs_sensor::config::SbirsEstimatedTrackingBackend::kImm;
  if (s == "kAngleCvKf")
    return sbirs_sensor::config::SbirsEstimatedTrackingBackend::kAngleCvKf;
  return sbirs_sensor::config::SbirsEstimatedTrackingBackend::kEkf;
}

inline sbirs_sensor::config::SbirsWeatherType SbirsWeatherTypeFromString(
    const std::string& s) {
  if (s == "kClear")
    return sbirs_sensor::config::SbirsWeatherType::kClear;
  if (s == "kCloudy")
    return sbirs_sensor::config::SbirsWeatherType::kCloudy;
  if (s == "kRain")
    return sbirs_sensor::config::SbirsWeatherType::kRain;
  if (s == "kFog")
    return sbirs_sensor::config::SbirsWeatherType::kFog;
  return sbirs_sensor::config::SbirsWeatherType::kClear;
}

inline sbirs_sensor::config::SbirsSeaState SbirsSeaStateFromString(
    const std::string& s) {
  if (s == "kLow")
    return sbirs_sensor::config::SbirsSeaState::kLow;
  if (s == "kMedium")
    return sbirs_sensor::config::SbirsSeaState::kMedium;
  if (s == "kHigh")
    return sbirs_sensor::config::SbirsSeaState::kHigh;
  return sbirs_sensor::config::SbirsSeaState::kLow;
}

}  // namespace examples

#endif  // EXAMPLES_SBIRS_CONFIG_LOADER_COMMON_H_
