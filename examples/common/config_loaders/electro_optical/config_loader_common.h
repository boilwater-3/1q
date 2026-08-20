#ifndef EXAMPLES_EOS_CONFIG_LOADER_COMMON_H_
#define EXAMPLES_EOS_CONFIG_LOADER_COMMON_H_

#include <string>

#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "json_reader.h"

namespace examples {

// -- enum helpers ------------------------------------------------------------

inline electro_optical_sensor::config::EosWorkMode EosWorkModeFromString(
    const std::string& s) {
  if (s == "kInfraredOnly")
    return electro_optical_sensor::config::EosWorkMode::kInfraredOnly;
  if (s == "kVisibleOnly")
    return electro_optical_sensor::config::EosWorkMode::kVisibleOnly;
  if (s == "kFused")
    return electro_optical_sensor::config::EosWorkMode::kFused;
  return electro_optical_sensor::config::EosWorkMode::kFused;
}

inline electro_optical_sensor::config::EosEnvironmentPreset EosPresetFromString(
    const std::string& s) {
  if (s == "kStandard")
    return electro_optical_sensor::config::EosEnvironmentPreset::kStandard;
  if (s == "kHumid")
    return electro_optical_sensor::config::EosEnvironmentPreset::kHumid;
  if (s == "kDusty")
    return electro_optical_sensor::config::EosEnvironmentPreset::kDusty;
  if (s == "kTurbulent")
    return electro_optical_sensor::config::EosEnvironmentPreset::kTurbulent;
  if (s == "kMaritime")
    return electro_optical_sensor::config::EosEnvironmentPreset::kMaritime;
  return electro_optical_sensor::config::EosEnvironmentPreset::kStandard;
}

}  // namespace examples

#endif  // EXAMPLES_EOS_CONFIG_LOADER_COMMON_H_
