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

inline electro_optical_sensor::config::EosEnvironmentModelType EosModelFromString(
    const std::string& s) {
  if (s == "kSimplified")
    return electro_optical_sensor::config::EosEnvironmentModelType::kSimplified;
  if (s == "kAdvanced")
    return electro_optical_sensor::config::EosEnvironmentModelType::kAdvanced;
  return electro_optical_sensor::config::EosEnvironmentModelType::kSimplified;
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

inline electro_optical_sensor::config::RadiativeTransferModel
RadiativeModelFromString(const std::string& s) {
  using R =
      electro_optical_sensor::config::RadiativeTransferModel;
  if (s == "kDerivedBeerLambert") return R::kDerivedBeerLambert;
  if (s == "kHumidityWeighted") return R::kHumidityWeighted;
  if (s == "kAdaptivePathRadiance") return R::kAdaptivePathRadiance;
  return R::kDerivedBeerLambert;
}

}  // namespace examples

#endif  // EXAMPLES_EOS_CONFIG_LOADER_COMMON_H_
