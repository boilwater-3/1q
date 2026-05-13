#ifndef EXAMPLES_EOS_CONFIG_LOADER_COMMON_H_
#define EXAMPLES_EOS_CONFIG_LOADER_COMMON_H_

#include <string>

#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/foundation/json_reader.h"

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

inline electro_optical_sensor::config::EosDetectionProfile EosDetectFromString(
    const std::string& s) {
  if (s == "kConservative")
    return electro_optical_sensor::config::EosDetectionProfile::kConservative;
  if (s == "kBalanced")
    return electro_optical_sensor::config::EosDetectionProfile::kBalanced;
  if (s == "kAggressive")
    return electro_optical_sensor::config::EosDetectionProfile::kAggressive;
  return electro_optical_sensor::config::EosDetectionProfile::kBalanced;
}

inline electro_optical_sensor::config::EosStrayLightProfile EosStrayFromString(
    const std::string& s) {
  if (s == "kDisabled")
    return electro_optical_sensor::config::EosStrayLightProfile::kDisabled;
  if (s == "kStandardHood")
    return electro_optical_sensor::config::EosStrayLightProfile::kStandardHood;
  if (s == "kEnhancedHood")
    return electro_optical_sensor::config::EosStrayLightProfile::kEnhancedHood;
  return electro_optical_sensor::config::EosStrayLightProfile::kDisabled;
}

inline electro_optical_sensor::environment::EosEnvironmentModelType EosModelFromString(
    const std::string& s) {
  if (s == "kSimplified")
    return electro_optical_sensor::environment::EosEnvironmentModelType::kSimplified;
  if (s == "kAdvanced")
    return electro_optical_sensor::environment::EosEnvironmentModelType::kAdvanced;
  return electro_optical_sensor::environment::EosEnvironmentModelType::kSimplified;
}

inline electro_optical_sensor::environment::EosEnvironmentPreset EosPresetFromString(
    const std::string& s) {
  if (s == "kStandard")
    return electro_optical_sensor::environment::EosEnvironmentPreset::kStandard;
  if (s == "kHumid")
    return electro_optical_sensor::environment::EosEnvironmentPreset::kHumid;
  if (s == "kDusty")
    return electro_optical_sensor::environment::EosEnvironmentPreset::kDusty;
  if (s == "kTurbulent")
    return electro_optical_sensor::environment::EosEnvironmentPreset::kTurbulent;
  if (s == "kMaritime")
    return electro_optical_sensor::environment::EosEnvironmentPreset::kMaritime;
  return electro_optical_sensor::environment::EosEnvironmentPreset::kStandard;
}

inline electro_optical_sensor::foundation::radiative_transfer::RadiativeTransferModel
RadiativeModelFromString(const std::string& s) {
  using R =
      electro_optical_sensor::foundation::radiative_transfer::RadiativeTransferModel;
  if (s == "kDerivedBeerLambert") return R::kDerivedBeerLambert;
  if (s == "kHumidityWeighted") return R::kHumidityWeighted;
  if (s == "kAdaptivePathRadiance") return R::kAdaptivePathRadiance;
  return R::kDerivedBeerLambert;
}

}  // namespace examples

#endif  // EXAMPLES_EOS_CONFIG_LOADER_COMMON_H_
