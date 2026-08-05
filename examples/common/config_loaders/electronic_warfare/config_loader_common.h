#ifndef EXAMPLES_ESR_CONFIG_LOADER_COMMON_H_
#define EXAMPLES_ESR_CONFIG_LOADER_COMMON_H_

#include <string>

#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "json_reader.h"

namespace examples {

// -- namespace aliases -------------------------------------------------------
namespace esr_cfg = electronic_surveillance_radar::config;

// -- enum helpers ------------------------------------------------------------

inline esr_cfg::EsrWorkMode EsrWorkModeFromString(const std::string& s) {
  if (s == "kEsm") return esr_cfg::EsrWorkMode::kEsm;
  if (s == "kHgesm") return esr_cfg::EsrWorkMode::kHgesm;
  if (s == "kRwr") return esr_cfg::EsrWorkMode::kRwr;
  return esr_cfg::EsrWorkMode::kEsm;
}

inline esr_cfg::EsrEnvironmentPreset EsrPresetFromString(const std::string& s) {
  if (s == "kStandard") return esr_cfg::EsrEnvironmentPreset::kStandard;
  if (s == "kLowClutter") return esr_cfg::EsrEnvironmentPreset::kLowClutter;
  if (s == "kDenseClutter") return esr_cfg::EsrEnvironmentPreset::kDenseClutter;
  if (s == "kJammed") return esr_cfg::EsrEnvironmentPreset::kJammed;
  return esr_cfg::EsrEnvironmentPreset::kStandard;
}

}  // namespace examples

#endif  // EXAMPLES_ESR_CONFIG_LOADER_COMMON_H_
