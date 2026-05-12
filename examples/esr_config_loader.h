#ifndef EXAMPLES_ESR_CONFIG_LOADER_H_
#define EXAMPLES_ESR_CONFIG_LOADER_H_

#include <string>

#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/foundation/json_reader.h"

namespace examples {

// -- namespace aliases -------------------------------------------------------
namespace esr_cfg = electronic_surveillance_radar::config;
namespace esr_env = electronic_surveillance_radar::environment;

// -- enum helpers ------------------------------------------------------------

inline esr_cfg::EsrWorkMode EsrWorkModeFromString(const std::string& s) {
  if (s == "kEsm") return esr_cfg::EsrWorkMode::kEsm;
  if (s == "kHgesm") return esr_cfg::EsrWorkMode::kHgesm;
  if (s == "kRwr") return esr_cfg::EsrWorkMode::kRwr;
  return esr_cfg::EsrWorkMode::kEsm;
}

inline esr_cfg::EsrDetectionProfile EsrDetectFromString(const std::string& s) {
  if (s == "kConservative") return esr_cfg::EsrDetectionProfile::kConservative;
  if (s == "kBalanced") return esr_cfg::EsrDetectionProfile::kBalanced;
  if (s == "kSensitive") return esr_cfg::EsrDetectionProfile::kSensitive;
  return esr_cfg::EsrDetectionProfile::kBalanced;
}

inline esr_cfg::EsrEnvironmentPreset EsrPresetFromString(const std::string& s) {
  if (s == "kStandard") return esr_cfg::EsrEnvironmentPreset::kStandard;
  if (s == "kLowClutter") return esr_cfg::EsrEnvironmentPreset::kLowClutter;
  if (s == "kDenseClutter") return esr_cfg::EsrEnvironmentPreset::kDenseClutter;
  if (s == "kJammed") return esr_cfg::EsrEnvironmentPreset::kJammed;
  return esr_cfg::EsrEnvironmentPreset::kStandard;
}

// -- struct loaders ----------------------------------------------------------

inline void LoadEsrHardware(const oneq::JsonValue& j,
                            esr_cfg::EsrHardwareConfig* v) {
  if (j.IsNull()) return;
  v->receiver_band_lower_hz = j["receiver_band_lower_hz"].AsDouble();
  v->receiver_band_upper_hz = j["receiver_band_upper_hz"].AsDouble();
  v->receiver_sensitivity_w = static_cast<float>(j["receiver_sensitivity_w"].AsDouble());
  v->integrated_receive_loss_db =
      static_cast<float>(j["integrated_receive_loss_db"].AsDouble());
  v->beam_az_width_deg = static_cast<float>(j["beam_az_width_deg"].AsDouble());
  v->beam_el_width_deg = static_cast<float>(j["beam_el_width_deg"].AsDouble());
  v->az_scan_range_deg = static_cast<float>(j["az_scan_range_deg"].AsDouble());
  v->el_scan_range_deg = static_cast<float>(j["el_scan_range_deg"].AsDouble());
  v->antenna_mount_az_deg = static_cast<float>(j["antenna_mount_az_deg"].AsDouble());
  v->antenna_mount_el_deg = static_cast<float>(j["antenna_mount_el_deg"].AsDouble());
}

inline void LoadEsrScanPolicy(const oneq::JsonValue& j,
                              esr_cfg::EsrScanPolicyConfig* v) {
  if (j.IsNull()) return;
  v->scan_center_az_deg = static_cast<float>(j["scan_center_az_deg"].AsDouble());
  v->scan_center_el_deg = static_cast<float>(j["scan_center_el_deg"].AsDouble());
  v->scan_rate_hz = static_cast<float>(j["scan_rate_hz"].AsDouble());
  v->use_explicit_scan_bounds = j["use_explicit_scan_bounds"].AsBool();
  v->scan_start_az_deg = static_cast<float>(j["scan_start_az_deg"].AsDouble());
  v->scan_end_az_deg = static_cast<float>(j["scan_end_az_deg"].AsDouble());
  v->scan_start_el_deg = static_cast<float>(j["scan_start_el_deg"].AsDouble());
  v->scan_end_el_deg = static_cast<float>(j["scan_end_el_deg"].AsDouble());
}

inline void LoadEsrMission(const oneq::JsonValue& j, esr_cfg::EsrMissionConfig* v) {
  if (j.IsNull()) return;
  v->power_on = j["power_on"].AsBool();
  v->work_mode = EsrWorkModeFromString(j["work_mode"].AsString());
  LoadEsrScanPolicy(j["scan"], &v->scan);
}

inline void LoadEsrDetection(const oneq::JsonValue& j,
                             esr_cfg::EsrDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  v->profile = EsrDetectFromString(j["profile"].AsString());
  v->use_profile_defaults = j["use_profile_defaults"].AsBool();
  v->min_detect_snr_db = static_cast<float>(j["min_detect_snr_db"].AsDouble());
  v->pfa = static_cast<float>(j["pfa"].AsDouble());
  v->pulse_count = static_cast<std::uint32_t>(j["pulse_count"].AsInt());
  v->threshold_scale = static_cast<float>(j["threshold_scale"].AsDouble());
  v->enable_statistical_detection = j["enable_statistical_detection"].AsBool();
}

inline void LoadEsrPolicy(const oneq::JsonValue& j, esr_cfg::EsrPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadEsrDetection(j["detection"], &v->detection);
}

inline void LoadEsrAtmosObs(const oneq::JsonValue& j,
                            oneq::foundation::AtmosphericObservation* v) {
  if (j.IsNull()) return;
  v->enable_physical_model = j["enable_physical_model"].AsBool();
  v->pressure_hpa = static_cast<float>(j["pressure_hpa"].AsDouble());
  v->temperature_k = static_cast<float>(j["temperature_k"].AsDouble());
  v->relative_humidity = static_cast<float>(j["relative_humidity"].AsDouble());
}

inline void LoadEsrAtmosCtx(const oneq::JsonValue& j,
                            oneq::foundation::SpaceWeatherContext* v) {
  if (j.IsNull()) return;
  v->solar_flux_f107a = static_cast<float>(j["solar_flux_f107a"].AsDouble());
  v->solar_flux_f107 = static_cast<float>(j["solar_flux_f107"].AsDouble());
  v->geomagnetic_ap = static_cast<float>(j["geomagnetic_ap"].AsDouble());
}

inline void LoadEsrScenario(const oneq::JsonValue& j,
                            esr_env::EsrEnvironmentScenarioConfig* v) {
  if (j.IsNull()) return;
  v->preset = EsrPresetFromString(j["preset"].AsString());
  LoadEsrAtmosObs(j["atmospheric_physics"], &v->atmospheric_physics);
  LoadEsrAtmosCtx(j["atmospheric_context"], &v->atmospheric_context);
}

inline void LoadEsrEnvironment(const oneq::JsonValue& j,
                               esr_cfg::EsrEnvironmentConfig* v) {
  if (j.IsNull()) return;
  LoadEsrScenario(j["scenario_config"], &v->scenario_config);
}

/// Load an EsrSessionConfig from a parsed JSON object.
inline void LoadEsrSessionConfig(
    const oneq::JsonValue& root,
    electronic_surveillance_radar::session::EsrSessionConfig* config) {
  LoadEsrHardware(root["hardware"], &config->hardware);
  LoadEsrMission(root["mission"], &config->mission);
  LoadEsrPolicy(root["policy"], &config->policy);
  LoadEsrEnvironment(root["environment"], &config->environment);
}

/// Load an EsrSessionConfig from a JSON file.
inline bool LoadEsrSessionConfigFromFile(
    const char* path, electronic_surveillance_radar::session::EsrSessionConfig* config,
    std::string* error_msg) {
  oneq::JsonValue root;
  if (!oneq::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != oneq::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadEsrSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_ESR_CONFIG_LOADER_H_
