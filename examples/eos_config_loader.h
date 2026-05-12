#ifndef EXAMPLES_EOS_CONFIG_LOADER_H_
#define EXAMPLES_EOS_CONFIG_LOADER_H_

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

// -- struct loaders ----------------------------------------------------------

inline void LoadEosHardware(const oneq::JsonValue& j,
                            electro_optical_sensor::config::EosHardwareConfig* v) {
  if (j.IsNull()) return;
  v->wavelength_lower_um = static_cast<float>(j["wavelength_lower_um"].AsDouble());
  v->wavelength_upper_um = static_cast<float>(j["wavelength_upper_um"].AsDouble());
  v->optical_aperture_m = static_cast<float>(j["optical_aperture_m"].AsDouble());
  v->focal_length_m = static_cast<float>(j["focal_length_m"].AsDouble());
}

inline void LoadEosMission(const oneq::JsonValue& j,
                           electro_optical_sensor::config::EosMissionConfig* v) {
  if (j.IsNull()) return;
  v->work_mode = EosWorkModeFromString(j["work_mode"].AsString());
  v->horizontal_fov_deg = static_cast<float>(j["horizontal_fov_deg"].AsDouble());
  v->vertical_fov_deg = static_cast<float>(j["vertical_fov_deg"].AsDouble());
  v->scan_rate_deg_per_sec =
      static_cast<float>(j["scan_rate_deg_per_sec"].AsDouble());
  v->frame_rate_hz = static_cast<float>(j["frame_rate_hz"].AsDouble());
  v->scan_start_az_deg = static_cast<float>(j["scan_start_az_deg"].AsDouble());
  v->scan_end_az_deg = static_cast<float>(j["scan_end_az_deg"].AsDouble());
  v->scan_center_el_deg = static_cast<float>(j["scan_center_el_deg"].AsDouble());
  v->boresight_depression_deg =
      static_cast<float>(j["boresight_depression_deg"].AsDouble());
}

inline void LoadEosDetection(const oneq::JsonValue& j,
                             electro_optical_sensor::config::EosDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  v->profile = EosDetectFromString(j["profile"].AsString());
  v->use_profile_defaults = j["use_profile_defaults"].AsBool();
  v->minimum_snr_db = static_cast<float>(j["minimum_snr_db"].AsDouble());
  v->detection_sensitivity_w =
      static_cast<float>(j["detection_sensitivity_w"].AsDouble());
  v->visible_reference_irradiance_w_m2 =
      static_cast<float>(j["visible_reference_irradiance_w_m2"].AsDouble());
}

inline void LoadEosStrayLight(const oneq::JsonValue& j,
                              electro_optical_sensor::config::EosStrayLightPolicyConfig* v) {
  if (j.IsNull()) return;
  v->profile = EosStrayFromString(j["profile"].AsString());
  v->use_profile_defaults = j["use_profile_defaults"].AsBool();
  v->enable_straylight_filter = j["enable_straylight_filter"].AsBool();
  v->hood_inner_half_angle_deg =
      static_cast<float>(j["hood_inner_half_angle_deg"].AsDouble());
  v->hood_outer_half_angle_deg =
      static_cast<float>(j["hood_outer_half_angle_deg"].AsDouble());
  v->hood_min_suppression_ratio =
      static_cast<float>(j["hood_min_suppression_ratio"].AsDouble());
  v->hood_max_suppression_ratio =
      static_cast<float>(j["hood_max_suppression_ratio"].AsDouble());
}

inline void LoadEosPolicy(const oneq::JsonValue& j,
                          electro_optical_sensor::config::EosPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadEosDetection(j["detection"], &v->detection);
  LoadEosStrayLight(j["stray_light"], &v->stray_light);
}

inline void LoadEosCustomOverrides(
    const oneq::JsonValue& j,
    electro_optical_sensor::environment::EosEnvironmentCustomOverrides* v) {
  if (j.IsNull()) return;
  v->radiative_transfer_model =
      RadiativeModelFromString(j["radiative_transfer_model"].AsString());
  v->aerosol_density_factor =
      static_cast<float>(j["aerosol_density_factor"].AsDouble());
  v->turbulence_factor = static_cast<float>(j["turbulence_factor"].AsDouble());
  v->enable_optical_countermeasure_extension =
      j["enable_optical_countermeasure_extension"].AsBool();
}

inline void LoadEosScenario(
    const oneq::JsonValue& j,
    electro_optical_sensor::environment::EosEnvironmentScenarioConfig* v) {
  if (j.IsNull()) return;
  v->model_type = EosModelFromString(j["model_type"].AsString());
  v->preset = EosPresetFromString(j["preset"].AsString());
  v->has_custom_overrides = j["has_custom_overrides"].AsBool();
  LoadEosCustomOverrides(j["custom_overrides"], &v->custom_overrides);
}

inline void LoadEosEnvironment(
    const oneq::JsonValue& j,
    electro_optical_sensor::config::EosEnvironmentConfig* v) {
  if (j.IsNull()) return;
  LoadEosScenario(j["scenario_config"], &v->scenario_config);
}

/// Load an EosSessionConfig from a parsed JSON object.
inline void LoadEosSessionConfig(
    const oneq::JsonValue& root,
    electro_optical_sensor::session::EosSessionConfig* config) {
  LoadEosHardware(root["hardware"], &config->hardware);
  LoadEosMission(root["mission"], &config->mission);
  LoadEosPolicy(root["policy"], &config->policy);
  LoadEosEnvironment(root["environment"], &config->environment);
}

/// Load an EosSessionConfig from a JSON file.
inline bool LoadEosSessionConfigFromFile(
    const char* path, electro_optical_sensor::session::EosSessionConfig* config,
    std::string* error_msg) {
  oneq::JsonValue root;
  if (!oneq::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != oneq::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadEosSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_EOS_CONFIG_LOADER_H_
