#ifndef EXAMPLES_EOS_CONFIG_LOADER_DETAIL_H_
#define EXAMPLES_EOS_CONFIG_LOADER_DETAIL_H_

#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/foundation/json_reader.h"
#include "config_loader_common.h"

namespace examples {

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
    electro_optical_sensor::environment::EosEnvironmentDefaultConfig* v) {
  if (j.IsNull()) return;
  LoadEosScenario(j["scenario_config"], &v->scenario_config);
}

}  // namespace examples

#endif  // EXAMPLES_EOS_CONFIG_LOADER_DETAIL_H_
