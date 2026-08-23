#ifndef EXAMPLES_EOS_CONFIG_LOADER_DETAIL_H_
#define EXAMPLES_EOS_CONFIG_LOADER_DETAIL_H_

#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "json_reader.h"
#include "config_loader_common.h"

namespace examples {

// -- struct loaders ----------------------------------------------------------

inline void LoadEosHardware(const examples::JsonValue& j,
                            electro_optical_sensor::config::EosHardwareConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("wavelength_lower_um")) {
    v->wavelength_lower_um = static_cast<float>(j["wavelength_lower_um"].AsDouble());
  }
  if (j.Has("wavelength_upper_um")) {
    v->wavelength_upper_um = static_cast<float>(j["wavelength_upper_um"].AsDouble());
  }
  if (j.Has("optical_aperture_m")) {
    v->optical_aperture_m = static_cast<float>(j["optical_aperture_m"].AsDouble());
  }
}

inline void LoadEosMission(const examples::JsonValue& j,
                           electro_optical_sensor::config::EosMissionConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("work_mode")) {
    v->work_mode = EosWorkModeFromString(j["work_mode"].AsString());
  }
  if (j.Has("horizontal_fov_deg")) {
    v->horizontal_fov_deg = static_cast<float>(j["horizontal_fov_deg"].AsDouble());
  }
  if (j.Has("vertical_fov_deg")) {
    v->vertical_fov_deg = static_cast<float>(j["vertical_fov_deg"].AsDouble());
  }
  if (j.Has("scan_rate_deg_per_sec")) {
    v->scan_rate_deg_per_sec =
        static_cast<float>(j["scan_rate_deg_per_sec"].AsDouble());
  }
  if (j.Has("frame_rate_hz")) {
    v->frame_rate_hz = static_cast<float>(j["frame_rate_hz"].AsDouble());
  }
  if (j.Has("scan_start_az_deg")) {
    v->scan_start_az_deg = static_cast<float>(j["scan_start_az_deg"].AsDouble());
  }
  if (j.Has("scan_end_az_deg")) {
    v->scan_end_az_deg = static_cast<float>(j["scan_end_az_deg"].AsDouble());
  }
  if (j.Has("scan_center_el_deg")) {
    v->scan_center_el_deg = static_cast<float>(j["scan_center_el_deg"].AsDouble());
  }
  if (j.Has("boresight_depression_deg")) {
    v->boresight_depression_deg =
        static_cast<float>(j["boresight_depression_deg"].AsDouble());
  }
}

inline void LoadEosDetection(const examples::JsonValue& j,
                             electro_optical_sensor::config::EosDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("minimum_snr_db")) {
    v->minimum_snr_db = static_cast<float>(j["minimum_snr_db"].AsDouble());
  }
  if (j.Has("detection_sensitivity_w")) {
    v->detection_sensitivity_w =
        static_cast<float>(j["detection_sensitivity_w"].AsDouble());
  }
  if (j.Has("visible_reference_irradiance_w_m2")) {
    v->visible_reference_irradiance_w_m2 =
        static_cast<float>(j["visible_reference_irradiance_w_m2"].AsDouble());
  }
}

inline void LoadEosStrayLight(const examples::JsonValue& j,
                              electro_optical_sensor::config::EosStrayLightPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("enable_straylight_filter")) {
    v->enable_straylight_filter = j["enable_straylight_filter"].AsBool();
  }
  if (j.Has("hood_inner_half_angle_deg")) {
    v->hood_inner_half_angle_deg =
        static_cast<float>(j["hood_inner_half_angle_deg"].AsDouble());
  }
  if (j.Has("hood_outer_half_angle_deg")) {
    v->hood_outer_half_angle_deg =
        static_cast<float>(j["hood_outer_half_angle_deg"].AsDouble());
  }
  if (j.Has("hood_min_suppression_ratio")) {
    v->hood_min_suppression_ratio =
        static_cast<float>(j["hood_min_suppression_ratio"].AsDouble());
  }
  if (j.Has("hood_max_suppression_ratio")) {
    v->hood_max_suppression_ratio =
        static_cast<float>(j["hood_max_suppression_ratio"].AsDouble());
  }
}

inline void LoadEosPolicy(const examples::JsonValue& j,
                          electro_optical_sensor::config::EosPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadEosDetection(j["detection"], &v->detection);
  LoadEosStrayLight(j["stray_light"], &v->stray_light);
}

inline void LoadEosScenario(
    const examples::JsonValue& j,
    electro_optical_sensor::config::EosEnvironmentScenarioConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("preset")) {
    v->preset = EosPresetFromString(j["preset"].AsString());
  }
  const examples::JsonValue& atmosphere = j["atmospheric_physics"];
  if (atmosphere.Has("enable_physical_model")) {
    v->atmospheric_physics.enable_physical_model =
        atmosphere["enable_physical_model"].AsBool();
  }
  if (atmosphere.Has("pressure_hpa")) {
    v->atmospheric_physics.pressure_hpa =
        static_cast<float>(atmosphere["pressure_hpa"].AsDouble());
  }
  if (atmosphere.Has("temperature_k")) {
    v->atmospheric_physics.temperature_k =
        static_cast<float>(atmosphere["temperature_k"].AsDouble());
  }
  if (atmosphere.Has("relative_humidity")) {
    v->atmospheric_physics.relative_humidity =
        static_cast<float>(atmosphere["relative_humidity"].AsDouble());
  }
}

inline void LoadEosEnvironment(
    const examples::JsonValue& j,
    electro_optical_sensor::config::EosEnvironmentConfig* v) {
  if (j.IsNull()) return;
  LoadEosScenario(j["scenario_config"], &v->scenario_config);
}

}  // namespace examples

#endif  // EXAMPLES_EOS_CONFIG_LOADER_DETAIL_H_
