#ifndef EXAMPLES_SAR_CONFIG_LOADER_DETAIL_H_
#define EXAMPLES_SAR_CONFIG_LOADER_DETAIL_H_

#include "json_reader.h"
#include "1q/sar/sar.hpp"
#include "config_loader_common.h"

namespace examples {

// -- SarHardwareConfig loader -------------------------------------------------

inline void LoadSarHardware(const examples::JsonValue& j,
                            sar::config::SarHardwareConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("carrier_frequency_hz")) {
    v->carrier_frequency_hz = j["carrier_frequency_hz"].AsDouble();
  }
  if (j.Has("bandwidth_hz")) {
    v->bandwidth_hz = j["bandwidth_hz"].AsDouble();
  }
  if (j.Has("pulse_width_s")) {
    v->pulse_width_s = j["pulse_width_s"].AsDouble();
  }
  if (j.Has("pulse_repetition_frequency_hz")) {
    v->pulse_repetition_frequency_hz =
        j["pulse_repetition_frequency_hz"].AsDouble();
  }
  if (j.Has("sample_rate_hz")) {
    v->sample_rate_hz = j["sample_rate_hz"].AsDouble();
  }
  if (j.Has("peak_power_w")) {
    v->peak_power_w = j["peak_power_w"].AsDouble();
  }
  if (j.Has("antenna_length_m")) {
    v->antenna_length_m = j["antenna_length_m"].AsDouble();
  }
  if (j.Has("antenna_width_m")) {
    v->antenna_width_m = j["antenna_width_m"].AsDouble();
  }
  if (j.Has("antenna_gain_db")) {
    v->antenna_gain_db = j["antenna_gain_db"].AsDouble();
  }
  if (j.Has("receiver_noise_figure_db")) {
    v->receiver_noise_figure_db = j["receiver_noise_figure_db"].AsDouble();
  }
  if (j.Has("system_loss_db")) {
    v->system_loss_db = j["system_loss_db"].AsDouble();
  }
}

// -- SarMissionConfig loader --------------------------------------------------

inline void LoadSarWaypoints(const examples::JsonValue& j,
                             sar::config::SarWaypointConfigList* v) {
  if (j.IsNull() || j.type() != examples::JsonValue::kArray) return;
  v->clear();
  for (std::size_t i = 0; i < j.Size(); ++i) {
    const auto& item = j[i];
    sar::config::SarWaypointConfig wp;
    wp.time_from_session_start_s =
        item["time_from_session_start_s"].AsDouble();
    wp.latitude_deg = item["latitude_deg"].AsDouble();
    wp.longitude_deg = item["longitude_deg"].AsDouble();
    wp.altitude_m = item["altitude_m"].AsDouble();
    v->push_back(wp);
  }
}

inline void LoadSarMission(const examples::JsonValue& j,
                           sar::config::SarMissionConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("scene_center_latitude_deg")) {
    v->scene_center_latitude_deg = j["scene_center_latitude_deg"].AsDouble();
  }
  if (j.Has("scene_center_longitude_deg")) {
    v->scene_center_longitude_deg = j["scene_center_longitude_deg"].AsDouble();
  }
  if (j.Has("scene_center_altitude_m")) {
    v->scene_center_altitude_m = j["scene_center_altitude_m"].AsDouble();
  }
  if (j.Has("nominal_slant_range_m")) {
    v->nominal_slant_range_m = j["nominal_slant_range_m"].AsDouble();
  }
  if (j.Has("platform_speed_mps")) {
    v->platform_speed_mps = j["platform_speed_mps"].AsDouble();
  }
  if (j.Has("range_sample_count")) {
    v->range_sample_count =
        static_cast<std::uint32_t>(j["range_sample_count"].AsInt());
  }
  if (j.Has("azimuth_pulse_count")) {
    v->azimuth_pulse_count =
        static_cast<std::uint32_t>(j["azimuth_pulse_count"].AsInt());
  }
  if (j.Has("desired_ground_range_resolution_m")) {
    v->desired_ground_range_resolution_m =
        j["desired_ground_range_resolution_m"].AsDouble();
  }
  if (j.Has("desired_azimuth_resolution_m")) {
    v->desired_azimuth_resolution_m =
        j["desired_azimuth_resolution_m"].AsDouble();
  }
  if (j.Has("l2_velocity_error_stddev_x_mps")) {
    v->l2_velocity_error_stddev_x_mps =
        j["l2_velocity_error_stddev_x_mps"].AsDouble();
  }
  if (j.Has("l2_velocity_error_stddev_y_mps")) {
    v->l2_velocity_error_stddev_y_mps =
        j["l2_velocity_error_stddev_y_mps"].AsDouble();
  }
  if (j.Has("l2_velocity_error_stddev_z_mps")) {
    v->l2_velocity_error_stddev_z_mps =
        j["l2_velocity_error_stddev_z_mps"].AsDouble();
  }
  if (j.Has("l2_random_seed")) {
    v->l2_random_seed =
        static_cast<std::uint32_t>(j["l2_random_seed"].AsInt());
  }
  LoadSarWaypoints(j["l3_waypoints"], &v->l3_waypoints);
}

// -- SarPolicyConfig loader ---------------------------------------------------

inline void LoadSarProcessing(const examples::JsonValue& j,
                              sar::config::SarPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("enable_raw_echo_generation")) {
    v->enable_raw_echo_generation = j["enable_raw_echo_generation"].AsBool();
  }
  if (j.Has("enable_l1_rda_imaging")) {
    v->enable_l1_rda_imaging = j["enable_l1_rda_imaging"].AsBool();
  }
  if (j.Has("enable_l2_motion_compensation")) {
    v->enable_l2_motion_compensation = j["enable_l2_motion_compensation"].AsBool();
  }
  if (j.Has("enable_l3_bp_imaging")) {
    v->enable_l3_bp_imaging = j["enable_l3_bp_imaging"].AsBool();
  }
  if (j.Has("enable_diagnostics")) {
    v->enable_diagnostics = j["enable_diagnostics"].AsBool();
  }
  if (j.Has("retain_focused_image")) {
    v->retain_focused_image = j["retain_focused_image"].AsBool();
  }
  if (j.Has("max_allowed_squint_angle_deg")) {
    v->max_allowed_squint_angle_deg =
        j["max_allowed_squint_angle_deg"].AsDouble();
  }
  if (j.Has("minimum_snr_db")) {
    v->minimum_snr_db = j["minimum_snr_db"].AsDouble();
  }
}

// -- SarEnvironmentConfig loader (placeholder, future-proofing) ---------------

inline void LoadSarEnvironment(const examples::JsonValue& j,
                               sar::config::SarEnvironmentConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("terrain_reference_altitude_m")) {
    v->terrain_reference_altitude_m = j["terrain_reference_altitude_m"].AsDouble();
  }
  if (j.Has("atmospheric_loss_db_per_km")) {
    v->atmospheric_loss_db_per_km = j["atmospheric_loss_db_per_km"].AsDouble();
  }
  if (j.Has("surface_backscatter_sigma0_db")) {
    v->surface_backscatter_sigma0_db =
        j["surface_backscatter_sigma0_db"].AsDouble();
  }
  if (j.Has("use_flat_earth_geometry")) {
    v->use_flat_earth_geometry = j["use_flat_earth_geometry"].AsBool();
  }
  if (j.Has("enable_atmospheric_attenuation")) {
    v->enable_atmospheric_attenuation =
        j["enable_atmospheric_attenuation"].AsBool();
  }
}

}  // namespace examples

#endif  // EXAMPLES_SAR_CONFIG_LOADER_DETAIL_H_
