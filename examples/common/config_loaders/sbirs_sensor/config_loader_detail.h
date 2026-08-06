#ifndef EXAMPLES_SBIRS_CONFIG_LOADER_DETAIL_H_
#define EXAMPLES_SBIRS_CONFIG_LOADER_DETAIL_H_

#include "1q/sbirs_sensor/sbirs_sensor.hpp"
#include "json_reader.h"
#include "config_loader_common.h"

namespace examples {

// -- struct loaders ----------------------------------------------------------

inline void LoadSbirsHardware(const examples::JsonValue& j,
                              sbirs_sensor::config::SbirsHardwareConfig* v) {
  if (j.IsNull()) return;
  v->wavelength_lower_um = static_cast<float>(j["wavelength_lower_um"].AsDouble());
  v->wavelength_upper_um = static_cast<float>(j["wavelength_upper_um"].AsDouble());
  v->optical_aperture_m = static_cast<float>(j["optical_aperture_m"].AsDouble());
  v->optical_transmission = static_cast<float>(j["optical_transmission"].AsDouble());
  v->detector_quantum_efficiency =
      static_cast<float>(j["detector_quantum_efficiency"].AsDouble());
  v->integration_time_sec = static_cast<float>(j["integration_time_sec"].AsDouble());
  v->noise_equivalent_power_w =
      static_cast<float>(j["noise_equivalent_power_w"].AsDouble());
  v->background_radiance_w_sr_m2 =
      static_cast<float>(j["background_radiance_w_sr_m2"].AsDouble());
  v->detector_temperature_k =
      static_cast<float>(j["detector_temperature_k"].AsDouble());
  v->readout_noise_rms_w = static_cast<float>(j["readout_noise_rms_w"].AsDouble());
}

inline void LoadSbirsMission(const examples::JsonValue& j,
                             sbirs_sensor::config::SbirsMissionConfig* v) {
  if (j.IsNull()) return;
  v->work_mode = SbirsWorkModeFromString(j["work_mode"].AsString());
  v->wide_field_fov_az_deg = static_cast<float>(j["wide_field_fov_az_deg"].AsDouble());
  v->wide_field_fov_el_deg = static_cast<float>(j["wide_field_fov_el_deg"].AsDouble());
  v->narrow_field_fov_az_deg =
      static_cast<float>(j["narrow_field_fov_az_deg"].AsDouble());
  v->narrow_field_fov_el_deg =
      static_cast<float>(j["narrow_field_fov_el_deg"].AsDouble());
  v->scan_start_az_deg = static_cast<float>(j["scan_start_az_deg"].AsDouble());
  v->scan_span_deg = static_cast<float>(j["scan_span_deg"].AsDouble());
  v->scan_direction = SbirsScanDirectionFromString(j["scan_direction"].AsString());
  v->scan_center_el_deg = static_cast<float>(j["scan_center_el_deg"].AsDouble());
  v->scan_rate_deg_per_sec = static_cast<float>(j["scan_rate_deg_per_sec"].AsDouble());
  v->min_range_m = static_cast<float>(j["min_range_m"].AsDouble());
  v->max_range_m = static_cast<float>(j["max_range_m"].AsDouble());
  v->frame_rate_hz = static_cast<float>(j["frame_rate_hz"].AsDouble());
  v->narrow_cue_latency_s = static_cast<float>(j["narrow_cue_latency_s"].AsDouble());
  v->narrow_pointing_settle_error_deg =
      static_cast<float>(j["narrow_pointing_settle_error_deg"].AsDouble());
  v->narrow_pointing_max_slew_rate_deg_per_sec =
      static_cast<float>(j["narrow_pointing_max_slew_rate_deg_per_sec"].AsDouble());
  v->narrow_pointing_settle_tolerance_deg =
      static_cast<float>(j["narrow_pointing_settle_tolerance_deg"].AsDouble());
}

inline void LoadSbirsDetection(const examples::JsonValue& j,
                               sbirs_sensor::config::SbirsDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  v->wide_min_snr_linear = static_cast<float>(j["wide_min_snr_linear"].AsDouble());
  v->narrow_min_snr_linear = static_cast<float>(j["narrow_min_snr_linear"].AsDouble());
}

inline void LoadSbirsErrorModel(const examples::JsonValue& j,
                                sbirs_sensor::config::SbirsErrorModelConfig* v) {
  if (j.IsNull()) return;
  v->range_fraction_sigma =
      static_cast<float>(j["range_fraction_sigma"].AsDouble());
  v->random_seed = static_cast<std::uint32_t>(j["random_seed"].AsInt());
  v->orbit_sigma_deg = static_cast<float>(j["orbit_sigma_deg"].AsDouble());
  v->attitude_sigma_deg = static_cast<float>(j["attitude_sigma_deg"].AsDouble());
  v->fov_sigma_deg = static_cast<float>(j["fov_sigma_deg"].AsDouble());
  v->detector_bandwidth_hz = static_cast<float>(j["detector_bandwidth_hz"].AsDouble());
}

inline void LoadSbirsPointingDisturbance(
    const examples::JsonValue& j,
    sbirs_sensor::config::SbirsPointingDisturbanceConfig* v) {
  if (j.IsNull()) return;
  v->common_attitude_sigma_deg =
      static_cast<float>(j["common_attitude_sigma_deg"].AsDouble());
  v->common_attitude_correlation_time_s =
      static_cast<float>(j["common_attitude_correlation_time_s"].AsDouble());
  v->channel_pointing_sigma_deg =
      static_cast<float>(j["channel_pointing_sigma_deg"].AsDouble());
  v->channel_pointing_correlation_time_s =
      static_cast<float>(j["channel_pointing_correlation_time_s"].AsDouble());
  v->channel_vibration_amplitude_deg =
      static_cast<float>(j["channel_vibration_amplitude_deg"].AsDouble());
  v->channel_vibration_frequency_hz =
      static_cast<float>(j["channel_vibration_frequency_hz"].AsDouble());
  v->random_seed = static_cast<std::uint32_t>(j["random_seed"].AsInt());
}

inline void LoadSbirsScheduler(const examples::JsonValue& j,
                               sbirs_sensor::config::SbirsSchedulerConfig* v) {
  if (j.IsNull()) return;
  v->max_concurrent_nfov_locks = static_cast<int>(j["max_concurrent_nfov_locks"].AsInt());
}

inline void LoadSbirsTracking(const examples::JsonValue& j,
                              sbirs_sensor::config::SbirsTrackingConfig* v) {
  if (j.IsNull()) return;
  v->tracking_mode = SbirsTrackingModeFromString(j["tracking_mode"].AsString());
  v->estimated_backend =
      SbirsEstimatedTrackingBackendFromString(j["estimated_backend"].AsString());
  v->process_noise_diff_coeff =
      static_cast<float>(j["process_noise_diff_coeff"].AsDouble());
  v->initial_position_std_m =
      static_cast<float>(j["initial_position_std_m"].AsDouble());
  v->initial_velocity_std_m_per_s =
      static_cast<float>(j["initial_velocity_std_m_per_s"].AsDouble());
  v->nis_gate_loss_cycles =
      static_cast<unsigned int>(j["nis_gate_loss_cycles"].AsInt());
  v->nfov_tracking_gate_loss_cycles =
      static_cast<unsigned int>(j["nfov_tracking_gate_loss_cycles"].AsInt());
  const examples::JsonValue& coeffs = j["imm_model_noise_diff_coeffs"];
  if (!coeffs.IsNull() && coeffs.type() == examples::JsonValue::kArray) {
    v->imm_model_noise_diff_coeffs.clear();
    for (std::size_t i = 0; i < coeffs.Size(); ++i) {
      v->imm_model_noise_diff_coeffs.push_back(
          static_cast<float>(coeffs[i].AsDouble()));
    }
  }
}

inline void LoadSbirsPolicy(const examples::JsonValue& j,
                            sbirs_sensor::config::SbirsPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadSbirsDetection(j["detection"], &v->detection);
  LoadSbirsErrorModel(j["error_model"], &v->error_model);
  LoadSbirsPointingDisturbance(j["pointing_disturbance"], &v->pointing_disturbance);
  LoadSbirsScheduler(j["scheduler"], &v->scheduler);
  LoadSbirsTracking(j["tracking"], &v->tracking);
}

inline void LoadSbirsEnvironment(const examples::JsonValue& j,
                                 sbirs_sensor::config::SbirsEnvironmentConfig* v) {
  if (j.IsNull()) return;
  v->weather_type = SbirsWeatherTypeFromString(j["weather_type"].AsString());
  v->sea_state = SbirsSeaStateFromString(j["sea_state"].AsString());
  v->temperature_c = static_cast<float>(j["temperature_c"].AsDouble());
  v->relative_humidity_percent =
      static_cast<float>(j["relative_humidity_percent"].AsDouble());
  v->visibility_km = static_cast<float>(j["visibility_km"].AsDouble());
  v->base_atmospheric_transmittance =
      static_cast<float>(j["base_atmospheric_transmittance"].AsDouble());
  v->humidity_visibility_interaction_weight =
      static_cast<float>(j["humidity_visibility_interaction_weight"].AsDouble());
  v->rain_humidity_interaction_weight =
      static_cast<float>(j["rain_humidity_interaction_weight"].AsDouble());
}

}  // namespace examples

#endif  // EXAMPLES_SBIRS_CONFIG_LOADER_DETAIL_H_
