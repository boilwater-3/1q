#ifndef EXAMPLES_ESR_CONFIG_LOADER_DETAIL_H_
#define EXAMPLES_ESR_CONFIG_LOADER_DETAIL_H_

#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/environment/AtmosphericTypes.h"
#include "json_reader.h"
#include "config_loader_common.h"

namespace examples {

namespace esr_env = electronic_surveillance_radar::session;

// -- struct loaders ----------------------------------------------------------

inline void LoadEsrHardware(const examples::JsonValue& j,
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
  v->antenna_peak_gain_dbi = static_cast<float>(j["antenna_peak_gain_dbi"].AsDouble());
  v->antenna_sidelobe_level_db =
      static_cast<float>(j["antenna_sidelobe_level_db"].AsDouble());
  v->antenna_backlobe_level_db =
      static_cast<float>(j["antenna_backlobe_level_db"].AsDouble());
  v->cross_polarization_isolation_db =
      static_cast<float>(j["cross_polarization_isolation_db"].AsDouble());
  v->minimum_far_field_range_m =
      static_cast<float>(j["minimum_far_field_range_m"].AsDouble());
  v->has_co_site_isolation = j["has_co_site_isolation"].AsBool();
  v->co_site_isolation_db = static_cast<float>(j["co_site_isolation_db"].AsDouble());
  v->maximum_linear_input_power_w =
      static_cast<float>(j["maximum_linear_input_power_w"].AsDouble());
  v->jamming_jn_threshold_db =
      static_cast<float>(j["jamming_jn_threshold_db"].AsDouble());
  v->jamming_snr_loss_threshold_db =
      static_cast<float>(j["jamming_snr_loss_threshold_db"].AsDouble());
}

inline void LoadEsrScanPolicy(const examples::JsonValue& j,
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

inline void LoadEsrMission(const examples::JsonValue& j, esr_cfg::EsrMissionConfig* v) {
  if (j.IsNull()) return;
  v->power_on = j["power_on"].AsBool();
  v->work_mode = EsrWorkModeFromString(j["work_mode"].AsString());
  LoadEsrScanPolicy(j["scan"], &v->scan);
}

inline void LoadEsrDetection(const examples::JsonValue& j,
                             esr_cfg::EsrDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  v->minimum_snr_db = static_cast<float>(j["minimum_snr_db"].AsDouble());
  v->pfa = static_cast<float>(j["pfa"].AsDouble());
  v->pulse_count = static_cast<std::uint32_t>(j["pulse_count"].AsInt());
  v->threshold_scale = static_cast<float>(j["threshold_scale"].AsDouble());
  v->enable_statistical_detection = j["enable_statistical_detection"].AsBool();
}

inline void LoadEsrPolicy(const examples::JsonValue& j, esr_cfg::EsrPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadEsrDetection(j["detection"], &v->detection);
}

inline void LoadEsrAtmosObs(const examples::JsonValue& j,
                            oneq::environment::AtmosphericObservation* v) {
  if (j.IsNull()) return;
  v->enable_physical_model = j["enable_physical_model"].AsBool();
  v->pressure_hpa = static_cast<float>(j["pressure_hpa"].AsDouble());
  v->temperature_k = static_cast<float>(j["temperature_k"].AsDouble());
  v->relative_humidity = static_cast<float>(j["relative_humidity"].AsDouble());
}

inline void LoadEsrAtmosCtx(const examples::JsonValue& j,
                            oneq::environment::SpaceWeatherContext* v) {
  if (j.IsNull()) return;
  v->solar_flux_f107a = static_cast<float>(j["solar_flux_f107a"].AsDouble());
  v->solar_flux_f107 = static_cast<float>(j["solar_flux_f107"].AsDouble());
  v->geomagnetic_ap = static_cast<float>(j["geomagnetic_ap"].AsDouble());
}

inline void LoadEsrScenario(const examples::JsonValue& j,
                            electronic_surveillance_radar::config::EsrEnvironmentScenarioConfig* v) {
  if (j.IsNull()) return;
  v->preset = EsrPresetFromString(j["preset"].AsString());
  LoadEsrAtmosObs(j["atmospheric_physics"], &v->atmospheric_physics);
  LoadEsrAtmosCtx(j["atmospheric_context"], &v->atmospheric_context);
}

inline void LoadEsrEnvironment(const examples::JsonValue& j,
                               esr_cfg::EsrEnvironmentConfig* v) {
  if (j.IsNull()) return;
  LoadEsrScenario(j["scenario_config"], &v->scenario_config);
}

}  // namespace examples

#endif  // EXAMPLES_ESR_CONFIG_LOADER_DETAIL_H_
