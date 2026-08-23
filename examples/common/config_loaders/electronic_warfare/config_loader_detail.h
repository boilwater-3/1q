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
  if (j.Has("receiver_band_lower_hz")) {
    v->receiver_band_lower_hz = j["receiver_band_lower_hz"].AsDouble();
  }
  if (j.Has("receiver_band_upper_hz")) {
    v->receiver_band_upper_hz = j["receiver_band_upper_hz"].AsDouble();
  }
  if (j.Has("receiver_sensitivity_w")) {
    v->receiver_sensitivity_w = static_cast<float>(j["receiver_sensitivity_w"].AsDouble());
  }
  if (j.Has("integrated_receive_loss_db")) {
    v->integrated_receive_loss_db =
        static_cast<float>(j["integrated_receive_loss_db"].AsDouble());
  }
  if (j.Has("beam_az_width_deg")) {
    v->beam_az_width_deg = static_cast<float>(j["beam_az_width_deg"].AsDouble());
  }
  if (j.Has("beam_el_width_deg")) {
    v->beam_el_width_deg = static_cast<float>(j["beam_el_width_deg"].AsDouble());
  }
  if (j.Has("az_scan_range_deg")) {
    v->az_scan_range_deg = static_cast<float>(j["az_scan_range_deg"].AsDouble());
  }
  if (j.Has("el_scan_range_deg")) {
    v->el_scan_range_deg = static_cast<float>(j["el_scan_range_deg"].AsDouble());
  }
  if (j.Has("antenna_peak_gain_dbi")) {
    v->antenna_peak_gain_dbi = static_cast<float>(j["antenna_peak_gain_dbi"].AsDouble());
  }
  if (j.Has("antenna_sidelobe_level_db")) {
    v->antenna_sidelobe_level_db =
        static_cast<float>(j["antenna_sidelobe_level_db"].AsDouble());
  }
  if (j.Has("antenna_backlobe_level_db")) {
    v->antenna_backlobe_level_db =
        static_cast<float>(j["antenna_backlobe_level_db"].AsDouble());
  }
  if (j.Has("cross_polarization_isolation_db")) {
    v->cross_polarization_isolation_db =
        static_cast<float>(j["cross_polarization_isolation_db"].AsDouble());
  }
  if (j.Has("minimum_far_field_range_m")) {
    v->minimum_far_field_range_m =
        static_cast<float>(j["minimum_far_field_range_m"].AsDouble());
  }
  if (j.Has("maximum_linear_input_power_w")) {
    v->maximum_linear_input_power_w =
        static_cast<float>(j["maximum_linear_input_power_w"].AsDouble());
  }
}

inline void LoadEsrOrientation(const examples::JsonValue& j,
                               esr_cfg::EsrOrientationConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("antenna_mount_az_deg")) {
    v->antenna_mount_az_deg = static_cast<float>(j["antenna_mount_az_deg"].AsDouble());
  }
  if (j.Has("antenna_mount_el_deg")) {
    v->antenna_mount_el_deg = static_cast<float>(j["antenna_mount_el_deg"].AsDouble());
  }
}

inline void LoadEsrScanPolicy(const examples::JsonValue& j,
                              esr_cfg::EsrScanPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("scan_center_az_deg")) {
    v->scan_center_az_deg = static_cast<float>(j["scan_center_az_deg"].AsDouble());
  }
  if (j.Has("scan_center_el_deg")) {
    v->scan_center_el_deg = static_cast<float>(j["scan_center_el_deg"].AsDouble());
  }
  if (j.Has("scan_rate_hz")) {
    v->scan_rate_hz = static_cast<float>(j["scan_rate_hz"].AsDouble());
  }
  if (j.Has("use_explicit_scan_bounds")) {
    v->use_explicit_scan_bounds = j["use_explicit_scan_bounds"].AsBool();
  }
  if (j.Has("scan_start_az_deg")) {
    v->scan_start_az_deg = static_cast<float>(j["scan_start_az_deg"].AsDouble());
  }
  if (j.Has("scan_end_az_deg")) {
    v->scan_end_az_deg = static_cast<float>(j["scan_end_az_deg"].AsDouble());
  }
  if (j.Has("scan_start_el_deg")) {
    v->scan_start_el_deg = static_cast<float>(j["scan_start_el_deg"].AsDouble());
  }
  if (j.Has("scan_end_el_deg")) {
    v->scan_end_el_deg = static_cast<float>(j["scan_end_el_deg"].AsDouble());
  }
}

inline void LoadEsrMission(const examples::JsonValue& j, esr_cfg::EsrMissionConfig* v) {
  if (j.IsNull()) return;
  // 电源状态由 EsrSessionConfig::sensor_enabled 承载（COMMON-OQ-4 字段提升）；
  // mission 域无电源字段。
  if (j.Has("work_mode")) {
    v->work_mode = EsrWorkModeFromString(j["work_mode"].AsString());
  }
  LoadEsrScanPolicy(j["scan"], &v->scan);
}

inline void LoadEsrDetection(const examples::JsonValue& j,
                             esr_cfg::EsrDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("minimum_snr_db")) {
    v->minimum_snr_db = static_cast<float>(j["minimum_snr_db"].AsDouble());
  }
  if (j.Has("pfa")) {
    v->pfa = static_cast<float>(j["pfa"].AsDouble());
  }
  if (j.Has("pulse_count")) {
    v->pulse_count = static_cast<std::uint32_t>(j["pulse_count"].AsInt());
  }
  if (j.Has("threshold_scale")) {
    v->threshold_scale = static_cast<float>(j["threshold_scale"].AsDouble());
  }
  if (j.Has("enable_statistical_detection")) {
    v->enable_statistical_detection = j["enable_statistical_detection"].AsBool();
  }
}

inline void LoadEsrPolicy(const examples::JsonValue& j, esr_cfg::EsrPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadEsrDetection(j["detection"], &v->detection);
}

inline void LoadEsrAtmosObs(const examples::JsonValue& j,
                            oneq::environment::AtmosphericObservation* v) {
  if (j.IsNull()) return;
  if (j.Has("enable_physical_model")) {
    v->enable_physical_model = j["enable_physical_model"].AsBool();
  }
  if (j.Has("pressure_hpa")) {
    v->pressure_hpa = static_cast<float>(j["pressure_hpa"].AsDouble());
  }
  if (j.Has("temperature_k")) {
    v->temperature_k = static_cast<float>(j["temperature_k"].AsDouble());
  }
  if (j.Has("relative_humidity")) {
    v->relative_humidity = static_cast<float>(j["relative_humidity"].AsDouble());
  }
}

inline void LoadEsrScenario(const examples::JsonValue& j,
                            electronic_surveillance_radar::config::EsrEnvironmentScenarioConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("preset")) {
    v->preset = EsrPresetFromString(j["preset"].AsString());
  }
  LoadEsrAtmosObs(j["atmospheric_physics"], &v->atmospheric_physics);
}

inline void LoadEsrEnvironment(const examples::JsonValue& j,
                               esr_cfg::EsrEnvironmentConfig* v) {
  if (j.IsNull()) return;
  LoadEsrScenario(j["scenario_config"], &v->scenario_config);
}

}  // namespace examples

#endif  // EXAMPLES_ESR_CONFIG_LOADER_DETAIL_H_
