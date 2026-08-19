#ifndef EXAMPLES_RIR_CONFIG_LOADER_DETAIL_H_
#define EXAMPLES_RIR_CONFIG_LOADER_DETAIL_H_

#include "1q/remote_identification_radar/remote_identification_radar.hpp"
#include "config_loader_common.h"
#include "json_reader.h"

namespace examples {

namespace rir_cfg = remote_identification_radar::config;

inline void LoadRirTransmitter(const examples::JsonValue& j,
                               rir_cfg::hardware::RirTransmitterConfig* v) {
  if (j.IsNull()) return;
  v->equipment_id = static_cast<std::uint64_t>(j["equipment_id"].AsInt());
  v->peak_power_w = static_cast<float>(j["peak_power_w"].AsDouble());
  v->frequency_hz = static_cast<float>(j["frequency_hz"].AsDouble());
  v->bandwidth_hz = static_cast<float>(j["bandwidth_hz"].AsDouble());
  v->pulse_width_s = static_cast<float>(j["pulse_width_s"].AsDouble());
  v->prf_hz = static_cast<float>(j["prf_hz"].AsDouble());
  v->transmit_loss_db = static_cast<float>(j["transmit_loss_db"].AsDouble());
  v->maximum_peak_power_w = static_cast<float>(j["maximum_peak_power_w"].AsDouble());
  v->maximum_duty_cycle = static_cast<float>(j["maximum_duty_cycle"].AsDouble());
  v->maximum_pulse_energy_j = static_cast<float>(j["maximum_pulse_energy_j"].AsDouble());
  const examples::JsonValue& plan = j["frequency_plan_hz"];
  if (!plan.IsNull() && plan.type() == examples::JsonValue::kArray) {
    v->frequency_plan_hz.clear();
    for (std::size_t i = 0U; i < plan.Size(); ++i) {
      v->frequency_plan_hz.push_back(plan[i].AsDouble());
    }
  }
}

inline void LoadRirAntennaPattern(const examples::JsonValue& j,
                                  rir_cfg::hardware::RirAntennaPatternConfig* v) {
  if (j.IsNull()) return;
  v->model_type = RirAntennaPatternModelTypeFromString(j["model_type"].AsString());
  v->max_sidelobe_level_db = static_cast<float>(j["max_sidelobe_level_db"].AsDouble());
  v->backlobe_level_db = static_cast<float>(j["backlobe_level_db"].AsDouble());
  v->scan_loss_coeff_db_per_deg2 =
      static_cast<float>(j["scan_loss_coeff_db_per_deg2"].AsDouble());
  v->max_scan_loss_db = static_cast<float>(j["max_scan_loss_db"].AsDouble());
  LoadRirAzEl(j["boresight_offset_deg"], &v->boresight_offset_deg);
}

inline void LoadRirAntenna(const examples::JsonValue& j, rir_cfg::hardware::RirAntennaConfig* v) {
  if (j.IsNull()) return;
  v->main_beam_gain_db = static_cast<float>(j["main_beam_gain_db"].AsDouble());
  v->nominal_az_beamwidth_deg = static_cast<float>(j["nominal_az_beamwidth_deg"].AsDouble());
  v->nominal_el_beamwidth_deg = static_cast<float>(j["nominal_el_beamwidth_deg"].AsDouble());
  v->antenna_length_m = static_cast<float>(j["antenna_length_m"].AsDouble());
  v->antenna_width_m = static_cast<float>(j["antenna_width_m"].AsDouble());
  LoadRirAntennaPattern(j["pattern"], &v->pattern);
  v->enable_directional_pattern = j["enable_directional_pattern"].AsBool();
}

inline void LoadRirReceiver(const examples::JsonValue& j,
                            rir_cfg::hardware::RirReceiverConfig* v) {
  if (j.IsNull()) return;
  v->equipment_id = static_cast<std::uint64_t>(j["equipment_id"].AsInt());
  v->noise_figure_db = static_cast<float>(j["noise_figure_db"].AsDouble());
  v->receive_loss_db = static_cast<float>(j["receive_loss_db"].AsDouble());
  v->cross_polarization_isolation_db =
      static_cast<float>(j["cross_polarization_isolation_db"].AsDouble());
  v->minimum_far_field_range_m = static_cast<float>(j["minimum_far_field_range_m"].AsDouble());
  v->has_co_site_isolation = j["has_co_site_isolation"].AsBool();
  v->co_site_isolation_db = static_cast<float>(j["co_site_isolation_db"].AsDouble());
  v->maximum_linear_input_power_w =
      static_cast<float>(j["maximum_linear_input_power_w"].AsDouble());
  v->preselector_bandwidth_hz = static_cast<float>(j["preselector_bandwidth_hz"].AsDouble());
  v->interference_observation_jn_gate_db =
      static_cast<float>(j["interference_observation_jn_gate_db"].AsDouble());
  v->scene_polarization = RfScenePolarizationFromString(j["scene_polarization"].AsString());
  const examples::JsonValue& paths = j["co_site_paths"];
  if (!paths.IsNull() && paths.type() == examples::JsonValue::kArray) {
    v->co_site_paths.clear();
    for (std::size_t i = 0U; i < paths.Size(); ++i) {
      const examples::JsonValue& path = paths[i];
      v->co_site_paths.emplace_back(
          static_cast<std::uint64_t>(path["transmitter_equipment_id"].AsInt()),
          static_cast<std::uint64_t>(path["receiver_equipment_id"].AsInt()),
          path["isolation_db"].AsDouble());
    }
  }
}

inline void LoadRirSignalProcessing(const examples::JsonValue& j,
                                    rir_cfg::hardware::RirSignalProcessingConfig* v) {
  if (j.IsNull()) return;
  v->target_processing_gain_db = static_cast<float>(j["target_processing_gain_db"].AsDouble());
  v->noise_processing_gain_db = static_cast<float>(j["noise_processing_gain_db"].AsDouble());
  v->clutter_suppression_gain_db =
      static_cast<float>(j["clutter_suppression_gain_db"].AsDouble());
  v->jamming_suppression_gain_db =
      static_cast<float>(j["jamming_suppression_gain_db"].AsDouble());
}

inline void LoadRirHardware(const examples::JsonValue& j, rir_cfg::RirHardwareConfig* v) {
  if (j.IsNull()) return;
  LoadRirTransmitter(j["transmitter"], &v->transmitter);
  LoadRirAntenna(j["antenna"], &v->antenna);
  LoadRirReceiver(j["receiver"], &v->receiver);
  LoadRirSignalProcessing(j["signal_processing"], &v->signal_processing);
}

inline void LoadRirScan(const examples::JsonValue& j, rir_cfg::RirScanConfig* v) {
  if (j.IsNull()) return;
  LoadRirAzElLimits(j["scan_limits_deg"], &v->scan_limits_deg);
  v->scan_start_position = ScanStartPositionFromString(j["scan_start_position"].AsString());
  v->scan_sequence = ScanSequenceFromString(j["scan_sequence"].AsString());
  v->step_scale = static_cast<float>(j["step_scale"].AsDouble());
}

inline void LoadRirMission(const examples::JsonValue& j, rir_cfg::RirMissionConfig* v) {
  if (j.IsNull()) return;
  v->work_mode = RirWorkModeFromString(j["work_mode"].AsString());
  v->max_range_m = static_cast<float>(j["max_range_m"].AsDouble());
  v->recognition_dwell_sec = static_cast<float>(j["recognition_dwell_sec"].AsDouble());
  LoadRirScan(j["scan"], &v->scan);
}

inline void LoadRirDetectionPolicy(const examples::JsonValue& j,
                                   rir_cfg::RirDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  v->cfar_pfa = static_cast<float>(j["cfar_pfa"].AsDouble());
  v->min_snr_db = static_cast<float>(j["min_snr_db"].AsDouble());
  v->min_detection_margin_db = static_cast<float>(j["min_detection_margin_db"].AsDouble());
  v->pulse_count = static_cast<int>(j["pulse_count"].AsInt());
  v->random_seed = static_cast<std::uint32_t>(j["random_seed"].AsInt());
  v->gate_mode = RirDetectionGateModeFromString(j["gate_mode"].AsString());
}

inline void LoadRirAssociationPolicy(const examples::JsonValue& j,
                                     rir_cfg::RirAssociationPolicyConfig* v) {
  if (j.IsNull()) return;
  v->distance_gate_sigma = static_cast<float>(j["distance_gate_sigma"].AsDouble());
}

inline void LoadRirTrackingPolicy(const examples::JsonValue& j,
                                  rir_cfg::RirTrackingPolicyConfig* v) {
  if (j.IsNull()) return;
  v->kalman_noise_diff_coeff = static_cast<float>(j["kalman_noise_diff_coeff"].AsDouble());
  v->kalman_measurement_noise_std =
      static_cast<float>(j["kalman_measurement_noise_std"].AsDouble());
}

inline void LoadRirLifecyclePolicy(const examples::JsonValue& j,
                                   rir_cfg::RirLifecyclePolicyConfig* v) {
  if (j.IsNull()) return;
  v->confirm_hits = static_cast<std::uint32_t>(j["confirm_hits"].AsInt());
  v->max_miss_before_lost = static_cast<std::uint32_t>(j["max_miss_before_lost"].AsInt());
  v->max_lost_cycles = static_cast<std::uint32_t>(j["max_lost_cycles"].AsInt());
  v->enable_imm_lifecycle = j["enable_imm_lifecycle"].AsBool();
  if (j.Has("model_count_hint")) {
    v->model_count_hint = static_cast<std::uint32_t>(j["model_count_hint"].AsInt());
  }
}

inline void LoadRirRecognitionFeatureWeights(const examples::JsonValue& j,
                                             rir_cfg::RirRecognitionFeatureWeights* v) {
  if (j.IsNull()) return;
  v->rcs_weight = static_cast<float>(j["rcs_weight"].AsDouble());
  v->motion_weight = static_cast<float>(j["motion_weight"].AsDouble());
  v->polarization_weight = static_cast<float>(j["polarization_weight"].AsDouble());
  v->range_profile_weight = static_cast<float>(j["range_profile_weight"].AsDouble());
}

inline void LoadRirRecognitionPolicy(const examples::JsonValue& j,
                                     rir_cfg::RirRecognitionPolicy* v) {
  if (j.IsNull()) return;
  v->enabled = j["enabled"].AsBool();
  v->min_confirmed_hits = static_cast<std::uint32_t>(j["min_confirmed_hits"].AsInt());
  v->accumulation_window_sec = static_cast<float>(j["accumulation_window_sec"].AsDouble());
  v->min_observation_count = static_cast<std::uint32_t>(j["min_observation_count"].AsInt());
  v->acceptance_score = static_cast<float>(j["acceptance_score"].AsDouble());
  v->minimum_margin = static_cast<float>(j["minimum_margin"].AsDouble());
  v->result_hold_sec = static_cast<float>(j["result_hold_sec"].AsDouble());
  LoadRirRecognitionFeatureWeights(j["feature_weights"], &v->feature_weights);
  v->database_path = j["database_path"].AsString();
}

inline void LoadRirPolicy(const examples::JsonValue& j, rir_cfg::RirPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadRirDetectionPolicy(j["detection"], &v->detection);
  LoadRirAssociationPolicy(j["association"], &v->association);
  LoadRirTrackingPolicy(j["tracking"], &v->tracking);
  LoadRirLifecyclePolicy(j["lifecycle"], &v->lifecycle);
  LoadRirRecognitionPolicy(j["recognition"], &v->recognition);
}

inline void LoadRirVegetationScatterPhysics(const examples::JsonValue& j,
                                            rir_cfg::RirVegetationScatterPhysicsConfig* v) {
  if (j.IsNull()) return;
  v->cover_profile = RirVegetationCoverProfileFromString(j["cover_profile"].AsString());
  v->enable_physical_model = j["enable_physical_model"].AsBool();
}

inline void LoadRirEnvironment(const examples::JsonValue& j, rir_cfg::RirEnvironmentConfig* v) {
  if (j.IsNull()) return;
  v->enable_environment_effects = j["enable_environment_effects"].AsBool();
  v->weather_attenuation_db = static_cast<float>(j["weather_attenuation_db"].AsDouble());
  LoadRirVegetationScatterPhysics(j["vegetation_scatter_physics"],
                                  &v->vegetation_scatter_physics);
}

}  // namespace examples

#endif  // EXAMPLES_RIR_CONFIG_LOADER_DETAIL_H_
