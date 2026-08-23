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
  if (j.Has("equipment_id")) {
    v->equipment_id = static_cast<std::uint64_t>(j["equipment_id"].AsInt());
  }
  if (j.Has("peak_power_w")) {
    v->peak_power_w = static_cast<float>(j["peak_power_w"].AsDouble());
  }
  if (j.Has("frequency_hz")) {
    v->frequency_hz = static_cast<float>(j["frequency_hz"].AsDouble());
  }
  if (j.Has("bandwidth_hz")) {
    v->bandwidth_hz = static_cast<float>(j["bandwidth_hz"].AsDouble());
  }
  if (j.Has("pulse_width_s")) {
    v->pulse_width_s = static_cast<float>(j["pulse_width_s"].AsDouble());
  }
  if (j.Has("prf_hz")) {
    v->prf_hz = static_cast<float>(j["prf_hz"].AsDouble());
  }
  if (j.Has("transmit_loss_db")) {
    v->transmit_loss_db = static_cast<float>(j["transmit_loss_db"].AsDouble());
  }
  if (j.Has("maximum_peak_power_w")) {
    v->maximum_peak_power_w = static_cast<float>(j["maximum_peak_power_w"].AsDouble());
  }
  if (j.Has("maximum_duty_cycle")) {
    v->maximum_duty_cycle = static_cast<float>(j["maximum_duty_cycle"].AsDouble());
  }
  if (j.Has("maximum_pulse_energy_j")) {
    v->maximum_pulse_energy_j = static_cast<float>(j["maximum_pulse_energy_j"].AsDouble());
  }
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
  if (j.Has("model_type")) {
    v->model_type = RirAntennaPatternModelTypeFromString(j["model_type"].AsString());
  }
  if (j.Has("max_sidelobe_level_db")) {
    v->max_sidelobe_level_db = static_cast<float>(j["max_sidelobe_level_db"].AsDouble());
  }
  if (j.Has("backlobe_level_db")) {
    v->backlobe_level_db = static_cast<float>(j["backlobe_level_db"].AsDouble());
  }
  if (j.Has("scan_loss_coeff_db_per_deg2")) {
    v->scan_loss_coeff_db_per_deg2 =
        static_cast<float>(j["scan_loss_coeff_db_per_deg2"].AsDouble());
  }
  if (j.Has("max_scan_loss_db")) {
    v->max_scan_loss_db = static_cast<float>(j["max_scan_loss_db"].AsDouble());
  }
  LoadRirAzEl(j["boresight_offset_deg"], &v->boresight_offset_deg);
}

inline void LoadRirAntenna(const examples::JsonValue& j, rir_cfg::hardware::RirAntennaConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("main_beam_gain_db")) {
    v->main_beam_gain_db = static_cast<float>(j["main_beam_gain_db"].AsDouble());
  }
  if (j.Has("nominal_az_beamwidth_deg")) {
    v->nominal_az_beamwidth_deg = static_cast<float>(j["nominal_az_beamwidth_deg"].AsDouble());
  }
  if (j.Has("nominal_el_beamwidth_deg")) {
    v->nominal_el_beamwidth_deg = static_cast<float>(j["nominal_el_beamwidth_deg"].AsDouble());
  }
  if (j.Has("antenna_length_m")) {
    v->antenna_length_m = static_cast<float>(j["antenna_length_m"].AsDouble());
  }
  if (j.Has("antenna_width_m")) {
    v->antenna_width_m = static_cast<float>(j["antenna_width_m"].AsDouble());
  }
  LoadRirAntennaPattern(j["pattern"], &v->pattern);
  if (j.Has("enable_directional_pattern")) {
    v->enable_directional_pattern = j["enable_directional_pattern"].AsBool();
  }
}

inline void LoadRirReceiver(const examples::JsonValue& j,
                            rir_cfg::hardware::RirReceiverConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("equipment_id")) {
    v->equipment_id = static_cast<std::uint64_t>(j["equipment_id"].AsInt());
  }
  if (j.Has("noise_figure_db")) {
    v->noise_figure_db = static_cast<float>(j["noise_figure_db"].AsDouble());
  }
  if (j.Has("receive_loss_db")) {
    v->receive_loss_db = static_cast<float>(j["receive_loss_db"].AsDouble());
  }
  if (j.Has("cross_polarization_isolation_db")) {
    v->cross_polarization_isolation_db =
        static_cast<float>(j["cross_polarization_isolation_db"].AsDouble());
  }
  if (j.Has("minimum_far_field_range_m")) {
    v->minimum_far_field_range_m = static_cast<float>(j["minimum_far_field_range_m"].AsDouble());
  }
  if (j.Has("has_co_site_isolation")) {
    v->has_co_site_isolation = j["has_co_site_isolation"].AsBool();
  }
  if (j.Has("co_site_isolation_db")) {
    v->co_site_isolation_db = static_cast<float>(j["co_site_isolation_db"].AsDouble());
  }
  if (j.Has("maximum_linear_input_power_w")) {
    v->maximum_linear_input_power_w =
        static_cast<float>(j["maximum_linear_input_power_w"].AsDouble());
  }
  if (j.Has("preselector_bandwidth_hz")) {
    v->preselector_bandwidth_hz = static_cast<float>(j["preselector_bandwidth_hz"].AsDouble());
  }
  if (j.Has("interference_observation_jn_gate_db")) {
    v->interference_observation_jn_gate_db =
        static_cast<float>(j["interference_observation_jn_gate_db"].AsDouble());
  }
  if (j.Has("scene_polarization")) {
    v->scene_polarization = RfScenePolarizationFromString(j["scene_polarization"].AsString());
  }
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
  if (j.Has("target_processing_gain_db")) {
    v->target_processing_gain_db = static_cast<float>(j["target_processing_gain_db"].AsDouble());
  }
  if (j.Has("noise_processing_gain_db")) {
    v->noise_processing_gain_db = static_cast<float>(j["noise_processing_gain_db"].AsDouble());
  }
  if (j.Has("clutter_suppression_gain_db")) {
    v->clutter_suppression_gain_db =
        static_cast<float>(j["clutter_suppression_gain_db"].AsDouble());
  }
  if (j.Has("jamming_suppression_gain_db")) {
    v->jamming_suppression_gain_db =
        static_cast<float>(j["jamming_suppression_gain_db"].AsDouble());
  }
}

inline void LoadRirRcsPhysics(const examples::JsonValue& j,
                              rir_cfg::hardware::RirRcsPhysicsConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("enable_physical_rcs")) {
    v->enable_physical_rcs = j["enable_physical_rcs"].AsBool();
  }
  if (j.Has("physics_mix_ratio")) {
    v->physics_mix_ratio = static_cast<float>(j["physics_mix_ratio"].AsDouble());
  }
  if (j.Has("cylinder_weight")) {
    v->cylinder_weight = static_cast<float>(j["cylinder_weight"].AsDouble());
  }
  if (j.Has("min_equivalent_radius_m")) {
    v->min_equivalent_radius_m = static_cast<float>(j["min_equivalent_radius_m"].AsDouble());
  }
  if (j.Has("max_equivalent_radius_m")) {
    v->max_equivalent_radius_m = static_cast<float>(j["max_equivalent_radius_m"].AsDouble());
  }
  if (j.Has("min_rcs_m2")) {
    v->min_rcs_m2 = static_cast<float>(j["min_rcs_m2"].AsDouble());
  }
  if (j.Has("max_rcs_m2")) {
    v->max_rcs_m2 = static_cast<float>(j["max_rcs_m2"].AsDouble());
  }
  if (j.Has("bistatic_psi_offset_deg")) {
    v->bistatic_psi_offset_deg = static_cast<float>(j["bistatic_psi_offset_deg"].AsDouble());
  }
}

inline void LoadRirHardware(const examples::JsonValue& j, rir_cfg::RirHardwareConfig* v) {
  if (j.IsNull()) return;
  LoadRirTransmitter(j["transmitter"], &v->transmitter);
  LoadRirAntenna(j["antenna"], &v->antenna);
  LoadRirReceiver(j["receiver"], &v->receiver);
  LoadRirRcsPhysics(j["rcs_physics"], &v->rcs_physics);
  LoadRirSignalProcessing(j["signal_processing"], &v->signal_processing);
}

inline void LoadRirScan(const examples::JsonValue& j, rir_cfg::RirScanConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("scan_start_position")) {
    v->scan_start_position = ScanStartPositionFromString(j["scan_start_position"].AsString());
  }
  if (j.Has("scan_sequence")) {
    v->scan_sequence = ScanSequenceFromString(j["scan_sequence"].AsString());
  }
  if (j.Has("step_scale")) {
    v->step_scale = static_cast<float>(j["step_scale"].AsDouble());
  }
}

inline void LoadRirOrientation(const examples::JsonValue& j, rir_cfg::RirOrientationConfig* v) {
  if (j.IsNull()) return;
  LoadRirAzElLimits(j["steerable_volume_deg"], &v->steerable_volume_deg);
}

inline void LoadRirMission(const examples::JsonValue& j, rir_cfg::RirMissionConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("work_mode")) {
    v->work_mode = RirWorkModeFromString(j["work_mode"].AsString());
  }
  LoadRirAzEl(j["scan_center_deg"], &v->scan_center_deg);
  if (j.Has("max_range_m")) {
    v->max_range_m = static_cast<float>(j["max_range_m"].AsDouble());
  }
  if (j.Has("recognition_dwell_sec")) {
    v->recognition_dwell_sec = static_cast<float>(j["recognition_dwell_sec"].AsDouble());
  }
  LoadRirScan(j["scan"], &v->scan);
}

inline void LoadRirDetectionPolicy(const examples::JsonValue& j,
                                   rir_cfg::RirDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("cfar_pfa")) {
    v->cfar_pfa = static_cast<float>(j["cfar_pfa"].AsDouble());
  }
  if (j.Has("min_snr_db")) {
    v->min_snr_db = static_cast<float>(j["min_snr_db"].AsDouble());
  }
  if (j.Has("min_detection_margin_db")) {
    v->min_detection_margin_db = static_cast<float>(j["min_detection_margin_db"].AsDouble());
  }
  if (j.Has("pulse_count")) {
    v->pulse_count = static_cast<int>(j["pulse_count"].AsInt());
  }
  if (j.Has("random_seed")) {
    v->random_seed = static_cast<std::uint32_t>(j["random_seed"].AsInt());
  }
  if (j.Has("gate_mode")) {
    v->gate_mode = RirDetectionGateModeFromString(j["gate_mode"].AsString());
  }
}

inline void LoadRirAssociationPolicy(const examples::JsonValue& j,
                                     rir_cfg::RirAssociationPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("distance_gate_sigma")) {
    v->distance_gate_sigma = static_cast<float>(j["distance_gate_sigma"].AsDouble());
  }
}

inline void LoadRirTrackingPolicy(const examples::JsonValue& j,
                                  rir_cfg::RirTrackingPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("kalman_noise_diff_coeff")) {
    v->kalman_noise_diff_coeff = static_cast<float>(j["kalman_noise_diff_coeff"].AsDouble());
  }
  if (j.Has("kalman_measurement_noise_std")) {
    v->kalman_measurement_noise_std =
        static_cast<float>(j["kalman_measurement_noise_std"].AsDouble());
  }
}

inline void LoadRirLifecyclePolicy(const examples::JsonValue& j,
                                   rir_cfg::RirLifecyclePolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("confirm_hits")) {
    v->confirm_hits = static_cast<std::uint32_t>(j["confirm_hits"].AsInt());
  }
  if (j.Has("max_miss_before_lost")) {
    v->max_miss_before_lost = static_cast<std::uint32_t>(j["max_miss_before_lost"].AsInt());
  }
  if (j.Has("max_lost_cycles")) {
    v->max_lost_cycles = static_cast<std::uint32_t>(j["max_lost_cycles"].AsInt());
  }
  if (j.Has("enable_imm_lifecycle")) {
    v->enable_imm_lifecycle = j["enable_imm_lifecycle"].AsBool();
  }
  if (j.Has("model_count_hint")) {
    v->model_count_hint = static_cast<std::uint32_t>(j["model_count_hint"].AsInt());
  }
}

inline void LoadRirRecognitionFeatureWeights(const examples::JsonValue& j,
                                             rir_cfg::RirRecognitionFeatureWeights* v) {
  if (j.IsNull()) return;
  if (j.Has("rcs_weight")) {
    v->rcs_weight = static_cast<float>(j["rcs_weight"].AsDouble());
  }
  if (j.Has("motion_weight")) {
    v->motion_weight = static_cast<float>(j["motion_weight"].AsDouble());
  }
  if (j.Has("polarization_weight")) {
    v->polarization_weight = static_cast<float>(j["polarization_weight"].AsDouble());
  }
  if (j.Has("range_profile_weight")) {
    v->range_profile_weight = static_cast<float>(j["range_profile_weight"].AsDouble());
  }
}

inline void LoadRirRecognitionPolicy(const examples::JsonValue& j,
                                     rir_cfg::RirRecognitionPolicy* v) {
  if (j.IsNull()) return;
  if (j.Has("enabled")) {
    v->enabled = j["enabled"].AsBool();
  }
  if (j.Has("min_confirmed_hits")) {
    v->min_confirmed_hits = static_cast<std::uint32_t>(j["min_confirmed_hits"].AsInt());
  }
  if (j.Has("accumulation_window_sec")) {
    v->accumulation_window_sec = static_cast<float>(j["accumulation_window_sec"].AsDouble());
  }
  if (j.Has("min_observation_count")) {
    v->min_observation_count = static_cast<std::uint32_t>(j["min_observation_count"].AsInt());
  }
  if (j.Has("acceptance_score")) {
    v->acceptance_score = static_cast<float>(j["acceptance_score"].AsDouble());
  }
  if (j.Has("minimum_margin")) {
    v->minimum_margin = static_cast<float>(j["minimum_margin"].AsDouble());
  }
  if (j.Has("result_hold_sec")) {
    v->result_hold_sec = static_cast<float>(j["result_hold_sec"].AsDouble());
  }
  LoadRirRecognitionFeatureWeights(j["feature_weights"], &v->feature_weights);
  if (j.Has("database_path")) {
    v->database_path = j["database_path"].AsString();
  }
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
  if (j.Has("cover_profile")) {
    v->cover_profile = RirVegetationCoverProfileFromString(j["cover_profile"].AsString());
  }
  if (j.Has("enable_physical_model")) {
    v->enable_physical_model = j["enable_physical_model"].AsBool();
  }
}

inline void LoadRirAtmosphericPhysics(const examples::JsonValue& j,
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

inline void LoadRirEnvironment(const examples::JsonValue& j, rir_cfg::RirEnvironmentConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("enable_environment_effects")) {
    v->enable_environment_effects = j["enable_environment_effects"].AsBool();
  }
  if (j.Has("weather_attenuation_db")) {
    v->weather_attenuation_db = static_cast<float>(j["weather_attenuation_db"].AsDouble());
  }
  LoadRirVegetationScatterPhysics(j["vegetation_scatter_physics"],
                                  &v->vegetation_scatter_physics);
  LoadRirAtmosphericPhysics(j["atmospheric_physics"], &v->atmospheric_physics);
}

}  // namespace examples

#endif  // EXAMPLES_RIR_CONFIG_LOADER_DETAIL_H_
