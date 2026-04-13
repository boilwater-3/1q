// Copyright 2026. All Rights Reserved.
//
// @file esr_trace_replayer.cpp
// @brief ESR Trace 回放示例：读取 JSONL，驱动 EsrTraceSession 重放并校验一致性。

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "1q/trace/TraceSink.h"
#include "common/trace_replay_common.h"

namespace {

using Json = nlohmann::ordered_json;
namespace esr = electronic_surveillance_radar;
using oneq::examples::replay::TraceRecord;

float GetFloat(const Json& json, const char* key, float default_value = 0.0f) {
  return json.contains(key) ? json[key].get<float>() : default_value;
}

double GetDouble(const Json& json, const char* key, double default_value = 0.0) {
  return json.contains(key) ? json[key].get<double>() : default_value;
}

int GetInt(const Json& json, const char* key, int default_value = 0) {
  return json.contains(key) ? json[key].get<int>() : default_value;
}

bool GetBool(const Json& json, const char* key, bool default_value = false) {
  return json.contains(key) ? json[key].get<bool>() : default_value;
}

oneq::foundation::Vector3f ParseVector3Array(const Json& json) {
  oneq::foundation::Vector3f value;
  if (json.is_array() && json.size() >= 3U) {
    value.x = json[0].get<float>();
    value.y = json[1].get<float>();
    value.z = json[2].get<float>();
  }
  return value;
}

oneq::foundation::EulerAnglesDeg ParseEuler(const Json& json) {
  oneq::foundation::EulerAnglesDeg value;
  value.yaw_deg = GetFloat(json, "yaw_deg", value.yaw_deg);
  value.pitch_deg = GetFloat(json, "pitch_deg", value.pitch_deg);
  value.roll_deg = GetFloat(json, "roll_deg", value.roll_deg);
  return value;
}

esr::environment::EsrAtmosphericPhysicsConfig ParseAtmosphericPhysics(const Json& json) {
  esr::environment::EsrAtmosphericPhysicsConfig value;
  value.enable_physical_model = GetBool(json, "enable_physical_model", value.enable_physical_model);
  value.frequency_hz = GetFloat(json, "frequency_hz", value.frequency_hz);
  value.path_length_m = GetFloat(json, "path_length_m", value.path_length_m);
  value.radar_altitude_m = GetFloat(json, "radar_altitude_m", value.radar_altitude_m);
  value.target_altitude_m = GetFloat(json, "target_altitude_m", value.target_altitude_m);
  value.elevation_deg = GetFloat(json, "elevation_deg", value.elevation_deg);
  value.pressure_hpa = GetFloat(json, "pressure_hpa", value.pressure_hpa);
  value.temperature_k = GetFloat(json, "temperature_k", value.temperature_k);
  value.relative_humidity = GetFloat(json, "relative_humidity", value.relative_humidity);
  value.k_factor = GetFloat(json, "k_factor", value.k_factor);
  value.day_of_year = GetInt(json, "day_of_year", value.day_of_year);
  value.solar_flux_f107a = GetFloat(json, "solar_flux_f107a", value.solar_flux_f107a);
  value.solar_flux_f107 = GetFloat(json, "solar_flux_f107", value.solar_flux_f107);
  value.geomagnetic_ap = GetFloat(json, "geomagnetic_ap", value.geomagnetic_ap);
  return value;
}

esr::session::EsrSessionConfig ParseSessionConfig(const Json& payload) {
  esr::session::EsrSessionConfig config;
  config.enable_layered_config = GetBool(payload, "enable_layered_config", config.enable_layered_config);

  if (payload.contains("layered_config")) {
    const Json& layered = payload["layered_config"];
    if (layered.contains("hardware")) {
      const Json& h = layered["hardware"];
      config.layered_config.hardware.receiver_band_lower_hz =
          GetDouble(h, "receiver_band_lower_hz", config.layered_config.hardware.receiver_band_lower_hz);
      config.layered_config.hardware.receiver_band_upper_hz =
          GetDouble(h, "receiver_band_upper_hz", config.layered_config.hardware.receiver_band_upper_hz);
      config.layered_config.hardware.receiver_sensitivity_w =
          GetFloat(h, "receiver_sensitivity_w", config.layered_config.hardware.receiver_sensitivity_w);
      config.layered_config.hardware.integrated_receive_loss_db =
          GetFloat(h, "integrated_receive_loss_db", config.layered_config.hardware.integrated_receive_loss_db);
      config.layered_config.hardware.beam_az_width_deg =
          GetFloat(h, "beam_az_width_deg", config.layered_config.hardware.beam_az_width_deg);
      config.layered_config.hardware.beam_el_width_deg =
          GetFloat(h, "beam_el_width_deg", config.layered_config.hardware.beam_el_width_deg);
      config.layered_config.hardware.az_scan_range_deg =
          GetFloat(h, "az_scan_range_deg", config.layered_config.hardware.az_scan_range_deg);
      config.layered_config.hardware.el_scan_range_deg =
          GetFloat(h, "el_scan_range_deg", config.layered_config.hardware.el_scan_range_deg);
      config.layered_config.hardware.antenna_mount_az_deg =
          GetFloat(h, "antenna_mount_az_deg", config.layered_config.hardware.antenna_mount_az_deg);
      config.layered_config.hardware.antenna_mount_el_deg =
          GetFloat(h, "antenna_mount_el_deg", config.layered_config.hardware.antenna_mount_el_deg);
    }
    if (layered.contains("mission")) {
      const Json& m = layered["mission"];
      config.layered_config.mission.power_on = GetBool(m, "power_on", config.layered_config.mission.power_on);
      config.layered_config.mission.work_mode =
          static_cast<esr::session::EsrWorkMode>(GetInt(m, "work_mode", static_cast<int>(config.layered_config.mission.work_mode)));
      config.layered_config.mission.scan_center_az_deg =
          GetFloat(m, "scan_center_az_deg", config.layered_config.mission.scan_center_az_deg);
      config.layered_config.mission.scan_center_el_deg =
          GetFloat(m, "scan_center_el_deg", config.layered_config.mission.scan_center_el_deg);
      config.layered_config.mission.scan_rate_hz =
          GetFloat(m, "scan_rate_hz", config.layered_config.mission.scan_rate_hz);
      config.layered_config.mission.scan_start_position =
          static_cast<esr::session::EsrScanStartPosition>(GetInt(m, "scan_start_position", static_cast<int>(config.layered_config.mission.scan_start_position)));
      config.layered_config.mission.scan_sequence =
          static_cast<esr::session::EsrScanSequence>(GetInt(m, "scan_sequence", static_cast<int>(config.layered_config.mission.scan_sequence)));
      config.layered_config.mission.use_explicit_scan_bounds =
          GetBool(m, "use_explicit_scan_bounds", config.layered_config.mission.use_explicit_scan_bounds);
      config.layered_config.mission.scan_start_az_deg =
          GetFloat(m, "scan_start_az_deg", config.layered_config.mission.scan_start_az_deg);
      config.layered_config.mission.scan_end_az_deg =
          GetFloat(m, "scan_end_az_deg", config.layered_config.mission.scan_end_az_deg);
      config.layered_config.mission.scan_start_el_deg =
          GetFloat(m, "scan_start_el_deg", config.layered_config.mission.scan_start_el_deg);
      config.layered_config.mission.scan_end_el_deg =
          GetFloat(m, "scan_end_el_deg", config.layered_config.mission.scan_end_el_deg);
    }
  }

  if (payload.contains("pipeline_config")) {
    const Json& p = payload["pipeline_config"];
    if (p.contains("detection")) {
      const Json& d = p["detection"];
      config.pipeline_config.detection.receiver_noise_floor_w =
          GetFloat(d, "receiver_noise_floor_w", config.pipeline_config.detection.receiver_noise_floor_w);
      config.pipeline_config.detection.min_detect_snr_db =
          GetFloat(d, "min_detect_snr_db", config.pipeline_config.detection.min_detect_snr_db);
      config.pipeline_config.detection.max_detect_range_m =
          GetFloat(d, "max_detect_range_m", config.pipeline_config.detection.max_detect_range_m);
      config.pipeline_config.detection.min_dynamic_range_margin_db =
          GetFloat(d, "min_dynamic_range_margin_db", config.pipeline_config.detection.min_dynamic_range_margin_db);
      config.pipeline_config.detection.boundary_resolution_m =
          GetFloat(d, "boundary_resolution_m", config.pipeline_config.detection.boundary_resolution_m);
      config.pipeline_config.detection.boundary_max_iterations =
          GetInt(d, "boundary_max_iterations", config.pipeline_config.detection.boundary_max_iterations);
    }
    if (p.contains("statistical_detection")) {
      const Json& d = p["statistical_detection"];
      config.pipeline_config.statistical_detection.pfa =
          GetFloat(d, "pfa", config.pipeline_config.statistical_detection.pfa);
      config.pipeline_config.statistical_detection.min_snr_db =
          GetFloat(d, "min_snr_db", config.pipeline_config.statistical_detection.min_snr_db);
      config.pipeline_config.statistical_detection.pulse_count =
          static_cast<std::uint32_t>(GetInt(d, "pulse_count", static_cast<int>(config.pipeline_config.statistical_detection.pulse_count)));
      config.pipeline_config.statistical_detection.integration_mode =
          static_cast<esr::extension::InterceptIntegrationMode>(GetInt(d, "integration_mode", static_cast<int>(config.pipeline_config.statistical_detection.integration_mode)));
      config.pipeline_config.statistical_detection.threshold_scale =
          GetFloat(d, "threshold_scale", config.pipeline_config.statistical_detection.threshold_scale);
      config.pipeline_config.statistical_detection.enable_statistical_detection =
          GetBool(d, "enable_statistical_detection", config.pipeline_config.statistical_detection.enable_statistical_detection);
    }
    if (p.contains("scan")) {
      const Json& s = p["scan"];
      config.pipeline_config.scan.scan_start_az_deg =
          GetFloat(s, "scan_start_az_deg", config.pipeline_config.scan.scan_start_az_deg);
      config.pipeline_config.scan.scan_end_az_deg =
          GetFloat(s, "scan_end_az_deg", config.pipeline_config.scan.scan_end_az_deg);
      config.pipeline_config.scan.scan_start_el_deg =
          GetFloat(s, "scan_start_el_deg", config.pipeline_config.scan.scan_start_el_deg);
      config.pipeline_config.scan.scan_end_el_deg =
          GetFloat(s, "scan_end_el_deg", config.pipeline_config.scan.scan_end_el_deg);
      config.pipeline_config.scan.az_step_deg =
          GetFloat(s, "az_step_deg", config.pipeline_config.scan.az_step_deg);
      config.pipeline_config.scan.el_step_deg =
          GetFloat(s, "el_step_deg", config.pipeline_config.scan.el_step_deg);
      config.pipeline_config.scan.scan_start_pos =
          GetInt(s, "scan_start_pos", config.pipeline_config.scan.scan_start_pos);
      config.pipeline_config.scan.scan_sequence =
          GetInt(s, "scan_sequence", config.pipeline_config.scan.scan_sequence);
    }
    if (p.contains("algorithm")) {
      const Json& a = p["algorithm"];
      config.pipeline_config.algorithm.random_seed =
          static_cast<unsigned int>(GetInt(a, "random_seed", static_cast<int>(config.pipeline_config.algorithm.random_seed)));
      config.pipeline_config.algorithm.angle_error_coefficient =
          GetFloat(a, "angle_error_coefficient", config.pipeline_config.algorithm.angle_error_coefficient);
    }
    if (p.contains("preprocess")) {
      const Json& pr = p["preprocess"];
      config.pipeline_config.preprocess.dedup_time_window_sec =
          GetFloat(pr, "dedup_time_window_sec", config.pipeline_config.preprocess.dedup_time_window_sec);
      config.pipeline_config.preprocess.dedup_rf_window_hz =
          GetDouble(pr, "dedup_rf_window_hz", config.pipeline_config.preprocess.dedup_rf_window_hz);
      config.pipeline_config.preprocess.dedup_pw_window_sec =
          GetDouble(pr, "dedup_pw_window_sec", config.pipeline_config.preprocess.dedup_pw_window_sec);
      config.pipeline_config.preprocess.dedup_az_window_deg =
          GetFloat(pr, "dedup_az_window_deg", config.pipeline_config.preprocess.dedup_az_window_deg);
      config.pipeline_config.preprocess.dedup_el_window_deg =
          GetFloat(pr, "dedup_el_window_deg", config.pipeline_config.preprocess.dedup_el_window_deg);
      config.pipeline_config.preprocess.normalize_quality =
          GetBool(pr, "normalize_quality", config.pipeline_config.preprocess.normalize_quality);
    }
    if (p.contains("cluster")) {
      const Json& c = p["cluster"];
      config.pipeline_config.cluster.radius = GetFloat(c, "radius", config.pipeline_config.cluster.radius);
      config.pipeline_config.cluster.min_points =
          static_cast<std::uint32_t>(GetInt(c, "min_points", static_cast<int>(config.pipeline_config.cluster.min_points)));
      config.pipeline_config.cluster.rf_scale_hz = GetFloat(c, "rf_scale_hz", config.pipeline_config.cluster.rf_scale_hz);
      config.pipeline_config.cluster.pw_scale_sec = GetFloat(c, "pw_scale_sec", config.pipeline_config.cluster.pw_scale_sec);
      config.pipeline_config.cluster.az_scale_deg = GetFloat(c, "az_scale_deg", config.pipeline_config.cluster.az_scale_deg);
      config.pipeline_config.cluster.el_scale_deg = GetFloat(c, "el_scale_deg", config.pipeline_config.cluster.el_scale_deg);
      config.pipeline_config.cluster.snr_scale_db = GetFloat(c, "snr_scale_db", config.pipeline_config.cluster.snr_scale_db);
    }
    if (p.contains("association")) {
      const Json& a = p["association"];
      config.pipeline_config.association.gate_distance =
          GetFloat(a, "gate_distance", config.pipeline_config.association.gate_distance);
      config.pipeline_config.association.confirm_hits =
          static_cast<std::uint32_t>(GetInt(a, "confirm_hits", static_cast<int>(config.pipeline_config.association.confirm_hits)));
      config.pipeline_config.association.max_missed_cycles =
          static_cast<std::uint32_t>(GetInt(a, "max_missed_cycles", static_cast<int>(config.pipeline_config.association.max_missed_cycles)));
      config.pipeline_config.association.confidence_alpha =
          GetFloat(a, "confidence_alpha", config.pipeline_config.association.confidence_alpha);
      config.pipeline_config.association.output_tentative =
          GetBool(a, "output_tentative", config.pipeline_config.association.output_tentative);
    }
    if (p.contains("suppression_model")) {
      const Json& s = p["suppression_model"];
      config.pipeline_config.suppression_model.suppression_noise_scale =
          GetFloat(s, "suppression_noise_scale", config.pipeline_config.suppression_model.suppression_noise_scale);
      config.pipeline_config.suppression_model.suppression_mark_threshold_w =
          GetFloat(s, "suppression_mark_threshold_w", config.pipeline_config.suppression_model.suppression_mark_threshold_w);
    }
    if (p.contains("deception_model")) {
      const Json& d = p["deception_model"];
      config.pipeline_config.deception_model.false_alarm_probability_scale =
          GetFloat(d, "false_alarm_probability_scale", config.pipeline_config.deception_model.false_alarm_probability_scale);
      config.pipeline_config.deception_model.confusion_probability_scale =
          GetFloat(d, "confusion_probability_scale", config.pipeline_config.deception_model.confusion_probability_scale);
      config.pipeline_config.deception_model.max_false_observations_per_emitter =
          static_cast<std::uint32_t>(GetInt(d, "max_false_observations_per_emitter", static_cast<int>(config.pipeline_config.deception_model.max_false_observations_per_emitter)));
      config.pipeline_config.deception_model.aoa_confusion_std_deg =
          GetFloat(d, "aoa_confusion_std_deg", config.pipeline_config.deception_model.aoa_confusion_std_deg);
      config.pipeline_config.deception_model.rf_confusion_ratio =
          GetFloat(d, "rf_confusion_ratio", config.pipeline_config.deception_model.rf_confusion_ratio);
      config.pipeline_config.deception_model.pw_confusion_ratio =
          GetFloat(d, "pw_confusion_ratio", config.pipeline_config.deception_model.pw_confusion_ratio);
      config.pipeline_config.deception_model.cluster_confidence_penalty_scale =
          GetFloat(d, "cluster_confidence_penalty_scale", config.pipeline_config.deception_model.cluster_confidence_penalty_scale);
    }
  }

  if (payload.contains("environment_default_config")) {
    const Json& env = payload["environment_default_config"];
    if (env.contains("model_config")) {
      const Json& m = env["model_config"];
      config.environment_default_config.model_config.default_clutter_noise_w =
          GetFloat(m, "default_clutter_noise_w", config.environment_default_config.model_config.default_clutter_noise_w);
      config.environment_default_config.model_config.jamming_detection_threshold_w =
          GetFloat(m, "jamming_detection_threshold_w", config.environment_default_config.model_config.jamming_detection_threshold_w);
      if (m.contains("atmospheric_physics")) {
        config.environment_default_config.model_config.atmospheric_physics =
            ParseAtmosphericPhysics(m["atmospheric_physics"]);
      }
    }
  }

  return config;
}

esr::session::EsrCycleInput ParseCycleInput(const Json& payload) {
  esr::session::EsrCycleInput input;
  input.cycle_index = payload.value("cycle_index", input.cycle_index);
  input.dt_sec = GetFloat(payload, "dt_sec", input.dt_sec);

  if (payload.contains("platform_pose")) {
    const Json& p = payload["platform_pose"];
    if (p.contains("position_m")) {
      input.platform_pose.position_m = ParseVector3Array(p["position_m"]);
    }
    if (p.contains("velocity_mps")) {
      input.platform_pose.velocity_mps = ParseVector3Array(p["velocity_mps"]);
    }
    if (p.contains("attitude_deg")) {
      input.platform_pose.attitude_deg = ParseEuler(p["attitude_deg"]);
    }
  }

  if (payload.contains("scene_emitters") && payload["scene_emitters"].is_array()) {
    input.scene_emitters.clear();
    for (std::size_t i = 0; i < payload["scene_emitters"].size(); ++i) {
      const Json& e = payload["scene_emitters"][i];
      esr::model::EmitterTruthState emitter;
      emitter.emitter_id = e.value("emitter_id", std::string());
      if (e.contains("pose")) {
        const Json& p = e["pose"];
        if (p.contains("position_m")) {
          emitter.pose.position_m = ParseVector3Array(p["position_m"]);
        }
        if (p.contains("velocity_mps")) {
          emitter.pose.velocity_mps = ParseVector3Array(p["velocity_mps"]);
        }
        if (p.contains("attitude_deg")) {
          emitter.pose.attitude_deg = ParseEuler(p["attitude_deg"]);
        }
      }
      emitter.carrier_hz = GetDouble(e, "carrier_hz", emitter.carrier_hz);
      emitter.bandwidth_hz = GetDouble(e, "bandwidth_hz", emitter.bandwidth_hz);
      emitter.tx_power_w = GetDouble(e, "tx_power_w", emitter.tx_power_w);
      emitter.pulse_width_s = GetDouble(e, "pulse_width_s", emitter.pulse_width_s);
      emitter.pri_s = GetDouble(e, "pri_s", emitter.pri_s);
      if (e.contains("beam_state")) {
        const Json& b = e["beam_state"];
        emitter.beam_state.center_az_deg = GetFloat(b, "center_az_deg", emitter.beam_state.center_az_deg);
        emitter.beam_state.center_el_deg = GetFloat(b, "center_el_deg", emitter.beam_state.center_el_deg);
        emitter.beam_state.az_beamwidth_deg =
            GetFloat(b, "az_beamwidth_deg", emitter.beam_state.az_beamwidth_deg);
        emitter.beam_state.el_beamwidth_deg =
            GetFloat(b, "el_beamwidth_deg", emitter.beam_state.el_beamwidth_deg);
        emitter.beam_state.beam_state_valid =
            GetBool(b, "beam_state_valid", emitter.beam_state.beam_state_valid);
      }
      emitter.is_emitting = GetBool(e, "is_emitting", emitter.is_emitting);
      input.scene_emitters.push_back(emitter);
    }
  }

  if (payload.contains("environment_scene_state")) {
    const Json& env = payload["environment_scene_state"];
    input.environment_scene_state.base_propagation_loss_db =
        GetFloat(env, "base_propagation_loss_db", input.environment_scene_state.base_propagation_loss_db);
    input.environment_scene_state.atmospheric_attenuation_db =
        GetFloat(env, "atmospheric_attenuation_db", input.environment_scene_state.atmospheric_attenuation_db);
    input.environment_scene_state.terrain_reflection_db =
        GetFloat(env, "terrain_reflection_db", input.environment_scene_state.terrain_reflection_db);
    input.environment_scene_state.clutter_noise_w =
        GetFloat(env, "clutter_noise_w", input.environment_scene_state.clutter_noise_w);
    input.environment_scene_state.spectrum_occupancy_ratio =
        GetFloat(env, "spectrum_occupancy_ratio", input.environment_scene_state.spectrum_occupancy_ratio);
    if (env.contains("atmospheric_physics")) {
      input.environment_scene_state.atmospheric_physics = ParseAtmosphericPhysics(env["atmospheric_physics"]);
    }
    if (env.contains("jammer_sources") && env["jammer_sources"].is_array()) {
      input.environment_scene_state.jammer_sources.clear();
      for (std::size_t i = 0; i < env["jammer_sources"].size(); ++i) {
        const Json& j = env["jammer_sources"][i];
        esr::environment::EsrJammerSource jammer;
        jammer.technique =
            static_cast<esr::environment::EsrJammingTechnique>(GetInt(j, "technique", static_cast<int>(jammer.technique)));
        jammer.active = GetBool(j, "active", jammer.active);
        jammer.center_hz = GetDouble(j, "center_hz", jammer.center_hz);
        jammer.bandwidth_hz = GetDouble(j, "bandwidth_hz", jammer.bandwidth_hz);
        jammer.power_w = GetFloat(j, "power_w", jammer.power_w);
        jammer.deception_risk = GetFloat(j, "deception_risk", jammer.deception_risk);
        jammer.confidence = GetFloat(j, "confidence", jammer.confidence);
        input.environment_scene_state.jammer_sources.push_back(jammer);
      }
    }
  }

  return input;
}

esr::session::EsrRuntimeConfigPatch ParseRuntimePatch(const Json& payload) {
  esr::session::EsrRuntimeConfigPatch patch;
  patch.has_sensor_enabled = GetBool(payload, "has_sensor_enabled", false);
  patch.sensor_enabled = GetBool(payload, "sensor_enabled", patch.sensor_enabled);
  patch.has_scan_rate_hz = GetBool(payload, "has_scan_rate_hz", false);
  patch.scan_rate_hz = GetFloat(payload, "scan_rate_hz", patch.scan_rate_hz);
  patch.has_integrated_receive_loss_db = GetBool(payload, "has_integrated_receive_loss_db", false);
  patch.integrated_receive_loss_db =
      GetFloat(payload, "integrated_receive_loss_db", patch.integrated_receive_loss_db);
  patch.has_fixed_receiver_window_hz = GetBool(payload, "has_fixed_receiver_window_hz", false);
  patch.receiver_lower_hz = GetDouble(payload, "receiver_lower_hz", patch.receiver_lower_hz);
  patch.receiver_upper_hz = GetDouble(payload, "receiver_upper_hz", patch.receiver_upper_hz);
  patch.has_use_fixed_receiver_window = GetBool(payload, "has_use_fixed_receiver_window", false);
  patch.use_fixed_receiver_window =
      GetBool(payload, "use_fixed_receiver_window", patch.use_fixed_receiver_window);
  patch.has_enable_statistical_detection = GetBool(payload, "has_enable_statistical_detection", false);
  patch.enable_statistical_detection =
      GetBool(payload, "enable_statistical_detection", patch.enable_statistical_detection);
  patch.has_enable_spectral_analysis = GetBool(payload, "has_enable_spectral_analysis", false);
  patch.enable_spectral_analysis =
      GetBool(payload, "enable_spectral_analysis", patch.enable_spectral_analysis);
  patch.has_detection_min_snr_db = GetBool(payload, "has_detection_min_snr_db", false);
  patch.detection_min_snr_db = GetFloat(payload, "detection_min_snr_db", patch.detection_min_snr_db);
  patch.has_observation_jam_mark_threshold_w =
      GetBool(payload, "has_observation_jam_mark_threshold_w", false);
  patch.observation_jam_mark_threshold_w =
      GetFloat(payload, "observation_jam_mark_threshold_w", patch.observation_jam_mark_threshold_w);

  patch.has_environment_runtime_config = GetBool(payload, "has_environment_runtime_config", false);
  if (patch.has_environment_runtime_config && payload.contains("environment_runtime_config")) {
    const Json& env = payload["environment_runtime_config"];
    patch.environment_runtime_config.has_model_config = GetBool(env, "has_model_config", false);
    if (patch.environment_runtime_config.has_model_config && env.contains("model_config")) {
      const Json& m = env["model_config"];
      patch.environment_runtime_config.model_config.default_clutter_noise_w =
          GetFloat(m, "default_clutter_noise_w", patch.environment_runtime_config.model_config.default_clutter_noise_w);
      patch.environment_runtime_config.model_config.jamming_detection_threshold_w =
          GetFloat(m, "jamming_detection_threshold_w", patch.environment_runtime_config.model_config.jamming_detection_threshold_w);
      if (m.contains("atmospheric_physics")) {
        patch.environment_runtime_config.model_config.atmospheric_physics =
            ParseAtmosphericPhysics(m["atmospheric_physics"]);
      }
    }
    patch.environment_runtime_config.has_jamming_detection_threshold_w =
        GetBool(env, "has_jamming_detection_threshold_w", false);
    patch.environment_runtime_config.jamming_detection_threshold_w =
        GetFloat(env, "jamming_detection_threshold_w", patch.environment_runtime_config.jamming_detection_threshold_w);
  }
  return patch;
}

std::string GetDefaultReplayPath(const char* argv0) {
  const std::string executable_path = (argv0 != nullptr) ? argv0 : "";
  const std::size_t last_sep = executable_path.find_last_of("/\\");
  const std::string executable_dir =
      (last_sep == std::string::npos) ? "." : executable_path.substr(0, last_sep);
  return executable_dir + "/1q-esr-trace-replay.jsonl";
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "usage: esr_trace_replayer <trace_jsonl_path> [replay_output_jsonl]" << std::endl;
    return 2;
  }

  const std::string trace_path = argv[1];
  const std::string replay_path = (argc >= 3) ? argv[2] : GetDefaultReplayPath(argv[0]);

  std::vector<TraceRecord> expected;
  std::string error_message;
  if (!oneq::examples::replay::LoadTraceRecords(trace_path, "electronic_surveillance_radar", &expected,
                                                 &error_message)) {
    std::cerr << "load trace failed: " << error_message << std::endl;
    return 1;
  }
  if (expected.empty()) {
    std::cerr << "no electronic_surveillance_radar records in trace: " << trace_path << std::endl;
    return 1;
  }

  esr::session::EsrSessionConfig config;
  bool has_config = false;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (expected[i].phase == "config") {
      config = ParseSessionConfig(expected[i].payload);
      has_config = true;
      break;
    }
  }
  if (!has_config) {
    std::cerr << "trace missing config phase" << std::endl;
    return 1;
  }

  std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::JsonlFileTraceSink(replay_path, false));
  esr::session::EsrTraceSession session(config, esr::session::EsrTraceSessionOptions{sink, true});

  for (std::size_t i = 0; i < expected.size(); ++i) {
    const TraceRecord& record = expected[i];
    if (record.phase == "input") {
      const esr::session::EsrCycleInput input = ParseCycleInput(record.payload);
      (void)session.StepWithResult(input);
    } else if (record.phase == "runtime_config_patch") {
      const esr::session::EsrRuntimeConfigPatch patch = ParseRuntimePatch(record.payload);
      session.ApplyRuntimeConfig(patch);
    }
  }

  std::vector<TraceRecord> actual;
  if (!oneq::examples::replay::LoadTraceRecords(replay_path, "electronic_surveillance_radar", &actual,
                                                 &error_message)) {
    std::cerr << "load replay trace failed: " << error_message << std::endl;
    return 1;
  }

  std::string diff_message;
  const bool ok = oneq::examples::replay::CompareTraceRecords(expected, actual, &diff_message);
  std::cout << "trace_in=" << trace_path << " replay_out=" << replay_path
            << " match=" << (ok ? "true" : "false") << std::endl;
  if (!ok) {
    std::cerr << "diff: " << diff_message << std::endl;
    return 1;
  }
  return 0;
}
