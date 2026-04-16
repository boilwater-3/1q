// Copyright 2026. All Rights Reserved.
//
// @file ar_trace_replayer.cpp
// @brief AR Trace 回放示例：读取 JSONL，驱动 RadarTraceSession 重放并校验一致性。

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/session/RadarTraceSession.h"
#include "1q/trace/TraceSink.h"
#include "common/trace_replay_common.h"

namespace {

using Json = nlohmann::ordered_json;
namespace ar = airborne_radar;
using oneq::examples::replay::TraceRecord;

float GetFloat(const Json& json, const char* key, float default_value = 0.0f) {
  return json.contains(key) ? json[key].get<float>() : default_value;
}

int GetInt(const Json& json, const char* key, int default_value = 0) {
  return json.contains(key) ? json[key].get<int>() : default_value;
}

bool GetBool(const Json& json, const char* key, bool default_value = false) {
  return json.contains(key) ? json[key].get<bool>() : default_value;
}

oneq::foundation::Vector3f ParseVector3(const Json& json) {
  oneq::foundation::Vector3f value;
  value.x = GetFloat(json, "x", 0.0f);
  value.y = GetFloat(json, "y", 0.0f);
  value.z = GetFloat(json, "z", 0.0f);
  return value;
}

oneq::foundation::EulerAnglesDeg ParseFoundationEuler(const Json& json) {
  oneq::foundation::EulerAnglesDeg value;
  value.yaw_deg = GetFloat(json, "yaw_deg", 0.0f);
  value.pitch_deg = GetFloat(json, "pitch_deg", 0.0f);
  value.roll_deg = GetFloat(json, "roll_deg", 0.0f);
  return value;
}

ar::model::EulerAnglesDeg ParseEuler(const Json& json) {
  ar::model::EulerAnglesDeg value;
  value.yaw_deg = GetFloat(json, "yaw_deg", 0.0f);
  value.pitch_deg = GetFloat(json, "pitch_deg", 0.0f);
  value.roll_deg = GetFloat(json, "roll_deg", 0.0f);
  return value;
}

ar::model::AzimuthElevationDeg ParseAzEl(const Json& json) {
  ar::model::AzimuthElevationDeg value;
  value.az_deg = GetFloat(json, "az_deg", 0.0f);
  value.el_deg = GetFloat(json, "el_deg", 0.0f);
  return value;
}

ar::model::AzimuthElevationLimitsDeg ParseAzElLimits(const Json& json) {
  ar::model::AzimuthElevationLimitsDeg value;
  value.az_min_deg = GetFloat(json, "az_min_deg", value.az_min_deg);
  value.az_max_deg = GetFloat(json, "az_max_deg", value.az_max_deg);
  value.el_min_deg = GetFloat(json, "el_min_deg", value.el_min_deg);
  value.el_max_deg = GetFloat(json, "el_max_deg", value.el_max_deg);
  return value;
}

oneq::foundation::PoseState ParsePose(const Json& json) {
  oneq::foundation::PoseState pose;
  if (json.contains("position_m")) {
    pose.position_m = ParseVector3(json["position_m"]);
  }
  if (json.contains("velocity_mps")) {
    pose.velocity_mps = ParseVector3(json["velocity_mps"]);
  }
  if (json.contains("attitude_deg")) {
    pose.attitude_deg = ParseFoundationEuler(json["attitude_deg"]);
  }
  return pose;
}

ar::config::SignalDetectionConfig ParseDetection(const Json& json) {
  ar::config::SignalDetectionConfig config;
  config.enable_physics_detection = GetBool(json, "enable_physics_detection", config.enable_physics_detection);
  config.min_detection_margin_db = GetFloat(json, "min_detection_margin_db", config.min_detection_margin_db);
  config.pulse_count = GetInt(json, "pulse_count", config.pulse_count);

  if (json.contains("radar_system")) {
    const Json& radar_system = json["radar_system"];
    if (radar_system.contains("transmitter")) {
      const Json& t = radar_system["transmitter"];
      config.transmitter.peak_power_w = GetFloat(t, "peak_power_w", config.transmitter.peak_power_w);
      config.transmitter.frequency_hz = GetFloat(t, "frequency_hz", config.transmitter.frequency_hz);
      config.transmitter.bandwidth_hz = GetFloat(t, "bandwidth_hz", config.transmitter.bandwidth_hz);
      config.transmitter.pulse_width_s = GetFloat(t, "pulse_width_s", config.transmitter.pulse_width_s);
      config.transmitter.prf_hz = GetFloat(t, "prf_hz", config.transmitter.prf_hz);
      config.transmitter.transmit_loss_db = GetFloat(t, "transmit_loss_db", config.transmitter.transmit_loss_db);
    }
    if (radar_system.contains("antenna")) {
      const Json& a = radar_system["antenna"];
      config.antenna.main_beam_gain_db = GetFloat(a, "main_beam_gain_db", config.antenna.main_beam_gain_db);
      config.antenna.nominal_az_beamwidth_deg =
          GetFloat(a, "nominal_az_beamwidth_deg", config.antenna.nominal_az_beamwidth_deg);
      config.antenna.nominal_el_beamwidth_deg =
          GetFloat(a, "nominal_el_beamwidth_deg", config.antenna.nominal_el_beamwidth_deg);
      config.antenna.enable_directional_pattern =
          GetBool(a, "enable_directional_pattern", config.antenna.enable_directional_pattern);
      if (a.contains("pattern")) {
        const Json& p = a["pattern"];
        config.antenna.pattern.model_type =
            static_cast<ar::config::AntennaPatternModelType>(GetInt(p, "model_type", static_cast<int>(config.antenna.pattern.model_type)));
        config.antenna.pattern.max_sidelobe_level_db =
            GetFloat(p, "max_sidelobe_level_db", config.antenna.pattern.max_sidelobe_level_db);
        config.antenna.pattern.backlobe_level_db =
            GetFloat(p, "backlobe_level_db", config.antenna.pattern.backlobe_level_db);
        config.antenna.pattern.scan_loss_coeff_db_per_deg2 =
            GetFloat(p, "scan_loss_coeff_db_per_deg2", config.antenna.pattern.scan_loss_coeff_db_per_deg2);
        config.antenna.pattern.max_scan_loss_db =
            GetFloat(p, "max_scan_loss_db", config.antenna.pattern.max_scan_loss_db);
        if (p.contains("boresight_offset_deg")) {
          config.antenna.pattern.boresight_offset_deg = ParseAzEl(p["boresight_offset_deg"]);
        }
      }
    }
    if (radar_system.contains("receiver")) {
      const Json& r = radar_system["receiver"];
      config.receiver.noise_figure_db = GetFloat(r, "noise_figure_db", config.receiver.noise_figure_db);
      config.receiver.receive_loss_db = GetFloat(r, "receive_loss_db", config.receiver.receive_loss_db);
    }
    if (radar_system.contains("detection")) {
      const Json& d = radar_system["detection"];
      config.detection_policy.cfar_pfa = GetFloat(d, "cfar_pfa", config.detection_policy.cfar_pfa);
      config.detection_policy.min_snr_db = GetFloat(d, "min_snr_db", config.detection_policy.min_snr_db);
    }
  }
  return config;
}

ar::config::SignalBeamControlConfig ParseBeamControl(const Json& json) {
  ar::config::SignalBeamControlConfig config;
  if (json.contains("platform_attitude_deg")) {
    config.platform_attitude_deg = ParseEuler(json["platform_attitude_deg"]);
  }
  if (json.contains("radar_orientation")) {
    const Json& r = json["radar_orientation"];
    if (r.contains("mount_angles_deg")) {
      config.radar_orientation.mount_angles_deg = ParseEuler(r["mount_angles_deg"]);
    }
    if (r.contains("scan_center_deg")) {
      config.radar_orientation.scan_center_deg = ParseAzEl(r["scan_center_deg"]);
    }
    if (r.contains("mechanical_scan_limits_deg")) {
      config.radar_orientation.mechanical_scan_limits_deg = ParseAzElLimits(r["mechanical_scan_limits_deg"]);
    }
    if (r.contains("electronic_scan_limits_deg")) {
      config.radar_orientation.electronic_scan_limits_deg = ParseAzElLimits(r["electronic_scan_limits_deg"]);
    }
    config.radar_orientation.scan_start_position =
        static_cast<oneq::foundation::ScanStartPosition>(GetInt(r, "scan_start_position", static_cast<int>(config.radar_orientation.scan_start_position)));
    config.radar_orientation.scan_sequence =
        static_cast<oneq::foundation::ScanSequence>(GetInt(r, "scan_sequence", static_cast<int>(config.radar_orientation.scan_sequence)));
    config.radar_orientation.work_sub_mode =
        static_cast<ar::model::RadarWorkSubMode>(GetInt(r, "work_sub_mode", static_cast<int>(config.radar_orientation.work_sub_mode)));
    if (r.contains("dwell_center_deg")) {
      config.radar_orientation.dwell_center_deg = ParseAzEl(r["dwell_center_deg"]);
    }
    config.radar_orientation.commanded_beamwidth_enabled =
        GetBool(r, "commanded_beamwidth_enabled", config.radar_orientation.commanded_beamwidth_enabled);
    if (r.contains("commanded_beamwidth_deg")) {
      const Json& c = r["commanded_beamwidth_deg"];
      config.radar_orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg =
          GetFloat(c, "commanded_az_beamwidth_deg", config.radar_orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg);
      config.radar_orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg =
          GetFloat(c, "commanded_el_beamwidth_deg", config.radar_orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg);
    }
    config.radar_orientation.stabilization_mode =
        static_cast<ar::model::StabilizationMode>(GetInt(r, "stabilization_mode", static_cast<int>(config.radar_orientation.stabilization_mode)));
  }
  return config;
}

ar::environment::AtmosphericPhysicsConfig ParseAtmosphericPhysics(const Json& json) {
  ar::environment::AtmosphericPhysicsConfig value;
  value.enable_physical_model = GetBool(json, "enable_physical_model", value.enable_physical_model);
  value.pressure_hpa = GetFloat(json, "pressure_hpa", value.pressure_hpa);
  value.temperature_k = GetFloat(json, "temperature_k", value.temperature_k);
  value.relative_humidity = GetFloat(json, "relative_humidity", value.relative_humidity);
  return value;
}

ar::environment::AtmosphericDerivedContext ParseAtmosphericContext(const Json& json) {
  ar::environment::AtmosphericDerivedContext value;
  value.k_factor = GetFloat(json, "k_factor", value.k_factor);
  value.day_of_year = GetInt(json, "day_of_year", value.day_of_year);
  value.solar_flux_f107a = GetFloat(json, "solar_flux_f107a", value.solar_flux_f107a);
  value.solar_flux_f107 = GetFloat(json, "solar_flux_f107", value.solar_flux_f107);
  value.geomagnetic_ap = GetFloat(json, "geomagnetic_ap", value.geomagnetic_ap);
  return value;
}

ar::environment::JammerEmitterState ParseJammerEmitter(const Json& json) {
  ar::environment::JammerEmitterState value;
  value.technique = static_cast<ar::environment::JammingTechnique>(GetInt(json, "technique", 0));
  value.power_db = GetFloat(json, "power_db", value.power_db);
  value.js_db = GetFloat(json, "js_db", value.js_db);
  value.has_direction_deg = GetBool(json, "has_direction_deg", value.has_direction_deg);
  value.azimuth_deg = GetFloat(json, "azimuth_deg", value.azimuth_deg);
  value.elevation_deg = GetFloat(json, "elevation_deg", value.elevation_deg);
  value.angular_span_deg = GetFloat(json, "angular_span_deg", value.angular_span_deg);
  value.confidence = GetFloat(json, "confidence", value.confidence);
  return value;
}

ar::environment::EnvironmentScenarioConfig ParseEnvironmentScenario(const Json& json) {
  ar::environment::EnvironmentScenarioConfig scenario;
  if (json.contains("atmospheric_physics")) {
    scenario.atmospheric_physics = ParseAtmosphericPhysics(json["atmospheric_physics"]);
  }
  if (json.contains("atmospheric_context")) {
    scenario.atmospheric_context = ParseAtmosphericContext(json["atmospheric_context"]);
  } else if (json.contains("atmospheric_physics")) {
    // Backward compatibility: older traces stored derived context together with atmospheric_physics.
    scenario.atmospheric_context = ParseAtmosphericContext(json["atmospheric_physics"]);
  }
  if (json.contains("vegetation_scatter_physics")) {
    const Json& veg = json["vegetation_scatter_physics"];
    scenario.vegetation_scatter_physics.enable_physical_model =
        GetBool(veg, "enable_physical_model", scenario.vegetation_scatter_physics.enable_physical_model);
    scenario.vegetation_scatter_physics.leaf_size_m =
        GetFloat(veg, "leaf_size_m", scenario.vegetation_scatter_physics.leaf_size_m);
    scenario.vegetation_scatter_physics.dielectric_constant_real =
        GetFloat(veg, "dielectric_constant_real",
                 scenario.vegetation_scatter_physics.dielectric_constant_real);
    scenario.vegetation_scatter_physics.leaf_count = static_cast<std::uint32_t>(
        GetInt(veg, "leaf_count", static_cast<int>(scenario.vegetation_scatter_physics.leaf_count)));
    scenario.vegetation_scatter_physics.canopy_radius_m =
        GetFloat(veg, "canopy_radius_m", scenario.vegetation_scatter_physics.canopy_radius_m);
    scenario.vegetation_scatter_physics.canopy_height_m =
        GetFloat(veg, "canopy_height_m", scenario.vegetation_scatter_physics.canopy_height_m);
  }
  if (json.contains("jammer_sources") && json["jammer_sources"].is_array()) {
    scenario.jammer_sources.clear();
    for (std::size_t i = 0; i < json["jammer_sources"].size(); ++i) {
      scenario.jammer_sources.push_back(ParseJammerEmitter(json["jammer_sources"][i]));
    }
  }
  return scenario;
}

ar::session::RadarSessionConfig ParseSessionConfig(const Json& payload) {
  ar::session::RadarSessionConfig config;
  if (payload.contains("signal_pipeline_config")) {
    const Json& pipeline = payload["signal_pipeline_config"];
    if (pipeline.contains("detection")) {
      config.detection = ParseDetection(pipeline["detection"]);
    }
    if (pipeline.contains("beam_control")) {
      config.beam_control = ParseBeamControl(pipeline["beam_control"]);
    }
    if (pipeline.contains("tracking")) {
      const Json& t = pipeline["tracking"];
      config.tracking.enable_kalman_filter =
          GetBool(t, "enable_kalman_filter", config.tracking.enable_kalman_filter);
      config.tracking.kalman_measurement_noise_std =
          GetFloat(t, "kalman_measurement_noise_std", config.tracking.kalman_measurement_noise_std);
    }
    if (pipeline.contains("lifecycle")) {
      const Json& l = pipeline["lifecycle"];
      if (l.contains("lifecycle_config")) {
        const Json& lc = l["lifecycle_config"];
        config.lifecycle.lifecycle_config.confirm_hits =
            static_cast<std::uint32_t>(GetInt(lc, "confirm_hits", static_cast<int>(config.lifecycle.lifecycle_config.confirm_hits)));
        config.lifecycle.lifecycle_config.max_miss_before_lost =
            static_cast<std::uint32_t>(GetInt(lc, "max_miss_before_lost", static_cast<int>(config.lifecycle.lifecycle_config.max_miss_before_lost)));
        config.lifecycle.lifecycle_config.max_lost_cycles =
            static_cast<std::uint32_t>(GetInt(lc, "max_lost_cycles", static_cast<int>(config.lifecycle.lifecycle_config.max_lost_cycles)));
      }
      config.lifecycle.enable_imm_lifecycle =
          GetBool(l, "enable_imm_lifecycle", config.lifecycle.enable_imm_lifecycle);
    }
  }

  if (payload.contains("environment_default_config")) {
    const Json& env = payload["environment_default_config"];
    config.environment_default_config.jamming_detection_threshold_db =
        GetFloat(env, "jamming_detection_threshold_db", config.environment_default_config.jamming_detection_threshold_db);
    if (env.contains("scenario_config")) {
      config.environment_default_config.scenario_config =
          ParseEnvironmentScenario(env["scenario_config"]);
    }
  }
  return config;
}

ar::model::TargetFeature ParseTargetFeature(const Json& json) {
  ar::model::TargetFeature feature;
  feature.external_target_id = json.value("external_target_id", static_cast<std::uint64_t>(0));
  if (json.contains("velocity_mps") && json["velocity_mps"].is_array() && json["velocity_mps"].size() >= 3U) {
    feature.current_track_velocity_x = json["velocity_mps"][0].get<float>();
    feature.current_track_velocity_y = json["velocity_mps"][1].get<float>();
    feature.current_track_velocity_z = json["velocity_mps"][2].get<float>();
  }
  feature.current_track_speed = GetFloat(json, "current_track_speed", feature.current_track_speed);
  feature.current_track_rcs = GetFloat(json, "current_track_rcs", feature.current_track_rcs);
  feature.range_m = GetFloat(json, "range_m", feature.range_m);
  feature.has_cartesian_position = GetBool(json, "has_cartesian_position", feature.has_cartesian_position);
  if (json.contains("position_m") && json["position_m"].is_array() && json["position_m"].size() >= 3U) {
    feature.position_x = json["position_m"][0].get<float>();
    feature.position_y = json["position_m"][1].get<float>();
    feature.position_z = json["position_m"][2].get<float>();
  }
  feature.target_swerling_type = GetInt(json, "target_swerling_type", feature.target_swerling_type);
  return feature;
}

ar::session::RadarCycleInput ParseCycleInput(const Json& payload) {
  ar::session::RadarCycleInput input;
  input.dt_sec = GetFloat(payload, "dt_sec", input.dt_sec);
  if (payload.contains("platform_pose")) {
    input.platform_pose = ParsePose(payload["platform_pose"]);
  }
  if (payload.contains("target_features") && payload["target_features"].is_array()) {
    input.target_features.clear();
    for (std::size_t i = 0; i < payload["target_features"].size(); ++i) {
      input.target_features.push_back(ParseTargetFeature(payload["target_features"][i]));
    }
  }
  return input;
}

ar::environment::EnvironmentSceneState ParseSceneState(const Json& payload) {
  ar::environment::EnvironmentSceneState scene;
  if (payload.contains("atmospheric_physics")) {
    scene.atmospheric_physics = ParseAtmosphericPhysics(payload["atmospheric_physics"]);
  }
  if (payload.contains("atmospheric_context")) {
    scene.atmospheric_context = ParseAtmosphericContext(payload["atmospheric_context"]);
  } else if (payload.contains("atmospheric_physics")) {
    // Backward compatibility: older traces stored derived context together with atmospheric_physics.
    scene.atmospheric_context = ParseAtmosphericContext(payload["atmospheric_physics"]);
  }
  if (payload.contains("jammer_emitters") && payload["jammer_emitters"].is_array()) {
    scene.jammer_emitters.clear();
    for (std::size_t i = 0; i < payload["jammer_emitters"].size(); ++i) {
      scene.jammer_emitters.push_back(ParseJammerEmitter(payload["jammer_emitters"][i]));
    }
  }
  return scene;
}

ar::config::RadarRuntimeConfigPatch ParseRuntimePatch(const Json& payload) {
  ar::config::RadarRuntimeConfigPatch patch;
  patch.has_signal_pipeline_config = GetBool(payload, "has_signal_pipeline_config", false);
  if (patch.has_signal_pipeline_config && payload.contains("signal_pipeline_config")) {
    const Json& pipeline = payload["signal_pipeline_config"];
    if (pipeline.contains("detection")) {
      patch.signal_pipeline_config.detection = ParseDetection(pipeline["detection"]);
    }
    if (pipeline.contains("beam_control")) {
      patch.signal_pipeline_config.beam_control = ParseBeamControl(pipeline["beam_control"]);
    }
    if (pipeline.contains("tracking")) {
      const Json& t = pipeline["tracking"];
      patch.signal_pipeline_config.tracking.enable_kalman_filter =
          GetBool(t, "enable_kalman_filter", patch.signal_pipeline_config.tracking.enable_kalman_filter);
      patch.signal_pipeline_config.tracking.kalman_measurement_noise_std =
          GetFloat(t, "kalman_measurement_noise_std", patch.signal_pipeline_config.tracking.kalman_measurement_noise_std);
    }
    if (pipeline.contains("lifecycle")) {
      const Json& l = pipeline["lifecycle"];
      if (l.contains("lifecycle_config")) {
        const Json& lc = l["lifecycle_config"];
        patch.signal_pipeline_config.lifecycle.lifecycle_config.confirm_hits =
            static_cast<std::uint32_t>(GetInt(lc, "confirm_hits", static_cast<int>(patch.signal_pipeline_config.lifecycle.lifecycle_config.confirm_hits)));
        patch.signal_pipeline_config.lifecycle.lifecycle_config.max_miss_before_lost =
            static_cast<std::uint32_t>(GetInt(lc, "max_miss_before_lost", static_cast<int>(patch.signal_pipeline_config.lifecycle.lifecycle_config.max_miss_before_lost)));
        patch.signal_pipeline_config.lifecycle.lifecycle_config.max_lost_cycles =
            static_cast<std::uint32_t>(GetInt(lc, "max_lost_cycles", static_cast<int>(patch.signal_pipeline_config.lifecycle.lifecycle_config.max_lost_cycles)));
      }
      patch.signal_pipeline_config.lifecycle.enable_imm_lifecycle =
          GetBool(l, "enable_imm_lifecycle", patch.signal_pipeline_config.lifecycle.enable_imm_lifecycle);
    }
  }

  patch.has_environment_runtime_config = GetBool(payload, "has_environment_runtime_config", false);
  if (patch.has_environment_runtime_config && payload.contains("environment_runtime_config")) {
    const Json& env = payload["environment_runtime_config"];
    patch.environment_runtime_config.has_scenario_config =
        GetBool(env, "has_scenario_config", false);
    if (patch.environment_runtime_config.has_scenario_config && env.contains("scenario_config")) {
      patch.environment_runtime_config.scenario_config =
          ParseEnvironmentScenario(env["scenario_config"]);
    }
    patch.environment_runtime_config.has_jamming_detection_threshold_db =
        GetBool(env, "has_jamming_detection_threshold_db", false);
    patch.environment_runtime_config.jamming_detection_threshold_db =
        GetFloat(env, "jamming_detection_threshold_db", patch.environment_runtime_config.jamming_detection_threshold_db);
  }

  patch.has_work_sub_mode = GetBool(payload, "has_work_sub_mode", false);
  patch.work_sub_mode =
      static_cast<ar::model::RadarWorkSubMode>(GetInt(payload, "work_sub_mode", static_cast<int>(patch.work_sub_mode)));
  patch.has_scan_center_deg = GetBool(payload, "has_scan_center_deg", false);
  if (patch.has_scan_center_deg && payload.contains("scan_center_deg")) {
    patch.scan_center_deg = ParseAzEl(payload["scan_center_deg"]);
  }
  patch.has_dwell_center_deg = GetBool(payload, "has_dwell_center_deg", false);
  if (patch.has_dwell_center_deg && payload.contains("dwell_center_deg")) {
    patch.dwell_center_deg = ParseAzEl(payload["dwell_center_deg"]);
  }
  patch.has_commanded_beamwidth_deg = GetBool(payload, "has_commanded_beamwidth_deg", false);
  if (patch.has_commanded_beamwidth_deg && payload.contains("commanded_beamwidth_deg")) {
    const Json& c = payload["commanded_beamwidth_deg"];
    patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg =
        GetFloat(c, "commanded_az_beamwidth_deg", patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg);
    patch.commanded_beamwidth_deg.commanded_el_beamwidth_deg =
        GetFloat(c, "commanded_el_beamwidth_deg", patch.commanded_beamwidth_deg.commanded_el_beamwidth_deg);
  }
  patch.has_commanded_beamwidth_enabled = GetBool(payload, "has_commanded_beamwidth_enabled", false);
  patch.commanded_beamwidth_enabled =
      GetBool(payload, "commanded_beamwidth_enabled", patch.commanded_beamwidth_enabled);
  return patch;
}

std::string GetDefaultReplayPath(const char* argv0) {
  const std::string executable_path = (argv0 != nullptr) ? argv0 : "";
  const std::size_t last_sep = executable_path.find_last_of("/\\");
  const std::string executable_dir =
      (last_sep == std::string::npos) ? "." : executable_path.substr(0, last_sep);
  return executable_dir + "/1q-ar-trace-replay.jsonl";
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "usage: ar_trace_replayer <trace_jsonl_path> [replay_output_jsonl]" << std::endl;
    return 2;
  }

  const std::string trace_path = argv[1];
  const std::string replay_path = (argc >= 3) ? argv[2] : GetDefaultReplayPath(argv[0]);

  std::vector<TraceRecord> expected;
  std::string error_message;
  if (!oneq::examples::replay::LoadTraceRecords(trace_path, "airborne_radar", &expected,
                                                 &error_message)) {
    std::cerr << "load trace failed: " << error_message << std::endl;
    return 1;
  }
  if (expected.empty()) {
    std::cerr << "no airborne_radar records in trace: " << trace_path << std::endl;
    return 1;
  }

  ar::session::RadarSessionConfig config;
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
  ar::session::RadarTraceSession session(config, ar::session::RadarTraceSessionOptions{sink, true});

  for (std::size_t i = 0; i < expected.size(); ++i) {
    const TraceRecord& record = expected[i];
    if (record.phase == "input") {
      if (record.payload.contains("cycle_input") && record.payload.contains("scene_state")) {
        const ar::session::RadarCycleInput input = ParseCycleInput(record.payload["cycle_input"]);
        const ar::environment::EnvironmentSceneState scene = ParseSceneState(record.payload["scene_state"]);
        (void)session.StepWithResult(input, scene);
      } else {
        const ar::session::RadarCycleInput input = ParseCycleInput(record.payload);
        (void)session.StepWithResult(input);
      }
    } else if (record.phase == "runtime_config") {
      const ar::config::RadarRuntimeConfigPatch patch = ParseRuntimePatch(record.payload);
      session.ApplyRuntimeConfig(patch);
    }
  }

  std::vector<TraceRecord> actual;
  if (!oneq::examples::replay::LoadTraceRecords(replay_path, "airborne_radar", &actual,
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
