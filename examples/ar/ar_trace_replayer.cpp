// Copyright 2026. All Rights Reserved.
//
// @file ar_trace_replayer.cpp
// @brief AR Trace 回放示例：读取 JSONL，驱动 RadarTraceSession 重放并校验一致性。

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

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

std::int64_t GetInt64(const Json& json, const char* key, std::int64_t default_value = 0) {
  return json.contains(key) ? json[key].get<std::int64_t>() : default_value;
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

ar::config::DetectionConfig ParseDetection(const Json& json) {
  ar::config::DetectionConfig config;
  config.enable_physics_detection =
      GetBool(json, "enable_physics_detection", config.enable_physics_detection);

  if (json.contains("swerling_model")) {
    config.swerling_model = static_cast<ar::config::semantic::SwerlingModel>(
        GetInt(json, "swerling_model", static_cast<int>(config.swerling_model)));
  }

  if (json.contains("transmitter")) {
    const Json& t = json["transmitter"];
    config.transmitter.peak_power_w = GetFloat(t, "peak_power_w", config.transmitter.peak_power_w);
    config.transmitter.frequency_hz = GetFloat(t, "frequency_hz", config.transmitter.frequency_hz);
    config.transmitter.bandwidth_hz = GetFloat(t, "bandwidth_hz", config.transmitter.bandwidth_hz);
    config.transmitter.pulse_width_s =
        GetFloat(t, "pulse_width_s", config.transmitter.pulse_width_s);
    config.transmitter.prf_hz = GetFloat(t, "prf_hz", config.transmitter.prf_hz);
    config.transmitter.transmit_loss_db =
        GetFloat(t, "transmit_loss_db", config.transmitter.transmit_loss_db);
  }
  if (json.contains("antenna")) {
    const Json& a = json["antenna"];
    config.antenna.main_beam_gain_db =
        GetFloat(a, "main_beam_gain_db", config.antenna.main_beam_gain_db);
    config.antenna.nominal_az_beamwidth_deg =
        GetFloat(a, "nominal_az_beamwidth_deg", config.antenna.nominal_az_beamwidth_deg);
    config.antenna.nominal_el_beamwidth_deg =
        GetFloat(a, "nominal_el_beamwidth_deg", config.antenna.nominal_el_beamwidth_deg);
    config.antenna.enable_directional_pattern =
        GetBool(a, "enable_directional_pattern", config.antenna.enable_directional_pattern);
    if (a.contains("pattern")) {
      const Json& p = a["pattern"];
      config.antenna.pattern.model_type = static_cast<ar::config::AntennaPatternModelType>(
          GetInt(p, "model_type", static_cast<int>(config.antenna.pattern.model_type)));
      config.antenna.pattern.max_sidelobe_level_db =
          GetFloat(p, "max_sidelobe_level_db", config.antenna.pattern.max_sidelobe_level_db);
      config.antenna.pattern.backlobe_level_db =
          GetFloat(p, "backlobe_level_db", config.antenna.pattern.backlobe_level_db);
      config.antenna.pattern.scan_loss_coeff_db_per_deg2 = GetFloat(
          p, "scan_loss_coeff_db_per_deg2", config.antenna.pattern.scan_loss_coeff_db_per_deg2);
      config.antenna.pattern.max_scan_loss_db =
          GetFloat(p, "max_scan_loss_db", config.antenna.pattern.max_scan_loss_db);
      if (p.contains("boresight_offset_deg")) {
        config.antenna.pattern.boresight_offset_deg = ParseAzEl(p["boresight_offset_deg"]);
      }
    }
  }
  if (json.contains("receiver")) {
    const Json& r = json["receiver"];
    config.receiver.noise_figure_db =
        GetFloat(r, "noise_figure_db", config.receiver.noise_figure_db);
    config.receiver.receive_loss_db =
        GetFloat(r, "receive_loss_db", config.receiver.receive_loss_db);
  }
  if (json.contains("detection_policy")) {
    const Json& p = json["detection_policy"];
    config.detection_policy.cfar_pfa = GetFloat(p, "cfar_pfa", config.detection_policy.cfar_pfa);
    config.detection_policy.min_snr_db =
        GetFloat(p, "min_snr_db", config.detection_policy.min_snr_db);
  }
  if (json.contains("rcs_physics")) {
    const Json& r = json["rcs_physics"];
    config.rcs_physics.enable_physical_rcs =
        GetBool(r, "enable_physical_rcs", config.rcs_physics.enable_physical_rcs);
    config.rcs_physics.frequency_hz = GetFloat(r, "frequency_hz", config.rcs_physics.frequency_hz);
    config.rcs_physics.physics_mix_ratio =
        GetFloat(r, "physics_mix_ratio", config.rcs_physics.physics_mix_ratio);
    config.rcs_physics.cylinder_weight =
        GetFloat(r, "cylinder_weight", config.rcs_physics.cylinder_weight);
    config.rcs_physics.min_equivalent_radius_m =
        GetFloat(r, "min_equivalent_radius_m", config.rcs_physics.min_equivalent_radius_m);
    config.rcs_physics.max_equivalent_radius_m =
        GetFloat(r, "max_equivalent_radius_m", config.rcs_physics.max_equivalent_radius_m);
    config.rcs_physics.min_rcs_m2 = GetFloat(r, "min_rcs_m2", config.rcs_physics.min_rcs_m2);
    config.rcs_physics.max_rcs_m2 = GetFloat(r, "max_rcs_m2", config.rcs_physics.max_rcs_m2);
    config.rcs_physics.bistatic_psi_offset_deg =
        GetFloat(r, "bistatic_psi_offset_deg", config.rcs_physics.bistatic_psi_offset_deg);
  }
  config.min_detection_margin_db =
      GetFloat(json, "min_detection_margin_db", config.min_detection_margin_db);
  config.pulse_count = GetInt(json, "pulse_count", config.pulse_count);
  return config;
}

ar::config::TrackingConfig ParseTracking(const Json& json) {
  ar::config::TrackingConfig config;
  config.enable_kalman_filter = GetBool(json, "enable_kalman_filter", config.enable_kalman_filter);
  config.kalman_measurement_noise_std =
      GetFloat(json, "kalman_measurement_noise_std", config.kalman_measurement_noise_std);
  if (json.contains("kalman_update_backend")) {
    config.kalman_update_backend = static_cast<ar::config::KalmanUpdateBackend>(
        GetInt(json, "kalman_update_backend", static_cast<int>(config.kalman_update_backend)));
  }
  return config;
}

ar::config::LifecycleConfig ParseLifecycle(const Json& json) {
  ar::config::LifecycleConfig config;
  config.confirm_hits = static_cast<std::uint32_t>(
      GetInt(json, "confirm_hits", static_cast<int>(config.confirm_hits)));
  config.max_miss_before_lost = static_cast<std::uint32_t>(
      GetInt(json, "max_miss_before_lost", static_cast<int>(config.max_miss_before_lost)));
  config.max_lost_cycles = static_cast<std::uint32_t>(
      GetInt(json, "max_lost_cycles", static_cast<int>(config.max_lost_cycles)));
  config.enable_imm_lifecycle = GetBool(json, "enable_imm_lifecycle", config.enable_imm_lifecycle);
  return config;
}

ar::model::RadarOrientationConfig ParseOrientation(const Json& json) {
  ar::model::RadarOrientationConfig orientation;
  if (json.contains("mount_angles_deg")) {
    orientation.mount_angles_deg = ParseEuler(json["mount_angles_deg"]);
  }
  if (json.contains("scan_center_deg")) {
    orientation.scan_center_deg = ParseAzEl(json["scan_center_deg"]);
  }
  if (json.contains("mechanical_scan_limits_deg")) {
    orientation.mechanical_scan_limits_deg = ParseAzElLimits(json["mechanical_scan_limits_deg"]);
  }
  if (json.contains("electronic_scan_limits_deg")) {
    orientation.electronic_scan_limits_deg = ParseAzElLimits(json["electronic_scan_limits_deg"]);
  }
  orientation.scan_start_position = static_cast<oneq::foundation::ScanStartPosition>(
      GetInt(json, "scan_start_position", static_cast<int>(orientation.scan_start_position)));
  orientation.scan_sequence = static_cast<oneq::foundation::ScanSequence>(
      GetInt(json, "scan_sequence", static_cast<int>(orientation.scan_sequence)));
  orientation.work_sub_mode = static_cast<ar::model::RadarWorkSubMode>(
      GetInt(json, "work_sub_mode", static_cast<int>(orientation.work_sub_mode)));
  orientation.commanded_beamwidth_enabled =
      GetBool(json, "commanded_beamwidth_enabled", orientation.commanded_beamwidth_enabled);
  if (json.contains("commanded_beamwidth_deg")) {
    const Json& c = json["commanded_beamwidth_deg"];
    orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg =
        GetFloat(c, "commanded_az_beamwidth_deg",
                 orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg);
    orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg =
        GetFloat(c, "commanded_el_beamwidth_deg",
                 orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg);
  }
  orientation.stabilization_mode = static_cast<ar::model::StabilizationMode>(
      GetInt(json, "stabilization_mode", static_cast<int>(orientation.stabilization_mode)));
  return orientation;
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
  value.has_simulation_unix_seconds =
      GetBool(json, "has_simulation_unix_seconds", value.has_simulation_unix_seconds);
  value.simulation_unix_seconds =
      GetInt64(json, "simulation_unix_seconds", value.simulation_unix_seconds);
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
  }
  if (json.contains("vegetation_scatter_physics")) {
    const Json& veg = json["vegetation_scatter_physics"];
    scenario.vegetation_scatter_physics.enable_physical_model = GetBool(
        veg, "enable_physical_model", scenario.vegetation_scatter_physics.enable_physical_model);
    if (veg.contains("cover_profile")) {
      scenario.vegetation_scatter_physics.cover_profile =
          static_cast<ar::environment::VegetationCoverProfile>(
              GetInt(veg, "cover_profile",
                     static_cast<int>(scenario.vegetation_scatter_physics.cover_profile)));
    }
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

  if (payload.contains("hardware")) {
    const Json& hw = payload["hardware"];
    if (hw.contains("detection")) {
      config.hardware.detection = ParseDetection(hw["detection"]);
    }
  }

  if (payload.contains("mission")) {
    const Json& mission = payload["mission"];
    if (mission.contains("orientation")) {
      config.mission.orientation = ParseOrientation(mission["orientation"]);
    }
  }

  if (payload.contains("policy")) {
    const Json& policy = payload["policy"];
    if (policy.contains("tracking")) {
      config.policy.tracking = ParseTracking(policy["tracking"]);
    }
    if (policy.contains("lifecycle")) {
      config.policy.lifecycle = ParseLifecycle(policy["lifecycle"]);
    }
  }

  if (payload.contains("environment")) {
    const Json& env = payload["environment"];
    if (env.contains("jamming_sensitivity_profile")) {
      config.environment.jamming_sensitivity_profile =
          static_cast<ar::environment::JammingSensitivityProfile>(
              GetInt(env, "jamming_sensitivity_profile",
                     static_cast<int>(config.environment.jamming_sensitivity_profile)));
    }
    if (env.contains("scenario_config")) {
      config.environment.scenario_config = ParseEnvironmentScenario(env["scenario_config"]);
    }
  }

  return config;
}

ar::model::TargetFeature ParseTargetFeature(const Json& json) {
  ar::model::TargetFeature feature;
  feature.external_target_id = json.value("external_target_id", static_cast<std::uint64_t>(0));
  if (json.contains("velocity_mps") && json["velocity_mps"].is_array() &&
      json["velocity_mps"].size() >= 3U) {
    feature.current_track_velocity_x = json["velocity_mps"][0].get<float>();
    feature.current_track_velocity_y = json["velocity_mps"][1].get<float>();
    feature.current_track_velocity_z = json["velocity_mps"][2].get<float>();
  }
  feature.current_track_speed = GetFloat(json, "current_track_speed", feature.current_track_speed);
  feature.current_track_rcs = GetFloat(json, "current_track_rcs", feature.current_track_rcs);
  feature.range_m = GetFloat(json, "range_m", feature.range_m);
  feature.has_cartesian_position =
      GetBool(json, "has_cartesian_position", feature.has_cartesian_position);
  if (json.contains("position_m") && json["position_m"].is_array() &&
      json["position_m"].size() >= 3U) {
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

  if (payload.contains("has_mission") && GetBool(payload, "has_mission", false) &&
      payload.contains("mission")) {
    const Json& mission = payload["mission"];
    patch.has_mission = true;
    if (mission.contains("orientation")) {
      patch.mission.orientation = ParseOrientation(mission["orientation"]);
    }
  }

  if (payload.contains("has_policy") && GetBool(payload, "has_policy", false) &&
      payload.contains("policy")) {
    const Json& policy = payload["policy"];
    patch.has_policy = true;
    if (policy.contains("tracking")) {
      patch.policy.tracking = ParseTracking(policy["tracking"]);
    }
    if (policy.contains("lifecycle")) {
      patch.policy.lifecycle = ParseLifecycle(policy["lifecycle"]);
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
    patch.environment_runtime_config.has_jamming_sensitivity_profile =
        GetBool(env, "has_jamming_sensitivity_profile", false);
    if (env.contains("jamming_sensitivity_profile")) {
      patch.environment_runtime_config.has_jamming_sensitivity_profile = true;
      patch.environment_runtime_config.jamming_sensitivity_profile =
          static_cast<ar::environment::JammingSensitivityProfile>(GetInt(
              env, "jamming_sensitivity_profile",
              static_cast<int>(patch.environment_runtime_config.jamming_sensitivity_profile)));
    }
  }

  patch.has_work_sub_mode = GetBool(payload, "has_work_sub_mode", false);
  patch.work_sub_mode = static_cast<ar::model::RadarWorkSubMode>(
      GetInt(payload, "work_sub_mode", static_cast<int>(patch.work_sub_mode)));
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
    patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg = GetFloat(
        c, "commanded_az_beamwidth_deg", patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg);
    patch.commanded_beamwidth_deg.commanded_el_beamwidth_deg = GetFloat(
        c, "commanded_el_beamwidth_deg", patch.commanded_beamwidth_deg.commanded_el_beamwidth_deg);
  }
  patch.has_commanded_beamwidth_enabled =
      GetBool(payload, "has_commanded_beamwidth_enabled", false);
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
        const ar::environment::EnvironmentSceneState scene =
            ParseSceneState(record.payload["scene_state"]);
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
