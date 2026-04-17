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

esr::environment::EsrAtmosphericObservation ParseAtmosphericObservation(const Json& json) {
  esr::environment::EsrAtmosphericObservation value;
  value.relative_humidity_ratio =
      GetFloat(json, "relative_humidity_ratio", value.relative_humidity_ratio);
  value.precipitation_rate_mmph =
      GetFloat(json, "precipitation_rate_mmph", value.precipitation_rate_mmph);
  value.visibility_km = GetFloat(json, "visibility_km", value.visibility_km);
  return value;
}

esr::session::EsrSessionConfig ParseSessionConfig(const Json& payload) {
  esr::session::EsrSessionConfig config;
  if (payload.contains("hardware")) {
    const Json& h = payload["hardware"];
    config.hardware.receiver_band_lower_hz =
        GetDouble(h, "receiver_band_lower_hz", config.hardware.receiver_band_lower_hz);
    config.hardware.receiver_band_upper_hz =
        GetDouble(h, "receiver_band_upper_hz", config.hardware.receiver_band_upper_hz);
    config.hardware.receiver_sensitivity_w =
        GetFloat(h, "receiver_sensitivity_w", config.hardware.receiver_sensitivity_w);
    config.hardware.integrated_receive_loss_db =
        GetFloat(h, "integrated_receive_loss_db", config.hardware.integrated_receive_loss_db);
    config.hardware.beam_az_width_deg =
        GetFloat(h, "beam_az_width_deg", config.hardware.beam_az_width_deg);
    config.hardware.beam_el_width_deg =
        GetFloat(h, "beam_el_width_deg", config.hardware.beam_el_width_deg);
    config.hardware.az_scan_range_deg =
        GetFloat(h, "az_scan_range_deg", config.hardware.az_scan_range_deg);
    config.hardware.el_scan_range_deg =
        GetFloat(h, "el_scan_range_deg", config.hardware.el_scan_range_deg);
    config.hardware.antenna_mount_az_deg =
        GetFloat(h, "antenna_mount_az_deg", config.hardware.antenna_mount_az_deg);
    config.hardware.antenna_mount_el_deg =
        GetFloat(h, "antenna_mount_el_deg", config.hardware.antenna_mount_el_deg);
  }
  if (payload.contains("mission")) {
    const Json& m = payload["mission"];
    config.mission.power_on = GetBool(m, "power_on", config.mission.power_on);
    config.mission.work_mode = static_cast<esr::session::EsrWorkMode>(
        GetInt(m, "work_mode", static_cast<int>(config.mission.work_mode)));
    config.mission.scan.scan_center_az_deg =
        GetFloat(m, "scan_center_az_deg", config.mission.scan.scan_center_az_deg);
    config.mission.scan.scan_center_el_deg =
        GetFloat(m, "scan_center_el_deg", config.mission.scan.scan_center_el_deg);
    config.mission.scan.scan_rate_hz =
        GetFloat(m, "scan_rate_hz", config.mission.scan.scan_rate_hz);
    config.mission.scan.scan_start_position = static_cast<esr::session::EsrScanStartPosition>(
        GetInt(m, "scan_start_position", static_cast<int>(config.mission.scan.scan_start_position)));
    config.mission.scan.scan_sequence = static_cast<esr::session::EsrScanSequence>(
        GetInt(m, "scan_sequence", static_cast<int>(config.mission.scan.scan_sequence)));
    config.mission.scan.use_explicit_scan_bounds =
        GetBool(m, "use_explicit_scan_bounds", config.mission.scan.use_explicit_scan_bounds);
    config.mission.scan.scan_start_az_deg =
        GetFloat(m, "scan_start_az_deg", config.mission.scan.scan_start_az_deg);
    config.mission.scan.scan_end_az_deg =
        GetFloat(m, "scan_end_az_deg", config.mission.scan.scan_end_az_deg);
    config.mission.scan.scan_start_el_deg =
        GetFloat(m, "scan_start_el_deg", config.mission.scan.scan_start_el_deg);
    config.mission.scan.scan_end_el_deg =
        GetFloat(m, "scan_end_el_deg", config.mission.scan.scan_end_el_deg);
  }
  if (payload.contains("detection_profile")) {
    config.detection.profile = static_cast<esr::config::EsrDetectionProfile>(
        GetInt(payload, "detection_profile", static_cast<int>(config.detection.profile)));
  }
  if (payload.contains("environment_preset")) {
    config.environment.preset = static_cast<esr::config::EsrEnvironmentPreset>(
        GetInt(payload, "environment_preset", static_cast<int>(config.environment.preset)));
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

  if (payload.contains("environment_observation")) {
    const Json& env = payload["environment_observation"];
    input.environment_observation.propagation_profile =
        static_cast<esr::environment::EsrPropagationEnvironmentProfile>(
            GetInt(env, "propagation_profile",
                   static_cast<int>(input.environment_observation.propagation_profile)));
    input.environment_observation.clutter_density =
        static_cast<esr::environment::EsrClutterDensityLevel>(
            GetInt(env, "clutter_density", static_cast<int>(input.environment_observation.clutter_density)));
    input.environment_observation.spectrum_occupancy_ratio =
        GetFloat(env, "spectrum_occupancy_ratio", input.environment_observation.spectrum_occupancy_ratio);
    if (env.contains("atmospheric_observation")) {
      input.environment_observation.atmospheric_observation =
          ParseAtmosphericObservation(env["atmospheric_observation"]);
    }
    if (env.contains("jammer_sources") && env["jammer_sources"].is_array()) {
      input.environment_observation.jammer_sources.clear();
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
        input.environment_observation.jammer_sources.push_back(jammer);
      }
    }
  }

  return input;
}

esr::session::EsrRuntimeConfigPatch ParseRuntimePatch(const Json& payload) {
  esr::session::EsrRuntimeConfigPatch patch;
  patch.has_sensor_enabled = GetBool(payload, "has_sensor_enabled", false);
  patch.sensor_enabled = GetBool(payload, "sensor_enabled", patch.sensor_enabled);
  patch.has_work_mode = GetBool(payload, "has_work_mode", false);
  patch.work_mode =
      static_cast<esr::config::EsrWorkMode>(GetInt(payload, "work_mode", static_cast<int>(patch.work_mode)));
  patch.has_scan_rate_hz = GetBool(payload, "has_scan_rate_hz", false);
  patch.scan_rate_hz = GetFloat(payload, "scan_rate_hz", patch.scan_rate_hz);
  patch.has_scan_start_position = GetBool(payload, "has_scan_start_position", false);
  patch.scan_start_position = static_cast<esr::config::EsrScanStartPosition>(
      GetInt(payload, "scan_start_position", static_cast<int>(patch.scan_start_position)));
  patch.has_scan_sequence = GetBool(payload, "has_scan_sequence", false);
  patch.scan_sequence = static_cast<esr::config::EsrScanSequence>(
      GetInt(payload, "scan_sequence", static_cast<int>(patch.scan_sequence)));
  patch.has_scan_center_az_deg = GetBool(payload, "has_scan_center_az_deg", false);
  patch.scan_center_az_deg = GetFloat(payload, "scan_center_az_deg", patch.scan_center_az_deg);
  patch.has_scan_center_el_deg = GetBool(payload, "has_scan_center_el_deg", false);
  patch.scan_center_el_deg = GetFloat(payload, "scan_center_el_deg", patch.scan_center_el_deg);
  patch.has_use_explicit_scan_bounds = GetBool(payload, "has_use_explicit_scan_bounds", false);
  patch.use_explicit_scan_bounds =
      GetBool(payload, "use_explicit_scan_bounds", patch.use_explicit_scan_bounds);
  patch.has_scan_start_az_deg = GetBool(payload, "has_scan_start_az_deg", false);
  patch.scan_start_az_deg = GetFloat(payload, "scan_start_az_deg", patch.scan_start_az_deg);
  patch.has_scan_end_az_deg = GetBool(payload, "has_scan_end_az_deg", false);
  patch.scan_end_az_deg = GetFloat(payload, "scan_end_az_deg", patch.scan_end_az_deg);
  patch.has_scan_start_el_deg = GetBool(payload, "has_scan_start_el_deg", false);
  patch.scan_start_el_deg = GetFloat(payload, "scan_start_el_deg", patch.scan_start_el_deg);
  patch.has_scan_end_el_deg = GetBool(payload, "has_scan_end_el_deg", false);
  patch.scan_end_el_deg = GetFloat(payload, "scan_end_el_deg", patch.scan_end_el_deg);
  patch.has_environment_preset = GetBool(payload, "has_environment_preset", false);
  patch.environment_preset = static_cast<esr::config::EsrEnvironmentPreset>(
      GetInt(payload, "environment_preset", static_cast<int>(patch.environment_preset)));
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
