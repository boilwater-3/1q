// Copyright 2026. All Rights Reserved.
//
// @file eos_trace_replayer.cpp
// @brief EOS Trace 回放示例：读取 JSONL，驱动 EosTraceSession 重放并校验一致性。

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/trace/TraceSink.h"
#include "common/trace_replay_common.h"

namespace {

using Json = nlohmann::ordered_json;
namespace eos = electro_optical_sensor;
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

oneq::foundation::PoseState ParsePose(const Json& json) {
  oneq::foundation::PoseState pose;
  if (json.contains("position_m")) {
    pose.position_m = ParseVector3Array(json["position_m"]);
  }
  if (json.contains("velocity_mps")) {
    pose.velocity_mps = ParseVector3Array(json["velocity_mps"]);
  }
  if (json.contains("attitude_deg")) {
    pose.attitude_deg = ParseEuler(json["attitude_deg"]);
  }
  return pose;
}

eos::session::EosSessionConfig ParseSessionConfig(const Json& payload) {
  eos::session::EosSessionConfig config;
  config.optical.wavelength_lower_um =
      GetFloat(payload, "wavelength_lower_um", config.optical.wavelength_lower_um);
  config.optical.wavelength_upper_um =
      GetFloat(payload, "wavelength_upper_um", config.optical.wavelength_upper_um);
  config.optical.optical_aperture_m =
      GetFloat(payload, "optical_aperture_m", config.optical.optical_aperture_m);
  config.optical.focal_length_m = GetFloat(payload, "focal_length_m", config.optical.focal_length_m);
  config.scan.work_mode = static_cast<eos::session::EosWorkMode>(
      GetInt(payload, "work_mode", static_cast<int>(config.scan.work_mode)));
  config.scan.horizontal_fov_deg =
      GetFloat(payload, "horizontal_fov_deg", config.scan.horizontal_fov_deg);
  config.scan.vertical_fov_deg = GetFloat(payload, "vertical_fov_deg", config.scan.vertical_fov_deg);
  config.scan.scan_rate_deg_per_sec =
      GetFloat(payload, "scan_rate_deg_per_sec", config.scan.scan_rate_deg_per_sec);
  config.scan.frame_rate_hz = GetFloat(payload, "frame_rate_hz", config.scan.frame_rate_hz);
  config.pointing.scan_start_az_deg =
      GetFloat(payload, "scan_start_az_deg", config.pointing.scan_start_az_deg);
  config.pointing.scan_end_az_deg =
      GetFloat(payload, "scan_end_az_deg", config.pointing.scan_end_az_deg);
  config.pointing.scan_center_el_deg =
      GetFloat(payload, "scan_center_el_deg", config.pointing.scan_center_el_deg);
  config.pointing.boresight_depression_deg =
      GetFloat(payload, "boresight_depression_deg", config.pointing.boresight_depression_deg);
  config.detection.profile = static_cast<eos::config::EosDetectionProfile>(
      GetInt(payload, "detection_profile", static_cast<int>(config.detection.profile)));
  config.stray_light.profile = static_cast<eos::config::EosStrayLightProfile>(
      GetInt(payload, "stray_light_profile", static_cast<int>(config.stray_light.profile)));
  config.environment.model_type = static_cast<eos::environment::EosEnvironmentModelType>(
      GetInt(payload, "environment_model_type", static_cast<int>(config.environment.model_type)));
  config.environment.preset = static_cast<eos::config::EosEnvironmentPreset>(
      GetInt(payload, "environment_preset", static_cast<int>(config.environment.preset)));
  return config;
}

eos::session::EosCycleInput ParseCycleInput(const Json& payload) {
  eos::session::EosCycleInput input;
  input.cycle_index = payload.value("cycle_index", input.cycle_index);
  input.dt_sec = GetFloat(payload, "dt_sec", input.dt_sec);
  if (payload.contains("platform_pose")) {
    input.platform_pose = ParsePose(payload["platform_pose"]);
  }
  input.solar_altitude_deg = GetFloat(payload, "solar_altitude_deg", input.solar_altitude_deg);
  input.solar_azimuth_deg = GetFloat(payload, "solar_azimuth_deg", input.solar_azimuth_deg);
  input.solar_irradiance_w_m2 = GetFloat(payload, "solar_irradiance_w_m2", input.solar_irradiance_w_m2);
  input.cloud_coverage_ratio = GetFloat(payload, "cloud_coverage_ratio", input.cloud_coverage_ratio);
  input.ambient_wind_speed_mps = GetFloat(payload, "ambient_wind_speed_mps", input.ambient_wind_speed_mps);
  input.day_night_type = static_cast<eos::session::DayNightType>(
      GetInt(payload, "day_night_type", static_cast<int>(input.day_night_type)));
  input.background_temperature_k =
      GetFloat(payload, "background_temperature_k", input.background_temperature_k);

  if (payload.contains("scene_targets") && payload["scene_targets"].is_array()) {
    input.scene_targets.clear();
    for (std::size_t i = 0; i < payload["scene_targets"].size(); ++i) {
      const Json& j = payload["scene_targets"][i];
      eos::session::EosTargetState target;
      target.target_id = j.value("target_id", target.target_id);
      target.range_m = GetFloat(j, "range_m", target.range_m);
      target.azimuth_deg = GetFloat(j, "azimuth_deg", target.azimuth_deg);
      target.elevation_deg = GetFloat(j, "elevation_deg", target.elevation_deg);
      target.apparent_temperature_k =
          GetFloat(j, "apparent_temperature_k", target.apparent_temperature_k);
      target.emissivity = GetFloat(j, "emissivity", target.emissivity);
      target.reflectance = GetFloat(j, "reflectance", target.reflectance);
      target.projected_area_m2 = GetFloat(j, "projected_area_m2", target.projected_area_m2);
      input.scene_targets.push_back(target);
    }
  }
  return input;
}

eos::session::EosRuntimeConfigPatch ParseRuntimePatch(const Json& payload) {
  eos::session::EosRuntimeConfigPatch patch;
  patch.has_work_mode = GetBool(payload, "has_work_mode", false);
  patch.work_mode =
      static_cast<eos::session::EosWorkMode>(GetInt(payload, "work_mode", static_cast<int>(patch.work_mode)));
  patch.has_scan_rate_deg_per_sec = GetBool(payload, "has_scan_rate_deg_per_sec", false);
  patch.scan_rate_deg_per_sec = GetFloat(payload, "scan_rate_deg_per_sec", patch.scan_rate_deg_per_sec);
  patch.has_frame_rate_hz = GetBool(payload, "has_frame_rate_hz", false);
  patch.frame_rate_hz = GetFloat(payload, "frame_rate_hz", patch.frame_rate_hz);
  patch.has_detection_profile = GetBool(payload, "has_detection_profile", false);
  patch.detection_profile = static_cast<eos::config::EosDetectionProfile>(
      GetInt(payload, "detection_profile", static_cast<int>(patch.detection_profile)));
  patch.has_stray_light_profile = GetBool(payload, "has_stray_light_profile", false);
  patch.stray_light_profile = static_cast<eos::config::EosStrayLightProfile>(
      GetInt(payload, "stray_light_profile", static_cast<int>(patch.stray_light_profile)));
  patch.has_environment_model_type = GetBool(payload, "has_environment_model_type", false);
  patch.environment_model_type = static_cast<eos::environment::EosEnvironmentModelType>(
      GetInt(payload, "environment_model_type", static_cast<int>(patch.environment_model_type)));
  patch.has_environment_preset = GetBool(payload, "has_environment_preset", false);
  patch.environment_preset = static_cast<eos::config::EosEnvironmentPreset>(
      GetInt(payload, "environment_preset", static_cast<int>(patch.environment_preset)));
  return patch;
}

std::string GetDefaultReplayPath(const char* argv0) {
  const std::string executable_path = (argv0 != nullptr) ? argv0 : "";
  const std::size_t last_sep = executable_path.find_last_of("/\\");
  const std::string executable_dir =
      (last_sep == std::string::npos) ? "." : executable_path.substr(0, last_sep);
  return executable_dir + "/1q-eos-trace-replay.jsonl";
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "usage: eos_trace_replayer <trace_jsonl_path> [replay_output_jsonl]" << std::endl;
    return 2;
  }

  const std::string trace_path = argv[1];
  const std::string replay_path = (argc >= 3) ? argv[2] : GetDefaultReplayPath(argv[0]);

  std::vector<TraceRecord> expected;
  std::string error_message;
  if (!oneq::examples::replay::LoadTraceRecords(trace_path, "electro_optical_sensor", &expected,
                                                 &error_message)) {
    std::cerr << "load trace failed: " << error_message << std::endl;
    return 1;
  }
  if (expected.empty()) {
    std::cerr << "no electro_optical_sensor records in trace: " << trace_path << std::endl;
    return 1;
  }

  eos::session::EosSessionConfig config;
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
  eos::session::EosTraceSession session(config, eos::session::EosTraceSessionOptions{sink, true});

  for (std::size_t i = 0; i < expected.size(); ++i) {
    const TraceRecord& record = expected[i];
    if (record.phase == "input") {
      const eos::session::EosCycleInput input = ParseCycleInput(record.payload);
      (void)session.StepWithResult(input);
    } else if (record.phase == "runtime_config_patch") {
      const eos::session::EosRuntimeConfigPatch patch = ParseRuntimePatch(record.payload);
      session.ApplyRuntimeConfig(patch);
    }
  }

  std::vector<TraceRecord> actual;
  if (!oneq::examples::replay::LoadTraceRecords(replay_path, "electro_optical_sensor", &actual,
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
