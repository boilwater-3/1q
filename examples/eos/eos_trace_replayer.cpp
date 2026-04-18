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
  config.hardware.wavelength_lower_um =
      GetFloat(payload, "wavelength_lower_um", config.hardware.wavelength_lower_um);
  config.hardware.wavelength_upper_um =
      GetFloat(payload, "wavelength_upper_um", config.hardware.wavelength_upper_um);
  config.hardware.optical_aperture_m =
      GetFloat(payload, "optical_aperture_m", config.hardware.optical_aperture_m);
  config.hardware.focal_length_m = GetFloat(payload, "focal_length_m", config.hardware.focal_length_m);

  const Json mission_payload = payload.contains("mission") ? payload["mission"] : payload;
  config.mission.work_mode = static_cast<eos::session::EosWorkMode>(
      GetInt(mission_payload, "work_mode", static_cast<int>(config.mission.work_mode)));
  config.mission.horizontal_fov_deg =
      GetFloat(mission_payload, "horizontal_fov_deg", config.mission.horizontal_fov_deg);
  config.mission.vertical_fov_deg =
      GetFloat(mission_payload, "vertical_fov_deg", config.mission.vertical_fov_deg);
  config.mission.scan_rate_deg_per_sec =
      GetFloat(mission_payload, "scan_rate_deg_per_sec", config.mission.scan_rate_deg_per_sec);
  config.mission.frame_rate_hz =
      GetFloat(mission_payload, "frame_rate_hz", config.mission.frame_rate_hz);
  config.mission.scan_start_az_deg =
      GetFloat(mission_payload, "scan_start_az_deg", config.mission.scan_start_az_deg);
  config.mission.scan_end_az_deg =
      GetFloat(mission_payload, "scan_end_az_deg", config.mission.scan_end_az_deg);
  config.mission.scan_center_el_deg =
      GetFloat(mission_payload, "scan_center_el_deg", config.mission.scan_center_el_deg);
  config.mission.boresight_depression_deg =
      GetFloat(mission_payload, "boresight_depression_deg", config.mission.boresight_depression_deg);

  const Json policy_payload = payload.contains("policy") ? payload["policy"] : payload;
  config.policy.detection.profile = static_cast<eos::config::EosDetectionProfile>(
      GetInt(policy_payload, "detection_profile",
             static_cast<int>(config.policy.detection.profile)));
  config.policy.detection.use_profile_defaults = GetBool(
      policy_payload, "detection_use_profile_defaults",
      config.policy.detection.use_profile_defaults);
  config.policy.detection.minimum_snr_db =
      GetFloat(policy_payload, "minimum_snr_db", config.policy.detection.minimum_snr_db);
  config.policy.detection.detection_sensitivity_w = GetFloat(
      policy_payload, "detection_sensitivity_w",
      config.policy.detection.detection_sensitivity_w);
  config.policy.detection.visible_reference_irradiance_w_m2 = GetFloat(
      policy_payload, "visible_reference_irradiance_w_m2",
      config.policy.detection.visible_reference_irradiance_w_m2);

  config.policy.stray_light.profile = static_cast<eos::config::EosStrayLightProfile>(
      GetInt(policy_payload, "stray_light_profile",
             static_cast<int>(config.policy.stray_light.profile)));
  config.policy.stray_light.use_profile_defaults = GetBool(
      policy_payload, "stray_light_use_profile_defaults",
      config.policy.stray_light.use_profile_defaults);
  config.policy.stray_light.enable_straylight_filter = GetBool(
      policy_payload, "enable_straylight_filter",
      config.policy.stray_light.enable_straylight_filter);
  config.policy.stray_light.hood_inner_half_angle_deg = GetFloat(
      policy_payload, "hood_inner_half_angle_deg",
      config.policy.stray_light.hood_inner_half_angle_deg);
  config.policy.stray_light.hood_outer_half_angle_deg = GetFloat(
      policy_payload, "hood_outer_half_angle_deg",
      config.policy.stray_light.hood_outer_half_angle_deg);
  config.policy.stray_light.hood_min_suppression_ratio = GetFloat(
      policy_payload, "hood_min_suppression_ratio",
      config.policy.stray_light.hood_min_suppression_ratio);
  config.policy.stray_light.hood_max_suppression_ratio = GetFloat(
      policy_payload, "hood_max_suppression_ratio",
      config.policy.stray_light.hood_max_suppression_ratio);

  const Json environment_payload =
      payload.contains("environment") ? payload["environment"] : payload;
  config.environment.model_type = static_cast<eos::environment::EosEnvironmentModelType>(
      GetInt(environment_payload, "environment_model_type",
             static_cast<int>(config.environment.model_type)));
  config.environment.preset = static_cast<eos::config::EosEnvironmentPreset>(
      GetInt(environment_payload, "environment_preset",
             static_cast<int>(config.environment.preset)));
  config.environment.use_preset_defaults = GetBool(
      environment_payload, "use_preset_defaults",
      config.environment.use_preset_defaults);
  config.environment.radiative_transfer_model =
      static_cast<eos::foundation::radiative_transfer::RadiativeTransferModel>(
          GetInt(environment_payload, "radiative_transfer_model",
                 static_cast<int>(config.environment.radiative_transfer_model)));
  config.environment.aerosol_density_factor = GetFloat(
      environment_payload, "aerosol_density_factor",
      config.environment.aerosol_density_factor);
  config.environment.turbulence_factor = GetFloat(
      environment_payload, "turbulence_factor",
      config.environment.turbulence_factor);
  config.environment.enable_optical_countermeasure_extension = GetBool(
      environment_payload, "enable_optical_countermeasure_extension",
      config.environment.enable_optical_countermeasure_extension);
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
  patch.has_mission = GetBool(payload, "has_mission", false);
  if (patch.has_mission && payload.contains("mission")) {
    const Json& mission_payload = payload["mission"];
    patch.mission.work_mode = static_cast<eos::session::EosWorkMode>(
        GetInt(mission_payload, "work_mode", static_cast<int>(patch.mission.work_mode)));
    patch.mission.horizontal_fov_deg =
        GetFloat(mission_payload, "horizontal_fov_deg", patch.mission.horizontal_fov_deg);
    patch.mission.vertical_fov_deg =
        GetFloat(mission_payload, "vertical_fov_deg", patch.mission.vertical_fov_deg);
    patch.mission.scan_rate_deg_per_sec =
        GetFloat(mission_payload, "scan_rate_deg_per_sec", patch.mission.scan_rate_deg_per_sec);
    patch.mission.frame_rate_hz =
        GetFloat(mission_payload, "frame_rate_hz", patch.mission.frame_rate_hz);
    patch.mission.scan_start_az_deg =
        GetFloat(mission_payload, "scan_start_az_deg", patch.mission.scan_start_az_deg);
    patch.mission.scan_end_az_deg =
        GetFloat(mission_payload, "scan_end_az_deg", patch.mission.scan_end_az_deg);
    patch.mission.scan_center_el_deg =
        GetFloat(mission_payload, "scan_center_el_deg", patch.mission.scan_center_el_deg);
    patch.mission.boresight_depression_deg =
        GetFloat(mission_payload, "boresight_depression_deg",
                 patch.mission.boresight_depression_deg);
  }

  patch.has_policy = GetBool(payload, "has_policy", false);
  if (patch.has_policy && payload.contains("policy")) {
    const Json& policy_payload = payload["policy"];
    patch.policy.detection.profile = static_cast<eos::config::EosDetectionProfile>(
        GetInt(policy_payload, "detection_profile",
               static_cast<int>(patch.policy.detection.profile)));
    patch.policy.detection.use_profile_defaults = GetBool(
        policy_payload, "detection_use_profile_defaults",
        patch.policy.detection.use_profile_defaults);
    patch.policy.detection.minimum_snr_db = GetFloat(
        policy_payload, "minimum_snr_db", patch.policy.detection.minimum_snr_db);
    patch.policy.detection.detection_sensitivity_w = GetFloat(
        policy_payload, "detection_sensitivity_w",
        patch.policy.detection.detection_sensitivity_w);
    patch.policy.detection.visible_reference_irradiance_w_m2 =
        GetFloat(policy_payload, "visible_reference_irradiance_w_m2",
                 patch.policy.detection.visible_reference_irradiance_w_m2);

    patch.policy.stray_light.profile = static_cast<eos::config::EosStrayLightProfile>(
        GetInt(policy_payload, "stray_light_profile",
               static_cast<int>(patch.policy.stray_light.profile)));
    patch.policy.stray_light.use_profile_defaults = GetBool(
        policy_payload, "stray_light_use_profile_defaults",
        patch.policy.stray_light.use_profile_defaults);
    patch.policy.stray_light.enable_straylight_filter = GetBool(
        policy_payload, "enable_straylight_filter",
        patch.policy.stray_light.enable_straylight_filter);
    patch.policy.stray_light.hood_inner_half_angle_deg = GetFloat(
        policy_payload, "hood_inner_half_angle_deg",
        patch.policy.stray_light.hood_inner_half_angle_deg);
    patch.policy.stray_light.hood_outer_half_angle_deg = GetFloat(
        policy_payload, "hood_outer_half_angle_deg",
        patch.policy.stray_light.hood_outer_half_angle_deg);
    patch.policy.stray_light.hood_min_suppression_ratio = GetFloat(
        policy_payload, "hood_min_suppression_ratio",
        patch.policy.stray_light.hood_min_suppression_ratio);
    patch.policy.stray_light.hood_max_suppression_ratio = GetFloat(
        policy_payload, "hood_max_suppression_ratio",
        patch.policy.stray_light.hood_max_suppression_ratio);
  }

  patch.has_environment = GetBool(payload, "has_environment", false);
  if (patch.has_environment && payload.contains("environment")) {
    const Json& environment_payload = payload["environment"];
    patch.environment.model_type =
        static_cast<eos::environment::EosEnvironmentModelType>(
            GetInt(environment_payload, "environment_model_type",
                   static_cast<int>(patch.environment.model_type)));
    patch.environment.preset = static_cast<eos::config::EosEnvironmentPreset>(
        GetInt(environment_payload, "environment_preset",
               static_cast<int>(patch.environment.preset)));
    patch.environment.use_preset_defaults =
        GetBool(environment_payload, "use_preset_defaults",
                patch.environment.use_preset_defaults);
    patch.environment.radiative_transfer_model =
        static_cast<eos::foundation::radiative_transfer::RadiativeTransferModel>(
            GetInt(environment_payload, "radiative_transfer_model",
                   static_cast<int>(patch.environment.radiative_transfer_model)));
    patch.environment.aerosol_density_factor =
        GetFloat(environment_payload, "aerosol_density_factor",
                 patch.environment.aerosol_density_factor);
    patch.environment.turbulence_factor =
        GetFloat(environment_payload, "turbulence_factor",
                 patch.environment.turbulence_factor);
    patch.environment.enable_optical_countermeasure_extension = GetBool(
        environment_payload, "enable_optical_countermeasure_extension",
        patch.environment.enable_optical_countermeasure_extension);
  }

  patch.has_work_mode = GetBool(payload, "has_work_mode", false);
  patch.work_mode =
      static_cast<eos::session::EosWorkMode>(GetInt(payload, "work_mode", static_cast<int>(patch.work_mode)));
  patch.has_scan_rate_deg_per_sec = GetBool(payload, "has_scan_rate_deg_per_sec", false);
  patch.scan_rate_deg_per_sec = GetFloat(payload, "scan_rate_deg_per_sec", patch.scan_rate_deg_per_sec);
  patch.has_frame_rate_hz = GetBool(payload, "has_frame_rate_hz", false);
  patch.frame_rate_hz = GetFloat(payload, "frame_rate_hz", patch.frame_rate_hz);
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
