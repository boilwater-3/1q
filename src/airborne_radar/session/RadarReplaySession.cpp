#include "1q/airborne_radar/session/RadarReplaySession.h"

#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#if defined(_WIN32)
#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"
#endif

namespace airborne_radar {
namespace session {
namespace {

std::size_t FindFieldValueStart(const std::string& json, const char* name) {
  const std::string key = std::string("\"") + name + "\":";
  const std::size_t key_pos = json.find(key);
  if (key_pos == std::string::npos) {
    return std::string::npos;
  }
  return key_pos + key.size();
}

float ReadFloat(const std::string& json, const char* name, float fallback) {
  const std::size_t value_pos = FindFieldValueStart(json, name);
  if (value_pos == std::string::npos) {
    return fallback;
  }
  return static_cast<float>(std::strtod(json.c_str() + value_pos, nullptr));
}

bool ReadBool(const std::string& json, const char* name, bool fallback) {
  const std::size_t value_pos = FindFieldValueStart(json, name);
  if (value_pos == std::string::npos) {
    return fallback;
  }
  if (json.compare(value_pos, 4U, "true") == 0) {
    return true;
  }
  if (json.compare(value_pos, 5U, "false") == 0) {
    return false;
  }
  return fallback;
}

int ReadInt(const std::string& json, const char* name, int fallback) {
  const std::size_t value_pos = FindFieldValueStart(json, name);
  if (value_pos == std::string::npos) {
    return fallback;
  }
  return static_cast<int>(std::strtol(json.c_str() + value_pos, nullptr, 10));
}

std::int64_t ReadInt64(const std::string& json, const char* name, std::int64_t fallback) {
  const std::size_t value_pos = FindFieldValueStart(json, name);
  if (value_pos == std::string::npos) {
    return fallback;
  }
  return static_cast<std::int64_t>(std::strtoll(json.c_str() + value_pos, nullptr, 10));
}

std::uint64_t ReadUInt64(const std::string& json, const char* name, std::uint64_t fallback) {
  const std::size_t value_pos = FindFieldValueStart(json, name);
  if (value_pos == std::string::npos) {
    return fallback;
  }
  return static_cast<std::uint64_t>(std::strtoull(json.c_str() + value_pos, nullptr, 10));
}

std::uint32_t ReadUInt32(const std::string& json, const char* name, std::uint32_t fallback) {
  return static_cast<std::uint32_t>(ReadUInt64(json, name, fallback));
}

std::string ExtractRawJsonValue(const std::string& json, const char* name) {
  const std::size_t value_pos = FindFieldValueStart(json, name);
  if (value_pos == std::string::npos || value_pos >= json.size()) {
    return "";
  }

  const char opening = json[value_pos];
  char closing = '\0';
  if (opening == '{') {
    closing = '}';
  } else if (opening == '[') {
    closing = ']';
  } else {
    return "";
  }

  int depth = 0;
  bool in_string = false;
  bool escaping = false;
  for (std::size_t i = value_pos; i < json.size(); ++i) {
    const char c = json[i];
    if (escaping) {
      escaping = false;
      continue;
    }
    if (c == '\\') {
      escaping = in_string;
      continue;
    }
    if (c == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (c == opening) {
      ++depth;
    } else if (c == closing) {
      --depth;
      if (depth == 0) {
        return json.substr(value_pos, i - value_pos + 1U);
      }
    }
  }
  return "";
}

std::vector<std::string> ExtractObjectArrayItems(const std::string& array_json) {
  std::vector<std::string> items;
  int depth = 0;
  bool in_string = false;
  bool escaping = false;
  std::size_t object_start = std::string::npos;

  for (std::size_t i = 0; i < array_json.size(); ++i) {
    const char c = array_json[i];
    if (escaping) {
      escaping = false;
      continue;
    }
    if (c == '\\') {
      escaping = in_string;
      continue;
    }
    if (c == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (c == '{') {
      if (depth == 0) {
        object_start = i;
      }
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0 && object_start != std::string::npos) {
        items.push_back(array_json.substr(object_start, i - object_start + 1U));
        object_start = std::string::npos;
      }
    }
  }

  return items;
}

bool ReadFloatArray3(const std::string& array_json, float* first, float* second, float* third) {
  if (array_json.empty() || array_json[0] != '[') {
    return false;
  }
  char* end = nullptr;
  *first = static_cast<float>(std::strtod(array_json.c_str() + 1U, &end));
  if (end == nullptr || *end != ',') {
    return false;
  }
  *second = static_cast<float>(std::strtod(end + 1, &end));
  if (end == nullptr || *end != ',') {
    return false;
  }
  *third = static_cast<float>(std::strtod(end + 1, nullptr));
  return true;
}

oneq::foundation::Vector3f ReadVector3(const std::string& json,
                                       const oneq::foundation::Vector3f& fallback) {
  oneq::foundation::Vector3f value = fallback;
  if (!json.empty() && json[0] == '[') {
    ReadFloatArray3(json, &value.x, &value.y, &value.z);
    return value;
  }
  value.x = ReadFloat(json, "x", value.x);
  value.y = ReadFloat(json, "y", value.y);
  value.z = ReadFloat(json, "z", value.z);
  return value;
}

oneq::foundation::EulerAnglesDeg ReadEulerAngles(const std::string& json,
                                                 const oneq::foundation::EulerAnglesDeg& fallback) {
  oneq::foundation::EulerAnglesDeg value = fallback;
  value.yaw_deg = ReadFloat(json, "yaw_deg", value.yaw_deg);
  value.pitch_deg = ReadFloat(json, "pitch_deg", value.pitch_deg);
  value.roll_deg = ReadFloat(json, "roll_deg", value.roll_deg);
  return value;
}

model::EulerAnglesDeg ReadModelEulerAngles(const std::string& json,
                                           const model::EulerAnglesDeg& fallback) {
  model::EulerAnglesDeg value = fallback;
  value.yaw_deg = ReadFloat(json, "yaw_deg", value.yaw_deg);
  value.pitch_deg = ReadFloat(json, "pitch_deg", value.pitch_deg);
  value.roll_deg = ReadFloat(json, "roll_deg", value.roll_deg);
  return value;
}

model::AzimuthElevationDeg ReadAzEl(const std::string& json,
                                    const model::AzimuthElevationDeg& fallback) {
  model::AzimuthElevationDeg value = fallback;
  value.az_deg = ReadFloat(json, "az_deg", value.az_deg);
  value.el_deg = ReadFloat(json, "el_deg", value.el_deg);
  return value;
}

model::AzimuthElevationLimitsDeg ReadAzElLimits(const std::string& json,
                                                const model::AzimuthElevationLimitsDeg& fallback) {
  model::AzimuthElevationLimitsDeg value = fallback;
  value.az_min_deg = ReadFloat(json, "az_min_deg", value.az_min_deg);
  value.az_max_deg = ReadFloat(json, "az_max_deg", value.az_max_deg);
  value.el_min_deg = ReadFloat(json, "el_min_deg", value.el_min_deg);
  value.el_max_deg = ReadFloat(json, "el_max_deg", value.el_max_deg);
  return value;
}

model::CommandedBeamwidthDeg ReadCommandedBeamwidth(const std::string& json,
                                                    const model::CommandedBeamwidthDeg& fallback) {
  model::CommandedBeamwidthDeg value = fallback;
  value.commanded_az_beamwidth_deg =
      ReadFloat(json, "commanded_az_beamwidth_deg", value.commanded_az_beamwidth_deg);
  value.commanded_el_beamwidth_deg =
      ReadFloat(json, "commanded_el_beamwidth_deg", value.commanded_el_beamwidth_deg);
  return value;
}

oneq::foundation::PoseState ReadPoseState(const std::string& json,
                                          const oneq::foundation::PoseState& fallback) {
  oneq::foundation::PoseState value = fallback;
  const std::string position_json = ExtractRawJsonValue(json, "position_m");
  if (!position_json.empty()) {
    value.position_m = ReadVector3(position_json, value.position_m);
  }
  const std::string velocity_json = ExtractRawJsonValue(json, "velocity_mps");
  if (!velocity_json.empty()) {
    value.velocity_mps = ReadVector3(velocity_json, value.velocity_mps);
  }
  const std::string attitude_json = ExtractRawJsonValue(json, "attitude_deg");
  if (!attitude_json.empty()) {
    value.attitude_deg = ReadEulerAngles(attitude_json, value.attitude_deg);
  }
  return value;
}

config::DetectionConfig ReadDetectionConfig(const std::string& json,
                                            const config::DetectionConfig& fallback) {
  config::DetectionConfig value = fallback;
  value.enable_physics_detection =
      ReadBool(json, "enable_physics_detection", value.enable_physics_detection);
  value.swerling_model = static_cast<config::profiles::SwerlingModel>(
      ReadInt(json, "swerling_model", static_cast<int>(value.swerling_model)));
  value.min_detection_margin_db =
      ReadFloat(json, "min_detection_margin_db", value.min_detection_margin_db);
  value.pulse_count = ReadInt(json, "pulse_count", value.pulse_count);

  const std::string transmitter_json = ExtractRawJsonValue(json, "transmitter");
  if (!transmitter_json.empty()) {
    value.transmitter.peak_power_w =
        ReadFloat(transmitter_json, "peak_power_w", value.transmitter.peak_power_w);
    value.transmitter.frequency_hz =
        ReadFloat(transmitter_json, "frequency_hz", value.transmitter.frequency_hz);
    value.transmitter.bandwidth_hz =
        ReadFloat(transmitter_json, "bandwidth_hz", value.transmitter.bandwidth_hz);
    value.transmitter.pulse_width_s =
        ReadFloat(transmitter_json, "pulse_width_s", value.transmitter.pulse_width_s);
    value.transmitter.prf_hz = ReadFloat(transmitter_json, "prf_hz", value.transmitter.prf_hz);
    value.transmitter.transmit_loss_db =
        ReadFloat(transmitter_json, "transmit_loss_db", value.transmitter.transmit_loss_db);
  }

  const std::string antenna_json = ExtractRawJsonValue(json, "antenna");
  if (!antenna_json.empty()) {
    value.antenna.main_beam_gain_db =
        ReadFloat(antenna_json, "main_beam_gain_db", value.antenna.main_beam_gain_db);
    value.antenna.nominal_az_beamwidth_deg =
        ReadFloat(antenna_json, "nominal_az_beamwidth_deg", value.antenna.nominal_az_beamwidth_deg);
    value.antenna.nominal_el_beamwidth_deg =
        ReadFloat(antenna_json, "nominal_el_beamwidth_deg", value.antenna.nominal_el_beamwidth_deg);
    value.antenna.enable_directional_pattern = ReadBool(antenna_json, "enable_directional_pattern",
                                                        value.antenna.enable_directional_pattern);

    const std::string pattern_json = ExtractRawJsonValue(antenna_json, "pattern");
    if (!pattern_json.empty()) {
      value.antenna.pattern.model_type = static_cast<config::detection::AntennaPatternModelType>(
          ReadInt(pattern_json, "model_type", static_cast<int>(value.antenna.pattern.model_type)));
      value.antenna.pattern.max_sidelobe_level_db = ReadFloat(
          pattern_json, "max_sidelobe_level_db", value.antenna.pattern.max_sidelobe_level_db);
      value.antenna.pattern.backlobe_level_db =
          ReadFloat(pattern_json, "backlobe_level_db", value.antenna.pattern.backlobe_level_db);
      value.antenna.pattern.scan_loss_coeff_db_per_deg2 =
          ReadFloat(pattern_json, "scan_loss_coeff_db_per_deg2",
                    value.antenna.pattern.scan_loss_coeff_db_per_deg2);
      value.antenna.pattern.max_scan_loss_db =
          ReadFloat(pattern_json, "max_scan_loss_db", value.antenna.pattern.max_scan_loss_db);
      const std::string offset_json = ExtractRawJsonValue(pattern_json, "boresight_offset_deg");
      if (!offset_json.empty()) {
        value.antenna.pattern.boresight_offset_deg =
            ReadAzEl(offset_json, value.antenna.pattern.boresight_offset_deg);
      }
    }
  }

  const std::string receiver_json = ExtractRawJsonValue(json, "receiver");
  if (!receiver_json.empty()) {
    value.receiver.noise_figure_db =
        ReadFloat(receiver_json, "noise_figure_db", value.receiver.noise_figure_db);
    value.receiver.receive_loss_db =
        ReadFloat(receiver_json, "receive_loss_db", value.receiver.receive_loss_db);
  }

  const std::string policy_json = ExtractRawJsonValue(json, "detection_policy");
  if (!policy_json.empty()) {
    value.detection_policy.cfar_pfa =
        ReadFloat(policy_json, "cfar_pfa", value.detection_policy.cfar_pfa);
    value.detection_policy.min_snr_db =
        ReadFloat(policy_json, "min_snr_db", value.detection_policy.min_snr_db);
  }

  const std::string rcs_json = ExtractRawJsonValue(json, "rcs_physics");
  if (!rcs_json.empty()) {
    value.rcs_physics.enable_physical_rcs =
        ReadBool(rcs_json, "enable_physical_rcs", value.rcs_physics.enable_physical_rcs);
    value.rcs_physics.frequency_hz =
        ReadFloat(rcs_json, "frequency_hz", value.rcs_physics.frequency_hz);
    value.rcs_physics.physics_mix_ratio =
        ReadFloat(rcs_json, "physics_mix_ratio", value.rcs_physics.physics_mix_ratio);
    value.rcs_physics.cylinder_weight =
        ReadFloat(rcs_json, "cylinder_weight", value.rcs_physics.cylinder_weight);
    value.rcs_physics.min_equivalent_radius_m =
        ReadFloat(rcs_json, "min_equivalent_radius_m", value.rcs_physics.min_equivalent_radius_m);
    value.rcs_physics.max_equivalent_radius_m =
        ReadFloat(rcs_json, "max_equivalent_radius_m", value.rcs_physics.max_equivalent_radius_m);
    value.rcs_physics.min_rcs_m2 = ReadFloat(rcs_json, "min_rcs_m2", value.rcs_physics.min_rcs_m2);
    value.rcs_physics.max_rcs_m2 = ReadFloat(rcs_json, "max_rcs_m2", value.rcs_physics.max_rcs_m2);
    value.rcs_physics.bistatic_psi_offset_deg =
        ReadFloat(rcs_json, "bistatic_psi_offset_deg", value.rcs_physics.bistatic_psi_offset_deg);
  }

  return value;
}

model::RadarOrientationConfig ReadOrientationConfig(const std::string& json,
                                                    const model::RadarOrientationConfig& fallback) {
  model::RadarOrientationConfig value = fallback;
  const std::string mount_json = ExtractRawJsonValue(json, "mount_angles_deg");
  if (!mount_json.empty()) {
    value.mount_angles_deg = ReadModelEulerAngles(mount_json, value.mount_angles_deg);
  }
  const std::string center_json = ExtractRawJsonValue(json, "scan_center_deg");
  if (!center_json.empty()) {
    value.scan_center_deg = ReadAzEl(center_json, value.scan_center_deg);
  }
  const std::string mechanical_json = ExtractRawJsonValue(json, "mechanical_scan_limits_deg");
  if (!mechanical_json.empty()) {
    value.mechanical_scan_limits_deg =
        ReadAzElLimits(mechanical_json, value.mechanical_scan_limits_deg);
  }
  const std::string electronic_json = ExtractRawJsonValue(json, "electronic_scan_limits_deg");
  if (!electronic_json.empty()) {
    value.electronic_scan_limits_deg =
        ReadAzElLimits(electronic_json, value.electronic_scan_limits_deg);
  }
  value.scan_start_position = static_cast<oneq::foundation::ScanStartPosition>(
      ReadInt(json, "scan_start_position", static_cast<int>(value.scan_start_position)));
  value.scan_sequence = static_cast<oneq::foundation::ScanSequence>(
      ReadInt(json, "scan_sequence", static_cast<int>(value.scan_sequence)));
  value.work_sub_mode = static_cast<model::RadarWorkSubMode>(
      ReadInt(json, "work_sub_mode", static_cast<int>(value.work_sub_mode)));
  value.commanded_beamwidth_enabled =
      ReadBool(json, "commanded_beamwidth_enabled", value.commanded_beamwidth_enabled);
  const std::string beamwidth_json = ExtractRawJsonValue(json, "commanded_beamwidth_deg");
  if (!beamwidth_json.empty()) {
    value.commanded_beamwidth_deg =
        ReadCommandedBeamwidth(beamwidth_json, value.commanded_beamwidth_deg);
  }
  value.stabilization_mode = static_cast<model::StabilizationMode>(
      ReadInt(json, "stabilization_mode", static_cast<int>(value.stabilization_mode)));
  return value;
}

config::BeamControlConfig ReadBeamControlConfig(const std::string& json,
                                                const config::BeamControlConfig& fallback) {
  config::BeamControlConfig value = fallback;
  const std::string pointing_json = ExtractRawJsonValue(json, "pointing");
  if (!pointing_json.empty()) {
    const std::string center_json = ExtractRawJsonValue(pointing_json, "default_scan_center_deg");
    if (!center_json.empty()) {
      value.pointing.default_scan_center_deg =
          ReadAzEl(center_json, value.pointing.default_scan_center_deg);
    }
    const std::string beamwidth_json = ExtractRawJsonValue(pointing_json, "nominal_beamwidth_deg");
    if (!beamwidth_json.empty()) {
      value.pointing.nominal_beamwidth_deg =
          ReadCommandedBeamwidth(beamwidth_json, value.pointing.nominal_beamwidth_deg);
    }
  }
  const std::string scheduler_json = ExtractRawJsonValue(json, "scheduler");
  if (!scheduler_json.empty()) {
    value.scheduler.azimuth_step_count_hint = ReadUInt32(scheduler_json, "azimuth_step_count_hint",
                                                         value.scheduler.azimuth_step_count_hint);
    value.scheduler.elevation_step_count_hint = ReadUInt32(
        scheduler_json, "elevation_step_count_hint", value.scheduler.elevation_step_count_hint);
    value.scheduler.prefer_dense_tas_sampling = ReadBool(
        scheduler_json, "prefer_dense_tas_sampling", value.scheduler.prefer_dense_tas_sampling);
  }
  return value;
}

config::RadarPolicyConfig ReadPolicyConfig(const std::string& json,
                                           const config::RadarPolicyConfig& fallback) {
  config::RadarPolicyConfig value = fallback;
  const std::string beam_json = ExtractRawJsonValue(json, "beam_control");
  if (!beam_json.empty()) {
    value.beam_control = ReadBeamControlConfig(beam_json, value.beam_control);
  }
  const std::string association_json = ExtractRawJsonValue(json, "association");
  if (!association_json.empty()) {
    value.association.unassigned_cost =
        ReadFloat(association_json, "unassigned_cost", value.association.unassigned_cost);
    value.association.use_distance_gate_hint = ReadBool(association_json, "use_distance_gate_hint",
                                                        value.association.use_distance_gate_hint);
    value.association.distance_gate_sigma_hint = ReadFloat(
        association_json, "distance_gate_sigma_hint", value.association.distance_gate_sigma_hint);
  }
  const std::string tracking_json = ExtractRawJsonValue(json, "tracking");
  if (!tracking_json.empty()) {
    value.tracking.enable_kalman_filter =
        ReadBool(tracking_json, "enable_kalman_filter", value.tracking.enable_kalman_filter);
    value.tracking.kalman_measurement_noise_std = ReadFloat(
        tracking_json, "kalman_measurement_noise_std", value.tracking.kalman_measurement_noise_std);
    value.tracking.kalman_update_backend = static_cast<config::tracking::KalmanUpdateBackend>(
        ReadInt(tracking_json, "kalman_update_backend",
                static_cast<int>(value.tracking.kalman_update_backend)));
    value.tracking.speed_decay_ratio_on_loss = ReadFloat(tracking_json, "speed_decay_ratio_on_loss",
                                                         value.tracking.speed_decay_ratio_on_loss);
    value.tracking.rcs_decay_ratio_on_loss =
        ReadFloat(tracking_json, "rcs_decay_ratio_on_loss", value.tracking.rcs_decay_ratio_on_loss);
  }
  const std::string lifecycle_json = ExtractRawJsonValue(json, "lifecycle");
  if (!lifecycle_json.empty()) {
    value.lifecycle.confirm_hits =
        ReadUInt32(lifecycle_json, "confirm_hits", value.lifecycle.confirm_hits);
    value.lifecycle.max_miss_before_lost =
        ReadUInt32(lifecycle_json, "max_miss_before_lost", value.lifecycle.max_miss_before_lost);
    value.lifecycle.max_lost_cycles =
        ReadUInt32(lifecycle_json, "max_lost_cycles", value.lifecycle.max_lost_cycles);
    value.lifecycle.enable_imm_lifecycle =
        ReadBool(lifecycle_json, "enable_imm_lifecycle", value.lifecycle.enable_imm_lifecycle);
  }
  const std::string imm_json = ExtractRawJsonValue(json, "imm");
  if (!imm_json.empty()) {
    value.imm.enable_imm_lifecycle =
        ReadBool(imm_json, "enable_imm_lifecycle", value.imm.enable_imm_lifecycle);
    value.imm.model_count_hint =
        ReadUInt32(imm_json, "model_count_hint", value.imm.model_count_hint);
  }
  return value;
}

bool ParseSessionConfig(const std::string& payload_json, RadarSessionConfig* config,
                        std::string* error) {
  if (payload_json.empty()) {
    *error = "empty RadarSessionConfig payload";
    return false;
  }

  const std::string hardware_json = ExtractRawJsonValue(payload_json, "hardware");
  if (!hardware_json.empty()) {
    const std::string detection_json = ExtractRawJsonValue(hardware_json, "detection");
    if (!detection_json.empty()) {
      config->hardware.detection = ReadDetectionConfig(detection_json, config->hardware.detection);
    }
  }

  const std::string mission_json = ExtractRawJsonValue(payload_json, "mission");
  if (!mission_json.empty()) {
    const std::string orientation_json = ExtractRawJsonValue(mission_json, "orientation");
    if (!orientation_json.empty()) {
      config->mission.orientation =
          ReadOrientationConfig(orientation_json, config->mission.orientation);
    }
  }

  const std::string policy_json = ExtractRawJsonValue(payload_json, "policy");
  if (!policy_json.empty()) {
    config->policy = ReadPolicyConfig(policy_json, config->policy);
  }

  config->jamming_sensitivity_profile = static_cast<environment::JammingSensitivityProfile>(
      ReadInt(payload_json, "jamming_sensitivity_profile",
              static_cast<int>(config->jamming_sensitivity_profile)));
  return true;
}

model::TargetFeature ReadTargetFeature(const std::string& json) {
  model::TargetFeature target;
  target.external_target_id = ReadUInt64(json, "external_target_id", target.external_target_id);

  const std::string velocity_json = ExtractRawJsonValue(json, "velocity_mps");
  if (!velocity_json.empty()) {
    ReadFloatArray3(velocity_json, &target.current_track_velocity_x,
                    &target.current_track_velocity_y, &target.current_track_velocity_z);
  }

  target.current_track_speed = ReadFloat(json, "current_track_speed", target.current_track_speed);
  target.current_track_rcs = ReadFloat(json, "current_track_rcs", target.current_track_rcs);
  target.range_m = ReadFloat(json, "range_m", target.range_m);
  target.has_cartesian_position =
      ReadBool(json, "has_cartesian_position", target.has_cartesian_position);

  const std::string position_json = ExtractRawJsonValue(json, "position_m");
  if (!position_json.empty()) {
    ReadFloatArray3(position_json, &target.position_x, &target.position_y, &target.position_z);
  }

  target.target_swerling_type = ReadInt(json, "target_swerling_type", target.target_swerling_type);
  return target;
}

bool ParseCycleInput(const std::string& payload_json, RadarCycleInput* input, std::string* error) {
  if (payload_json.empty()) {
    *error = "empty RadarCycleInput payload";
    return false;
  }

  input->dt_sec = ReadFloat(payload_json, "dt_sec", input->dt_sec);

  const std::string pose_json = ExtractRawJsonValue(payload_json, "platform_pose");
  if (!pose_json.empty()) {
    input->platform_pose = ReadPoseState(pose_json, input->platform_pose);
  }

  const std::string targets_json = ExtractRawJsonValue(payload_json, "target_features");
  if (!targets_json.empty()) {
    const std::vector<std::string> targets = ExtractObjectArrayItems(targets_json);
    input->target_features.clear();
    for (std::size_t i = 0; i < targets.size(); ++i) {
      input->target_features.push_back(ReadTargetFeature(targets[i]));
    }
  }
  return true;
}

environment::JammerEmitterState ReadJammerEmitterState(const std::string& json) {
  environment::JammerEmitterState emitter;
  emitter.technique = static_cast<environment::JammingTechnique>(
      ReadInt(json, "technique", static_cast<int>(emitter.technique)));
  emitter.power_db = ReadFloat(json, "power_db", emitter.power_db);
  emitter.js_db = ReadFloat(json, "js_db", emitter.js_db);
  emitter.has_direction_deg = ReadBool(json, "has_direction_deg", emitter.has_direction_deg);
  emitter.azimuth_deg = ReadFloat(json, "azimuth_deg", emitter.azimuth_deg);
  emitter.elevation_deg = ReadFloat(json, "elevation_deg", emitter.elevation_deg);
  emitter.angular_span_deg = ReadFloat(json, "angular_span_deg", emitter.angular_span_deg);
  emitter.confidence = ReadFloat(json, "confidence", emitter.confidence);
  return emitter;
}

bool ParseSceneState(const std::string& payload_json,
                     environment::EnvironmentSceneState* scene_state, std::string* error) {
  if (payload_json.empty()) {
    *error = "empty EnvironmentSceneState payload";
    return false;
  }

  const std::string atmospheric_json = ExtractRawJsonValue(payload_json, "atmospheric_physics");
  if (!atmospheric_json.empty()) {
    scene_state->atmospheric_physics.enable_physical_model =
        ReadBool(atmospheric_json, "enable_physical_model",
                 scene_state->atmospheric_physics.enable_physical_model);
    scene_state->atmospheric_physics.pressure_hpa =
        ReadFloat(atmospheric_json, "pressure_hpa", scene_state->atmospheric_physics.pressure_hpa);
    scene_state->atmospheric_physics.temperature_k = ReadFloat(
        atmospheric_json, "temperature_k", scene_state->atmospheric_physics.temperature_k);
    scene_state->atmospheric_physics.relative_humidity = ReadFloat(
        atmospheric_json, "relative_humidity", scene_state->atmospheric_physics.relative_humidity);
  }

  const std::string context_json = ExtractRawJsonValue(payload_json, "atmospheric_context");
  if (!context_json.empty()) {
    scene_state->atmospheric_context.has_simulation_unix_seconds =
        ReadBool(context_json, "has_simulation_unix_seconds",
                 scene_state->atmospheric_context.has_simulation_unix_seconds);
    scene_state->atmospheric_context.simulation_unix_seconds =
        ReadInt64(context_json, "simulation_unix_seconds",
                  scene_state->atmospheric_context.simulation_unix_seconds);
    scene_state->atmospheric_context.solar_flux_f107a = ReadFloat(
        context_json, "solar_flux_f107a", scene_state->atmospheric_context.solar_flux_f107a);
    scene_state->atmospheric_context.solar_flux_f107 = ReadFloat(
        context_json, "solar_flux_f107", scene_state->atmospheric_context.solar_flux_f107);
    scene_state->atmospheric_context.geomagnetic_ap =
        ReadFloat(context_json, "geomagnetic_ap", scene_state->atmospheric_context.geomagnetic_ap);
  }

  const std::string vegetation_json =
      ExtractRawJsonValue(payload_json, "vegetation_scatter_physics");
  if (!vegetation_json.empty()) {
    scene_state->vegetation_scatter_physics.cover_profile =
        static_cast<environment::VegetationCoverProfile>(
            ReadInt(vegetation_json, "cover_profile",
                    static_cast<int>(scene_state->vegetation_scatter_physics.cover_profile)));
    scene_state->vegetation_scatter_physics.enable_physical_model =
        ReadBool(vegetation_json, "enable_physical_model",
                 scene_state->vegetation_scatter_physics.enable_physical_model);
  }

  const std::string jammers_json = ExtractRawJsonValue(payload_json, "jammer_emitters");
  if (!jammers_json.empty()) {
    const std::vector<std::string> emitters = ExtractObjectArrayItems(jammers_json);
    scene_state->jammer_emitters.clear();
    for (std::size_t i = 0; i < emitters.size(); ++i) {
      scene_state->jammer_emitters.push_back(ReadJammerEmitterState(emitters[i]));
    }
  }

  return true;
}

bool ParseEnvironmentScenarioConfig(const std::string& payload_json,
                                    environment::EnvironmentScenarioConfig* scenario_config,
                                    std::string* error) {
  environment::EnvironmentSceneState scene_state;
  scene_state.atmospheric_physics = scenario_config->atmospheric_physics;
  scene_state.atmospheric_context = scenario_config->atmospheric_context;
  scene_state.vegetation_scatter_physics = scenario_config->vegetation_scatter_physics;
  scene_state.jammer_emitters = scenario_config->jammer_sources;

  std::string scene_payload = payload_json;
  const std::string jammer_sources_json = ExtractRawJsonValue(payload_json, "jammer_sources");
  if (!jammer_sources_json.empty() &&
      ExtractRawJsonValue(payload_json, "jammer_emitters").empty()) {
    scene_payload = payload_json;
    const std::size_t key_pos = scene_payload.find("\"jammer_sources\":");
    if (key_pos != std::string::npos) {
      scene_payload.replace(key_pos, std::string("\"jammer_sources\"").size(),
                            "\"jammer_emitters\"");
    }
  }

  if (!ParseSceneState(scene_payload, &scene_state, error)) {
    return false;
  }

  scenario_config->atmospheric_physics = scene_state.atmospheric_physics;
  scenario_config->atmospheric_context = scene_state.atmospheric_context;
  scenario_config->vegetation_scatter_physics = scene_state.vegetation_scatter_physics;
  scenario_config->jammer_sources = scene_state.jammer_emitters;
  return true;
}

bool ParseEnvironmentRuntimeConfigPatch(const std::string& payload_json,
                                        environment::EnvironmentRuntimeConfigPatch* patch,
                                        std::string* error) {
  if (payload_json.empty()) {
    *error = "empty EnvironmentRuntimeConfigPatch payload";
    return false;
  }

  patch->has_scenario_config =
      ReadBool(payload_json, "has_scenario_config", patch->has_scenario_config);
  const std::string scenario_json = ExtractRawJsonValue(payload_json, "scenario_config");
  if (!scenario_json.empty()) {
    if (!ParseEnvironmentScenarioConfig(scenario_json, &patch->scenario_config, error)) {
      return false;
    }
  }

  patch->has_jamming_sensitivity_profile = ReadBool(payload_json, "has_jamming_sensitivity_profile",
                                                    patch->has_jamming_sensitivity_profile);
  patch->jamming_sensitivity_profile = static_cast<environment::JammingSensitivityProfile>(
      ReadInt(payload_json, "jamming_sensitivity_profile",
              static_cast<int>(patch->jamming_sensitivity_profile)));
  return true;
}

bool ParseRuntimeConfigPatch(const std::string& payload_json,
                             config::RadarRuntimeConfigPatch* patch, std::string* error) {
  if (payload_json.empty()) {
    *error = "empty RadarRuntimeConfigPatch payload";
    return false;
  }

  patch->has_mission = ReadBool(payload_json, "has_mission", patch->has_mission);
  const std::string mission_json = ExtractRawJsonValue(payload_json, "mission");
  if (!mission_json.empty()) {
    const std::string orientation_json = ExtractRawJsonValue(mission_json, "orientation");
    if (!orientation_json.empty()) {
      patch->mission.orientation =
          ReadOrientationConfig(orientation_json, patch->mission.orientation);
    }
  }

  patch->has_policy = ReadBool(payload_json, "has_policy", patch->has_policy);
  const std::string policy_json = ExtractRawJsonValue(payload_json, "policy");
  if (!policy_json.empty()) {
    patch->policy = ReadPolicyConfig(policy_json, patch->policy);
  }

  patch->has_environment_runtime_config = ReadBool(payload_json, "has_environment_runtime_config",
                                                   patch->has_environment_runtime_config);
  const std::string environment_json =
      ExtractRawJsonValue(payload_json, "environment_runtime_config");
  if (!environment_json.empty()) {
    if (!ParseEnvironmentRuntimeConfigPatch(environment_json, &patch->environment_runtime_config,
                                            error)) {
      return false;
    }
  }

  patch->has_work_sub_mode = ReadBool(payload_json, "has_work_sub_mode", patch->has_work_sub_mode);
  patch->work_sub_mode = static_cast<config::RadarWorkSubMode>(
      ReadInt(payload_json, "work_sub_mode", static_cast<int>(patch->work_sub_mode)));

  patch->has_scan_center_deg =
      ReadBool(payload_json, "has_scan_center_deg", patch->has_scan_center_deg);
  const std::string scan_center_json = ExtractRawJsonValue(payload_json, "scan_center_deg");
  if (!scan_center_json.empty()) {
    patch->scan_center_deg = ReadAzEl(scan_center_json, patch->scan_center_deg);
  }

  patch->has_dwell_center_deg =
      ReadBool(payload_json, "has_dwell_center_deg", patch->has_dwell_center_deg);
  const std::string dwell_center_json = ExtractRawJsonValue(payload_json, "dwell_center_deg");
  if (!dwell_center_json.empty()) {
    patch->dwell_center_deg = ReadAzEl(dwell_center_json, patch->dwell_center_deg);
  }

  patch->has_commanded_beamwidth_deg =
      ReadBool(payload_json, "has_commanded_beamwidth_deg", patch->has_commanded_beamwidth_deg);
  const std::string beamwidth_json = ExtractRawJsonValue(payload_json, "commanded_beamwidth_deg");
  if (!beamwidth_json.empty()) {
    patch->commanded_beamwidth_deg =
        ReadCommandedBeamwidth(beamwidth_json, patch->commanded_beamwidth_deg);
  }

  patch->has_commanded_beamwidth_enabled = ReadBool(payload_json, "has_commanded_beamwidth_enabled",
                                                    patch->has_commanded_beamwidth_enabled);
  patch->commanded_beamwidth_enabled =
      ReadBool(payload_json, "commanded_beamwidth_enabled", patch->commanded_beamwidth_enabled);
  return true;
}

std::string MakeOutputPayload(const output::TrackOutputFrame& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"TrackOutputFrame\","
         << "\"cycle_index\":" << output.cycle_index << ","
         << "\"published_track_count\":" << output.published_track_count << "}";
  return stream.str();
}

std::string MakeResultPayload(const RadarCycleResult& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"RadarCycleResult\","
         << "\"validation_issue_count\":" << output.validation_issues.size() << ","
         << "\"executed_this_cycle\":" << (output.executed_this_cycle ? "true" : "false") << "}";
  return stream.str();
}

struct RadarReplayState {
  std::unique_ptr<RadarSession> session{};
  RadarCycleInput pending_input{};
  bool has_pending_input{false};
  environment::EnvironmentSceneState pending_scene_state{};
  bool has_pending_scene_state{false};
  RadarCycleResult latest_result{};
  output::TrackOutputFrame latest_frame{};
  std::string latest_result_payload{};
  std::string latest_frame_payload{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload_json{};
};

bool TrackOutputSummaryEqual(const output::TrackOutputFrame& left,
                             const output::TrackOutputFrame& right) {
  return left.cycle_index == right.cycle_index &&
         left.published_track_count == right.published_track_count;
}

bool CycleResultSummaryEqual(const RadarCycleResult& left, const RadarCycleResult& right) {
  return TrackOutputSummaryEqual(left.track_output_frame, right.track_output_frame) &&
         left.validation_issues.size() == right.validation_issues.size() &&
         left.executed_this_cycle == right.executed_this_cycle;
}

bool ExecutePendingCycle(RadarReplayState* state, std::string* error) {
  if (!state->session) {
    *error = "AR replay cannot execute before session_config";
    return false;
  }
  if (!state->has_pending_input) {
    *error = "AR replay cycle_output arrived before cycle_input";
    return false;
  }

  RadarCycleResult result;
  if (state->has_pending_scene_state) {
    result = state->session->StepWithResult(state->pending_input, state->pending_scene_state);
  } else {
    result = state->session->StepWithResult(state->pending_input);
  }
  state->latest_result = result;
  state->latest_frame = result.track_output_frame;
  state->latest_result_payload = MakeResultPayload(result);
  state->latest_frame_payload = MakeOutputPayload(result.track_output_frame);
  state->has_pending_input = false;
  state->has_pending_scene_state = false;
  return true;
}

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "RadarSessionConfig") {
    *error = "AR replay expected RadarSessionConfig session_config";
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  RadarSessionConfig config;
#if defined(_WIN32)
  if (event.payload_encoding == "flatbuffers") {
    if (!DecodeSessionConfigFlatbuffer(event.payload_bytes, &config, error)) {
      return false;
    }
  } else
#endif
      if (!ParseSessionConfig(event.payload_json, &config, error)) {
    return false;
  }
  state->session.reset(new RadarSession(RadarSessionFactory::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  if (event.payload_type != "RadarCycleInput") {
    *error = "AR replay expected RadarCycleInput cycle_input";
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received cycle_input before session_config";
    return false;
  }

  RadarCycleInput input;
#if defined(_WIN32)
  if (event.payload_encoding == "flatbuffers") {
    if (!DecodeCycleInputFlatbuffer(event.payload_bytes, &input, error)) {
      return false;
    }
  } else
#endif
      if (!ParseCycleInput(event.payload_json, &input, error)) {
    return false;
  }

  state->pending_input = input;
  state->has_pending_input = true;
  state->has_pending_scene_state = false;
  return true;
}

bool OnSceneState(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  if (event.payload_type != "EnvironmentSceneState") {
    *error = "AR replay expected EnvironmentSceneState scene_state";
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  environment::EnvironmentSceneState scene_state;
#if defined(_WIN32)
  if (event.payload_encoding == "flatbuffers") {
    if (!DecodeSceneStateFlatbuffer(event.payload_bytes, &scene_state, error)) {
      return false;
    }
  } else
#endif
      if (!ParseSceneState(event.payload_json, &scene_state, error)) {
    return false;
  }
  state->pending_scene_state = scene_state;
  state->has_pending_scene_state = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  if (event.payload_type != "RadarRuntimeConfigPatch") {
    *error = "AR replay expected RadarRuntimeConfigPatch runtime_config_patch";
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received runtime_config_patch before session_config";
    return false;
  }

  config::RadarRuntimeConfigPatch patch;
#if defined(_WIN32)
  if (event.payload_encoding == "flatbuffers") {
    if (!DecodeRuntimeConfigPatchFlatbuffer(event.payload_bytes, &patch, error)) {
      return false;
    }
  } else
#endif
      if (!ParseRuntimeConfigPatch(event.payload_json, &patch, error)) {
    return false;
  }
  state->session->ApplyRuntimeConfig(patch);
  return true;
}

bool OnCycleOutput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                   std::string* actual_output_json, std::string* error) {
  RadarCycleResult expected_result;
  output::TrackOutputFrame expected_frame;
  bool has_expected_result = false;
  bool has_expected_frame = false;
#if defined(_WIN32)
  if (event.payload_encoding == "flatbuffers") {
    if (event.payload_type == "RadarCycleResult") {
      if (!DecodeCycleResultFlatbuffer(event.payload_bytes, &expected_result, error)) {
        return false;
      }
      has_expected_result = true;
    } else if (event.payload_type == "TrackOutputFrame") {
      if (!DecodeTrackOutputFrameFlatbuffer(event.payload_bytes, &expected_frame, error)) {
        return false;
      }
      has_expected_frame = true;
    } else {
      *error = "AR replay does not support cycle_output payload type: " + event.payload_type;
      return false;
    }
  }
#endif

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  if (!ExecutePendingCycle(state, error)) {
    return false;
  }

#if defined(_WIN32)
  if (event.payload_encoding == "flatbuffers") {
    if (event.payload_type == "RadarCycleResult") {
      const bool match =
          has_expected_result && CycleResultSummaryEqual(expected_result, state->latest_result);
      *actual_output_json = match ? event.payload_json : state->latest_result_payload;
      return true;
    }
    if (event.payload_type == "TrackOutputFrame") {
      const bool match =
          has_expected_frame && TrackOutputSummaryEqual(expected_frame, state->latest_frame);
      *actual_output_json = match ? event.payload_json : state->latest_frame_payload;
      return true;
    }
  }
#endif

  if (event.payload_type == "RadarCycleResult") {
    *actual_output_json = state->latest_result_payload;
    return true;
  }
  if (event.payload_type == "TrackOutputFrame") {
    *actual_output_json = state->latest_frame_payload;
    return true;
  }
  *error = "AR replay does not support cycle_output payload type: " + event.payload_type;
  return false;
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* /*error*/) {
  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload_json = event.payload_json;
  return true;
}

}  // namespace

RadarReplaySessionResult ReplayRadarTrace(const std::string& trace_dir) {
  RadarReplaySessionResult result;

  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "airborne_radar";
  expectation.require_module_match = true;

  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.ok = false;
    result.first_error = result.report.first_error;
    return result;
  }

  RadarReplayState state;
  oneq::replay::ReplayTracePlaybackCallbacks callbacks;
  callbacks.user_data = &state;
  callbacks.on_session_config = OnSessionConfig;
  callbacks.on_cycle_input = OnCycleInput;
  callbacks.on_scene_state = OnSceneState;
  callbacks.on_runtime_config_patch = OnRuntimeConfigPatch;
  callbacks.on_cycle_output = OnCycleOutput;
  callbacks.on_failure_marker = OnFailureMarker;

  oneq::replay::ReplayTracePlaybackOptions options;
  options.require_output_callback = true;
  options.stop_on_first_divergence = true;
  options.stop_on_failure_marker = true;

  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
  result.reached_failure_marker = state.reached_failure_marker;
  result.failure_marker_payload_json = state.failure_marker_payload_json;
  if (!result.ok) {
    result.first_error = result.playback.first_error;
  }
  return result;
}

}  // namespace session
}  // namespace airborne_radar
