#include "1q/airborne_radar/session/RadarReplaySession.h"

#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/RadarSessionFactory.h"

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

std::uint64_t ReadUInt64(const std::string& json, const char* name,
                         std::uint64_t fallback) {
  const std::size_t value_pos = FindFieldValueStart(json, name);
  if (value_pos == std::string::npos) {
    return fallback;
  }
  return static_cast<std::uint64_t>(std::strtoull(json.c_str() + value_pos, nullptr, 10));
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

bool ReadFloatArray3(const std::string& array_json, float* first, float* second,
                     float* third) {
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

oneq::foundation::EulerAnglesDeg ReadEulerAngles(
    const std::string& json,
    const oneq::foundation::EulerAnglesDeg& fallback) {
  oneq::foundation::EulerAnglesDeg value = fallback;
  value.yaw_deg = ReadFloat(json, "yaw_deg", value.yaw_deg);
  value.pitch_deg = ReadFloat(json, "pitch_deg", value.pitch_deg);
  value.roll_deg = ReadFloat(json, "roll_deg", value.roll_deg);
  return value;
}

oneq::foundation::PoseState ReadPoseState(
    const std::string& json,
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

model::TargetFeature ReadTargetFeature(const std::string& json) {
  model::TargetFeature target;
  target.external_target_id = ReadUInt64(json, "external_target_id",
                                         target.external_target_id);

  const std::string velocity_json = ExtractRawJsonValue(json, "velocity_mps");
  if (!velocity_json.empty()) {
    ReadFloatArray3(velocity_json, &target.current_track_velocity_x,
                    &target.current_track_velocity_y,
                    &target.current_track_velocity_z);
  }

  target.current_track_speed = ReadFloat(json, "current_track_speed",
                                         target.current_track_speed);
  target.current_track_rcs = ReadFloat(json, "current_track_rcs",
                                       target.current_track_rcs);
  target.range_m = ReadFloat(json, "range_m", target.range_m);
  target.has_cartesian_position = ReadBool(json, "has_cartesian_position",
                                           target.has_cartesian_position);

  const std::string position_json = ExtractRawJsonValue(json, "position_m");
  if (!position_json.empty()) {
    ReadFloatArray3(position_json, &target.position_x, &target.position_y,
                    &target.position_z);
  }

  target.target_swerling_type = ReadInt(json, "target_swerling_type",
                                        target.target_swerling_type);
  return target;
}

bool ParseCycleInput(const std::string& payload_json, RadarCycleInput* input,
                     std::string* error) {
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

std::string MakeOutputPayload(const output::TrackOutputFrame& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"TrackOutputFrame\","
         << "\"cycle_index\":" << output.cycle_index << ","
         << "\"published_track_count\":" << output.published_track_count
         << "}";
  return stream.str();
}

std::string MakeResultPayload(const RadarCycleResult& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"RadarCycleResult\","
         << "\"validation_issue_count\":" << output.validation_issues.size() << ","
         << "\"executed_this_cycle\":" << (output.executed_this_cycle ? "true" : "false")
         << "}";
  return stream.str();
}

struct RadarReplayState {
  std::unique_ptr<RadarSession> session{};
  std::string latest_result_payload{};
  std::string latest_frame_payload{};
};

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "RadarSessionConfig") {
    *error = "AR replay expected RadarSessionConfig session_config";
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  RadarSessionConfig config;
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
  if (!ParseCycleInput(event.payload_json, &input, error)) {
    return false;
  }

  const RadarCycleResult result = state->session->StepWithResult(input);
  state->latest_result_payload = MakeResultPayload(result);
  state->latest_frame_payload = MakeOutputPayload(result.track_output_frame);
  return true;
}

bool OnCycleOutput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                   std::string* actual_output_json, std::string* error) {
  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
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
  callbacks.on_cycle_output = OnCycleOutput;

  oneq::replay::ReplayTracePlaybackOptions options;
  options.require_output_callback = true;
  options.stop_on_first_divergence = true;

  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
  if (!result.ok) {
    result.first_error = result.playback.first_error;
  }
  return result;
}

}  // namespace session
}  // namespace airborne_radar
