#include "1q/airborne_radar/session/RadarTraceSession.h"

#include <cstddef>
#include <sstream>
#include <string>

#include "1q/airborne_radar/session/RadarSessionFactory.h"

namespace airborne_radar {
namespace session {
namespace {

template <typename T>
std::string MakeFlatbuffersPayload(const char* type_name, const T& /*value*/) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"" << type_name << "\""
         << "}";
  return stream.str();
}

std::string MakeInputPayload(const RadarCycleInput& input) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"RadarCycleInput\","
         << "\"dt_sec\":" << input.dt_sec << ","
         << "\"platform_pose\":{"
         << "\"position_m\":[" << input.platform_pose.position_m.x << ","
         << input.platform_pose.position_m.y << "," << input.platform_pose.position_m.z << "],"
         << "\"velocity_mps\":[" << input.platform_pose.velocity_mps.x << ","
         << input.platform_pose.velocity_mps.y << "," << input.platform_pose.velocity_mps.z
         << "],"
         << "\"attitude_deg\":{"
         << "\"yaw_deg\":" << input.platform_pose.attitude_deg.yaw_deg << ","
         << "\"pitch_deg\":" << input.platform_pose.attitude_deg.pitch_deg << ","
         << "\"roll_deg\":" << input.platform_pose.attitude_deg.roll_deg << "}"
         << "},"
         << "\"target_features\":[";
  for (std::size_t i = 0; i < input.target_features.size(); ++i) {
    const model::TargetFeature& target = input.target_features[i];
    if (i > 0U) {
      stream << ",";
    }
    stream << "{"
           << "\"external_target_id\":" << target.external_target_id << ","
           << "\"velocity_mps\":[" << target.current_track_velocity_x << ","
           << target.current_track_velocity_y << "," << target.current_track_velocity_z << "],"
           << "\"current_track_speed\":" << target.current_track_speed << ","
           << "\"current_track_rcs\":" << target.current_track_rcs << ","
           << "\"range_m\":" << target.range_m << ","
           << "\"has_cartesian_position\":"
           << (target.has_cartesian_position ? "true" : "false") << ","
           << "\"position_m\":[" << target.position_x << "," << target.position_y << ","
           << target.position_z << "],"
           << "\"target_swerling_type\":" << target.target_swerling_type << "}";
  }
  stream << "]}";
  return stream.str();
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

std::string MakeSceneInputPayload(const RadarCycleInput& input,
                                  const environment::EnvironmentSceneState& scene_state) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"RadarCycleInputWithScene\","
         << "\"cycle_input\":" << MakeInputPayload(input) << ","
         << "\"jammer_emitter_count\":" << scene_state.jammer_emitters.size()
         << "}";
  return stream.str();
}

}  // namespace

RadarTraceSession::RadarTraceSession(const RadarSessionConfig& config,
                                     RadarTraceSessionOptions options)
    : session_(RadarSessionFactory::Create(config)),
      sink_(std::move(options.sink)),
      replay_writer_(std::move(options.replay_writer)) {
  if (replay_writer_ && options.trace_config_on_construct) {
    RecordReplay("session_config", "RadarSessionConfig",
                 MakeFlatbuffersPayload("RadarSessionConfig", config));
  }
  if (sink_ && options.trace_config_on_construct) {
    Record("config", MakeFlatbuffersPayload("RadarSessionConfig", config));
  }
}

output::TrackOutputFrame RadarTraceSession::Step(const RadarCycleInput& input) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "RadarCycleInput", MakeInputPayload(input));
  }
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const output::TrackOutputFrame output = session_.Step(input);
  if (replay_writer_) {
    RecordReplay("cycle_output", "TrackOutputFrame", MakeOutputPayload(output),
                 output.cycle_index);
  }
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

output::TrackOutputFrame RadarTraceSession::Step(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "RadarCycleInput", MakeInputPayload(input));
    RecordReplay("scene_state", "EnvironmentSceneState",
                 MakeFlatbuffersPayload("EnvironmentSceneState", scene_state));
  }
  if (sink_) {
    Record("input", MakeSceneInputPayload(input, scene_state));
  }
  const output::TrackOutputFrame output = session_.Step(input, scene_state);
  if (replay_writer_) {
    RecordReplay("cycle_output", "TrackOutputFrame", MakeOutputPayload(output),
                 output.cycle_index);
  }
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(const RadarCycleInput& input) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "RadarCycleInput", MakeInputPayload(input));
  }
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const RadarCycleResult output = session_.StepWithResult(input);
  if (replay_writer_) {
    RecordReplay("cycle_output", "RadarCycleResult", MakeResultPayload(output),
                 output.track_output_frame.cycle_index);
  }
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "RadarCycleInput", MakeInputPayload(input));
    RecordReplay("scene_state", "EnvironmentSceneState",
                 MakeFlatbuffersPayload("EnvironmentSceneState", scene_state));
  }
  if (sink_) {
    Record("input", MakeSceneInputPayload(input, scene_state));
  }
  const RadarCycleResult output = session_.StepWithResult(input, scene_state);
  if (replay_writer_) {
    RecordReplay("cycle_output", "RadarCycleResult", MakeResultPayload(output),
                 output.track_output_frame.cycle_index);
  }
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

void RadarTraceSession::ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch) {
  if (replay_writer_) {
    RecordReplay("runtime_config_patch", "RadarRuntimeConfigPatch",
                 MakeFlatbuffersPayload("RadarRuntimeConfigPatch", patch));
  }
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config", MakeFlatbuffersPayload("RadarRuntimeConfigPatch", patch));
  }
}

const std::vector<extension::control::RadarCommand>& RadarTraceSession::GetSubmittedCommands()
    const {
  return session_.GetSubmittedCommands();
}

bool RadarTraceSession::HasLatestControlProfile() const {
  return session_.HasLatestControlProfile();
}

const extension::control::RadarControlProfile& RadarTraceSession::GetLatestControlProfile() const {
  return session_.GetLatestControlProfile();
}

extension::AssociationQualityMetrics RadarTraceSession::GetLastAssociationQualityMetrics() const {
  return session_.GetLastAssociationQualityMetrics();
}

RadarSession& RadarTraceSession::session() { return session_; }

const RadarSession& RadarTraceSession::session() const { return session_; }

void RadarTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("airborne_radar", phase, payload_json);
}

void RadarTraceSession::RecordReplay(const std::string& event_type,
                                     const std::string& payload_type,
                                     const std::string& payload_json) const {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_json = payload_json;
  replay_writer_->WriteEvent(event);
}

void RadarTraceSession::RecordReplay(const std::string& event_type,
                                     const std::string& payload_type,
                                     const std::string& payload_json,
                                     std::uint32_t cycle_index) const {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_json = payload_json;
  event.has_cycle_index = true;
  event.cycle_index = cycle_index;
  replay_writer_->WriteEvent(event);
}

}  // namespace session
}  // namespace airborne_radar
