#include "1q/airborne_radar/session/RadarTraceSession.h"

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
         << "\"target_feature_count\":" << input.target_features.size()
         << "}";
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
         << "\"dt_sec\":" << input.dt_sec << ","
         << "\"target_feature_count\":" << input.target_features.size() << ","
         << "\"jammer_emitter_count\":" << scene_state.jammer_emitters.size()
         << "}";
  return stream.str();
}

}  // namespace

RadarTraceSession::RadarTraceSession(const RadarSessionConfig& config,
                                     RadarTraceSessionOptions options)
    : session_(RadarSessionFactory::Create(config)), sink_(std::move(options.sink)) {
  if (sink_ && options.trace_config_on_construct) {
    Record("config", MakeFlatbuffersPayload("RadarSessionConfig", config));
  }
}

output::TrackOutputFrame RadarTraceSession::Step(const RadarCycleInput& input) {
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const output::TrackOutputFrame output = session_.Step(input);
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

output::TrackOutputFrame RadarTraceSession::Step(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (sink_) {
    Record("input", MakeSceneInputPayload(input, scene_state));
  }
  const output::TrackOutputFrame output = session_.Step(input, scene_state);
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(const RadarCycleInput& input) {
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const RadarCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (sink_) {
    Record("input", MakeSceneInputPayload(input, scene_state));
  }
  const RadarCycleResult output = session_.StepWithResult(input, scene_state);
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

void RadarTraceSession::ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch) {
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

}  // namespace session
}  // namespace airborne_radar

