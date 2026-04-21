#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"

#include <sstream>
#include <string>

namespace electronic_surveillance_radar {
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

std::string MakeInputPayload(const session::EsrCycleInput& input) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EsrCycleInput\","
         << "\"cycle_index\":" << input.cycle_index << ","
         << "\"dt_sec\":" << input.dt_sec << ","
         << "\"scene_emitter_count\":" << input.scene_emitters.size()
         << "}";
  return stream.str();
}

std::string MakeOutputPayload(const output::EsrOutputFrame& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EsrOutputFrame\","
         << "\"observation_count\":"
         << output.observation_output.observations.size() << ","
         << "\"hypothesis_count\":" << output.emitter_output.hypotheses.size()
         << "}";
  return stream.str();
}

std::string MakeResultPayload(const session::EsrCycleResult& result) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EsrCycleResult\","
         << "\"has_validation_error\":" << (result.has_validation_error ? "true" : "false")
         << ","
         << "\"validation_issue_count\":" << result.validation_issues.size()
         << "}";
  return stream.str();
}

}  // namespace

EsrTraceSession::EsrTraceSession(session::EsrSessionConfig config, EsrTraceSessionOptions options)
    : session_(config), sink_(std::move(options.sink)) {
  if (sink_ && options.trace_config_on_construct) {
    Record("config", MakeFlatbuffersPayload("EsrSessionConfig", config));
  }
}

output::EsrOutputFrame EsrTraceSession::Step(const session::EsrCycleInput& input) {
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const output::EsrOutputFrame output = session_.Step(input);
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

session::EsrCycleResult EsrTraceSession::StepWithResult(const session::EsrCycleInput& input) {
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const session::EsrCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

void EsrTraceSession::ApplyRuntimeConfig(const session::EsrRuntimeConfigPatch& patch) {
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config_patch", MakeFlatbuffersPayload("EsrRuntimeConfigPatch", patch));
  }
}

session::EsrSession& EsrTraceSession::session() { return session_; }

const session::EsrSession& EsrTraceSession::session() const { return session_; }

void EsrTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("electronic_surveillance_radar", phase, payload_json);
}

}  // namespace session
}  // namespace electronic_surveillance_radar

