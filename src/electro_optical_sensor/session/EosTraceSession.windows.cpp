#include "1q/electro_optical_sensor/session/EosTraceSession.h"

#include <sstream>
#include <string>

namespace electro_optical_sensor {
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

std::string MakeInputPayload(const EosCycleInput& input) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EosCycleInput\","
         << "\"cycle_index\":" << input.cycle_index << ","
         << "\"dt_sec\":" << input.dt_sec << ","
         << "\"scene_target_count\":" << input.scene_targets.size()
         << "}";
  return stream.str();
}

std::string MakeOutputPayload(const output::EosOutputFrame& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EosOutputFrame\","
         << "\"cycle_index\":" << output.cycle_index << ","
         << "\"detection_count\":" << output.detections.size()
         << "}";
  return stream.str();
}

std::string MakeResultPayload(const model::EosCycleResult& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EosCycleResult\","
         << "\"has_validation_error\":" << (output.has_validation_error ? "true" : "false")
         << ","
         << "\"executed_this_cycle\":" << (output.executed_this_cycle ? "true" : "false")
         << "}";
  return stream.str();
}

}  // namespace

EosTraceSession::EosTraceSession(EosSessionConfig config, EosTraceSessionOptions options)
    : session_(EosSessionFactory::Create(config)), sink_(std::move(options.sink)) {
  if (sink_ && options.trace_config_on_construct) {
    Record("config", MakeFlatbuffersPayload("EosSessionConfig", config));
  }
}

output::EosOutputFrame EosTraceSession::Step(const EosCycleInput& input) {
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const output::EosOutputFrame output = session_.Step(input);
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

model::EosCycleResult EosTraceSession::StepWithResult(const EosCycleInput& input) {
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const model::EosCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

void EosTraceSession::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config_patch", MakeFlatbuffersPayload("EosRuntimeConfigPatch", patch));
  }
}

EosSession& EosTraceSession::session() { return session_; }

const EosSession& EosTraceSession::session() const { return session_; }

void EosTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("electro_optical_sensor", phase, payload_json);
}

}  // namespace session
}  // namespace electro_optical_sensor
