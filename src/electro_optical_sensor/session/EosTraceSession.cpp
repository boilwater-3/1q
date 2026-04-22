#include "1q/electro_optical_sensor/session/EosTraceSession.h"

#include <string>

#include "EosReplayFlatbufferCodec.h"
#include "1q/replay/ReplayTrace.h"

namespace electro_optical_sensor {
namespace session {

EosTraceSession::EosTraceSession(EosSessionConfig config, EosTraceSessionOptions options)
    : session_(EosSessionFactory::Create(config)),
      sink_(std::move(options.sink)),
      replay_writer_(std::move(options.replay_writer)) {
  if (replay_writer_ && options.trace_config_on_construct) {
    WriteReplayEvent("session_config", "EosSessionConfig",
                     EncodeEosSessionConfig(config));
  }
}

output::EosOutputFrame EosTraceSession::Step(const EosCycleInput& input) {
  if (replay_writer_) {
    WriteReplayEvent("cycle_input", "EosCycleInput",
                     EncodeEosCycleInput(input), input.cycle_index);
    pending_input_written_ = true;
  }
  if (sink_) {
    sink_->Record("electro_optical_sensor", "input", std::to_string(input.cycle_index));
  }
  const output::EosOutputFrame output = session_.Step(input);
  if (replay_writer_) {
    WriteReplayEvent("cycle_output", "EosOutputFrame",
                     EncodeEosOutputFrame(output), output.cycle_index);
    pending_input_written_ = false;
  }
  return output;
}

model::EosCycleResult EosTraceSession::StepWithResult(const EosCycleInput& input) {
  if (replay_writer_) {
    if (pending_input_written_) {
      // P2-B: 连续两次 cycle_input 无中间 output，记录 warning 后继续
      oneq::replay::ReplayTraceEvent warn_ev;
      warn_ev.module = "electro_optical_sensor";
      warn_ev.event_type = "warning";
      warn_ev.payload_type = "ConsecutiveCycleInputWarning";
      warn_ev.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      replay_writer_->WriteEvent(warn_ev);
    }
    WriteReplayEvent("cycle_input", "EosCycleInput",
                     EncodeEosCycleInput(input), input.cycle_index);
    pending_input_written_ = true;
  }
  const model::EosCycleResult result = session_.StepWithResult(input);
  if (replay_writer_) {
    WriteReplayEvent("cycle_output", "EosCycleResult",
                     EncodeEosCycleResult(result), result.output_frame.cycle_index);
    pending_input_written_ = false;
    // P1-A: 自动 failure_marker
    if (result.has_validation_error) {
      oneq::replay::ReplayTraceFailure failure;
      failure.error_code = "validation_error";
      failure.message = "EosCycleResult has_validation_error=true";
      failure.cycle_index = result.output_frame.cycle_index;
      failure.has_cycle_index = true;
      replay_writer_->WriteFailureMarker(failure);
    }
  }
  return result;
}

void EosTraceSession::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  if (replay_writer_) {
    // 先写后 apply，保证回放时配置变更在执行前可重放
    WriteReplayEvent("runtime_config_patch", "EosRuntimeConfigPatch",
                     EncodeEosRuntimeConfigPatch(patch));
  }
  session_.ApplyRuntimeConfig(patch);
}

EosSession& EosTraceSession::session() { return session_; }
const EosSession& EosTraceSession::session() const { return session_; }

void EosTraceSession::WriteReplayEvent(const std::string& event_type,
                                       const std::string& payload_type,
                                       const std::string& payload_bytes) const {
  oneq::replay::ReplayTraceEvent ev;
  ev.module = "electro_optical_sensor";
  ev.event_type = event_type;
  ev.payload_type = payload_type;
  ev.payload_encoding = "flatbuffers";
  ev.payload_bytes = payload_bytes;
  replay_writer_->WriteEvent(ev);
}

void EosTraceSession::WriteReplayEvent(const std::string& event_type,
                                       const std::string& payload_type,
                                       const std::string& payload_bytes,
                                       std::uint32_t cycle_index) const {
  oneq::replay::ReplayTraceEvent ev;
  ev.module = "electro_optical_sensor";
  ev.event_type = event_type;
  ev.payload_type = payload_type;
  ev.payload_encoding = "flatbuffers";
  ev.payload_bytes = payload_bytes;
  ev.has_cycle_index = true;
  ev.cycle_index = cycle_index;
  replay_writer_->WriteEvent(ev);
}

}  // namespace session
}  // namespace electro_optical_sensor
