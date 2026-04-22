#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"

#include <string>

#include "EsrReplayFlatbufferCodec.h"
#include "1q/replay/ReplayTrace.h"

namespace electronic_surveillance_radar {
namespace session {

EsrTraceSession::EsrTraceSession(EsrSessionConfig config, EsrTraceSessionOptions options)
    : session_(config),
      sink_(std::move(options.sink)),
      replay_writer_(std::move(options.replay_writer)) {
  if (replay_writer_ && options.trace_config_on_construct) {
    WriteReplayEvent("session_config", "EsrSessionConfig",
                     EncodeEsrSessionConfig(config));
  }
}

output::EsrOutputFrame EsrTraceSession::Step(const EsrCycleInput& input) {
  if (replay_writer_) {
    WriteReplayEvent("cycle_input", "EsrCycleInput",
                     EncodeEsrCycleInput(input), input.cycle_index);
    pending_input_written_ = true;
  }
  const output::EsrOutputFrame output = session_.Step(input);
  if (replay_writer_) {
    WriteReplayEvent("cycle_output", "EsrOutputFrame",
                     EncodeEsrOutputFrame(output),
                     output.observation_output.cycle_index);
    pending_input_written_ = false;
  }
  return output;
}

EsrCycleResult EsrTraceSession::StepWithResult(const EsrCycleInput& input) {
  if (replay_writer_) {
    if (pending_input_written_) {
      oneq::replay::ReplayTraceEvent warn;
      warn.module = "electronic_surveillance_radar";
      warn.event_type = "warning";
      warn.payload_type = "ConsecutiveCycleInputWarning";
      warn.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      replay_writer_->WriteEvent(warn);
    }
    WriteReplayEvent("cycle_input", "EsrCycleInput",
                     EncodeEsrCycleInput(input), input.cycle_index);
    pending_input_written_ = true;
  }
  const EsrCycleResult result = session_.StepWithResult(input);
  if (replay_writer_) {
    WriteReplayEvent("cycle_output", "EsrCycleResult",
                     EncodeEsrCycleResult(result),
                     result.output_frame.observation_output.cycle_index);
    pending_input_written_ = false;
    if (result.has_validation_error) {
      oneq::replay::ReplayTraceFailure failure;
      failure.error_code = "validation_error";
      failure.message = "EsrCycleResult has_validation_error=true";
      failure.cycle_index = result.output_frame.observation_output.cycle_index;
      failure.has_cycle_index = true;
      replay_writer_->WriteFailureMarker(failure);
    }
  }
  return result;
}

void EsrTraceSession::ApplyRuntimeConfig(const EsrRuntimeConfigPatch& patch) {
  if (replay_writer_) {
    WriteReplayEvent("runtime_config_patch", "EsrRuntimeConfigPatch",
                     EncodeEsrRuntimeConfigPatch(patch));
  }
  session_.ApplyRuntimeConfig(patch);
}

EsrSession& EsrTraceSession::session() { return session_; }
const EsrSession& EsrTraceSession::session() const { return session_; }

void EsrTraceSession::WriteReplayEvent(const std::string& event_type,
                                       const std::string& payload_type,
                                       const std::string& payload_bytes) const {
  oneq::replay::ReplayTraceEvent ev;
  ev.module = "electronic_surveillance_radar";
  ev.event_type = event_type;
  ev.payload_type = payload_type;
  ev.payload_encoding = "flatbuffers";
  ev.payload_bytes = payload_bytes;
  replay_writer_->WriteEvent(ev);
}

void EsrTraceSession::WriteReplayEvent(const std::string& event_type,
                                       const std::string& payload_type,
                                       const std::string& payload_bytes,
                                       std::uint32_t cycle_index) const {
  oneq::replay::ReplayTraceEvent ev;
  ev.module = "electronic_surveillance_radar";
  ev.event_type = event_type;
  ev.payload_type = payload_type;
  ev.payload_encoding = "flatbuffers";
  ev.payload_bytes = payload_bytes;
  ev.has_cycle_index = true;
  ev.cycle_index = cycle_index;
  replay_writer_->WriteEvent(ev);
}

}  // namespace session
}  // namespace electronic_surveillance_radar
