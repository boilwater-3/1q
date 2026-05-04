#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"

#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "EsrReplayFlatbufferCodec.h"

namespace electronic_surveillance_radar {
namespace session {

struct EsrTraceSession::Impl {
  Impl(EsrSessionConfig config, EsrTraceSessionOptions options)
      : session(config),
        sink(std::move(options.sink)),
        replay_writer(std::move(options.replay_writer)) {
    if (sink && options.trace_config_on_construct) {
      sink->Record("electronic_surveillance_radar", "config", "{}");
    }
    if (replay_writer && options.trace_config_on_construct) {
      WriteReplayEvent("session_config", "EsrSessionConfig", EncodeEsrSessionConfig(config));
    }
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes) const {
    oneq::replay::ReplayTraceEvent ev;
    ev.module = "electronic_surveillance_radar";
    ev.event_type = event_type;
    ev.payload_type = payload_type;
    ev.payload_encoding = "flatbuffers";
    ev.payload_bytes = payload_bytes;
    replay_writer->WriteEvent(ev);
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent ev;
    ev.module = "electronic_surveillance_radar";
    ev.event_type = event_type;
    ev.payload_type = payload_type;
    ev.payload_encoding = "flatbuffers";
    ev.payload_bytes = payload_bytes;
    ev.has_cycle_index = true;
    ev.cycle_index = cycle_index;
    replay_writer->WriteEvent(ev);
  }

  EsrSession session;
  std::shared_ptr<oneq::trace::TraceSink> sink;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
  bool pending_input_written{false};
};

EsrTraceSession::EsrTraceSession(EsrSessionConfig config, EsrTraceSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}

EsrTraceSession::~EsrTraceSession() = default;
EsrTraceSession::EsrTraceSession(EsrTraceSession&&) noexcept = default;
EsrTraceSession& EsrTraceSession::operator=(EsrTraceSession&&) noexcept = default;

session::EsrOutputFrame EsrTraceSession::Step(const EsrCycleInput& input) {
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_input", "EsrCycleInput", EncodeEsrCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  if (impl_->sink) {
    impl_->sink->Record("electronic_surveillance_radar", "input",
                        std::to_string(input.cycle_index));
  }
  const session::EsrOutputFrame output = impl_->session.Step(input);
  if (impl_->sink) {
    impl_->sink->Record("electronic_surveillance_radar", "output",
                        std::to_string(output.cycle_index));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EsrOutputFrame", EncodeEsrOutputFrame(output),
                            output.cycle_index);
    impl_->pending_input_written = false;
  }
  return output;
}

EsrCycleResult EsrTraceSession::StepWithResult(const EsrCycleInput& input) {
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      oneq::replay::ReplayTraceEvent warn;
      warn.module = "electronic_surveillance_radar";
      warn.event_type = "warning";
      warn.payload_type = "ConsecutiveCycleInputWarning";
      warn.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warn);
    }
    impl_->WriteReplayEvent("cycle_input", "EsrCycleInput", EncodeEsrCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  if (impl_->sink) {
    impl_->sink->Record("electronic_surveillance_radar", "input",
                        std::to_string(input.cycle_index));
  }
  const EsrCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("electronic_surveillance_radar", "output",
                        std::to_string(result.output_frame.cycle_index));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EsrCycleResult", EncodeEsrCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (result.has_validation_error) {
      oneq::replay::ReplayTraceFailure failure;
      failure.error_code = "validation_error";
      failure.message = "EsrCycleResult has_validation_error=true";
      failure.cycle_index = result.input_cycle_index;
      failure.has_cycle_index = true;
      impl_->replay_writer->WriteFailureMarker(failure);
    }
  }
  return result;
}

void EsrTraceSession::ApplyRuntimeConfig(const EsrRuntimeConfigPatch& patch) {
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("runtime_config_patch", "EsrRuntimeConfigPatch",
                            EncodeEsrRuntimeConfigPatch(patch));
  }
  impl_->session.ApplyRuntimeConfig(patch);
}

EsrSession& EsrTraceSession::session() { return impl_->session; }
const EsrSession& EsrTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace electronic_surveillance_radar
