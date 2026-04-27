#include "1q/electro_optical_sensor/session/EosTraceSession.h"

#include <string>
#include <utility>

#include "EosReplayFlatbufferCodec.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"

namespace electro_optical_sensor {
namespace session {

struct EosTraceSession::Impl {
  Impl(EosSessionConfig config, EosTraceSessionOptions options)
      : session(EosSessionFactory::Create(config)),
        sink(std::move(options.sink)),
        replay_writer(std::move(options.replay_writer)) {
    if (sink && options.trace_config_on_construct) {
      sink->Record("electro_optical_sensor", "config", "{}");
    }
    if (replay_writer && options.trace_config_on_construct) {
      WriteReplayEvent("session_config", "EosSessionConfig", EncodeEosSessionConfig(config));
    }
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes) const {
    oneq::replay::ReplayTraceEvent ev;
    ev.module = "electro_optical_sensor";
    ev.event_type = event_type;
    ev.payload_type = payload_type;
    ev.payload_encoding = "flatbuffers";
    ev.payload_bytes = payload_bytes;
    replay_writer->WriteEvent(ev);
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent ev;
    ev.module = "electro_optical_sensor";
    ev.event_type = event_type;
    ev.payload_type = payload_type;
    ev.payload_encoding = "flatbuffers";
    ev.payload_bytes = payload_bytes;
    ev.has_cycle_index = true;
    ev.cycle_index = cycle_index;
    replay_writer->WriteEvent(ev);
  }

  EosSession session;
  std::shared_ptr<oneq::trace::TraceSink> sink;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
  bool pending_input_written{false};
};

EosTraceSession::EosTraceSession(EosSessionConfig config, EosTraceSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}

EosTraceSession::~EosTraceSession() = default;
EosTraceSession::EosTraceSession(EosTraceSession&&) noexcept = default;
EosTraceSession& EosTraceSession::operator=(EosTraceSession&&) noexcept = default;

session::EosOutputFrame EosTraceSession::Step(const EosCycleInput& input) {
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_input", "EosCycleInput", EncodeEosCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  if (impl_->sink) {
    impl_->sink->Record("electro_optical_sensor", "input", std::to_string(input.cycle_index));
  }
  const session::EosOutputFrame output = impl_->session.Step(input);
  if (impl_->sink) {
    impl_->sink->Record("electro_optical_sensor", "output", std::to_string(output.cycle_index));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EosOutputFrame", EncodeEosOutputFrame(output),
                            output.cycle_index);
    impl_->pending_input_written = false;
  }
  return output;
}

::electro_optical_sensor::session::EosCycleResult EosTraceSession::StepWithResult(const EosCycleInput& input) {
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      // P2-B: 连续两次 cycle_input 无中间 output，记录 warning 后继续
      oneq::replay::ReplayTraceEvent warn_ev;
      warn_ev.module = "electro_optical_sensor";
      warn_ev.event_type = "warning";
      warn_ev.payload_type = "ConsecutiveCycleInputWarning";
      warn_ev.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warn_ev);
    }
    impl_->WriteReplayEvent("cycle_input", "EosCycleInput", EncodeEosCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  if (impl_->sink) {
    impl_->sink->Record("electro_optical_sensor", "input", std::to_string(input.cycle_index));
  }
  const ::electro_optical_sensor::session::EosCycleResult result =
      impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("electro_optical_sensor", "output",
                        std::to_string(result.output_frame.cycle_index));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EosCycleResult", EncodeEosCycleResult(result),
                            result.output_frame.cycle_index);
    impl_->pending_input_written = false;
    // P1-A: 自动 failure_marker
    if (result.has_validation_error) {
      oneq::replay::ReplayTraceFailure failure;
      failure.error_code = "validation_error";
      failure.message = "EosCycleResult has_validation_error=true";
      failure.cycle_index = result.output_frame.cycle_index;
      failure.has_cycle_index = true;
      impl_->replay_writer->WriteFailureMarker(failure);
    }
  }
  return result;
}

void EosTraceSession::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  if (impl_->replay_writer) {
    // 先写后 apply，保证回放时配置变更在执行前可重放
    impl_->WriteReplayEvent("runtime_config_patch", "EosRuntimeConfigPatch",
                            EncodeEosRuntimeConfigPatch(patch));
  }
  impl_->session.ApplyRuntimeConfig(patch);
}

EosSession& EosTraceSession::session() { return impl_->session; }
const EosSession& EosTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace electro_optical_sensor
