#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

#include <sstream>
#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "EosReplayFlatbufferCodec.h"

namespace electro_optical_sensor {
namespace session {
namespace {

std::string BuildEosInputPayload(const EosCycleInput& input) {
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << input.cycle_index << ","
     << "\"dt_sec\":" << input.dt_sec << ","
     << "\"platform_altitude_m\":" << input.platform_altitude_m << ","
     << "\"scene_target_count\":" << input.scene.size()
     << "}";
  return os.str();
}

std::string BuildEosOutputPayload(const EosCycleResult& result) {
  const auto& frame = result.output_frame;
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << frame.cycle_index << ","
     << "\"scan_azimuth_deg\":" << frame.scan_azimuth_deg << ","
     << "\"executed\":" << (result.status == EosCycleStatus::kCompleted ? "true" : "false") << ","
     << "\"status\":" << static_cast<int>(result.status) << ","
     << "\"detection_count\":" << frame.detections.size() << ","
     << "\"validation_error\":" << (HasValidationError(result.issues) ? "true" : "false") << ","
     << "\"issue_count\":" << result.issues.size()
     << "}";
  return os.str();
}

}  // namespace

struct EosTraceSession::Impl {
  Impl(config::EosSessionConfig config, EosTraceSessionOptions options)
      : session(EosSession::Create(config)),
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

  void WriteValidationFailureMarker(const EosCycleResult& result) const {
    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "EOS_VALIDATION_ERROR";
    failure.message = "EosCycleResult input validation rejected";
    failure.location = "EosTraceSession::StepWithResult";
    failure.cycle_index = result.input_cycle_index;
    failure.has_cycle_index = true;
    const std::string failure_bytes = EncodeEosFailureMarker(failure);
    replay_writer->WriteFailureMarker(failure, failure_bytes);
  }

  EosSession session;
  std::shared_ptr<oneq::trace::TraceSink> sink;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
  bool pending_input_written{false};
};

EosTraceSession::EosTraceSession(config::EosSessionConfig config, EosTraceSessionOptions options)
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
    impl_->sink->Record("electro_optical_sensor", "input", BuildEosInputPayload(input));
  }
  const session::EosCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("electro_optical_sensor", "output", BuildEosOutputPayload(result));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EosCycleResult", EncodeEosCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result.output_frame;
}

::electro_optical_sensor::session::EosCycleResult EosTraceSession::StepWithResult(
    const EosCycleInput& input) {
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
    impl_->sink->Record("electro_optical_sensor", "input", BuildEosInputPayload(input));
  }
  const ::electro_optical_sensor::session::EosCycleResult result =
      impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("electro_optical_sensor", "output", BuildEosOutputPayload(result));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EosCycleResult", EncodeEosCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result;
}

bool EosTraceSession::TryApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch) {
  const bool accepted = impl_->session.TryApplyRuntimeConfig(patch);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("runtime_config_patch", "EosRuntimeConfigPatch",
                            EncodeEosRuntimeConfigPatch(patch));
  }
  return accepted;
}

EosSession& EosTraceSession::session() { return impl_->session; }
const EosSession& EosTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace electro_optical_sensor
