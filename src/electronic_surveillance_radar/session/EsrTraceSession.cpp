#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

#include <sstream>
#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "EsrReplayFlatbufferCodec.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

std::string BuildEsrInputPayload(const EsrCycleInput& input) {
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << input.cycle_index << ","
     << "\"dt_sec\":" << input.dt_sec << ","
     << "\"platform_yaw_deg\":" << input.platform_attitude_deg.yaw_deg << ","
     << "\"rf_emission_count\":" << input.rf_emissions.emissions.size()
     << "}";
  return os.str();
}

std::string BuildEsrOutputPayload(const EsrCycleResult& result) {
  const auto& frame = result.output_frame;
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << frame.cycle_index << ","
     << "\"batch_id\":" << frame.batch_id << ","
     << "\"status\":" << static_cast<int>(result.status) << ","
     << "\"raw_observation_count\":" << frame.observation_output.raw_observation_count << ","
     << "\"observation_count\":" << frame.observation_output.observations.size() << ","
     << "\"cluster_count\":" << frame.observation_output.cluster_count << ","
     << "\"hypothesis_count\":" << frame.emitter_output.hypotheses.size() << ","
     << "\"validation_error\":" << (result.has_validation_error ? "true" : "false") << ","
     << "\"diagnostic_count\":" << result.diagnostics.size()
     << "}";
  return os.str();
}

}  // namespace

struct EsrTraceSession::Impl {
  Impl(config::EsrSessionConfig config, EsrTraceSessionOptions options)
      : session(EsrSession::Create(config)),
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

  void WriteValidationFailureMarker(const EsrCycleResult& result) const {
    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "ESR_VALIDATION_ERROR";
    failure.message = "EsrCycleResult has_validation_error=true";
    failure.location = "EsrTraceSession::StepWithResult";
    failure.cycle_index = result.input_cycle_index;
    failure.has_cycle_index = true;
    const std::string failure_bytes = EncodeEsrFailureMarker(failure);
    replay_writer->WriteFailureMarker(failure, failure_bytes);
  }

  EsrSession session;
  std::shared_ptr<oneq::trace::TraceSink> sink;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
  bool pending_input_written{false};
};

EsrTraceSession::EsrTraceSession(config::EsrSessionConfig config, EsrTraceSessionOptions options)
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
    impl_->sink->Record("electronic_surveillance_radar", "input", BuildEsrInputPayload(input));
  }
  const session::EsrCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("electronic_surveillance_radar", "output", BuildEsrOutputPayload(result));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EsrCycleResult", EncodeEsrCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (result.has_validation_error) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result.output_frame;
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
    impl_->sink->Record("electronic_surveillance_radar", "input", BuildEsrInputPayload(input));
  }
  const EsrCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("electronic_surveillance_radar", "output", BuildEsrOutputPayload(result));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "EsrCycleResult", EncodeEsrCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (result.has_validation_error) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result;
}

EsrRuntimeConfigApplyResult EsrTraceSession::TryApplyRuntimeConfig(
    const config::EsrRuntimeConfigPatch& patch) {
  const EsrRuntimeConfigApplyResult result =
      impl_->session.ApplyRuntimeConfigWithResult(patch);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent(
        "runtime_config_patch", "EsrRuntimeConfigPatchEvent",
        EncodeEsrRuntimeConfigPatchEvent(patch, result));
  }
  return result;
}

EsrSession& EsrTraceSession::session() { return impl_->session; }
const EsrSession& EsrTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace electronic_surveillance_radar
