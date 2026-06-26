#include "1q/sar/session/SarTraceSession.h"
#include "1q/sar/session/SarSession.h"

#include <sstream>
#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "SarReplayFlatbufferCodec.h"

namespace sar {
namespace session {
namespace {

bool HasExternalRawIq(const SarCycleInput& input) {
  return input.raw_iq.pulse_count != 0U || input.raw_iq.samples_per_pulse != 0U ||
         !input.raw_iq.i_values.empty() || !input.raw_iq.q_values.empty() ||
         !input.raw_iq.pulse_states.empty() || !input.raw_iq.ideal_pulse_states.empty();
}

std::string BuildSarInputPayload(const SarCycleInput& input) {
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << input.cycle_index << ","
     << "\"dt_sec\":" << input.dt_sec << ","
     << "\"point_target_count\":" << input.point_targets.size() << ","
     << "\"has_external_raw_iq\":" << (HasExternalRawIq(input) ? "true" : "false") << "}";
  return os.str();
}

std::string BuildSarOutputPayload(const SarCycleResult& result) {
  const SarOutputFrame& frame = result.output_frame;
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << frame.cycle_index << ","
     << "\"completed_stage\":" << static_cast<int>(frame.completed_stage) << ","
     << "\"executed\":" << (result.executed_this_cycle ? "true" : "false") << ","
     << "\"has_error\":" << (result.has_error ? "true" : "false") << ","
     << "\"has_l1_image\":" << (frame.has_l1_image ? "true" : "false") << ","
     << "\"has_l3_bp_image\":" << (frame.has_l3_bp_image ? "true" : "false") << ","
     << "\"phase_reference_mode\":" << static_cast<int>(frame.phase_reference_mode) << ","
     << "\"phase_reference_applied\":"
     << (frame.phase_reference_applied ? "true" : "false") << ","
     << "\"image_quality_mainlobe_method\":"
     << static_cast<int>(frame.image_quality_mainlobe_method) << ","
     << "\"has_image_quality_metrics\":"
     << (frame.has_image_quality_metrics ? "true" : "false") << ","
     << "\"image_resolution_m_valid\":"
     << (frame.image_resolution_m_valid ? "true" : "false") << ","
     << "\"range_width_3db_bins\":" << frame.range_width_3db_bins << ","
     << "\"azimuth_width_3db_bins\":" << frame.azimuth_width_3db_bins << ","
     << "\"range_resolution_3db_m\":" << frame.range_resolution_3db_m << ","
     << "\"azimuth_resolution_3db_m\":" << frame.azimuth_resolution_3db_m << ","
     << "\"image_entropy_nats\":" << frame.image_entropy_nats << ","
     << "\"image_contrast\":" << frame.image_contrast << "}";
  return os.str();
}

}  // namespace

struct SarTraceSession::Impl {
  Impl(SarSession initial_session, SarTraceSessionOptions session_options)
      : session(std::move(initial_session)),
        sink(std::move(session_options.sink)),
        replay_writer(std::move(session_options.replay_writer)) {}

  Impl(config::SarSessionConfig config, SarTraceSessionOptions session_options)
      : session(SarSession::Create(config)),
        sink(std::move(session_options.sink)),
        replay_writer(std::move(session_options.replay_writer)) {
    if (sink && session_options.trace_config_on_construct) {
      sink->Record("sar", "config", "{}");
    }
    if (replay_writer && session_options.trace_config_on_construct) {
      WriteReplayEvent("session_config", "SarSessionConfig", EncodeSarSessionConfig(config));
    }
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "sar";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload_bytes;
    replay_writer->WriteEvent(event);
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "sar";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload_bytes;
    event.has_cycle_index = true;
    event.cycle_index = cycle_index;
    replay_writer->WriteEvent(event);
  }

  void WriteFailureMarker(const SarCycleResult& result) const {
    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = result.abort_reason.empty() ? "sar_error" : result.abort_reason;
    failure.message = "SarCycleResult has_error=true";
    failure.has_cycle_index = true;
    failure.cycle_index = result.input_cycle_index;
    replay_writer->WriteFailureMarker(failure);
  }

  SarSession session;
  std::shared_ptr<oneq::trace::TraceSink> sink{};
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool pending_input_written{false};
};

SarTraceSession::SarTraceSession() : impl_(new Impl(SarSession{}, SarTraceSessionOptions{})) {}

SarTraceSession::SarTraceSession(SarSession session)
    : impl_(new Impl(std::move(session), SarTraceSessionOptions{})) {}

SarTraceSession::SarTraceSession(config::SarSessionConfig config, SarTraceSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}

SarTraceSession::~SarTraceSession() = default;
SarTraceSession::SarTraceSession(SarTraceSession&&) noexcept = default;
SarTraceSession& SarTraceSession::operator=(SarTraceSession&&) noexcept = default;

SarOutputFrame SarTraceSession::Step(const SarCycleInput& input) {
  return StepWithResult(input).output_frame;
}

SarCycleResult SarTraceSession::StepWithResult(const SarCycleInput& input) {
  if (impl_->replay_writer && HasExternalRawIq(input)) {
    SarCycleResult result;
    result.input_cycle_index = input.cycle_index;
    result.output_frame.cycle_index = input.cycle_index;
    result.has_error = true;
    result.abort_reason = "external_raw_iq_replay_unsupported";
    SarDiagnosticIssue issue;
    issue.severity = SarDiagnosticSeverity::kError;
    issue.code = "sar.external_raw_iq_replay_unsupported";
    issue.message =
        "External raw IQ cannot be recorded by the current summary-only SAR replay schema.";
    result.diagnostics.push_back(issue);
    return result;
  }
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      oneq::replay::ReplayTraceEvent warning;
      warning.module = "sar";
      warning.event_type = "warning";
      warning.payload_type = "ConsecutiveCycleInputWarning";
      warning.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warning);
    }
    impl_->WriteReplayEvent("cycle_input", "SarCycleInput", EncodeSarCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  if (impl_->sink) {
    impl_->sink->Record("sar", "input", BuildSarInputPayload(input));
  }

  const SarCycleResult result = impl_->session.StepWithResult(input);

  if (impl_->sink) {
    impl_->sink->Record("sar", "output", BuildSarOutputPayload(result));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "SarCycleResult", EncodeSarCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (result.has_error) {
      impl_->WriteFailureMarker(result);
    }
  }
  return result;
}

void SarTraceSession::ApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("runtime_config_patch", "SarRuntimeConfigPatch",
                            EncodeSarRuntimeConfigPatch(patch));
  }
  impl_->session.ApplyRuntimeConfig(patch);
}

SarSession& SarTraceSession::session() { return impl_->session; }
const SarSession& SarTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace sar
