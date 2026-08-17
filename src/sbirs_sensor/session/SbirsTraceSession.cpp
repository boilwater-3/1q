#include "1q/sbirs_sensor/session/SbirsTraceSession.h"

#include <sstream>
#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "sbirs_sensor/session/SbirsReplayFlatbufferCodec.h"

namespace sbirs_sensor {
namespace session {
namespace {

std::string BuildSbirsInputPayload(const SbirsCycleInput& input) {
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << input.cycle_index << ","
     << "\"dt_sec\":" << input.dt_sec << ","
     << "\"has_satellite_position\":" << (input.has_satellite_position ? "true" : "false") << ","
     << "\"has_satellite_velocity\":"
     << (input.has_satellite_velocity_ecef_m_per_s ? "true" : "false") << ","
     << "\"has_satellite_attitude\":" << (input.has_satellite_attitude ? "true" : "false") << ","
     << "\"scene_target_count\":" << input.scene.size() << "}";
  return os.str();
}

std::string BuildSbirsOutputPayload(const SbirsCycleResult& result) {
  const SbirsOutputFrame& frame = result.output_frame;
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << frame.cycle_index << ","
     << "\"scan_azimuth_rad\":" << frame.scan_azimuth_rad << ","
     << "\"executed\":" << (result.status == SbirsCycleStatus::kCompleted ? "true" : "false") << ","
     << "\"status\":" << static_cast<int>(result.status) << ","
     << "\"detection_count\":" << frame.detections.size() << ","
     << "\"validation_error\":" << (HasValidationError(result.issues) ? "true" : "false") << ","
     << "\"issue_count\":" << result.issues.size() << "}";
  return os.str();
}

}  // namespace

struct SbirsTraceSession::Impl {
  Impl(config::SbirsSessionConfig session_config, SbirsTraceSessionOptions session_options)
      : session(SbirsSession::Create(session_config)),
        sink(std::move(session_options.sink)),
        replay_writer(std::move(session_options.replay_writer)) {
    if (sink && session_options.trace_config_on_construct) {
      sink->Record("sbirs_sensor", "config", "{}");
    }
    if (replay_writer && session_options.trace_config_on_construct) {
      WriteReplayEvent("session_config", "SbirsSessionConfig",
                       EncodeSbirsSessionConfig(session_config));
    }
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "sbirs_sensor";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload_bytes;
    replay_writer->WriteEvent(event);
  }

  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "sbirs_sensor";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload_bytes;
    event.has_cycle_index = true;
    event.cycle_index = cycle_index;
    replay_writer->WriteEvent(event);
  }

  void WriteValidationFailureMarker(const SbirsCycleResult& result) const {
    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "SBIRS_VALIDATION_ERROR";
    failure.message = "SbirsCycleResult input validation rejected";
    failure.location = "SbirsTraceSession::StepWithResult";
    failure.cycle_index = result.input_cycle_index;
    failure.has_cycle_index = true;
    replay_writer->WriteFailureMarker(failure, EncodeSbirsFailureMarker(failure));
  }

  SbirsSession session;
  std::shared_ptr<oneq::trace::TraceSink> sink;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
  bool pending_input_written{false};
};

SbirsTraceSession::SbirsTraceSession(config::SbirsSessionConfig config,
                                     SbirsTraceSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}

SbirsTraceSession::~SbirsTraceSession() = default;
SbirsTraceSession::SbirsTraceSession(SbirsTraceSession&&) noexcept = default;
SbirsTraceSession& SbirsTraceSession::operator=(SbirsTraceSession&&) noexcept = default;

SbirsOutputFrame SbirsTraceSession::Step(const SbirsCycleInput& input) {
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_input", "SbirsCycleInput", EncodeSbirsCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  if (impl_->sink) {
    impl_->sink->Record("sbirs_sensor", "input", BuildSbirsInputPayload(input));
  }
  const SbirsCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("sbirs_sensor", "output", BuildSbirsOutputPayload(result));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "SbirsCycleResult", EncodeSbirsCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result.output_frame;
}

SbirsCycleResult SbirsTraceSession::StepWithResult(const SbirsCycleInput& input) {
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      oneq::replay::ReplayTraceEvent warning;
      warning.module = "sbirs_sensor";
      warning.event_type = "warning";
      warning.payload_type = "ConsecutiveCycleInputWarning";
      warning.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warning);
    }
    impl_->WriteReplayEvent("cycle_input", "SbirsCycleInput", EncodeSbirsCycleInput(input),
                            input.cycle_index);
    impl_->pending_input_written = true;
  }
  if (impl_->sink) {
    impl_->sink->Record("sbirs_sensor", "input", BuildSbirsInputPayload(input));
  }
  const SbirsCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("sbirs_sensor", "output", BuildSbirsOutputPayload(result));
  }
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("cycle_output", "SbirsCycleResult", EncodeSbirsCycleResult(result),
                            result.input_cycle_index);
    impl_->pending_input_written = false;
    if (HasValidationError(result.issues)) {
      impl_->WriteValidationFailureMarker(result);
    }
  }
  return result;
}

bool SbirsTraceSession::TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch) {
  const bool accepted = impl_->session.TryApplyRuntimeConfig(patch);
  if (impl_->replay_writer) {
    impl_->WriteReplayEvent("runtime_config_patch", "SbirsRuntimeConfigPatch",
                            EncodeSbirsRuntimeConfigPatch(patch));
  }
  return accepted;
}

SbirsSession& SbirsTraceSession::session() { return impl_->session; }
const SbirsSession& SbirsTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace sbirs_sensor
