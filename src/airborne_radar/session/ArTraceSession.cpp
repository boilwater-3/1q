#include "1q/airborne_radar/session/ArTraceSession.h"
#include "1q/airborne_radar/session/ArSession.h"

#include <sstream>
#include <string>
#include <utility>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/trace/TraceSink.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

std::string BuildArInputPayload(const ArCycleInput& input) {
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << input.cycle_index << ","
     << "\"dt_sec\":" << input.dt_sec << ","
     << "\"platform_altitude_m\":" << input.platform_altitude_m << ","
     << "\"has_environment\":" << (input.has_environment ? "true" : "false") << ","
     << "\"scene_target_count\":" << input.scene.size()
     << "}";
  return os.str();
}

std::string BuildArOutputPayload(const ArCycleResult& result) {
  const auto& frame = result.track_output_frame;
  std::size_t confirmed_count = 0U;
  for (const auto& track : frame.tracks) {
    if (track.status == session::TrackStatus::kConfirmed) {
      ++confirmed_count;
    }
  }

  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << frame.cycle_index << ","
     << "\"batch_id\":" << frame.batch_id << ","
     << "\"executed\":" << (result.executed_this_cycle ? "true" : "false") << ","
     << "\"track_count\":" << frame.tracks.size() << ","
     << "\"confirmed_track_count\":" << confirmed_count << ","
     << "\"assoc_priors\":" << result.association_quality_metrics.prior_track_count << ","
     << "\"assoc_detections\":" << result.association_quality_metrics.detection_count << ","
     << "\"assoc_matches\":" << result.association_quality_metrics.matched_count << ","
     << "\"assoc_new_tracks\":" << result.association_quality_metrics.new_track_count << ","
     << "\"assoc_missed\":" << result.association_quality_metrics.missed_track_count << ","
     << "\"assoc_match_rate\":" << result.association_quality_metrics.match_rate << ","
     << "\"validation_error\":" << (result.has_validation_error ? "true" : "false")
     << "}";
  return os.str();
}

void WriteSessionConfigReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                              const config::ArSessionConfig& config) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "session_config";
  event.payload_type = "ArSessionConfig";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeSessionConfigFlatbuffer(config);
  writer->WriteEvent(event);
}

void WriteRuntimeConfigPatchReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                                   const config::ArRuntimeConfigPatch& patch) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "runtime_config_patch";
  event.payload_type = "ArRuntimeConfigPatch";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeRuntimeConfigPatchFlatbuffer(patch);
  writer->WriteEvent(event);
}

void WriteExternalDecisionReplay(
    const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
    const session::ExternalDecisionResponse& response) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "decision_input";
  event.payload_type = "ExternalDecisionResponse";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeExternalDecisionResponseFlatbuffer(response);
  event.has_cycle_index = true;
  event.cycle_index = response.source_cycle_index;
  writer->WriteEvent(event);
}

void WriteCycleResultReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                            const ArReplayCycleRecord& record) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "cycle_output";
  event.payload_type = "ArReplayCycleRecord";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeReplayCycleRecordFlatbuffer(record);
  event.has_cycle_index = true;
  event.cycle_index = record.result.input_cycle_index;
  writer->WriteEvent(event);
}

void MaybeWriteValidationFailureMarker(
    const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
    const ArCycleResult& result) {
  if (!writer || !result.has_validation_error) {
    return;
  }
  oneq::replay::ReplayTraceFailure failure;
  failure.error_code = "AR_VALIDATION_ERROR";
  failure.message = "ArCycleResult has_validation_error set";
  failure.location = "ArTraceSession::StepWithResult";
  failure.has_cycle_index = true;
  failure.cycle_index = result.input_cycle_index;
  const std::string failure_bytes = EncodeFailureMarkerFlatbuffer(failure, false, 0U);
  writer->WriteFailureMarker(failure, failure_bytes);
}

void WriteCycleInputEvent(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                          const ArCycleInput& input) {
  oneq::replay::ReplayTraceEvent ev;
  ev.module = "airborne_radar";
  ev.event_type = "cycle_input";
  ev.payload_type = "ArCycleInput";
  ev.payload_encoding = "flatbuffers";
  ev.payload_bytes = EncodeCycleInputFlatbuffer(input);
  ev.has_cycle_index = true;
  ev.cycle_index = input.cycle_index;
  writer->WriteEvent(ev);
}

}  // namespace

struct ArTraceSession::Impl {
  Impl(ArSession s, std::shared_ptr<oneq::trace::TraceSink> sk,
       std::shared_ptr<oneq::replay::ReplayTraceWriter> rw)
      : session(std::move(s)), sink(std::move(sk)), replay_writer(std::move(rw)) {}

  ArSession session;
  std::shared_ptr<oneq::trace::TraceSink> sink;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
  bool pending_input_written{false};
};

ArTraceSession::ArTraceSession(const config::ArSessionConfig& config,
                               ArTraceSessionOptions options)
    : impl_(new Impl(ArSession::Create(config), std::move(options.sink),
                     std::move(options.replay_writer))) {
  if (options.trace_config_on_construct) {
    if (impl_->sink) {
      impl_->sink->Record("airborne_radar", "config", "{}");
    }
    if (impl_->replay_writer) {
      WriteSessionConfigReplay(impl_->replay_writer, config);
    }
  }
}

ArTraceSession::ArTraceSession(ArTraceSession&& other) noexcept = default;
ArTraceSession& ArTraceSession::operator=(ArTraceSession&& other) noexcept = default;
ArTraceSession::~ArTraceSession() = default;

session::TrackOutputFrame ArTraceSession::Step(const ArCycleInput& input) {
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "input", BuildArInputPayload(input));
  }
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      oneq::replay::ReplayTraceEvent warn_ev;
      warn_ev.module = "airborne_radar";
      warn_ev.event_type = "warning";
      warn_ev.payload_type = "ConsecutiveCycleInputWarning";
      warn_ev.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warn_ev);
    }
    WriteCycleInputEvent(impl_->replay_writer, input);
    impl_->pending_input_written = true;
  }
  const ArCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "output", BuildArOutputPayload(result));
  }
  if (impl_->replay_writer) {
    ArReplayCycleRecord record;
    record.result = result;
    record.decision_state = ArSessionReplayAccess::CaptureDecisionState(impl_->session);
    WriteCycleResultReplay(impl_->replay_writer, record);
    MaybeWriteValidationFailureMarker(impl_->replay_writer, result);
    impl_->pending_input_written = false;
  }
  return result.track_output_frame;
}

ArCycleResult ArTraceSession::StepWithResult(const ArCycleInput& input) {
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "input", BuildArInputPayload(input));
  }
  if (impl_->replay_writer) {
    if (impl_->pending_input_written) {
      oneq::replay::ReplayTraceEvent warn_ev;
      warn_ev.module = "airborne_radar";
      warn_ev.event_type = "warning";
      warn_ev.payload_type = "ConsecutiveCycleInputWarning";
      warn_ev.payload_inline = "{\"message\":\"consecutive cycle_input without cycle_output\"}";
      impl_->replay_writer->WriteEvent(warn_ev);
    }
    WriteCycleInputEvent(impl_->replay_writer, input);
    impl_->pending_input_written = true;
  }
  const ArCycleResult output = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "output", BuildArOutputPayload(output));
  }
  if (impl_->replay_writer) {
    ArReplayCycleRecord record;
    record.result = output;
    record.decision_state = ArSessionReplayAccess::CaptureDecisionState(impl_->session);
    WriteCycleResultReplay(impl_->replay_writer, record);
    MaybeWriteValidationFailureMarker(impl_->replay_writer, output);
    impl_->pending_input_written = false;
  }
  return output;
}

void ArTraceSession::ApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch) {
  (void)TryApplyRuntimeConfig(patch);
}

bool ArTraceSession::TryApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch) {
  if (!impl_->session.TryApplyRuntimeConfig(patch)) {
    return false;
  }
  if (impl_->replay_writer) {
    WriteRuntimeConfigPatchReplay(impl_->replay_writer, patch);
  }
  return true;
}

session::ExternalDecisionSubmitStatus ArTraceSession::SubmitExternalDecision(
    const session::ExternalDecisionResponse& response) {
  const session::ExternalDecisionSubmitStatus status =
      impl_->session.SubmitExternalDecision(response);
  if (status == session::ExternalDecisionSubmitStatus::kAccepted && impl_->replay_writer) {
    WriteExternalDecisionReplay(impl_->replay_writer, response);
  }
  return status;
}

const std::vector<session::ArCommand>& ArTraceSession::GetSubmittedCommands()
    const {
  return impl_->session.GetSubmittedCommands();
}

bool ArTraceSession::HasLatestControlProfile() const {
  return impl_->session.HasLatestControlProfile();
}

const session::ArControlProfile& ArTraceSession::GetLatestControlProfile() const {
  return impl_->session.GetLatestControlProfile();
}

session::AssociationQualityMetrics ArTraceSession::GetLastAssociationQualityMetrics() const {
  return impl_->session.GetLastAssociationQualityMetrics();
}

const ArSession& ArTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace airborne_radar
