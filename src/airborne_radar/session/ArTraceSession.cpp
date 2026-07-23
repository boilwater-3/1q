#include "1q/airborne_radar/session/ArTraceSession.h"

#include <sstream>
#include <string>
#include <utility>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/trace/TraceSink.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

void WriteReplayEvent(
    const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
    const char* event_type, const char* payload_type,
    const std::string& payload_bytes, const ArCycleInput& input) {
  if (!writer) {
    return;
  }
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = payload_bytes;
  event.has_cycle_index = true;
  event.cycle_index = input.cycle_index;
  event.has_sim_time_sec = true;
  event.sim_time_sec = input.cycle_start_time_s;
  writer->WriteEvent(event);
}

void WriteSessionConfigReplay(
    const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
    const config::ArSessionConfig& config) {
  if (!writer) {
    return;
  }
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "session_config";
  event.payload_type = "ArSessionConfig";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeSessionConfigFlatbuffer(config);
  writer->WriteEvent(event);
}

std::string BuildCyclePayload(const ArCycleInput& input,
                              const ArCycleResult* result) {
  std::ostringstream stream;
  stream << "{\"cycle_index\":" << input.cycle_index
         << ",\"cycle_start_time_s\":" << input.cycle_start_time_s
         << ",\"external_emission_count\":"
         << input.interference.emissions.size();
  if (result != nullptr) {
    stream << ",\"status\":" << static_cast<int>(result->status)
           << ",\"track_count\":" << result->track_output_frame.tracks.size()
           << ",\"ar_emission_count\":"
           << result->emission_frame.emissions.size()
           << ",\"interference_observation_count\":"
           << result->interference_observations.size()
           << ",\"receiver_impairment\":"
           << static_cast<int>(result->receiver_impairment);
  }
  stream << "}";
  return stream.str();
}

}  // namespace

struct ArTraceSession::Impl {
  Impl(ArSession value, std::shared_ptr<oneq::trace::TraceSink> trace_sink,
       std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : session(std::move(value)),
        sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)) {}

  ArSession session;
  std::shared_ptr<oneq::trace::TraceSink> sink;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
};

ArTraceSession::ArTraceSession(const config::ArSessionConfig& config,
                               ArTraceSessionOptions options)
    : impl_(new Impl(ArSession::Create(config), std::move(options.sink),
                     std::move(options.replay_writer))) {
  if (options.trace_config_on_construct) {
    if (impl_->sink) {
      impl_->sink->Record("airborne_radar", "config", "{}");
    }
    WriteSessionConfigReplay(impl_->replay_writer, config);
  }
}

ArTraceSession::ArTraceSession(ArTraceSession&& other) noexcept = default;
ArTraceSession& ArTraceSession::operator=(ArTraceSession&& other) noexcept =
    default;
ArTraceSession::~ArTraceSession() = default;

TrackOutputFrame ArTraceSession::Step(const ArCycleInput& input) {
  return StepWithResult(input).track_output_frame;
}

ArCycleResult ArTraceSession::StepWithResult(const ArCycleInput& input) {
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "cycle_input",
                        BuildCyclePayload(input, nullptr));
  }
  WriteReplayEvent(impl_->replay_writer, "cycle_input", "ArCycleInputV3",
                   EncodeCycleInputFlatbuffer(input), input);

  const ArCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "cycle_output",
                        BuildCyclePayload(input, &result));
  }
  ArCycleReplayRecord record;
  record.result = result;
  record.session_state =
      ArSessionReplayAccess::CaptureSessionState(impl_->session);
  WriteReplayEvent(impl_->replay_writer, "cycle_output",
                   "ArCycleReplayRecordV3",
                   EncodeCycleReplayRecordFlatbuffer(record), input);
  return result;
}

void ArTraceSession::ApplyRuntimeConfig(
    const config::ArRuntimeConfigPatch& patch) {
  (void)TryApplyRuntimeConfig(patch);
}

bool ArTraceSession::TryApplyRuntimeConfig(
    const config::ArRuntimeConfigPatch& patch) {
  const bool accepted = impl_->session.TryApplyRuntimeConfig(patch);
  if (impl_->replay_writer) {
    oneq::replay::ReplayTraceEvent event;
    event.module = "airborne_radar";
    event.event_type = "runtime_config_patch";
    event.payload_type = "ArRuntimeConfigAttemptV3";
    event.payload_encoding = "flatbuffers";
    event.payload_bytes =
        EncodeRuntimeConfigAttemptFlatbuffer(patch, accepted);
    impl_->replay_writer->WriteEvent(event);
  }
  return accepted;
}

session::ExternalDecisionSubmitStatus ArTraceSession::SubmitExternalDecision(
    const session::ExternalDecisionResponse& response) {
  const session::ExternalDecisionSubmitStatus status =
      impl_->session.SubmitExternalDecision(response);
  if (impl_->replay_writer) {
    oneq::replay::ReplayTraceEvent event;
    event.module = "airborne_radar";
    event.event_type = "decision_input";
    event.payload_type = "ArExternalDecisionAttemptV3";
    event.payload_encoding = "flatbuffers";
    event.payload_bytes =
        EncodeExternalDecisionAttemptFlatbuffer(response, status);
    event.has_cycle_index = true;
    event.cycle_index = response.source_cycle_index;
    impl_->replay_writer->WriteEvent(event);
  }
  return status;
}

const ArSession& ArTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace airborne_radar
