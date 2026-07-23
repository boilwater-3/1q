#include "1q/airborne_radar/session/ArTraceSession.h"

#include <cmath>
#include <limits>
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

void SetWorldCycleMetadata(std::uint64_t world_cycle_index, double world_time_s,
                           oneq::replay::ReplayTraceEvent* event) {
  if (event == nullptr) {
    return;
  }
  if (world_cycle_index <= std::numeric_limits<std::uint32_t>::max()) {
    event->has_cycle_index = true;
    event->cycle_index = static_cast<std::uint32_t>(world_cycle_index);
  }
  if (std::isfinite(world_time_s)) {
    event->has_sim_time_sec = true;
    event->sim_time_sec = world_time_s;
  }
}

void WriteReplayEvent(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                      const char* event_type, const char* payload_type,
                      const std::string& payload_bytes, std::uint64_t world_cycle_index,
                      double world_time_s) {
  if (!writer) {
    return;
  }
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = payload_bytes;
  SetWorldCycleMetadata(world_cycle_index, world_time_s, &event);
  writer->WriteEvent(event);
}

void WriteSessionConfigReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
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

std::string BuildPreparePayload(const ArPrepareCycleInput& input,
                                const ArPrepareCycleResult* result) {
  std::ostringstream stream;
  stream << "{\"world_cycle_index\":" << input.world_cycle_index
         << ",\"window_start_time_s\":" << input.window_start_time_s;
  if (result != nullptr) {
    stream << ",\"status\":" << static_cast<int>(result->status)
           << ",\"has_emission\":" << (result->has_emission ? "true" : "false")
           << ",\"emission_id\":" << result->emission.identity.emission_id;
  }
  stream << "}";
  return stream.str();
}

std::string BuildCompletePayload(const ArPreparedCycleToken& token,
                                 const ArCompleteCycleResult* result) {
  std::ostringstream stream;
  stream << "{\"token\":" << token.value << ",\"world_cycle_index\":" << token.world_cycle_index;
  if (result != nullptr) {
    stream << ",\"status\":" << static_cast<int>(result->status)
           << ",\"track_count\":" << result->track_output_frame.tracks.size()
           << ",\"interference_observation_count\":" << result->interference_observations.size()
           << ",\"has_decision_observation\":"
           << (result->has_decision_observation ? "true" : "false")
           << ",\"receiver_impairment\":" << static_cast<int>(result->receiver_impairment);
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
ArTraceSession& ArTraceSession::operator=(ArTraceSession&& other) noexcept = default;
ArTraceSession::~ArTraceSession() = default;

ArPrepareCycleResult ArTraceSession::PrepareCycle(const ArPrepareCycleInput& input) {
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "prepare_input", BuildPreparePayload(input, nullptr));
  }
  WriteReplayEvent(impl_->replay_writer, "cycle_input", "ArPrepareCycleInputV2",
                   EncodePrepareCycleInputFlatbuffer(input), input.world_cycle_index,
                   input.window_start_time_s);

  ArPrepareCycleResult result = impl_->session.PrepareCycle(input);
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "prepare_output", BuildPreparePayload(input, &result));
  }
  ArPrepareReplayRecord record;
  record.result = result;
  record.session_state = ArSessionReplayAccess::CaptureSessionState(impl_->session);
  WriteReplayEvent(impl_->replay_writer, "cycle_output", "ArPrepareReplayRecordV2",
                   EncodePrepareReplayRecordFlatbuffer(record), input.world_cycle_index,
                   input.window_start_time_s);
  return result;
}

ArCompleteCycleResult ArTraceSession::CompleteCycle(const ArPreparedCycleToken& token,
                                                    const ArCompleteCycleInput& input) {
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "complete_input", BuildCompletePayload(token, nullptr));
  }
  ArCompleteReplayOperationInput operation;
  operation.token = token;
  operation.input = input;
  WriteReplayEvent(impl_->replay_writer, "cycle_input", "ArCompleteReplayOperationInputV2",
                   EncodeCompleteReplayOperationInputFlatbuffer(operation), token.world_cycle_index,
                   input.rf_scene.window_start_time_s);

  ArCompleteCycleResult result = impl_->session.CompleteCycle(token, input);
  if (impl_->sink) {
    impl_->sink->Record("airborne_radar", "complete_output", BuildCompletePayload(token, &result));
  }
  ArCompleteReplayRecord record;
  record.result = result;
  record.session_state = ArSessionReplayAccess::CaptureSessionState(impl_->session);
  WriteReplayEvent(impl_->replay_writer, "cycle_output", "ArCompleteReplayRecordV2",
                   EncodeCompleteReplayRecordFlatbuffer(record), token.world_cycle_index,
                   input.rf_scene.window_start_time_s);
  return result;
}

ArAbandonCycleStatus ArTraceSession::AbandonCycle(const ArPreparedCycleToken& token) {
  ArAbandonReplayOperationInput operation;
  operation.token = token;
  WriteReplayEvent(impl_->replay_writer, "cycle_input", "ArAbandonReplayOperationInputV2",
                   EncodeAbandonReplayOperationInputFlatbuffer(operation), token.world_cycle_index,
                   std::numeric_limits<double>::quiet_NaN());
  const ArAbandonCycleStatus status = impl_->session.AbandonCycle(token);
  ArAbandonReplayRecord record;
  record.status = status;
  record.session_state = ArSessionReplayAccess::CaptureSessionState(impl_->session);
  WriteReplayEvent(impl_->replay_writer, "cycle_output", "ArAbandonReplayRecordV2",
                   EncodeAbandonReplayRecordFlatbuffer(record), token.world_cycle_index,
                   std::numeric_limits<double>::quiet_NaN());
  return status;
}

void ArTraceSession::ApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch) {
  (void)TryApplyRuntimeConfig(patch);
}

bool ArTraceSession::TryApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch) {
  const bool accepted = impl_->session.TryApplyRuntimeConfig(patch);
  if (impl_->replay_writer) {
    oneq::replay::ReplayTraceEvent event;
    event.module = "airborne_radar";
    event.event_type = "runtime_config_patch";
    event.payload_type = "ArRuntimeConfigAttemptV2";
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = EncodeRuntimeConfigAttemptFlatbuffer(patch, accepted);
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
    event.payload_type = "ArExternalDecisionAttemptV2";
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = EncodeExternalDecisionAttemptFlatbuffer(response, status);
    event.has_cycle_index = true;
    event.cycle_index = response.source_cycle_index;
    impl_->replay_writer->WriteEvent(event);
  }
  return status;
}

const ArSession& ArTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace airborne_radar
