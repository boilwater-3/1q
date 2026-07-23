#include "1q/airborne_radar/session/ArReplaySession.h"

#include <memory>
#include <string>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

enum class PendingOperation {
  kNone = 0,
  kPrepare,
  kComplete,
  kAbandon,
};

struct ArReplayState {
  std::unique_ptr<ArSession> session{};
  PendingOperation pending_operation{PendingOperation::kNone};
  ArPrepareCycleInput prepare_input{};
  ArCompleteReplayOperationInput complete_operation{};
  ArAbandonReplayOperationInput abandon_operation{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "ArSessionConfig") {
    *error = "AR replay expected ArSessionConfig session_config";
    return false;
  }
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (state->session) {
    *error = "AR replay received duplicate session_config";
    return false;
  }
  config::ArSessionConfig config;
  if (!DecodeSessionConfigFlatbuffer(event.payload_bytes, &config, error)) {
    return false;
  }
  state->session.reset(new ArSession(ArSession::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received cycle_input before session_config";
    return false;
  }
  if (state->pending_operation != PendingOperation::kNone) {
    *error = "AR replay received consecutive cycle_input without cycle_output";
    return false;
  }

  if (event.payload_type == "ArPrepareCycleInputV2") {
    if (!DecodePrepareCycleInputFlatbuffer(event.payload_bytes, &state->prepare_input, error)) {
      return false;
    }
    state->pending_operation = PendingOperation::kPrepare;
    return true;
  }
  if (event.payload_type == "ArCompleteReplayOperationInputV2") {
    if (!DecodeCompleteReplayOperationInputFlatbuffer(event.payload_bytes,
                                                      &state->complete_operation, error)) {
      return false;
    }
    state->pending_operation = PendingOperation::kComplete;
    return true;
  }
  if (event.payload_type == "ArAbandonReplayOperationInputV2") {
    if (!DecodeAbandonReplayOperationInputFlatbuffer(event.payload_bytes, &state->abandon_operation,
                                                     error)) {
      return false;
    }
    state->pending_operation = PendingOperation::kAbandon;
    return true;
  }
  *error = "AR replay rejects legacy or unknown cycle_input payload type: " + event.payload_type;
  return false;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  if (event.payload_type != "ArRuntimeConfigAttemptV2") {
    *error = "AR replay rejects legacy runtime_config_patch payload type: " + event.payload_type;
    return false;
  }
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received runtime_config_patch before session_config";
    return false;
  }
  config::ArRuntimeConfigPatch patch;
  bool expected_accepted = false;
  if (!DecodeRuntimeConfigAttemptFlatbuffer(event.payload_bytes, &patch, &expected_accepted,
                                            error)) {
    return false;
  }
  const bool actual_accepted = state->session->TryApplyRuntimeConfig(patch);
  if (actual_accepted != expected_accepted) {
    *error = "AR replay runtime patch apply result divergence";
    return false;
  }
  return true;
}

bool OnDecisionInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "ArExternalDecisionAttemptV2") {
    *error = "AR replay rejects legacy decision_input payload type: " + event.payload_type;
    return false;
  }
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received decision_input before session_config";
    return false;
  }
  ExternalDecisionResponse response;
  ExternalDecisionSubmitStatus expected_status =
      ExternalDecisionSubmitStatus::kNoPendingObservation;
  if (!DecodeExternalDecisionAttemptFlatbuffer(event.payload_bytes, &response, &expected_status,
                                               error)) {
    return false;
  }
  const ExternalDecisionSubmitStatus actual_status =
      state->session->SubmitExternalDecision(response);
  if (actual_status != expected_status) {
    *error = "AR replay external decision submit result divergence";
    return false;
  }
  return true;
}

oneq::replay::ReplayTraceOutputStatus CompareEncodedRecord(const std::string& expected_payload,
                                                           const std::string& actual_payload,
                                                           const char* payload_type,
                                                           std::string* actual_output,
                                                           std::string* error) {
  if (expected_payload == actual_payload) {
    actual_output->clear();
    return oneq::replay::ReplayTraceOutputStatus::kHandledByModule;
  }
  *actual_output = std::string("{\"payload_type\":\"") + payload_type +
                   "\",\"actual_size\":" + std::to_string(actual_payload.size()) + "}";
  *error = std::string("AR replay output divergence (") + payload_type + ")";
  return oneq::replay::ReplayTraceOutputStatus::kDivergence;
}

oneq::replay::ReplayTraceOutputStatus OnPrepareOutput(
    const oneq::replay::ReplayTraceReadEvent& event, ArReplayState* state,
    std::string* actual_output, std::string* error) {
  if (event.payload_type != "ArPrepareReplayRecordV2") {
    *error = "AR replay Prepare expected ArPrepareReplayRecordV2 cycle_output";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  ArPrepareReplayRecord expected;
  if (!DecodePrepareReplayRecordFlatbuffer(event.payload_bytes, &expected, error)) {
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  ArPrepareReplayRecord actual;
  actual.result = state->session->PrepareCycle(state->prepare_input);
  actual.session_state = ArSessionReplayAccess::CaptureSessionState(*state->session);
  state->pending_operation = PendingOperation::kNone;
  return CompareEncodedRecord(event.payload_bytes, EncodePrepareReplayRecordFlatbuffer(actual),
                              "ArPrepareReplayRecordV2", actual_output, error);
}

oneq::replay::ReplayTraceOutputStatus OnCompleteOutput(
    const oneq::replay::ReplayTraceReadEvent& event, ArReplayState* state,
    std::string* actual_output, std::string* error) {
  if (event.payload_type != "ArCompleteReplayRecordV2") {
    *error = "AR replay Complete expected ArCompleteReplayRecordV2 cycle_output";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  ArCompleteReplayRecord expected;
  if (!DecodeCompleteReplayRecordFlatbuffer(event.payload_bytes, &expected, error)) {
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  ArCompleteReplayRecord actual;
  actual.result = state->session->CompleteCycle(state->complete_operation.token,
                                                state->complete_operation.input);
  actual.session_state = ArSessionReplayAccess::CaptureSessionState(*state->session);
  state->pending_operation = PendingOperation::kNone;
  return CompareEncodedRecord(event.payload_bytes, EncodeCompleteReplayRecordFlatbuffer(actual),
                              "ArCompleteReplayRecordV2", actual_output, error);
}

oneq::replay::ReplayTraceOutputStatus OnAbandonOutput(
    const oneq::replay::ReplayTraceReadEvent& event, ArReplayState* state,
    std::string* actual_output, std::string* error) {
  if (event.payload_type != "ArAbandonReplayRecordV2") {
    *error = "AR replay Abandon expected ArAbandonReplayRecordV2 cycle_output";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  ArAbandonReplayRecord expected;
  if (!DecodeAbandonReplayRecordFlatbuffer(event.payload_bytes, &expected, error)) {
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  ArAbandonReplayRecord actual;
  actual.status = state->session->AbandonCycle(state->abandon_operation.token);
  actual.session_state = ArSessionReplayAccess::CaptureSessionState(*state->session);
  state->pending_operation = PendingOperation::kNone;
  return CompareEncodedRecord(event.payload_bytes, EncodeAbandonReplayRecordFlatbuffer(actual),
                              "ArAbandonReplayRecordV2", actual_output, error);
}

oneq::replay::ReplayTraceOutputStatus OnCycleOutput(const oneq::replay::ReplayTraceReadEvent& event,
                                                    void* user_data, std::string* actual_output,
                                                    std::string* error) {
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received cycle_output before session_config";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  switch (state->pending_operation) {
    case PendingOperation::kPrepare:
      return OnPrepareOutput(event, state, actual_output, error);
    case PendingOperation::kComplete:
      return OnCompleteOutput(event, state, actual_output, error);
    case PendingOperation::kAbandon:
      return OnAbandonOutput(event, state, actual_output, error);
    case PendingOperation::kNone:
      *error = "AR replay cycle_output arrived before cycle_input";
      return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  *error = "AR replay pending operation is invalid";
  return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload = event.payload_bytes;
  return DecodeFailureMarkerFlatbuffer(event.payload_bytes, &state->failure_marker_data, error);
}

}  // namespace

ArReplaySessionResult ReplayArTrace(const std::string& trace_dir) {
  ArReplaySessionResult result;
  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "airborne_radar";
  expectation.require_module_match = true;
  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.first_error = result.report.first_error;
    return result;
  }

  ArReplayState state;
  oneq::replay::ReplayTracePlaybackCallbacks callbacks;
  callbacks.user_data = &state;
  callbacks.on_session_config = OnSessionConfig;
  callbacks.on_cycle_input = OnCycleInput;
  callbacks.on_decision_input = OnDecisionInput;
  callbacks.on_runtime_config_patch = OnRuntimeConfigPatch;
  callbacks.on_cycle_output = OnCycleOutput;
  callbacks.on_failure_marker = OnFailureMarker;

  oneq::replay::ReplayTracePlaybackOptions options;
  options.require_output_callback = true;
  options.stop_on_first_divergence = true;
  options.stop_on_failure_marker = false;
  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
  if (result.ok && state.pending_operation != PendingOperation::kNone) {
    result.ok = false;
    result.playback.ok = false;
    result.first_error = "AR replay ended with pending cycle_input without cycle_output";
    result.playback.first_error = result.first_error;
  }
  result.reached_failure_marker = state.reached_failure_marker;
  result.failure_marker_payload = state.failure_marker_payload;
  result.failure_marker_data = state.failure_marker_data;
  if (!result.ok && result.first_error.empty()) {
    result.first_error = result.playback.first_error;
  }
  return result;
}

}  // namespace session
}  // namespace airborne_radar
