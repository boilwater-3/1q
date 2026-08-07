#include "1q/airborne_radar/session/ArReplaySession.h"

#include <memory>
#include <string>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

struct ArReplayState {
  std::unique_ptr<ArSession> session{};
  bool has_pending_cycle{false};
  ArCycleInput pending_input{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event,
                     void* user_data, std::string* error) {
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

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event,
                  void* user_data, std::string* error) {
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received cycle_input before session_config";
    return false;
  }
  if (state->has_pending_cycle) {
    *error = "AR replay received consecutive cycle_input without cycle_output";
    return false;
  }
  if (event.payload_type != "ArCycleInputV3") {
    *error =
        "AR replay rejects legacy or unknown cycle_input payload type: " +
        event.payload_type;
    return false;
  }
  if (!DecodeCycleInputFlatbuffer(event.payload_bytes, &state->pending_input,
                                  error)) {
    return false;
  }
  state->has_pending_cycle = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event,
                          void* user_data, std::string* error) {
  if (event.payload_type != "ArRuntimeConfigAttemptV3") {
    *error =
        "AR replay rejects legacy runtime_config_patch payload type: " +
        event.payload_type;
    return false;
  }
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received runtime_config_patch before session_config";
    return false;
  }
  config::ArRuntimeConfigPatch patch;
  bool expected_accepted = false;
  if (!DecodeRuntimeConfigAttemptFlatbuffer(event.payload_bytes, &patch,
                                            &expected_accepted, error)) {
    return false;
  }
  const bool actual_accepted =
      state->session->TryApplyRuntimeConfig(patch);
  if (actual_accepted != expected_accepted) {
    *error = "AR replay runtime patch apply result divergence";
    return false;
  }
  return true;
}

bool OnDecisionInput(const oneq::replay::ReplayTraceReadEvent& event,
                     void* user_data, std::string* error) {
  if (event.payload_type != "ArControlProfilePayload") {
    *error =
        "AR replay rejects unknown decision_input payload type: " +
        event.payload_type;
    return false;
  }
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received decision_input before session_config";
    return false;
  }
  session::ArControlProfile profile;
  if (!DecodeArControlProfileFlatbuffer(event.payload_bytes, &profile, error)) {
    return false;
  }
  session::ExternalDecisionOverride override_decision;
  override_decision.profile = profile;
  const session::ExternalDecisionSubmitStatus status =
      state->session->SubmitExternalDecision(std::move(override_decision));
  if (status != session::ExternalDecisionSubmitStatus::kAccepted) {
    *error = "AR replay decision_input override rejected";
    return false;
  }
  return true;
}

oneq::replay::ReplayTraceOutputStatus OnCycleOutput(
    const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
    std::string* actual_output, std::string* error) {
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received cycle_output before session_config";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  if (!state->has_pending_cycle) {
    *error = "AR replay cycle_output arrived before cycle_input";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  if (event.payload_type != "ArCycleReplayRecordV3") {
    *error = "AR replay expected ArCycleReplayRecordV3 cycle_output";
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  ArCycleReplayRecord expected;
  if (!DecodeCycleReplayRecordFlatbuffer(event.payload_bytes, &expected,
                                         error)) {
    return oneq::replay::ReplayTraceOutputStatus::kOtherFailure;
  }
  ArCycleReplayRecord actual;
  actual.result = state->session->StepWithResult(state->pending_input);
  actual.session_state =
      ArSessionReplayAccess::CaptureSessionState(*state->session);
  state->has_pending_cycle = false;
  const std::string actual_payload =
      EncodeCycleReplayRecordFlatbuffer(actual);
  if (event.payload_bytes == actual_payload) {
    actual_output->clear();
    return oneq::replay::ReplayTraceOutputStatus::kHandledByModule;
  }
  // Field-level issues comparison for targeted error reporting.
  if (expected.result.issues.size() != actual.result.issues.size()) {
    *actual_output =
        "{\"payload_type\":\"ArCycleReplayRecordV3\",\"actual_size\":" +
        std::to_string(actual_payload.size()) +
        ",\"expected_issue_count\":" +
        std::to_string(expected.result.issues.size()) +
        ",\"actual_issue_count\":" +
        std::to_string(actual.result.issues.size()) + "}";
    *error = "AR replay issues divergence";
    return oneq::replay::ReplayTraceOutputStatus::kDivergence;
  }
  *actual_output =
      "{\"payload_type\":\"ArCycleReplayRecordV3\",\"actual_size\":" +
      std::to_string(actual_payload.size()) + "}";
  *error = "AR replay output divergence (ArCycleReplayRecordV3)";
  return oneq::replay::ReplayTraceOutputStatus::kDivergence;
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event,
                     void* user_data, std::string* error) {
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload = event.payload_bytes;
  return DecodeFailureMarkerFlatbuffer(event.payload_bytes,
                                       &state->failure_marker_data, error);
}

}  // namespace

ArReplaySessionResult ReplayArTrace(const std::string& trace_dir) {
  ArReplaySessionResult result;
  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "airborne_radar";
  expectation.require_module_match = true;
  result.report =
      oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
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
  result.playback =
      oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
  if (result.ok && state.has_pending_cycle) {
    result.ok = false;
    result.playback.ok = false;
    result.first_error =
        "AR replay ended with pending cycle_input without cycle_output";
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
