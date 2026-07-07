#include "1q/sbirs_sensor/session/SbirsReplaySession.h"

#include <memory>
#include <string>

#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "sbirs_sensor/session/SbirsReplayFlatbufferCodec.h"

namespace sbirs_sensor {
namespace session {
namespace {

struct SbirsReplayState {
  std::unique_ptr<SbirsSession> session{};
  SbirsCycleInput pending_input{};
  bool has_pending_input{false};
  SbirsCycleResult latest_result{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool DetectionEqual(const output::SbirsDetectionRecord& left,
                    const output::SbirsDetectionRecord& right) {
  return left.detection_id == right.detection_id && left.azimuth_deg == right.azimuth_deg &&
         left.elevation_deg == right.elevation_deg &&
         left.infrared_snr_linear == right.infrared_snr_linear &&
         left.observation_stage == right.observation_stage && left.detected == right.detected;
}

bool OutputFrameEqual(const SbirsOutputFrame& left, const SbirsOutputFrame& right) {
  if (left.cycle_index != right.cycle_index || left.scan_azimuth_deg != right.scan_azimuth_deg ||
      left.detections.size() != right.detections.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.detections.size(); ++i) {
    if (!DetectionEqual(left.detections[i], right.detections[i])) {
      return false;
    }
  }
  return true;
}

bool AttributionEqual(const attribution::SbirsDetectionAttributionRecord& left,
                      const attribution::SbirsDetectionAttributionRecord& right) {
  return left.detection_id == right.detection_id && left.target_id == right.target_id &&
         left.target_name == right.target_name &&
         left.estimated_range_m == right.estimated_range_m &&
         left.used_truth_assist == right.used_truth_assist &&
         left.capture_failure_reason == right.capture_failure_reason;
}

bool AttributionListEqual(const attribution::SbirsDetectionAttributionRecordList& left,
                          const attribution::SbirsDetectionAttributionRecordList& right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (!AttributionEqual(left[i], right[i])) {
      return false;
    }
  }
  return true;
}

bool ValidationIssueEqual(const ValidationIssue& left, const ValidationIssue& right) {
  return left.severity == right.severity && left.location.kind == right.location.kind &&
         left.location.entity_index == right.location.entity_index && left.message == right.message;
}

bool ValidationIssueListEqual(const ValidationIssueList& left, const ValidationIssueList& right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (!ValidationIssueEqual(left[i], right[i])) {
      return false;
    }
  }
  return true;
}

bool CycleResultEqual(const SbirsCycleResult& left, const SbirsCycleResult& right) {
  return left.input_cycle_index == right.input_cycle_index &&
         OutputFrameEqual(left.output_frame, right.output_frame) &&
         AttributionListEqual(left.detection_attributions, right.detection_attributions) &&
         ValidationIssueListEqual(left.validation_issues, right.validation_issues) &&
         left.has_validation_error == right.has_validation_error &&
         left.executed_this_cycle == right.executed_this_cycle &&
         left.reused_previous_output == right.reused_previous_output &&
         left.abort_reason == right.abort_reason;
}

bool ExecutePendingCycle(SbirsReplayState* state, std::string* error) {
  if (!state->session) {
    *error = "SBIRS replay cannot execute before session_config";
    return false;
  }
  if (!state->has_pending_input) {
    *error = "SBIRS replay cycle_output arrived before cycle_input";
    return false;
  }
  state->latest_result = state->session->StepWithResult(state->pending_input);
  state->has_pending_input = false;
  return true;
}

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "SbirsSessionConfig") {
    *error = "SBIRS replay expected SbirsSessionConfig session_config";
    return false;
  }
  config::SbirsSessionConfig config;
  if (!DecodeSbirsSessionConfig(event.payload_bytes, &config)) {
    *error = "SBIRS replay failed to decode session_config";
    return false;
  }
  SbirsReplayState* state = static_cast<SbirsReplayState*>(user_data);
  state->session.reset(new SbirsSession(SbirsSession::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  if (event.payload_type != "SbirsCycleInput") {
    *error = "SBIRS replay expected SbirsCycleInput cycle_input";
    return false;
  }
  SbirsReplayState* state = static_cast<SbirsReplayState*>(user_data);
  if (!state->session) {
    *error = "SBIRS replay received cycle_input before session_config";
    return false;
  }
  if (state->has_pending_input) {
    *error = "SBIRS replay received consecutive cycle_input without intervening cycle_output";
    return false;
  }
  if (!DecodeSbirsCycleInput(event.payload_bytes, &state->pending_input)) {
    *error = "SBIRS replay failed to decode cycle_input";
    return false;
  }
  state->has_pending_input = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  if (event.payload_type != "SbirsRuntimeConfigPatch") {
    *error = "SBIRS replay expected SbirsRuntimeConfigPatch runtime_config_patch";
    return false;
  }
  SbirsReplayState* state = static_cast<SbirsReplayState*>(user_data);
  if (!state->session) {
    *error = "SBIRS replay received runtime_config_patch before session_config";
    return false;
  }
  config::SbirsRuntimeConfigPatch patch;
  if (!DecodeSbirsRuntimeConfigPatch(event.payload_bytes, &patch)) {
    *error = "SBIRS replay failed to decode runtime_config_patch";
    return false;
  }
  state->session->ApplyRuntimeConfig(patch);
  return true;
}

oneq::replay::ReplayTraceOutputStatus OnCycleOutput(const oneq::replay::ReplayTraceReadEvent& event,
                                                    void* user_data, std::string* actual_output,
                                                    std::string* error) {
  using oneq::replay::ReplayTraceOutputStatus;
  if (event.payload_type != "SbirsCycleResult" && event.payload_type != "SbirsOutputFrame") {
    *error = "SBIRS replay does not support cycle_output payload type: " + event.payload_type;
    return ReplayTraceOutputStatus::kOtherFailure;
  }
  SbirsReplayState* state = static_cast<SbirsReplayState*>(user_data);
  if (!ExecutePendingCycle(state, error)) {
    return ReplayTraceOutputStatus::kOtherFailure;
  }
  if (event.payload_type == "SbirsCycleResult") {
    SbirsCycleResult expected;
    if (!DecodeSbirsCycleResult(event.payload_bytes, &expected)) {
      *error = "SBIRS replay failed to decode SbirsCycleResult";
      return ReplayTraceOutputStatus::kOtherFailure;
    }
    if (!CycleResultEqual(expected, state->latest_result)) {
      *error = "SBIRS replay output divergence (SbirsCycleResult)";
      actual_output->clear();
      return ReplayTraceOutputStatus::kDivergence;
    }
    actual_output->clear();
    return ReplayTraceOutputStatus::kHandledByModule;
  }
  SbirsOutputFrame expected_frame;
  if (!DecodeSbirsOutputFrame(event.payload_bytes, &expected_frame)) {
    *error = "SBIRS replay failed to decode SbirsOutputFrame";
    return ReplayTraceOutputStatus::kOtherFailure;
  }
  if (!OutputFrameEqual(expected_frame, state->latest_result.output_frame)) {
    *error = "SBIRS replay output divergence (SbirsOutputFrame)";
    actual_output->clear();
    return ReplayTraceOutputStatus::kDivergence;
  }
  actual_output->clear();
  return ReplayTraceOutputStatus::kHandledByModule;
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  SbirsReplayState* state = static_cast<SbirsReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload = event.payload_bytes;
  if (event.payload_bytes.empty()) {
    return true;
  }
  return DecodeSbirsFailureMarker(event.payload_bytes, &state->failure_marker_data, error);
}

}  // namespace

SbirsReplaySessionResult ReplaySbirsTrace(const std::string& trace_dir) {
  SbirsReplaySessionResult result;

  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "sbirs_sensor";
  expectation.require_module_match = true;

  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.ok = false;
    result.first_error = result.report.first_error;
    return result;
  }

  SbirsReplayState state;
  oneq::replay::ReplayTracePlaybackCallbacks callbacks;
  callbacks.user_data = &state;
  callbacks.on_session_config = OnSessionConfig;
  callbacks.on_cycle_input = OnCycleInput;
  callbacks.on_runtime_config_patch = OnRuntimeConfigPatch;
  callbacks.on_cycle_output = OnCycleOutput;
  callbacks.on_failure_marker = OnFailureMarker;

  oneq::replay::ReplayTracePlaybackOptions options;
  options.require_output_callback = true;
  options.stop_on_first_divergence = true;
  options.stop_on_failure_marker = true;

  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
  if (result.ok && state.has_pending_input) {
    result.ok = false;
    result.playback.ok = false;
    result.first_error = "SBIRS replay ended with pending cycle_input without cycle_output";
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
}  // namespace sbirs_sensor
