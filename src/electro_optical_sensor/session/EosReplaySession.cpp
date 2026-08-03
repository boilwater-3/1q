#include "1q/electro_optical_sensor/session/EosReplaySession.h"

#include <memory>
#include <string>
#include <vector>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "electro_optical_sensor/session/EosReplayFlatbufferCodec.h"

namespace electro_optical_sensor {
namespace session {
namespace {

struct EosReplayState {
  std::unique_ptr<EosSession> session{};
  EosCycleInput pending_input{};
  bool has_pending_input{false};
  EosCycleResult latest_result{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool EosDetectionRecordEqual(const output::EosDetectionRecord& left,
                             const output::EosDetectionRecord& right) {
  return left.detection_id == right.detection_id && left.range_m == right.range_m &&
         left.azimuth_deg == right.azimuth_deg && left.elevation_deg == right.elevation_deg &&
         left.infrared_snr_linear == right.infrared_snr_linear &&
         left.visible_snr_linear == right.visible_snr_linear &&
         left.fused_snr_linear == right.fused_snr_linear &&
         left.fused_snr_db == right.fused_snr_db && left.detected == right.detected;
}

bool EosOutputFrameEqual(const EosOutputFrame& left, const EosOutputFrame& right) {
  if (left.cycle_index != right.cycle_index || left.scan_azimuth_deg != right.scan_azimuth_deg ||
      left.detections.size() != right.detections.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.detections.size(); ++i) {
    if (!EosDetectionRecordEqual(left.detections[i], right.detections[i])) {
      return false;
    }
  }
  return true;
}

bool EosDetectionAttributionEqual(const attribution::EosDetectionAttributionRecord& left,
                                  const attribution::EosDetectionAttributionRecord& right) {
  return left.detection_id == right.detection_id && left.target_id == right.target_id &&
         left.target_name == right.target_name;
}

bool EosDetectionAttributionListEqual(const attribution::EosDetectionAttributionRecordList& left,
                                      const attribution::EosDetectionAttributionRecordList& right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (!EosDetectionAttributionEqual(left[i], right[i])) {
      return false;
    }
  }
  return true;
}

bool EosValidationIssueEqual(const ValidationIssue& left, const ValidationIssue& right) {
  return left.severity == right.severity && left.code == right.code &&
         left.location.kind == right.location.kind &&
         left.location.entity_index == right.location.entity_index && left.field == right.field &&
         left.message == right.message;
}

bool EosValidationIssueListEqual(const ValidationIssueList& left,
                                 const ValidationIssueList& right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (!EosValidationIssueEqual(left[i], right[i])) {
      return false;
    }
  }
  return true;
}

bool EosCycleResultEqual(const EosCycleResult& left, const EosCycleResult& right) {
  return left.input_cycle_index == right.input_cycle_index &&
         EosOutputFrameEqual(left.output_frame, right.output_frame) &&
         EosDetectionAttributionListEqual(left.detection_attributions,
                                          right.detection_attributions) &&
         EosValidationIssueListEqual(left.validation_issues, right.validation_issues) &&
         left.has_validation_error == right.has_validation_error &&
         left.executed_this_cycle == right.executed_this_cycle &&
         left.abort_reason == right.abort_reason;
}

bool ExecutePendingCycle(EosReplayState* state, std::string* error) {
  if (!state->session) {
    *error = "EOS replay cannot execute before session_config";
    return false;
  }
  if (!state->has_pending_input) {
    *error = "EOS replay cycle_output arrived before cycle_input";
    return false;
  }

  EosCycleResult result = state->session->StepWithResult(state->pending_input);
  state->latest_result = result;
  state->has_pending_input = false;
  return true;
}

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "EosSessionConfig") {
    *error = "EOS replay expected EosSessionConfig session_config";
    return false;
  }

  EosReplayState* state = static_cast<EosReplayState*>(user_data);
  config::EosSessionConfig config;
  if (!DecodeEosSessionConfig(event.payload_bytes, &config)) {
    *error = "EOS replay failed to decode session_config";
    return false;
  }
  state->session.reset(new EosSession(EosSession::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  if (event.payload_type != "EosCycleInput") {
    *error = "EOS replay expected EosCycleInput cycle_input";
    return false;
  }

  EosReplayState* state = static_cast<EosReplayState*>(user_data);
  if (!state->session) {
    *error = "EOS replay received cycle_input before session_config";
    return false;
  }

  if (state->has_pending_input) {
    *error = "EOS replay received consecutive cycle_input without intervening cycle_output";
    return false;
  }

  EosCycleInput input;
  if (!DecodeEosCycleInput(event.payload_bytes, &input)) {
    *error = "EOS replay failed to decode cycle_input";
    return false;
  }

  state->pending_input = input;
  state->has_pending_input = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  if (event.payload_type != "EosRuntimeConfigPatch") {
    *error = "EOS replay expected EosRuntimeConfigPatch runtime_config_patch";
    return false;
  }

  EosReplayState* state = static_cast<EosReplayState*>(user_data);
  if (!state->session) {
    *error = "EOS replay received runtime_config_patch before session_config";
    return false;
  }

  config::EosRuntimeConfigPatch patch;
  if (!DecodeEosRuntimeConfigPatch(event.payload_bytes, &patch)) {
    *error = "EOS replay failed to decode runtime_config_patch";
    return false;
  }
  state->session->TryApplyRuntimeConfig(patch);
  return true;
}

oneq::replay::ReplayTraceOutputStatus OnCycleOutput(const oneq::replay::ReplayTraceReadEvent& event,
                                                     void* user_data, std::string* actual_output,
                                                     std::string* error) {
  using oneq::replay::ReplayTraceOutputStatus;
  if (event.payload_type != "EosCycleResult" && event.payload_type != "EosOutputFrame") {
    *error = "EOS replay does not support cycle_output payload type: " + event.payload_type;
    return ReplayTraceOutputStatus::kOtherFailure;
  }

  EosReplayState* state = static_cast<EosReplayState*>(user_data);
  if (!ExecutePendingCycle(state, error)) {
    return ReplayTraceOutputStatus::kOtherFailure;
  }

  if (event.payload_type == "EosCycleResult") {
    EosCycleResult expected_result;
    if (!DecodeEosCycleResult(event.payload_bytes, &expected_result)) {
      *error = "EOS replay failed to decode EosCycleResult";
      return ReplayTraceOutputStatus::kOtherFailure;
    }
    if (!EosCycleResultEqual(expected_result, state->latest_result)) {
      *error = "EOS replay output divergence (EosCycleResult)";
      actual_output->clear();
      return ReplayTraceOutputStatus::kDivergence;
    }
    actual_output->clear();
    return ReplayTraceOutputStatus::kHandledByModule;
  }

  {
    EosOutputFrame expected_frame;
    if (!DecodeEosOutputFrame(event.payload_bytes, &expected_frame)) {
      *error = "EOS replay failed to decode EosOutputFrame";
      return ReplayTraceOutputStatus::kOtherFailure;
    }
    if (!EosOutputFrameEqual(expected_frame, state->latest_result.output_frame)) {
      *error = "EOS replay output divergence (EosOutputFrame)";
      actual_output->clear();
      return ReplayTraceOutputStatus::kDivergence;
    }
    actual_output->clear();
    return ReplayTraceOutputStatus::kHandledByModule;
  }
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  EosReplayState* state = static_cast<EosReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload = event.payload_bytes;
  // 空 payload 兼容旧版单参写入的失败标记：仅置 reached，不解码。
  if (event.payload_bytes.empty()) {
    return true;
  }
  oneq::replay::ReplayTraceFailure decoded;
  if (!DecodeEosFailureMarker(event.payload_bytes, &decoded, error)) {
    return false;
  }
  state->failure_marker_data = decoded;
  return true;
}

}  // namespace

EosReplaySessionResult ReplayEosTrace(const std::string& trace_dir) {
  EosReplaySessionResult result;

  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "electro_optical_sensor";
  expectation.require_module_match = true;

  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.ok = false;
    result.first_error = result.report.first_error;
    return result;
  }

  EosReplayState state;
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
  // Failure markers diagnose recoverable rejected cycles; they do not make the
  // remaining trace ineligible for deterministic replay.
  options.stop_on_failure_marker = false;

  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
  if (result.ok && state.has_pending_input) {
    result.ok = false;
    result.playback.ok = false;
    result.first_error = "EOS replay ended with pending cycle_input without cycle_output";
    result.playback.first_error = result.first_error;
  }
  result.reached_failure_marker = state.reached_failure_marker;
  result.failure_marker_payload = state.failure_marker_payload;
  result.failure_marker_data = state.failure_marker_data;
  if (!result.ok) {
    result.first_error = result.playback.first_error;
  }
  return result;
}

}  // namespace session
}  // namespace electro_optical_sensor
