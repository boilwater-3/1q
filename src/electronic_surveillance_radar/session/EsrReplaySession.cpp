#include "1q/electronic_surveillance_radar/session/EsrReplaySession.h"

#include <memory>
#include <string>
#include <vector>

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

struct EsrReplayState {
  std::unique_ptr<EsrSession> session{};
  EsrCycleInput pending_input{};
  bool has_pending_input{false};
  EsrCycleResult latest_result{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool EmitterObservationEqual(const session::EmitterObservation& left,
                             const session::EmitterObservation& right) {
  return left.observation_id == right.observation_id && left.timestamp_s == right.timestamp_s &&
         left.aoa_az_deg == right.aoa_az_deg && left.aoa_el_deg == right.aoa_el_deg &&
         left.rf_hz == right.rf_hz && left.bandwidth_hz == right.bandwidth_hz &&
         left.pri_s == right.pri_s && left.pulse_width_s == right.pulse_width_s &&
         left.rf_std_hz == right.rf_std_hz &&
         left.bandwidth_std_hz == right.bandwidth_std_hz &&
         left.pri_std_s == right.pri_std_s &&
         left.pulse_width_std_s == right.pulse_width_std_s &&
         left.amplitude_db == right.amplitude_db && left.snr_db == right.snr_db &&
         left.quality == right.quality;
}

bool EmitterHypothesisEqual(const session::EmitterHypothesis& left,
                            const session::EmitterHypothesis& right) {
  if (left.hypothesis_id != right.hypothesis_id || left.mode != right.mode ||
      left.threat_level != right.threat_level || left.bearing_az_deg != right.bearing_az_deg ||
      left.bearing_el_deg != right.bearing_el_deg ||
      left.bearing_std_deg != right.bearing_std_deg || left.confidence != right.confidence ||
      left.estimated_center_frequency_hz != right.estimated_center_frequency_hz ||
      left.estimated_bandwidth_hz != right.estimated_bandwidth_hz ||
      left.estimated_pri_s != right.estimated_pri_s ||
      left.estimated_pulse_width_s != right.estimated_pulse_width_s ||
      left.center_frequency_std_hz != right.center_frequency_std_hz ||
      left.bandwidth_std_hz != right.bandwidth_std_hz ||
      left.pri_std_s != right.pri_std_s ||
      left.pulse_width_std_s != right.pulse_width_std_s ||
      left.last_seen_cycle != right.last_seen_cycle ||
      left.candidate_classes.size() != right.candidate_classes.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.candidate_classes.size(); ++i) {
    if (left.candidate_classes[i] != right.candidate_classes[i]) {
      return false;
    }
  }
  return true;
}

bool TruthAssociationRecordEqual(const session::TruthAssociationRecord& left,
                                 const session::TruthAssociationRecord& right) {
  return left.observation_id == right.observation_id &&
         left.truth_emitter_id == right.truth_emitter_id && left.matched == right.matched &&
         left.confidence == right.confidence;
}

bool ObservationOutputFrameEqual(const session::ObservationOutputFrame& left,
                                 const session::ObservationOutputFrame& right) {
  if (left.raw_observation_count != right.raw_observation_count ||
      left.cluster_count != right.cluster_count ||
      left.receiver_center_frequency_hz != right.receiver_center_frequency_hz ||
      left.receiver_bandwidth_hz != right.receiver_bandwidth_hz ||
      left.receiver_saturated != right.receiver_saturated ||
      left.observations.size() != right.observations.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.observations.size(); ++i) {
    if (!EmitterObservationEqual(left.observations[i], right.observations[i])) {
      return false;
    }
  }
  return true;
}

bool EmitterOutputFrameEqual(const session::EmitterOutputFrame& left,
                             const session::EmitterOutputFrame& right) {
  if (left.hypotheses.size() != right.hypotheses.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.hypotheses.size(); ++i) {
    if (!EmitterHypothesisEqual(left.hypotheses[i], right.hypotheses[i])) {
      return false;
    }
  }
  return true;
}

bool TruthEvaluationFrameEqual(const session::TruthEvaluationFrame& left,
                               const session::TruthEvaluationFrame& right) {
  if (left.associations.size() != right.associations.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.associations.size(); ++i) {
    if (!TruthAssociationRecordEqual(left.associations[i], right.associations[i])) {
      return false;
    }
  }
  return true;
}

bool EsrOutputFrameEqual(const EsrOutputFrame& left, const EsrOutputFrame& right) {
  return left.cycle_index == right.cycle_index && left.batch_id == right.batch_id &&
         ObservationOutputFrameEqual(left.observation_output, right.observation_output) &&
         EmitterOutputFrameEqual(left.emitter_output, right.emitter_output) &&
         TruthEvaluationFrameEqual(left.truth_evaluation_output, right.truth_evaluation_output);
}

bool EsrValidationIssueEqual(const ValidationIssue& left, const ValidationIssue& right) {
  return left.severity == right.severity && left.code == right.code &&
         left.location.kind == right.location.kind &&
         left.location.entity_index == right.location.entity_index && left.field == right.field &&
         left.message == right.message;
}

bool EsrValidationIssueListEqual(const ValidationIssueList& left,
                                 const ValidationIssueList& right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (!EsrValidationIssueEqual(left[i], right[i])) {
      return false;
    }
  }
  return true;
}

bool EsrCycleResultEqual(const EsrCycleResult& left, const EsrCycleResult& right) {
  return left.input_cycle_index == right.input_cycle_index &&
         EsrOutputFrameEqual(left.output_frame, right.output_frame) &&
         EsrValidationIssueListEqual(left.validation_issues, right.validation_issues) &&
         left.has_validation_error == right.has_validation_error &&
         left.executed_this_cycle == right.executed_this_cycle &&
         left.reused_previous_output == right.reused_previous_output &&
         left.abort_reason == right.abort_reason;
}

bool ExecutePendingCycle(EsrReplayState* state, std::string* error) {
  if (!state->session) {
    *error = "ESR replay cannot execute before session_config";
    return false;
  }
  if (!state->has_pending_input) {
    *error = "ESR replay cycle_output arrived before cycle_input";
    return false;
  }

  EsrCycleResult result = state->session->StepWithResult(state->pending_input);
  state->latest_result = result;
  state->has_pending_input = false;
  return true;
}

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "EsrSessionConfig") {
    *error = "ESR replay expected EsrSessionConfig session_config";
    return false;
  }

  EsrReplayState* state = static_cast<EsrReplayState*>(user_data);
  config::EsrSessionConfig config;
  if (!DecodeEsrSessionConfig(event.payload_bytes, &config)) {
    *error = "ESR replay failed to decode session_config";
    return false;
  }
  state->session.reset(new EsrSession(EsrSession::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  if (event.payload_type != "EsrCycleInput") {
    *error = "ESR replay expected EsrCycleInput cycle_input";
    return false;
  }

  EsrReplayState* state = static_cast<EsrReplayState*>(user_data);
  if (!state->session) {
    *error = "ESR replay received cycle_input before session_config";
    return false;
  }

  if (state->has_pending_input) {
    *error = "ESR replay received consecutive cycle_input without intervening cycle_output";
    return false;
  }

  EsrCycleInput input;
  if (!DecodeEsrCycleInput(event.payload_bytes, &input)) {
    *error = "ESR replay failed to decode cycle_input";
    return false;
  }

  state->pending_input = input;
  state->has_pending_input = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  if (event.payload_type != "EsrRuntimeConfigPatch") {
    *error = "ESR replay expected EsrRuntimeConfigPatch runtime_config_patch";
    return false;
  }

  EsrReplayState* state = static_cast<EsrReplayState*>(user_data);
  if (!state->session) {
    *error = "ESR replay received runtime_config_patch before session_config";
    return false;
  }

  config::EsrRuntimeConfigPatch patch;
  if (!DecodeEsrRuntimeConfigPatch(event.payload_bytes, &patch)) {
    *error = "ESR replay failed to decode runtime_config_patch";
    return false;
  }
  state->session->ApplyRuntimeConfig(patch);
  return true;
}

oneq::replay::ReplayTraceOutputStatus OnCycleOutput(const oneq::replay::ReplayTraceReadEvent& event,
                                                     void* user_data, std::string* actual_output,
                                                     std::string* error) {
  using oneq::replay::ReplayTraceOutputStatus;
  if (event.payload_type != "EsrCycleResult" && event.payload_type != "EsrOutputFrame") {
    *error = "ESR replay does not support cycle_output payload type: " + event.payload_type;
    return ReplayTraceOutputStatus::kOtherFailure;
  }

  EsrReplayState* state = static_cast<EsrReplayState*>(user_data);
  if (!ExecutePendingCycle(state, error)) {
    return ReplayTraceOutputStatus::kOtherFailure;
  }

  if (event.payload_type == "EsrCycleResult") {
    EsrCycleResult expected_result;
    if (!DecodeEsrCycleResult(event.payload_bytes, &expected_result)) {
      *error = "ESR replay failed to decode EsrCycleResult";
      return ReplayTraceOutputStatus::kOtherFailure;
    }
    if (!EsrCycleResultEqual(expected_result, state->latest_result)) {
      *error = "ESR replay output divergence (EsrCycleResult)";
      actual_output->clear();
      return ReplayTraceOutputStatus::kDivergence;
    }
    actual_output->clear();
    return ReplayTraceOutputStatus::kHandledByModule;
  }

  {
    EsrOutputFrame expected_frame;
    if (!DecodeEsrOutputFrame(event.payload_bytes, &expected_frame)) {
      *error = "ESR replay failed to decode EsrOutputFrame";
      return ReplayTraceOutputStatus::kOtherFailure;
    }
    if (!EsrOutputFrameEqual(expected_frame, state->latest_result.output_frame)) {
      *error = "ESR replay output divergence (EsrOutputFrame)";
      actual_output->clear();
      return ReplayTraceOutputStatus::kDivergence;
    }
    actual_output->clear();
    return ReplayTraceOutputStatus::kHandledByModule;
  }
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  EsrReplayState* state = static_cast<EsrReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload = event.payload_bytes;
  // 空 payload 兼容旧版单参写入的失败标记：仅置 reached，不解码。
  if (event.payload_bytes.empty()) {
    return true;
  }
  oneq::replay::ReplayTraceFailure decoded;
  if (!DecodeEsrFailureMarker(event.payload_bytes, &decoded, error)) {
    return false;
  }
  state->failure_marker_data = decoded;
  return true;
}

}  // namespace

EsrReplaySessionResult ReplayEsrTrace(const std::string& trace_dir) {
  EsrReplaySessionResult result;

  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "electronic_surveillance_radar";
  expectation.require_module_match = true;

  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.ok = false;
    result.first_error = result.report.first_error;
    return result;
  }

  EsrReplayState state;
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
  // A rejected cycle does not terminate the live session. Replay must preserve
  // that boundary and compare any following recovery cycles.
  options.stop_on_failure_marker = false;

  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
  if (result.ok && state.has_pending_input) {
    result.ok = false;
    result.playback.ok = false;
    result.first_error = "ESR replay ended with pending cycle_input without cycle_output";
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
}  // namespace electronic_surveillance_radar
