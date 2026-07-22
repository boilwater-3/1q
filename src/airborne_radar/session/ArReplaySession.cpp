#include "1q/airborne_radar/session/ArReplaySession.h"

#include <memory>
#include <string>
#include <vector>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

struct ArReplayState {
  std::unique_ptr<ArSession> session{};
  ArCycleInput pending_input{};
  bool has_pending_input{false};
  ArCycleResult latest_result{};
  ArDecisionReplayState latest_decision_state{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool TrackStateSnapshotEqual(const session::TrackStateSnapshot& left,
                             const session::TrackStateSnapshot& right) {
  return left.association_key == right.association_key &&
         left.external_target_id == right.external_target_id &&
         left.target_name == right.target_name && left.status == right.status &&
         left.position_x == right.position_x && left.position_y == right.position_y &&
         left.position_z == right.position_z && left.velocity_x == right.velocity_x &&
         left.velocity_y == right.velocity_y && left.velocity_z == right.velocity_z &&
         left.speed == right.speed && left.acceleration_x == right.acceleration_x &&
         left.acceleration_y == right.acceleration_y &&
         left.acceleration_z == right.acceleration_z && left.acceleration == right.acceleration &&
         left.rcs == right.rcs && left.hit_count == right.hit_count &&
         left.miss_count == right.miss_count &&
         left.target_type == right.target_type &&
         left.target_probability == right.target_probability;
}

bool TrackOutputFrameEqual(const session::TrackOutputFrame& left,
                           const session::TrackOutputFrame& right) {
  if (left.cycle_index != right.cycle_index || left.tracks.size() != right.tracks.size() ||
      left.batch_id != right.batch_id) {
    return false;
  }
  for (std::size_t i = 0; i < left.tracks.size(); ++i) {
    if (!TrackStateSnapshotEqual(left.tracks[i], right.tracks[i])) {
      return false;
    }
  }
  return true;
}

bool ValidationIssueEqual(const ValidationIssue& left, const ValidationIssue& right) {
  return left.severity == right.severity && left.code == right.code &&
         left.location.kind == right.location.kind &&
         left.location.entity_index == right.location.entity_index && left.field == right.field &&
         left.message == right.message;
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

bool ArCommandsEqual(const std::vector<session::ArCommand>& left,
                     const std::vector<session::ArCommand>& right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (left[i].type != right[i].type || left[i].source != right[i].source) {
      return false;
    }
  }
  return true;
}

bool ArControlProfileEqual(const session::ArControlProfile& left,
                           const session::ArControlProfile& right) {
  return left.version == right.version &&
         left.enable_lpi_power_control == right.enable_lpi_power_control &&
         left.lpi_power_scale == right.lpi_power_scale &&
         left.enable_lpi_beamforming == right.enable_lpi_beamforming &&
         left.lpi_dwell_scale == right.lpi_dwell_scale &&
         left.enable_agility_frequency == right.enable_agility_frequency &&
         left.agility_frequency_hop_phase == right.agility_frequency_hop_phase &&
         left.enable_sidelobe_canceller == right.enable_sidelobe_canceller &&
         left.enable_adaptive_beamforming == right.enable_adaptive_beamforming &&
         left.enable_eccm_rejitter == right.enable_eccm_rejitter &&
         left.eccm_burnthrough_gain == right.eccm_burnthrough_gain;
}

bool AssociationQualityMetricsEqual(const session::AssociationQualityMetrics& left,
                                    const session::AssociationQualityMetrics& right) {
  return left.prior_track_count == right.prior_track_count &&
         left.detection_count == right.detection_count &&
         left.matched_count == right.matched_count &&
         left.new_track_count == right.new_track_count &&
         left.missed_track_count == right.missed_track_count &&
         left.match_rate == right.match_rate && left.new_track_rate == right.new_track_rate &&
         left.missed_track_rate == right.missed_track_rate &&
         left.mean_match_cost == right.mean_match_cost &&
         left.p95_match_cost == right.p95_match_cost &&
         left.association_stress == right.association_stress;
}

bool ArInterferenceObservationEqual(const session::ArInterferenceObservation& left,
                                    const session::ArInterferenceObservation& right) {
  return left.observation_id == right.observation_id &&
         left.estimated_bearing_azimuth_deg == right.estimated_bearing_azimuth_deg &&
         left.estimated_bearing_elevation_deg == right.estimated_bearing_elevation_deg &&
         left.estimated_off_boresight_deg == right.estimated_off_boresight_deg &&
         left.estimated_center_frequency_hz == right.estimated_center_frequency_hz &&
         left.estimated_bandwidth_hz == right.estimated_bandwidth_hz &&
         left.estimated_waveform_kind == right.estimated_waveform_kind &&
         left.jammer_to_noise_db == right.jammer_to_noise_db &&
         left.bearing_standard_deviation_deg == right.bearing_standard_deviation_deg &&
         left.frequency_standard_deviation_hz == right.frequency_standard_deviation_hz &&
         left.bandwidth_standard_deviation_hz == right.bandwidth_standard_deviation_hz;
}

bool DecisionInputFrameEqual(const session::DecisionInputFrame& left,
                             const session::DecisionInputFrame& right) {
  if (left.cycle_index != right.cycle_index || left.batch_id != right.batch_id ||
      left.association_quality_info.match_rate != right.association_quality_info.match_rate ||
      left.association_quality_info.new_track_rate !=
          right.association_quality_info.new_track_rate ||
      left.association_quality_info.missed_track_rate !=
          right.association_quality_info.missed_track_rate ||
      left.association_quality_info.mean_match_cost !=
          right.association_quality_info.mean_match_cost ||
      left.association_quality_info.p95_match_cost !=
          right.association_quality_info.p95_match_cost ||
      left.association_quality_info.association_stress !=
          right.association_quality_info.association_stress ||
      left.perception_quality_info.input_target_count !=
          right.perception_quality_info.input_target_count ||
      left.perception_quality_info.detection_count !=
          right.perception_quality_info.detection_count ||
      left.perception_quality_info.detection_rate !=
          right.perception_quality_info.detection_rate ||
      left.perception_quality_info.detection_stress !=
          right.perception_quality_info.detection_stress ||
      left.interference_observations.size() != right.interference_observations.size() ||
      left.tracks.size() != right.tracks.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < left.interference_observations.size(); ++index) {
    if (!ArInterferenceObservationEqual(left.interference_observations[index],
                                        right.interference_observations[index])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < left.tracks.size(); ++i) {
    if (!TrackStateSnapshotEqual(left.tracks[i], right.tracks[i])) {
      return false;
    }
  }
  return true;
}

bool TacticalProposalEqual(const session::TacticalProposal& left,
                           const session::TacticalProposal& right) {
  return left.directive.type == right.directive.type &&
         left.directive.source == right.directive.source &&
         left.directive.has_requested_value == right.directive.has_requested_value &&
         left.directive.requested_value == right.directive.requested_value &&
         left.priority == right.priority && left.rationale == right.rationale;
}

bool TacticalProposalsEqual(const std::vector<session::TacticalProposal>& left,
                            const std::vector<session::TacticalProposal>& right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (!TacticalProposalEqual(left[i], right[i])) {
      return false;
    }
  }
  return true;
}

bool ExternalDecisionResponseEqual(const session::ExternalDecisionResponse& left,
                                   const session::ExternalDecisionResponse& right) {
  return left.source_cycle_index == right.source_cycle_index &&
         left.source_batch_id == right.source_batch_id &&
         TacticalProposalsEqual(left.proposals, right.proposals);
}

bool DecisionObservationEqual(const session::DecisionObservation& left,
                              const session::DecisionObservation& right) {
  return DecisionInputFrameEqual(left.input_frame, right.input_frame) &&
         ArControlProfileEqual(left.active_control_profile,
                               right.active_control_profile);
}

bool CycleResultEqual(const ArCycleResult& left, const ArCycleResult& right) {
  const bool base_equal = left.input_cycle_index == right.input_cycle_index &&
         TrackOutputFrameEqual(left.track_output_frame, right.track_output_frame) &&
         ArCommandsEqual(left.submitted_commands, right.submitted_commands) &&
         ValidationIssueListEqual(left.validation_issues, right.validation_issues) &&
         left.has_validation_error == right.has_validation_error &&
         left.executed_this_cycle == right.executed_this_cycle &&
         left.abort_reason == right.abort_reason &&
         left.reused_previous_output == right.reused_previous_output &&
         left.has_control_profile == right.has_control_profile &&
         ArControlProfileEqual(left.control_profile, right.control_profile) &&
         AssociationQualityMetricsEqual(left.association_quality_metrics,
                                        right.association_quality_metrics);
  return base_equal &&
         left.has_decision_observation == right.has_decision_observation &&
         DecisionObservationEqual(left.decision_observation,
                                  right.decision_observation) &&
         left.applied_decision_source == right.applied_decision_source &&
         left.applied_decision_cycle_index == right.applied_decision_cycle_index &&
         left.applied_decision_batch_id == right.applied_decision_batch_id;
}

bool DecisionReplayStateEqual(const ArDecisionReplayState& left,
                              const ArDecisionReplayState& right) {
  const bool pending_external_equal =
      left.has_pending_external_decision == right.has_pending_external_decision &&
      (!left.has_pending_external_decision ||
       ExternalDecisionResponseEqual(left.pending_external_decision,
                                     right.pending_external_decision));
  return left.has_pending_internal_decision == right.has_pending_internal_decision &&
         left.pending_internal_cycle_index == right.pending_internal_cycle_index &&
         left.pending_internal_batch_id == right.pending_internal_batch_id &&
         TacticalProposalsEqual(left.pending_internal_proposals,
                                right.pending_internal_proposals) &&
         left.applied_decision_source == right.applied_decision_source &&
         left.applied_decision_cycle_index == right.applied_decision_cycle_index &&
         left.applied_decision_batch_id == right.applied_decision_batch_id &&
         TacticalProposalsEqual(left.applied_decision_proposals,
                                right.applied_decision_proposals) &&
         pending_external_equal &&
         left.reducer_state.lpi_hold_cycles_remaining ==
             right.reducer_state.lpi_hold_cycles_remaining &&
         left.reducer_state.eccm_hold_cycles_remaining ==
             right.reducer_state.eccm_hold_cycles_remaining &&
         left.reducer_state.lpi_cooldown_cycles_remaining ==
             right.reducer_state.lpi_cooldown_cycles_remaining &&
         left.reducer_state.eccm_cooldown_cycles_remaining ==
             right.reducer_state.eccm_cooldown_cycles_remaining;
}

bool ExecutePendingCycle(ArReplayState* state, std::string* error) {
  if (!state->session) {
    *error = "AR replay cannot execute before session_config";
    return false;
  }
  if (!state->has_pending_input) {
    *error = "AR replay cycle_output arrived before cycle_input";
    return false;
  }

  ArCycleResult result = state->session->StepWithResult(state->pending_input);
  state->latest_result = result;
  state->latest_decision_state =
      ArSessionReplayAccess::CaptureDecisionState(*state->session);
  state->has_pending_input = false;
  return true;
}

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "ArSessionConfig") {
    *error = "AR replay expected ArSessionConfig session_config";
    return false;
  }

  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  config::ArSessionConfig config;
  if (!DecodeSessionConfigFlatbuffer(event.payload_bytes, &config, error)) {
    return false;
  }
  state->session.reset(new ArSession(ArSession::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  if (event.payload_type != "ArCycleInput") {
    *error = "AR replay expected ArCycleInput cycle_input";
    return false;
  }

  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received cycle_input before session_config";
    return false;
  }

  // P2-B: 连续两个 cycle_input 说明 trace 质量有问题——前一个 input 从未执行。
  if (state->has_pending_input) {
    *error = "AR replay received consecutive cycle_input without intervening cycle_output";
    return false;
  }

  ArCycleInput input;
  if (!DecodeCycleInputFlatbuffer(event.payload_bytes, &input, error)) {
    return false;
  }

  state->pending_input = input;
  state->has_pending_input = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  if (event.payload_type != "ArRuntimeConfigPatch") {
    *error = "AR replay expected ArRuntimeConfigPatch runtime_config_patch";
    return false;
  }

  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received runtime_config_patch before session_config";
    return false;
  }

  config::ArRuntimeConfigPatch patch;
  if (!DecodeRuntimeConfigPatchFlatbuffer(event.payload_bytes, &patch, error)) {
    return false;
  }
  state->session->ApplyRuntimeConfig(patch);
  return true;
}

bool OnDecisionInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "ExternalDecisionResponse") {
    *error = "AR replay expected ExternalDecisionResponse decision_input";
    return false;
  }

  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received decision_input before session_config";
    return false;
  }
  if (state->has_pending_input) {
    *error = "AR replay received decision_input after cycle_input but before cycle_output";
    return false;
  }

  session::ExternalDecisionResponse response;
  if (!DecodeExternalDecisionResponseFlatbuffer(event.payload_bytes, &response, error)) {
    return false;
  }
  if (state->session->SubmitExternalDecision(response) !=
      session::ExternalDecisionSubmitStatus::kAccepted) {
    *error = "AR replay could not apply recorded external decision at its causal boundary";
    return false;
  }
  return true;
}

oneq::replay::ReplayTraceOutputStatus OnCycleOutput(const oneq::replay::ReplayTraceReadEvent& event,
                                                     void* user_data, std::string* actual_output,
                                                     std::string* error) {
  using oneq::replay::ReplayTraceOutputStatus;
  if (event.payload_type != "ArReplayCycleRecord") {
    *error = "AR replay does not support cycle_output payload type: " + event.payload_type;
    return ReplayTraceOutputStatus::kOtherFailure;
  }
  ArReplayCycleRecord expected_record;
  if (!DecodeReplayCycleRecordFlatbuffer(event.payload_bytes, &expected_record, error)) {
    return ReplayTraceOutputStatus::kOtherFailure;
  }

  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  if (!ExecutePendingCycle(state, error)) {
    return ReplayTraceOutputStatus::kOtherFailure;
  }

  const bool match = CycleResultEqual(expected_record.result, state->latest_result) &&
                     DecisionReplayStateEqual(expected_record.decision_state,
                                              state->latest_decision_state);
  if (!match) {
    *error = "AR replay output divergence (ArReplayCycleRecord)";
    actual_output->clear();
    return ReplayTraceOutputStatus::kDivergence;
  }
  actual_output->clear();
  return ReplayTraceOutputStatus::kHandledByModule;
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  ArReplayState* state = static_cast<ArReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload = event.payload_bytes;
  oneq::replay::ReplayTraceFailure decoded;
  if (!DecodeFailureMarkerFlatbuffer(event.payload_bytes, &decoded, error)) {
    return false;
  }
  state->failure_marker_data = decoded;
  return true;
}

}  // namespace

ArReplaySessionResult ReplayArTrace(const std::string& trace_dir) {
  ArReplaySessionResult result;

  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "airborne_radar";
  expectation.require_module_match = true;

  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.ok = false;
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
  // Validation failures are recoverable session boundaries. Keep the marker in
  // the report, but continue so a valid recovery cycle is replayed as well.
  options.stop_on_failure_marker = false;

  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
  if (result.ok && state.has_pending_input) {
    result.ok = false;
    result.playback.ok = false;
    result.first_error = "AR replay ended with pending cycle_input without cycle_output";
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
}  // namespace airborne_radar
