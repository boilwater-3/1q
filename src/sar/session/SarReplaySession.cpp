#include "1q/sar/session/SarReplaySession.h"
#include "1q/sar/session/SarSession.h"

#include <memory>
#include <string>

#include "SarReplayFlatbufferCodec.h"

namespace sar {
namespace session {
namespace {

struct SarReplayState {
  std::unique_ptr<SarSession> session{};
  SarCycleInput pending_input{};
  bool has_pending_input{false};
  SarCycleResult latest_result{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool SarDiagnosticEqual(const SarDiagnosticIssue& left, const SarDiagnosticIssue& right) {
  return left.severity == right.severity && left.code == right.code &&
         left.message == right.message;
}

bool SarOutputFrameEqual(const SarOutputFrame& left, const SarOutputFrame& right) {
  return left.cycle_index == right.cycle_index && left.completed_stage == right.completed_stage &&
         left.range_sample_count == right.range_sample_count &&
         left.azimuth_pulse_count == right.azimuth_pulse_count &&
         left.center_slant_range_m == right.center_slant_range_m &&
         left.estimated_snr_db == right.estimated_snr_db &&
         left.phase_reference_mode == right.phase_reference_mode &&
         left.image_quality_mainlobe_method == right.image_quality_mainlobe_method &&
         left.range_width_3db_bins == right.range_width_3db_bins &&
         left.azimuth_width_3db_bins == right.azimuth_width_3db_bins &&
         left.range_resolution_3db_m == right.range_resolution_3db_m &&
         left.azimuth_resolution_3db_m == right.azimuth_resolution_3db_m &&
         left.image_entropy_nats == right.image_entropy_nats &&
         left.image_contrast == right.image_contrast &&
         left.has_raw_echo == right.has_raw_echo &&
         left.has_range_compressed_echo == right.has_range_compressed_echo &&
         left.has_l1_image == right.has_l1_image &&
         left.has_l3_bp_image == right.has_l3_bp_image &&
         left.has_image_quality_metrics == right.has_image_quality_metrics &&
         left.image_resolution_m_valid == right.image_resolution_m_valid &&
         left.phase_reference_applied == right.phase_reference_applied;
}

bool SarCycleResultEqual(const SarCycleResult& left, const SarCycleResult& right) {
  if (left.input_cycle_index != right.input_cycle_index ||
      !SarOutputFrameEqual(left.output_frame, right.output_frame) ||
      left.diagnostics.size() != right.diagnostics.size() || left.has_error != right.has_error ||
      left.executed_this_cycle != right.executed_this_cycle ||
      left.reused_previous_output != right.reused_previous_output ||
      left.abort_reason != right.abort_reason) {
    return false;
  }
  for (std::size_t i = 0U; i < left.diagnostics.size(); ++i) {
    if (!SarDiagnosticEqual(left.diagnostics[i], right.diagnostics[i])) {
      return false;
    }
  }
  return true;
}

bool ExecutePendingCycle(SarReplayState* state, std::string* error) {
  if (!state->session) {
    *error = "SAR replay cannot execute before session_config";
    return false;
  }
  if (!state->has_pending_input) {
    *error = "SAR replay cycle_output arrived before cycle_input";
    return false;
  }

  state->latest_result = state->session->StepWithResult(state->pending_input);
  state->has_pending_input = false;
  return true;
}

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "SarSessionConfig") {
    *error = "SAR replay expected SarSessionConfig session_config";
    return false;
  }

  config::SarSessionConfig config;
  if (!DecodeSarSessionConfig(event.payload_bytes, &config)) {
    *error = "SAR replay failed to decode session_config";
    return false;
  }
  SarReplayState* state = static_cast<SarReplayState*>(user_data);
  state->session.reset(new SarSession(SarSession::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  if (event.payload_type != "SarCycleInput") {
    *error = "SAR replay expected SarCycleInput cycle_input";
    return false;
  }

  SarReplayState* state = static_cast<SarReplayState*>(user_data);
  if (!state->session) {
    *error = "SAR replay received cycle_input before session_config";
    return false;
  }
  if (state->has_pending_input) {
    *error = "SAR replay received consecutive cycle_input without intervening cycle_output";
    return false;
  }

  SarCycleInput input;
  if (!DecodeSarCycleInput(event.payload_bytes, &input)) {
    *error = "SAR replay failed to decode cycle_input";
    return false;
  }
  state->pending_input = input;
  state->has_pending_input = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  if (event.payload_type != "SarRuntimeConfigPatch") {
    *error = "SAR replay expected SarRuntimeConfigPatch runtime_config_patch";
    return false;
  }

  SarReplayState* state = static_cast<SarReplayState*>(user_data);
  if (!state->session) {
    *error = "SAR replay received runtime_config_patch before session_config";
    return false;
  }

  config::SarRuntimeConfigPatch patch;
  if (!DecodeSarRuntimeConfigPatch(event.payload_bytes, &patch)) {
    *error = "SAR replay failed to decode runtime_config_patch";
    return false;
  }
  state->session->ApplyRuntimeConfig(patch);
  return true;
}

bool OnCycleOutput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                   std::string* actual_output, std::string* error) {
  if (event.payload_type != "SarCycleResult" && event.payload_type != "SarOutputFrame") {
    *error = "SAR replay does not support cycle_output payload type: " + event.payload_type;
    return false;
  }

  SarReplayState* state = static_cast<SarReplayState*>(user_data);
  if (!ExecutePendingCycle(state, error)) {
    return false;
  }

  if (event.payload_type == "SarCycleResult") {
    SarCycleResult expected;
    if (!DecodeSarCycleResult(event.payload_bytes, &expected)) {
      *error = "SAR replay failed to decode SarCycleResult";
      return false;
    }
    if (!SarCycleResultEqual(expected, state->latest_result)) {
      *error = "SAR replay output divergence (SarCycleResult)";
      return false;
    }
    actual_output->clear();
    return true;
  }

  SarOutputFrame expected_frame;
  if (!DecodeSarOutputFrame(event.payload_bytes, &expected_frame)) {
    *error = "SAR replay failed to decode SarOutputFrame";
    return false;
  }
  if (!SarOutputFrameEqual(expected_frame, state->latest_result.output_frame)) {
    *error = "SAR replay output divergence (SarOutputFrame)";
    return false;
  }
  actual_output->clear();
  return true;
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* /*error*/) {
  SarReplayState* state = static_cast<SarReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload = event.payload_bytes;
  return true;
}

}  // namespace

SarReplaySessionResult ReplaySarTrace(const std::string& trace_dir) {
  SarReplaySessionResult result;

  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "sar";
  expectation.require_module_match = true;

  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.ok = false;
    result.first_error = result.report.first_error;
    return result;
  }

  SarReplayState state;
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
    result.first_error = "SAR replay ended with pending cycle_input without cycle_output";
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
}  // namespace sar
