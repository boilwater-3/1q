#include "1q/airborne_radar/session/RadarReplaySession.h"

#include <memory>
#include <string>

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

struct RadarReplayState {
  std::unique_ptr<RadarSession> session{};
  RadarCycleInput pending_input{};
  bool has_pending_input{false};
  environment::EnvironmentSceneState pending_scene_state{};
  bool has_pending_scene_state{false};
  RadarCycleResult latest_result{};
  session::TrackOutputFrame latest_frame{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool TrackStateSnapshotEqual(const model::TrackStateSnapshot& left,
                             const model::TrackStateSnapshot& right) {
  return left.association_key == right.association_key &&
         left.external_target_id == right.external_target_id &&
         left.status == right.status &&
         left.position_x == right.position_x &&
         left.position_y == right.position_y &&
         left.position_z == right.position_z &&
         left.velocity_x == right.velocity_x &&
         left.velocity_y == right.velocity_y &&
         left.velocity_z == right.velocity_z &&
         left.speed == right.speed &&
         left.rcs == right.rcs &&
         left.jamming_detected == right.jamming_detected &&
         left.hit_count == right.hit_count &&
         left.miss_count == right.miss_count;
}

bool TrackOutputFrameEqual(const session::TrackOutputFrame& left,
                           const session::TrackOutputFrame& right) {
  if (left.cycle_index != right.cycle_index ||
      left.tracks.size() != right.tracks.size() ||
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

bool CycleResultEqual(const RadarCycleResult& left, const RadarCycleResult& right) {
  return TrackOutputFrameEqual(left.track_output_frame, right.track_output_frame) &&
         left.validation_issues.size() == right.validation_issues.size() &&
         left.executed_this_cycle == right.executed_this_cycle;
}

bool ExecutePendingCycle(RadarReplayState* state, std::string* error) {
  if (!state->session) {
    *error = "AR replay cannot execute before session_config";
    return false;
  }
  if (!state->has_pending_input) {
    *error = "AR replay cycle_output arrived before cycle_input";
    return false;
  }

  RadarCycleResult result;
  if (state->has_pending_scene_state) {
    result = state->session->StepWithResult(state->pending_input, state->pending_scene_state);
  } else {
    result = state->session->StepWithResult(state->pending_input);
  }
  state->latest_result = result;
  state->latest_frame = result.track_output_frame;
  state->has_pending_input = false;
  state->has_pending_scene_state = false;
  return true;
}

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "RadarSessionConfig") {
    *error = "AR replay expected RadarSessionConfig session_config";
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  RadarSessionConfig config;
  if (!DecodeSessionConfigFlatbuffer(event.payload_bytes, &config, error)) {
    return false;
  }
  state->session.reset(new RadarSession(RadarSessionFactory::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  if (event.payload_type != "RadarCycleInput") {
    *error = "AR replay expected RadarCycleInput cycle_input";
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received cycle_input before session_config";
    return false;
  }

  // P2-B: 连续两个 cycle_input 说明 trace 质量有问题——前一个 input 从未执行。
  if (state->has_pending_input) {
    *error = "AR replay received consecutive cycle_input without intervening cycle_output";
    return false;
  }

  RadarCycleInput input;
  if (!DecodeCycleInputFlatbuffer(event.payload_bytes, &input, error)) {
    return false;
  }

  state->pending_input = input;
  state->has_pending_input = true;
  state->has_pending_scene_state = false;
  return true;
}

bool OnSceneState(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  if (event.payload_type != "EnvironmentSceneState") {
    *error = "AR replay expected EnvironmentSceneState scene_state";
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  environment::EnvironmentSceneState scene_state;
  if (!DecodeSceneStateFlatbuffer(event.payload_bytes, &scene_state, error)) {
    return false;
  }
  state->pending_scene_state = scene_state;
  state->has_pending_scene_state = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  if (event.payload_type != "RadarRuntimeConfigPatch") {
    *error = "AR replay expected RadarRuntimeConfigPatch runtime_config_patch";
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  if (!state->session) {
    *error = "AR replay received runtime_config_patch before session_config";
    return false;
  }

  config::RadarRuntimeConfigPatch patch;
  if (!DecodeRuntimeConfigPatchFlatbuffer(event.payload_bytes, &patch, error)) {
    return false;
  }
  state->session->ApplyRuntimeConfig(patch);
  return true;
}

bool OnCycleOutput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                   std::string* actual_output, std::string* error) {
  RadarCycleResult expected_result;
  session::TrackOutputFrame expected_frame;
  bool has_expected_result = false;
  bool has_expected_frame = false;

  if (event.payload_type == "RadarCycleResult") {
    if (!DecodeCycleResultFlatbuffer(event.payload_bytes, &expected_result, error)) {
      return false;
    }
    has_expected_result = true;
  } else if (event.payload_type == "TrackOutputFrame") {
    if (!DecodeTrackOutputFrameFlatbuffer(event.payload_bytes, &expected_frame, error)) {
      return false;
    }
    has_expected_frame = true;
  } else {
    *error = "AR replay does not support cycle_output payload type: " + event.payload_type;
    return false;
  }

  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  if (!ExecutePendingCycle(state, error)) {
    return false;
  }

  if (event.payload_type == "RadarCycleResult") {
    const bool match = has_expected_result && CycleResultEqual(expected_result, state->latest_result);
    if (!match) {
      *error = "AR replay output divergence (RadarCycleResult)";
      return false;
    }
    actual_output->clear();
    return true;
  }
  if (event.payload_type == "TrackOutputFrame") {
    const bool match = has_expected_frame && TrackOutputFrameEqual(expected_frame, state->latest_frame);
    if (!match) {
      *error = "AR replay output divergence (TrackOutputFrame)";
      return false;
    }
    actual_output->clear();
    return true;
  }
  *error = "AR replay does not support cycle_output payload type: " + event.payload_type;
  return false;
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
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

RadarReplaySessionResult ReplayRadarTrace(const std::string& trace_dir) {
  RadarReplaySessionResult result;

  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "airborne_radar";
  expectation.require_module_match = true;

  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.ok = false;
    result.first_error = result.report.first_error;
    return result;
  }

  RadarReplayState state;
  oneq::replay::ReplayTracePlaybackCallbacks callbacks;
  callbacks.user_data = &state;
  callbacks.on_session_config = OnSessionConfig;
  callbacks.on_cycle_input = OnCycleInput;
  callbacks.on_scene_state = OnSceneState;
  callbacks.on_runtime_config_patch = OnRuntimeConfigPatch;
  callbacks.on_cycle_output = OnCycleOutput;
  callbacks.on_failure_marker = OnFailureMarker;

  oneq::replay::ReplayTracePlaybackOptions options;
  options.require_output_callback = true;
  options.stop_on_first_divergence = true;
  options.stop_on_failure_marker = true;

  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok;
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
