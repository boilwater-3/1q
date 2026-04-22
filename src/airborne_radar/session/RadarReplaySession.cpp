#include "1q/airborne_radar/session/RadarReplaySession.h"

#include <memory>
#include <sstream>
#include <string>

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

std::string JsonBool(bool value) { return value ? "true" : "false"; }

std::string MakeOutputPayload(const output::TrackOutputFrame& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"type\":\"TrackOutputFrame\","
         << "\"cycle_index\":" << output.cycle_index << ","
         << "\"published_track_count\":" << output.published_track_count << "}";
  return stream.str();
}

std::string MakeResultPayload(const RadarCycleResult& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"type\":\"RadarCycleResult\","
         << "\"validation_issue_count\":" << output.validation_issues.size() << ","
         << "\"executed_this_cycle\":" << JsonBool(output.executed_this_cycle) << "}";
  return stream.str();
}

struct RadarReplayState {
  std::unique_ptr<RadarSession> session{};
  RadarCycleInput pending_input{};
  bool has_pending_input{false};
  environment::EnvironmentSceneState pending_scene_state{};
  bool has_pending_scene_state{false};
  RadarCycleResult latest_result{};
  output::TrackOutputFrame latest_frame{};
  std::string latest_result_payload{};
  std::string latest_frame_payload{};
  bool reached_failure_marker{false};
  std::string failure_marker_payload_json{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
};

bool TrackOutputSummaryEqual(const output::TrackOutputFrame& left,
                             const output::TrackOutputFrame& right) {
  return left.cycle_index == right.cycle_index &&
         left.published_track_count == right.published_track_count;
}

bool CycleResultSummaryEqual(const RadarCycleResult& left, const RadarCycleResult& right) {
  return TrackOutputSummaryEqual(left.track_output_frame, right.track_output_frame) &&
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
  state->latest_result_payload = MakeResultPayload(result);
  state->latest_frame_payload = MakeOutputPayload(result.track_output_frame);
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
                   std::string* actual_output_json, std::string* error) {
  RadarCycleResult expected_result;
  output::TrackOutputFrame expected_frame;
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
    const bool match =
        has_expected_result && CycleResultSummaryEqual(expected_result, state->latest_result);
    *actual_output_json = match ? event.payload_json : state->latest_result_payload;
    return true;
  }
  if (event.payload_type == "TrackOutputFrame") {
    const bool match =
        has_expected_frame && TrackOutputSummaryEqual(expected_frame, state->latest_frame);
    *actual_output_json = match ? event.payload_json : state->latest_frame_payload;
    return true;
  }
  *error = "AR replay does not support cycle_output payload type: " + event.payload_type;
  return false;
}

bool OnFailureMarker(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  RadarReplayState* state = static_cast<RadarReplayState*>(user_data);
  state->reached_failure_marker = true;
  state->failure_marker_payload_json = event.payload_json;
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
  result.failure_marker_payload_json = state.failure_marker_payload_json;
  result.failure_marker_data = state.failure_marker_data;
  if (!result.ok) {
    result.first_error = result.playback.first_error;
  }
  return result;
}

}  // namespace session
}  // namespace airborne_radar
