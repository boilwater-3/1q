#include "1q/airborne_radar/session/RadarTraceSession.h"

#include <string>
#include <utility>

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

void WriteSessionConfigReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                              const RadarSessionConfig& config) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "session_config";
  event.payload_type = "RadarSessionConfig";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeSessionConfigFlatbuffer(config);
  writer->WriteEvent(event);
}

void WriteSceneStateReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                           const environment::EnvironmentSceneState& scene_state) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "scene_state";
  event.payload_type = "EnvironmentSceneState";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeSceneStateFlatbuffer(scene_state);
  writer->WriteEvent(event);
}

void WriteRuntimeConfigPatchReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                                   const config::RadarRuntimeConfigPatch& patch) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "runtime_config_patch";
  event.payload_type = "RadarRuntimeConfigPatch";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeRuntimeConfigPatchFlatbuffer(patch);
  writer->WriteEvent(event);
}

void WriteTrackOutputReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                            const session::TrackOutputFrame& output_frame) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "cycle_output";
  event.payload_type = "TrackOutputFrame";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeTrackOutputFrameFlatbuffer(output_frame);
  event.has_cycle_index = true;
  event.cycle_index = output_frame.cycle_index;
  writer->WriteEvent(event);
}

void WriteCycleResultReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                            const RadarCycleResult& result) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "cycle_output";
  event.payload_type = "RadarCycleResult";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeCycleResultFlatbuffer(result);
  event.has_cycle_index = true;
  event.cycle_index = result.track_output_frame.cycle_index;
  writer->WriteEvent(event);
}

// P1-A: 若 result 携带 validation error，自动落盘 failure_marker。
void MaybeWriteValidationFailureMarker(
    const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
    const RadarCycleResult& result) {
  if (!writer || !result.has_validation_error) {
    return;
  }
  oneq::replay::ReplayTraceFailure failure;
  failure.error_code = "AR_VALIDATION_ERROR";
  failure.message = "RadarCycleResult has_validation_error set";
  failure.location = "RadarTraceSession::StepWithResult";
  failure.has_cycle_index = true;
  failure.cycle_index = result.track_output_frame.cycle_index;
  const std::string failure_bytes = EncodeFailureMarkerFlatbuffer(failure, false, 0U);
  writer->WriteFailureMarker(failure, failure_bytes);
}

// P2-B: 写 cycle_input 事件并更新 pending 标志（直接由成员函数操作成员）。
void WriteCycleInputEvent(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                          const RadarCycleInput& input) {
  oneq::replay::ReplayTraceEvent ev;
  ev.module = "airborne_radar";
  ev.event_type = "cycle_input";
  ev.payload_type = "RadarCycleInput";
  ev.payload_encoding = "flatbuffers";
  ev.payload_bytes = EncodeCycleInputFlatbuffer(input);
  writer->WriteEvent(ev);
}

}  // namespace

RadarTraceSession::RadarTraceSession(const RadarSessionConfig& config,
                                     RadarTraceSessionOptions options)
    : session_(RadarSessionFactory::Create(config)),
      replay_writer_(std::move(options.replay_writer)) {
  if (replay_writer_ && options.trace_config_on_construct) {
    WriteSessionConfigReplay(replay_writer_, config);
  }
}

session::TrackOutputFrame RadarTraceSession::Step(const RadarCycleInput& input) {
  if (replay_writer_) {
    pending_input_written_ = true;
    WriteCycleInputEvent(replay_writer_, input);
  }
  const session::TrackOutputFrame output = session_.Step(input);
  if (replay_writer_) {
    WriteTrackOutputReplay(replay_writer_, output);
    pending_input_written_ = false;
  }
  return output;
}

session::TrackOutputFrame RadarTraceSession::Step(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (replay_writer_) {
    pending_input_written_ = true;
    WriteCycleInputEvent(replay_writer_, input);
    WriteSceneStateReplay(replay_writer_, scene_state);
  }
  const session::TrackOutputFrame output = session_.Step(input, scene_state);
  if (replay_writer_) {
    WriteTrackOutputReplay(replay_writer_, output);
    pending_input_written_ = false;
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(const RadarCycleInput& input) {
  if (replay_writer_) {
    pending_input_written_ = true;
    WriteCycleInputEvent(replay_writer_, input);
  }
  const RadarCycleResult output = session_.StepWithResult(input);
  if (replay_writer_) {
    WriteCycleResultReplay(replay_writer_, output);
    pending_input_written_ = false;
    MaybeWriteValidationFailureMarker(replay_writer_, output);
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (replay_writer_) {
    pending_input_written_ = true;
    WriteCycleInputEvent(replay_writer_, input);
    WriteSceneStateReplay(replay_writer_, scene_state);
  }
  const RadarCycleResult output = session_.StepWithResult(input, scene_state);
  if (replay_writer_) {
    WriteCycleResultReplay(replay_writer_, output);
    pending_input_written_ = false;
    MaybeWriteValidationFailureMarker(replay_writer_, output);
  }
  return output;
}

void RadarTraceSession::ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch) {
  if (replay_writer_) {
    WriteRuntimeConfigPatchReplay(replay_writer_, patch);
  }
  session_.ApplyRuntimeConfig(patch);
}

const std::vector<extension::control::RadarCommand>& RadarTraceSession::GetSubmittedCommands()
    const {
  return session_.GetSubmittedCommands();
}

bool RadarTraceSession::HasLatestControlProfile() const {
  return session_.HasLatestControlProfile();
}

const extension::control::RadarControlProfile& RadarTraceSession::GetLatestControlProfile() const {
  return session_.GetLatestControlProfile();
}

extension::AssociationQualityMetrics RadarTraceSession::GetLastAssociationQualityMetrics() const {
  return session_.GetLastAssociationQualityMetrics();
}

RadarSession& RadarTraceSession::session() { return session_; }

const RadarSession& RadarTraceSession::session() const { return session_; }

}  // namespace session
}  // namespace airborne_radar
