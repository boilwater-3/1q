#include "1q/airborne_radar/session/RadarTraceSession.h"

#include <sstream>
#include <string>
#include <utility>

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/trace/TraceSink.h"
#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

std::string BuildRadarInputPayload(const RadarCycleInput& input) {
  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << input.cycle_index << ","
     << "\"dt_sec\":" << input.dt_sec << ","
     << "\"platform_altitude_m\":" << input.platform_altitude_m << ","
     << "\"scene_target_count\":" << input.scene.size()
     << "}";
  return os.str();
}

std::string BuildRadarOutputPayload(const RadarCycleResult& result) {
  const auto& frame = result.track_output_frame;
  std::size_t confirmed_count = 0U;
  for (const auto& track : frame.tracks) {
    if (track.status == model::TrackStatus::kConfirmed) {
      ++confirmed_count;
    }
  }

  std::ostringstream os;
  os << "{"
     << "\"cycle_index\":" << frame.cycle_index << ","
     << "\"batch_id\":" << frame.batch_id << ","
     << "\"executed\":" << (result.executed_this_cycle ? "true" : "false") << ","
     << "\"track_count\":" << frame.tracks.size() << ","
     << "\"confirmed_track_count\":" << confirmed_count << ","
     << "\"assoc_priors\":" << result.association_quality_metrics.prior_track_count << ","
     << "\"assoc_detections\":" << result.association_quality_metrics.detection_count << ","
     << "\"assoc_matches\":" << result.association_quality_metrics.matched_count << ","
     << "\"assoc_new_tracks\":" << result.association_quality_metrics.new_track_count << ","
     << "\"assoc_missed\":" << result.association_quality_metrics.missed_track_count << ","
     << "\"assoc_match_rate\":" << result.association_quality_metrics.match_rate << ","
     << "\"validation_error\":" << (result.has_validation_error ? "true" : "false")
     << "}";
  return os.str();
}

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

void WriteCycleResultReplay(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                            const RadarCycleResult& result) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "cycle_output";
  event.payload_type = "RadarCycleResult";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = EncodeCycleResultFlatbuffer(result);
  event.has_cycle_index = true;
  event.cycle_index = result.input_cycle_index;
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
  failure.cycle_index = result.input_cycle_index;
  const std::string failure_bytes = EncodeFailureMarkerFlatbuffer(failure, false, 0U);
  writer->WriteFailureMarker(failure, failure_bytes);
}

void WriteCycleInputEvent(const std::shared_ptr<oneq::replay::ReplayTraceWriter>& writer,
                          const RadarCycleInput& input) {
  oneq::replay::ReplayTraceEvent ev;
  ev.module = "airborne_radar";
  ev.event_type = "cycle_input";
  ev.payload_type = "RadarCycleInput";
  ev.payload_encoding = "flatbuffers";
  ev.payload_bytes = EncodeCycleInputFlatbuffer(input);
  ev.has_cycle_index = true;
  ev.cycle_index = input.cycle_index;
  writer->WriteEvent(ev);
}

}  // namespace

RadarTraceSession::RadarTraceSession(const RadarSessionConfig& config,
                                     RadarTraceSessionOptions options)
    : session_(RadarSessionFactory::Create(config)),
      sink_(std::move(options.sink)),
      replay_writer_(std::move(options.replay_writer)) {
  if (sink_ && options.trace_config_on_construct) {
    sink_->Record("airborne_radar", "config", "{}");
  }
  if (replay_writer_ && options.trace_config_on_construct) {
    WriteSessionConfigReplay(replay_writer_, config);
  }
}

session::TrackOutputFrame RadarTraceSession::Step(const RadarCycleInput& input) {
  if (sink_) {
    sink_->Record("airborne_radar", "input", BuildRadarInputPayload(input));
  }
  if (replay_writer_) {
    WriteCycleInputEvent(replay_writer_, input);
  }
  const RadarCycleResult result = session_.StepWithResult(input);
  if (sink_) {
    sink_->Record("airborne_radar", "output", BuildRadarOutputPayload(result));
  }
  if (replay_writer_) {
    WriteCycleResultReplay(replay_writer_, result);
    MaybeWriteValidationFailureMarker(replay_writer_, result);
  }
  return result.track_output_frame;
}

RadarCycleResult RadarTraceSession::StepWithResult(const RadarCycleInput& input) {
  if (sink_) {
    sink_->Record("airborne_radar", "input", BuildRadarInputPayload(input));
  }
  if (replay_writer_) {
    WriteCycleInputEvent(replay_writer_, input);
  }
  const RadarCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    sink_->Record("airborne_radar", "output", BuildRadarOutputPayload(output));
  }
  if (replay_writer_) {
    WriteCycleResultReplay(replay_writer_, output);
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
