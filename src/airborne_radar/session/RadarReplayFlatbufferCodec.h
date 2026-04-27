#ifndef AIRBORNE_RADAR_SESSION_RADAR_REPLAY_FLATBUFFER_CODEC_H_
#define AIRBORNE_RADAR_SESSION_RADAR_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/replay/ReplayTrace.h"

namespace airborne_radar {
namespace session {

std::string EncodeCycleInputFlatbuffer(const RadarCycleInput& input);
bool DecodeCycleInputFlatbuffer(const std::string& payload_bytes, RadarCycleInput* input,
                                std::string* error);
std::string EncodeTrackOutputFrameFlatbuffer(const session::TrackOutputFrame& output_frame);
bool DecodeTrackOutputFrameFlatbuffer(const std::string& payload_bytes,
                                      session::TrackOutputFrame* output_frame, std::string* error);
std::string EncodeCycleResultFlatbuffer(const RadarCycleResult& result);
bool DecodeCycleResultFlatbuffer(const std::string& payload_bytes, RadarCycleResult* result,
                                 std::string* error);
std::string EncodeSessionConfigFlatbuffer(const RadarSessionConfig& config);
bool DecodeSessionConfigFlatbuffer(const std::string& payload_bytes, RadarSessionConfig* config,
                                   std::string* error);
std::string EncodeRuntimeConfigPatchFlatbuffer(const config::RadarRuntimeConfigPatch& patch);
bool DecodeRuntimeConfigPatchFlatbuffer(const std::string& payload_bytes,
                                        config::RadarRuntimeConfigPatch* patch, std::string* error);
std::string EncodeSceneStateFlatbuffer(const environment::EnvironmentSceneState& scene_state);
bool DecodeSceneStateFlatbuffer(const std::string& payload_bytes,
                                environment::EnvironmentSceneState* scene_state,
                                std::string* error);
std::string EncodeFailureMarkerFlatbuffer(const oneq::replay::ReplayTraceFailure& failure,
                                          bool has_last_event_sequence,
                                          std::uint64_t last_event_sequence);
bool DecodeFailureMarkerFlatbuffer(const std::string& payload_bytes,
                                   oneq::replay::ReplayTraceFailure* failure, std::string* error);

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RADAR_REPLAY_FLATBUFFER_CODEC_H_
