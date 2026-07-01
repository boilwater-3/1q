#ifndef AIRBORNE_RADAR_SESSION_AR_REPLAY_FLATBUFFER_CODEC_H_
#define AIRBORNE_RADAR_SESSION_AR_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/replay/ReplayTrace.h"

namespace airborne_radar {
namespace session {

std::string EncodeCycleInputFlatbuffer(const ArCycleInput& input);
bool DecodeCycleInputFlatbuffer(const std::string& payload_bytes, ArCycleInput* input,
                                std::string* error);
std::string EncodeTrackOutputFrameFlatbuffer(const session::TrackOutputFrame& output_frame);
bool DecodeTrackOutputFrameFlatbuffer(const std::string& payload_bytes,
                                      session::TrackOutputFrame* output_frame, std::string* error);
std::string EncodeCycleResultFlatbuffer(const ArCycleResult& result);
bool DecodeCycleResultFlatbuffer(const std::string& payload_bytes, ArCycleResult* result,
                                 std::string* error);
std::string EncodeSessionConfigFlatbuffer(const config::ArSessionConfig& config);
bool DecodeSessionConfigFlatbuffer(const std::string& payload_bytes, config::ArSessionConfig* config,
                                   std::string* error);
std::string EncodeRuntimeConfigPatchFlatbuffer(const config::ArRuntimeConfigPatch& patch);
bool DecodeRuntimeConfigPatchFlatbuffer(const std::string& payload_bytes,
                                        config::ArRuntimeConfigPatch* patch, std::string* error);
std::string EncodeFailureMarkerFlatbuffer(const oneq::replay::ReplayTraceFailure& failure,
                                          bool has_last_event_sequence,
                                          std::uint64_t last_event_sequence);
bool DecodeFailureMarkerFlatbuffer(const std::string& payload_bytes,
                                   oneq::replay::ReplayTraceFailure* failure, std::string* error);

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_AR_REPLAY_FLATBUFFER_CODEC_H_
