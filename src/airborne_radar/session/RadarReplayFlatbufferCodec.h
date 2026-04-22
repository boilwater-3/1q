#ifndef AIRBORNE_RADAR_SESSION_RADAR_REPLAY_FLATBUFFER_CODEC_H_
#define AIRBORNE_RADAR_SESSION_RADAR_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/airborne_radar/session/RadarCycleInput.h"

namespace airborne_radar {
namespace session {

std::string EncodeCycleInputFlatbuffer(const RadarCycleInput& input);
bool DecodeCycleInputFlatbuffer(const std::string& payload_bytes,
                                RadarCycleInput* input,
                                std::string* error);

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RADAR_REPLAY_FLATBUFFER_CODEC_H_
