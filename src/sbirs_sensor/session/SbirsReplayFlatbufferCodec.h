/**
 * @file SbirsReplayFlatbufferCodec.h
 * @brief SBIRS-inspired replay payload 的 FlatBuffers encode/decode 接口。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_FLATBUFFER_CODEC_H_
#define ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/replay/ReplayTrace.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

std::string EncodeSbirsCycleInput(const SbirsCycleInput& value);
std::string EncodeSbirsOutputFrame(const SbirsOutputFrame& value);
std::string EncodeSbirsCycleResult(const SbirsCycleResult& value);
std::string EncodeSbirsSessionConfig(const config::SbirsSessionConfig& value);
std::string EncodeSbirsRuntimeConfigPatch(const config::SbirsRuntimeConfigPatch& value);
std::string EncodeSbirsFailureMarker(const oneq::replay::ReplayTraceFailure& failure);

bool DecodeSbirsCycleInput(const std::string& bytes, SbirsCycleInput* out);
bool DecodeSbirsOutputFrame(const std::string& bytes, SbirsOutputFrame* out);
bool DecodeSbirsCycleResult(const std::string& bytes, SbirsCycleResult* out);
bool DecodeSbirsSessionConfig(const std::string& bytes, config::SbirsSessionConfig* out);
bool DecodeSbirsRuntimeConfigPatch(const std::string& bytes, config::SbirsRuntimeConfigPatch* out);
bool DecodeSbirsFailureMarker(const std::string& bytes, oneq::replay::ReplayTraceFailure* failure,
                              std::string* error);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_FLATBUFFER_CODEC_H_
