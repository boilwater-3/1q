/**
 * @file EosReplayFlatbufferCodec.h
 * @brief EOS replay payload 的 FlatBuffers encode/decode 接口。
 *
 * 与 AR 模块的 ArReplayFlatbufferCodec 对应，统一使用 payload_bytes +
 * payload_encoding="flatbuffers"，不依赖 nlohmann::json 或平台特定代码。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_FLATBUFFER_CODEC_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/replay/ReplayTrace.h"

namespace electro_optical_sensor {
namespace session {

// ---- Encode (C++ -> bytes) ----

std::string EncodeEosCycleInput(const EosCycleInput& value);
std::string EncodeEosOutputFrame(const session::EosOutputFrame& value);
std::string EncodeEosCycleResult(const ::electro_optical_sensor::session::EosCycleResult& value);
std::string EncodeEosSessionConfig(const config::EosSessionConfig& value);
std::string EncodeEosRuntimeConfigPatch(const config::EosRuntimeConfigPatch& value);
std::string EncodeEosFailureMarker(const oneq::replay::ReplayTraceFailure& failure);

// ---- Decode (bytes -> C++) ----

bool DecodeEosCycleInput(const std::string& bytes, EosCycleInput* out);
bool DecodeEosOutputFrame(const std::string& bytes, session::EosOutputFrame* out);
bool DecodeEosCycleResult(const std::string& bytes, ::electro_optical_sensor::session::EosCycleResult* out);
bool DecodeEosSessionConfig(const std::string& bytes, config::EosSessionConfig* out);
bool DecodeEosRuntimeConfigPatch(const std::string& bytes, config::EosRuntimeConfigPatch* out);
bool DecodeEosFailureMarker(const std::string& bytes, oneq::replay::ReplayTraceFailure* failure,
                            std::string* error);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_FLATBUFFER_CODEC_H_
