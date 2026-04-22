/**
 * @file EosReplayFlatbufferCodec.h
 * @brief EOS replay payload 的 FlatBuffers encode/decode 接口。
 *
 * 与 AR 模块的 RadarReplayFlatbufferCodec 对应，统一使用 payload_bytes +
 * payload_encoding="flatbuffers"，不依赖 nlohmann::json 或平台特定代码。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_FLATBUFFER_CODEC_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/output/EosOutputFrame.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"

namespace electro_optical_sensor {
namespace session {

// ---- Encode (C++ -> bytes) ----

std::string EncodeEosCycleInput(const EosCycleInput& value);
std::string EncodeEosOutputFrame(const output::EosOutputFrame& value);
std::string EncodeEosCycleResult(const ::electro_optical_sensor::session::EosCycleResult& value);
std::string EncodeEosSessionConfig(const EosSessionConfig& value);
std::string EncodeEosRuntimeConfigPatch(const EosRuntimeConfigPatch& value);

// ---- Decode (bytes -> C++) ----

bool DecodeEosCycleInput(const std::string& bytes, EosCycleInput* out);
bool DecodeEosOutputFrame(const std::string& bytes, output::EosOutputFrame* out);
bool DecodeEosCycleResult(const std::string& bytes, ::electro_optical_sensor::session::EosCycleResult* out);
bool DecodeEosSessionConfig(const std::string& bytes, EosSessionConfig* out);
bool DecodeEosRuntimeConfigPatch(const std::string& bytes, EosRuntimeConfigPatch* out);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_FLATBUFFER_CODEC_H_
