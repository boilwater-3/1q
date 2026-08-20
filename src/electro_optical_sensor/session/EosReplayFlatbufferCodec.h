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

/**
 * @brief 将 EosCycleInput 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的周期输入。
 * @return FlatBuffers 编码字节串。
 */
std::string EncodeEosCycleInput(const EosCycleInput& value);

/**
 * @brief 将 EosOutputFrame 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的输出帧。
 * @return FlatBuffers 编码字节串。
 */
std::string EncodeEosOutputFrame(const session::EosOutputFrame& value);

/**
 * @brief 将 EosCycleResult 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的周期聚合结果。
 * @return FlatBuffers 编码字节串。
 */
std::string EncodeEosCycleResult(const ::electro_optical_sensor::session::EosCycleResult& value);

/**
 * @brief 将 EosSessionConfig 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的会话配置。
 * @return FlatBuffers 编码字节串。
 */
std::string EncodeEosSessionConfig(const config::EosSessionConfig& value);

/**
 * @brief 将 EosRuntimeConfigPatch 序列化为 FlatBuffers payload。
 * @param[in] value 待序列化的运行期配置补丁。
 * @return FlatBuffers 编码字节串。
 */
std::string EncodeEosRuntimeConfigPatch(const config::EosRuntimeConfigPatch& value);

/**
 * @brief 将 ReplayTraceFailure 失败标记序列化为 FlatBuffers payload。
 * @param[in] failure 待序列化的失败标记。
 * @return FlatBuffers 编码字节串。
 */
std::string EncodeEosFailureMarker(const oneq::replay::ReplayTraceFailure& failure);

// ---- Decode (bytes -> C++) ----

/**
 * @brief 将 FlatBuffers payload 反序列化为 EosCycleInput。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] out 输出周期输入。
 * @return 校验通过并解析成功返回 true；buffer 校验失败返回 false。
 */
bool DecodeEosCycleInput(const std::string& bytes, EosCycleInput* out);

/**
 * @brief 将 FlatBuffers payload 反序列化为 EosOutputFrame。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] out 输出输出帧。
 * @return 校验通过并解析成功返回 true；buffer 校验失败返回 false。
 */
bool DecodeEosOutputFrame(const std::string& bytes, session::EosOutputFrame* out);

/**
 * @brief 将 FlatBuffers payload 反序列化为 EosCycleResult。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] out 输出周期聚合结果。
 * @return 校验通过并解析成功返回 true；buffer 校验失败返回 false。
 */
bool DecodeEosCycleResult(const std::string& bytes, ::electro_optical_sensor::session::EosCycleResult* out);

/**
 * @brief 将 FlatBuffers payload 反序列化为 EosSessionConfig。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] out 输出会话配置。
 * @return 校验通过并解析成功返回 true；buffer 校验失败返回 false。
 */
bool DecodeEosSessionConfig(const std::string& bytes, config::EosSessionConfig* out);

/**
 * @brief 将 FlatBuffers payload 反序列化为 EosRuntimeConfigPatch。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] out 输出运行期配置补丁。
 * @return 校验通过并解析成功返回 true；buffer 校验失败返回 false。
 */
bool DecodeEosRuntimeConfigPatch(const std::string& bytes, config::EosRuntimeConfigPatch* out);

/**
 * @brief 将 FlatBuffers payload 反序列化为失败标记。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] failure 输出失败标记；为 nullptr 时直接返回 false。
 * @param[out] error 解析失败时写入的简短错误信息。
 * @return 校验通过并解析成功返回 true；`failure` 为空、buffer 校验或解析失败返回 false。
 */
bool DecodeEosFailureMarker(const std::string& bytes, oneq::replay::ReplayTraceFailure* failure,
                            std::string* error);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_FLATBUFFER_CODEC_H_
