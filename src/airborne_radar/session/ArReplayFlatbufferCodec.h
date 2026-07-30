/**
 * @file ArReplayFlatbufferCodec.h
 * @brief 提供 AR 会话回放（replay）相关结构的 FlatBuffers 序列化/反序列化编解码。
 *
 * 涵盖 RF v2 单周期输入/结果、会话配置、运行期补丁与失败标记等记录的编解码。
 * 所有 Decode* 函数在解析失败时返回 false 并通过输出参数回填错误描述。
 */

#ifndef AIRBORNE_RADAR_SESSION_AR_REPLAY_FLATBUFFER_CODEC_H_
#define AIRBORNE_RADAR_SESSION_AR_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/replay/ReplayTrace.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"

namespace airborne_radar {
namespace session {

/** @brief 编解码 AR 单周期用户输入与结果记录。 */
std::string EncodeCycleInputFlatbuffer(const ArCycleInput& input);
bool DecodeCycleInputFlatbuffer(const std::string& payload_bytes, ArCycleInput* input,
                                std::string* error);
std::string EncodeCycleReplayRecordFlatbuffer(const ArCycleReplayRecord& record);
bool DecodeCycleReplayRecordFlatbuffer(const std::string& payload_bytes,
                                       ArCycleReplayRecord* record, std::string* error);

/** @brief 编解码带接受状态的运行期补丁尝试。 */
std::string EncodeRuntimeConfigAttemptFlatbuffer(const config::ArRuntimeConfigPatch& patch,
                                                 bool accepted);
bool DecodeRuntimeConfigAttemptFlatbuffer(const std::string& payload_bytes,
                                          config::ArRuntimeConfigPatch* patch, bool* accepted,
                                          std::string* error);
/**
 * @brief 将四域会话配置编码为 FlatBuffers 字节串。
 * @param[in] config 待编码的会话配置。
 * @return 序列化后的二进制 payload（std::string）。
 */
std::string EncodeSessionConfigFlatbuffer(const config::ArSessionConfig& config);
/**
 * @brief 从 FlatBuffers 字节串解码四域会话配置。
 * @param[in] payload_bytes 序列化后的二进制 payload。
 * @param[out] config 解码成功的会话配置输出指针（非空）。
 * @param[out] error 解析失败时回填的错误描述。
 * @return 解析成功返回 true；失败返回 false 并填充 error。
 */
bool DecodeSessionConfigFlatbuffer(const std::string& payload_bytes, config::ArSessionConfig* config,
                                   std::string* error);
/**
 * @brief 将运行期配置补丁编码为 FlatBuffers 字节串。
 * @param[in] patch 待编码的运行期配置补丁。
 * @return 序列化后的二进制 payload（std::string）。
 */
std::string EncodeRuntimeConfigPatchFlatbuffer(const config::ArRuntimeConfigPatch& patch);
/**
 * @brief 从 FlatBuffers 字节串解码运行期配置补丁。
 * @param[in] payload_bytes 序列化后的二进制 payload。
 * @param[out] patch 解码成功的配置补丁输出指针（非空）。
 * @param[out] error 解析失败时回填的错误描述。
 * @return 解析成功返回 true；失败返回 false 并填充 error。
 */
bool DecodeRuntimeConfigPatchFlatbuffer(const std::string& payload_bytes,
                                        config::ArRuntimeConfigPatch* patch, std::string* error);
/**
 * @brief 将回放失败标记编码为 FlatBuffers 字节串。
 * @param[in] failure 待编码的失败信息。
 * @param[in] has_last_event_sequence 是否附带最后事件序号。
 * @param[in] last_event_sequence 失败前最后已记录的事件序号（仅在 has_last_event_sequence 为 true 时有效）。
 * @return 序列化后的二进制 payload（std::string）。
 */
std::string EncodeFailureMarkerFlatbuffer(const oneq::replay::ReplayTraceFailure& failure,
                                          bool has_last_event_sequence,
                                          std::uint64_t last_event_sequence);
/**
 * @brief 从 FlatBuffers 字节串解码回放失败标记。
 * @param[in] payload_bytes 序列化后的二进制 payload。
 * @param[out] failure 解码成功的失败信息输出指针（非空）。
 * @param[out] error 解析失败时回填的错误描述。
 * @return 解析成功返回 true；失败返回 false 并填充 error。
 */
bool DecodeFailureMarkerFlatbuffer(const std::string& payload_bytes,
                                   oneq::replay::ReplayTraceFailure* failure, std::string* error);

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_AR_REPLAY_FLATBUFFER_CODEC_H_
