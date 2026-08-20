/**
 * @file SarReplayFlatbufferCodec.h
 * @brief SAR replay payload 的 FlatBuffers encode/decode 接口。
 */

#ifndef SAR_SESSION_SAR_REPLAY_FLATBUFFER_CODEC_H_
#define SAR_SESSION_SAR_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

/**
 * @brief 将单周期输入载荷序列化为 FlatBuffers 字节串。
 * @param[in] value 待序列化的单周期输入。
 * @return FlatBuffers 编码后的字节串。
 */
std::string EncodeSarCycleInput(const SarCycleInput& value);
/**
 * @brief 将单周期结果序列化为 FlatBuffers 字节串。
 * @param[in] value 待序列化的单周期结果。
 * @return FlatBuffers 编码后的字节串。
 */
std::string EncodeSarCycleResult(const SarCycleResult& value);
/**
 * @brief 将会话配置序列化为 FlatBuffers 字节串。
 * @param[in] value 待序列化的会话配置。
 * @return FlatBuffers 编码后的字节串。
 */
std::string EncodeSarSessionConfig(const config::SarSessionConfig& value);
/**
 * @brief 将运行期配置补丁序列化为 FlatBuffers 字节串。
 * @param[in] value 待序列化的运行期配置补丁。
 * @return FlatBuffers 编码后的字节串。
 */
std::string EncodeSarRuntimeConfigPatch(const config::SarRuntimeConfigPatch& value);

/**
 * @brief 从 FlatBuffers 字节串反序列化单周期输入载荷。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] out 反序列化结果（失败时不修改）。
 * @return 校验通过且成功解码返回 true；非法或校验失败返回 false。
 */
bool DecodeSarCycleInput(const std::string& bytes, SarCycleInput* out);
/**
 * @brief 从 FlatBuffers 字节串反序列化单周期结果。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] out 反序列化结果（失败时不修改）。
 * @return 校验通过且成功解码返回 true；非法或校验失败返回 false。
 */
bool DecodeSarCycleResult(const std::string& bytes, SarCycleResult* out);
/**
 * @brief 从 FlatBuffers 字节串反序列化会话配置。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] out 反序列化结果（失败时不修改）。
 * @return 校验通过且成功解码返回 true；非法或校验失败返回 false。
 */
bool DecodeSarSessionConfig(const std::string& bytes, config::SarSessionConfig* out);
/**
 * @brief 从 FlatBuffers 字节串反序列化运行期配置补丁。
 * @param[in] bytes FlatBuffers 编码字节串。
 * @param[out] out 反序列化结果（失败时不修改）。
 * @return 校验通过且成功解码返回 true；非法或校验失败返回 false。
 */
bool DecodeSarRuntimeConfigPatch(const std::string& bytes, config::SarRuntimeConfigPatch* out);

}  // namespace session
}  // namespace sar

#endif  // SAR_SESSION_SAR_REPLAY_FLATBUFFER_CODEC_H_
