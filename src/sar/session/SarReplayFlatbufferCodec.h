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

std::string EncodeSarCycleInput(const SarCycleInput& value);
std::string EncodeSarOutputFrame(const SarOutputFrame& value);
std::string EncodeSarCycleResult(const SarCycleResult& value);
std::string EncodeSarSessionConfig(const config::SarSessionConfig& value);
std::string EncodeSarRuntimeConfigPatch(const config::SarRuntimeConfigPatch& value);

bool DecodeSarCycleInput(const std::string& bytes, SarCycleInput* out);
bool DecodeSarOutputFrame(const std::string& bytes, SarOutputFrame* out);
bool DecodeSarCycleResult(const std::string& bytes, SarCycleResult* out);
bool DecodeSarSessionConfig(const std::string& bytes, config::SarSessionConfig* out);
bool DecodeSarRuntimeConfigPatch(const std::string& bytes, config::SarRuntimeConfigPatch* out);

}  // namespace session
}  // namespace sar

#endif  // SAR_SESSION_SAR_REPLAY_FLATBUFFER_CODEC_H_
