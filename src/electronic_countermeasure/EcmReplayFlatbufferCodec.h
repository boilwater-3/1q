/**
 * @file EcmReplayFlatbufferCodec.h
 * @brief 声明 ECM replay FlatBuffers payload 编解码接口。
 */

#ifndef ELECTRONIC_COUNTERMEASURE_ECM_REPLAY_FLATBUFFER_CODEC_H_
#define ELECTRONIC_COUNTERMEASURE_ECM_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/electronic_countermeasure/EcmTypes.h"

namespace electronic_countermeasure {
namespace session {

std::string EncodeEcmSessionConfig(const config::EcmSessionConfig& value);
bool DecodeEcmSessionConfig(const std::string& bytes, config::EcmSessionConfig* output);
std::string EncodeEcmRuntimeConfigPatch(const config::EcmRuntimeConfigPatch& value);
bool DecodeEcmRuntimeConfigPatch(const std::string& bytes,
                                 config::EcmRuntimeConfigPatch* output);
std::string EncodeEcmCycleInput(const EcmCycleInput& value);
bool DecodeEcmCycleInput(const std::string& bytes, EcmCycleInput* output);
std::string EncodeEcmCycleResult(const EcmCycleResult& value);
bool DecodeEcmCycleResult(const std::string& bytes, EcmCycleResult* output);

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ELECTRONIC_COUNTERMEASURE_ECM_REPLAY_FLATBUFFER_CODEC_H_
