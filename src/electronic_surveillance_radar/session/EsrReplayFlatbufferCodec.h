/**
 * @file EsrReplayFlatbufferCodec.h
 * @brief ESR replay payload 的 FlatBuffers encode/decode 接口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_FLATBUFFER_CODEC_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/output/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"

namespace electronic_surveillance_radar {
namespace session {

std::string EncodeEsrCycleInput(const EsrCycleInput& value);
std::string EncodeEsrOutputFrame(const output::EsrOutputFrame& value);
std::string EncodeEsrCycleResult(const EsrCycleResult& value);
std::string EncodeEsrSessionConfig(const EsrSessionConfig& value);
std::string EncodeEsrRuntimeConfigPatch(const EsrRuntimeConfigPatch& value);

bool DecodeEsrCycleInput(const std::string& bytes, EsrCycleInput* out);
bool DecodeEsrOutputFrame(const std::string& bytes, output::EsrOutputFrame* out);
bool DecodeEsrCycleResult(const std::string& bytes, EsrCycleResult* out);
bool DecodeEsrSessionConfig(const std::string& bytes, EsrSessionConfig* out);
bool DecodeEsrRuntimeConfigPatch(const std::string& bytes, EsrRuntimeConfigPatch* out);

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_FLATBUFFER_CODEC_H_
