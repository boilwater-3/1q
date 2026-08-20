/**
 * @file RirReplayFlatbufferCodec.h
 * @brief 提供远程识别雷达会话回放（replay）相关结构的 FlatBuffers 编解码。
 *
 * 阶段 1 范围：单周期结果记录 + 会话状态（active_database_version）编解码；
 * 逐周期 replay 比较语义（浮点容差 1e-5f）由 replay 测试与调用方承担。
 * 会话配置 replay（rir_session_replay.fbs）列为阶段 2 评估项。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_FLATBUFFER_CODEC_H_
#define REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_FLATBUFFER_CODEC_H_

#include <string>

#include "remote_identification_radar/session/RirReplayCycleRecord.h"

namespace remote_identification_radar {
namespace session {

/** @brief 编解码 RIR 单周期结果与状态记录。 */
std::string EncodeCycleReplayRecordFlatbuffer(const RirCycleReplayRecord& record);
bool DecodeCycleReplayRecordFlatbuffer(const std::string& payload_bytes,
                                       RirCycleReplayRecord* record, std::string* error);

}  // namespace session
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_REPLAY_FLATBUFFER_CODEC_H_
