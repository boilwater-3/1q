/**
 * @file EosReplaySession.h
 * @brief 基于 replay trace 事件的 EOS 回放入口。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_SESSION_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosReplaySessionResult 描述一次 EOS trace 回放的结果。
 */
struct ONEQ_API EosReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};      /**< trace 目录兼容性报告 */
  oneq::replay::ReplayTracePlaybackResult playback{};  /**< 逐事件回放执行结果 */
  bool ok{false};                                      /**< 回放是否全程成功且无输出分歧 */
  bool reached_failure_marker{false};                  /**< 是否命中预期失败标记 */
  std::string failure_marker_payload{};                /**< 命中的失败标记原始 payload */
  oneq::replay::ReplayTraceFailure failure_marker_data{}; /**< 解码后的失败标记结构 */
  std::string first_error{};                           /**< 首条错误信息，成功时为空 */
};

/**
 * @brief 从 trace 目录回放 EOS 会话并逐周期比对输出。
 *
 * 依据记录的事件重建会话与周期输入，重放运行期补丁，并将每周期实际输出与
 * 记录输出逐字段比对，遇到输出分歧或失败标记即停止。
 *
 * @param[in] trace_dir trace 目录路径。
 * @return 回放结果，包含兼容性报告、回放执行结果与首条错误信息。
 */
ONEQ_API EosReplaySessionResult ReplayEosTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_SESSION_H_
