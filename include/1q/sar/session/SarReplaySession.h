/**
 * @file SarReplaySession.h
 * @brief 定义 SAR replay 会话占位门面。
 */

#ifndef ONEQ_SAR_SESSION_SAR_REPLAY_SESSION_H_
#define ONEQ_SAR_SESSION_SAR_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"
namespace sar {
namespace session {

/**
 * @brief SAR replay 会话执行结果。
 */
struct ONEQ_API SarReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};     /**< replay 报告 */
  oneq::replay::ReplayTracePlaybackResult playback{};  /**< 回放执行结果 */
  bool ok{false};                                      /**< 回放是否成功完成 */
  bool reached_failure_marker{false};                  /**< 是否命中失败标记并提前停止 */
  std::string failure_marker_payload{};                /**< 命中的失败标记载荷 */
  oneq::replay::ReplayTraceFailure failure_marker_data{}; /**< 失败标记解析数据 */
  std::string first_error{};                           /**< 首条错误信息 */
};

/**
 * @brief 回放指定 SAR trace 目录。
 *
 * 逐事件回放 trace，命中失败标记时提前停止并把标记载荷写入结果。
 * @param[in] trace_dir SAR replay trace 目录路径。
 * @return 回放结果（含报告、回放状态与失败标记信息）。
 */
ONEQ_API SarReplaySessionResult ReplaySarTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_REPLAY_SESSION_H_
