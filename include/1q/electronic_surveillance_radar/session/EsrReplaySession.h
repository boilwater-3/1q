/**
 * @file EsrReplaySession.h
 * @brief 基于 replay trace 事件提供 ESR 回放入口。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_SESSION_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrReplaySessionResult 描述一次 ESR trace 回放结果。
 */
struct ONEQ_API EsrReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};     /**< 回放就绪性与兼容性报告 */
  oneq::replay::ReplayTracePlaybackResult playback{}; /**< 回放执行明细（含状态与事件计数） */
  bool ok{false};                                     /**< 回放整体是否成功 */
  bool reached_failure_marker{false};                 /**< 是否到达 trace 中的失败标记 */
  std::string failure_marker_payload{};               /**< 失败标记原始 payload（空则未解码） */
  oneq::replay::ReplayTraceFailure failure_marker_data{}; /**< 已解码的失败标记结构 */
  std::string first_error{};                          /**< 首条错误描述，成功时为空 */
};

/**
 * @brief 回放指定目录下的 ESR trace 并比对输出。
 *
 * 读取 trace 事件重建会话与逐周期输入，重算输出并与记录输出比对，
 * 遇到输出分歧或失败标记时停止（stop_on_first_divergence / stop_on_failure_marker）。
 *
 * @param[in] trace_dir replay trace 目录路径。
 * @return 回放结果（含报告、回放明细、失败标记与首条错误）。
 */
ONEQ_API EsrReplaySessionResult ReplayEsrTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_SESSION_H_
