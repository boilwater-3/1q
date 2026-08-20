/**
 * @file EcmReplaySession.h
 * @brief 定义 ECM replay trace 的确定性回放入口。
 */

#ifndef ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_REPLAY_SESSION_H_
#define ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace electronic_countermeasure {
namespace session {

/** @brief ECM trace 回放报告。 */
struct ONEQ_API EcmReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};
  oneq::replay::ReplayTracePlaybackResult playback{};
  bool ok{false};
  std::string first_error{};
};

/** @brief 重建 ECM 会话并逐周期比较实际发射、决策和累积资源状态。 */
ONEQ_API EcmReplaySessionResult ReplayEcmTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_REPLAY_SESSION_H_
