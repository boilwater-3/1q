/**
 * @file SarReplaySession.h
 * @brief 定义 SAR replay 会话占位门面。
 */

#ifndef ONEQ_SAR_SESSION_SAR_REPLAY_SESSION_H_
#define ONEQ_SAR_SESSION_SAR_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"
#include "1q/sar/session/SarSession.h"

namespace sar {
namespace session {

struct ONEQ_API SarReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};
  oneq::replay::ReplayTracePlaybackResult playback{};
  bool ok{false};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
  std::string first_error{};
};

/**
 * @brief SAR replay 会话。
 */
class ONEQ_API SarReplaySession {
 public:
  SarReplaySession();
  explicit SarReplaySession(SarSession session);

  SarCycleResult StepWithResult(const SarCycleInput& input);

 private:
  SarSession session_;
};

ONEQ_API SarReplaySessionResult ReplaySarTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_REPLAY_SESSION_H_
