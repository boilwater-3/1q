/**
 * @file ArReplaySession.h
 * @brief AR module primary replay entry point.
 *
 * Primary header for replay session.
 * Include this for new code; RadarReplaySession.h is the deprecated compat wrapper.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_REPLAY_SESSION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace airborne_radar {
namespace session {

struct ONEQ_API ArReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};
  oneq::replay::ReplayTracePlaybackResult playback{};
  bool ok{false};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
  std::string first_error{};
};

ONEQ_API ArReplaySessionResult ReplayArTrace(const std::string& trace_dir);

// 兼容别名：旧名称在 wrapper 阶段保留。
using RadarReplaySessionResult = ArReplaySessionResult;

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_REPLAY_SESSION_H_
