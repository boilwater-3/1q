/**
 * @file SbirsReplaySession.h
 * @brief 提供 SBIRS-inspired replay trace 回放入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_SESSION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace sbirs_sensor {
namespace session {

struct ONEQ_API SbirsReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};
  oneq::replay::ReplayTracePlaybackResult playback{};
  bool ok{false};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
  std::string first_error{};
};

ONEQ_API SbirsReplaySessionResult ReplaySbirsTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_SESSION_H_
