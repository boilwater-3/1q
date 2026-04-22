/**
 * @file RadarReplaySession.h
 * @brief Provides an AR replay entry point backed by replay trace events.
 */

#ifndef AIRBORNE_RADAR_SESSION_RADAR_REPLAY_SESSION_H_
#define AIRBORNE_RADAR_SESSION_RADAR_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace airborne_radar {
namespace session {

struct ONEQ_API RadarReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};
  oneq::replay::ReplayTracePlaybackResult playback{};
  bool ok{false};
  bool reached_failure_marker{false};
  std::string failure_marker_payload_json{};
  std::string first_error{};
};

ONEQ_API RadarReplaySessionResult ReplayRadarTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RADAR_REPLAY_SESSION_H_
