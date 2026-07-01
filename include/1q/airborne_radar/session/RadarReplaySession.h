/**
 * @file RadarReplaySession.h
 * @brief Deprecated compat wrapper — include ArReplaySession.h instead.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_REPLAY_SESSION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_REPLAY_SESSION_H_

#include "1q/airborne_radar/session/ArReplaySession.h"

namespace airborne_radar {
namespace session {

inline RadarReplaySessionResult ReplayRadarTrace(const std::string& trace_dir) {
  return ReplayArTrace(trace_dir);
}

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_REPLAY_SESSION_H_
