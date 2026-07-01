/**
 * @file ArReplaySession.h
 * @brief AR module primary aliases for replay sessions.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_REPLAY_SESSION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_REPLAY_SESSION_H_

#include "1q/airborne_radar/session/RadarReplaySession.h"

namespace airborne_radar {
namespace session {

using ArReplaySessionResult = RadarReplaySessionResult;

inline ArReplaySessionResult ReplayArTrace(const std::string& trace_dir) {
  return ReplayRadarTrace(trace_dir);
}

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_REPLAY_SESSION_H_
