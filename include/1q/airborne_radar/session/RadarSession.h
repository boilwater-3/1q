/**
 * @file RadarSession.h
 * @brief Deprecated compat wrapper — include ArSession.h instead.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_SESSION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_SESSION_H_

#include "1q/airborne_radar/session/ArSession.h"

namespace airborne_radar {
namespace session {
using RadarSession = ArSession;
}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_SESSION_H_
