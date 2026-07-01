/**
 * @file ArTrackLifecycleRecorder.h
 * @brief AR module primary aliases for track lifecycle recording.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_LIFECYCLE_RECORDER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_LIFECYCLE_RECORDER_H_

#include "1q/airborne_radar/session/RadarTrackLifecycleRecorder.h"

namespace airborne_radar {
namespace session {

using ArTrackLifecycleEventKind = RadarTrackLifecycleEventKind;
using ArTrackLifecycleReason = RadarTrackLifecycleReason;
using ArTrackLifecycleEvent = RadarTrackLifecycleEvent;
using ArTrackLifecycleRecorderConfig = RadarTrackLifecycleRecorderConfig;
using ArTrackLifecycleRecorder = RadarTrackLifecycleRecorder;

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_LIFECYCLE_RECORDER_H_
