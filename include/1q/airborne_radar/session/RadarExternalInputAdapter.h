/**
 * @file RadarExternalInputAdapter.h
 * @brief Deprecated compat wrapper — include ArExternalInputAdapter.h instead.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_INPUT_ADAPTER_H_

#include "1q/airborne_radar/session/ArExternalInputAdapter.h"

namespace airborne_radar {
namespace session {

inline bool TryMakeRadarPoseFromExternalKinematics(const RadarExternalPoseInput& input,
                                                    oneq::coordinate::LocalFrameReference* reference,
                                                    oneq::foundation::PoseState* platform_pose,
                                                    RadarCoordinateStatus* status = nullptr) {
  return TryMakeArPoseFromExternalKinematics(input, reference, platform_pose, status);
}

inline bool TryMakeTargetFromExternalKinematics(
    const RadarExternalTargetInput& target_input,
    const oneq::coordinate::LocalFrameReference& reference,
    oneq::foundation::Vector3f radar_local_velocity_mps, RadarSceneTarget* target,
    RadarCoordinateStatus* status = nullptr) {
  return TryMakeArTargetFromExternalKinematics(target_input, reference, radar_local_velocity_mps,
                                               target, status);
}

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_INPUT_ADAPTER_H_
