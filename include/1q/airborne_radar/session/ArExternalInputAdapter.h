/**
 * @file ArExternalInputAdapter.h
 * @brief AR module primary aliases for external input adaptation.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_EXTERNAL_INPUT_ADAPTER_H_

#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"

namespace airborne_radar {
namespace session {

using ArExternalPoseInput = RadarExternalPoseInput;
using ArExternalTargetInput = RadarExternalTargetInput;
using ArCoordinateStatus = RadarCoordinateStatus;

inline bool TryMakeArPoseFromExternalKinematics(const ArExternalPoseInput& input,
                                                oneq::coordinate::LocalFrameReference* reference,
                                                oneq::foundation::PoseState* platform_pose,
                                                ArCoordinateStatus* status = nullptr) {
  return TryMakeRadarPoseFromExternalKinematics(input, reference, platform_pose, status);
}

inline bool TryMakeArTargetFromExternalKinematics(
    const ArExternalTargetInput& target_input,
    const oneq::coordinate::LocalFrameReference& reference,
    oneq::foundation::Vector3f radar_local_velocity_mps, ArSceneTarget* target,
    ArCoordinateStatus* status = nullptr) {
  return TryMakeTargetFromExternalKinematics(target_input, reference, radar_local_velocity_mps,
                                             target, status);
}

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_EXTERNAL_INPUT_ADAPTER_H_
