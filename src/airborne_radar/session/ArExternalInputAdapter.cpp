#include "1q/airborne_radar/session/ArExternalInputAdapter.h"

#include <cmath>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "common/coordinate/CoordinateUtils.h"
#include "common/validation/ValidationUtils.h"

namespace airborne_radar {
namespace session {

namespace {

using oneq::common::coordinate_utils::RotateEnuPositionToLocal;
using oneq::common::coordinate_utils::RotateEnuVelocityToLocal;
using oneq::common::coordinate_utils::ToFoundationEuler;
using oneq::common::coordinate_utils::ToFoundationVector;
using oneq::internal::validation::IsFinite;

bool IsFiniteVector3f(const oneq::foundation::Vector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

}  // namespace

oneq::coordinate::EulerAnglesDeg ComposeRadarAttitudeDeg(
    const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
    const oneq::coordinate::EulerAnglesDeg& radar_mount_angles_deg) {
  return oneq::coordinate::ComposeAttitudeDeg(platform_attitude_deg, radar_mount_angles_deg);
}

bool TryMakeArPoseFromExternalKinematics(const ArExternalPoseInput& input,
                                         oneq::coordinate::LocalFrameReference* reference,
                                         oneq::foundation::PoseState* platform_pose,
                                         ArCoordinateStatus* status) {
  if (status != nullptr) {
    *status = ArCoordinateStatus::kOk;
  }

  if (reference == nullptr || platform_pose == nullptr) {
    if (status != nullptr) {
      *status = ArCoordinateStatus::kNullOutput;
    }
    return false;
  }

  oneq::coordinate::LlaPositionDegM radar_lla;
  if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &radar_lla)) {
    if (status != nullptr) {
      *status = ArCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }
  reference->origin_lla = radar_lla;
  reference->frame_attitude_deg =
      ComposeRadarAttitudeDeg(input.platform_attitude_deg, input.radar_mount_angles_deg);

  platform_pose->position_m = oneq::foundation::Vector3f{};

  if (!oneq::coordinate::IsFinite(input.platform_velocity_mps)) {
    platform_pose->velocity_mps = oneq::foundation::Vector3f{};
  } else {
    oneq::coordinate::EnuVelocityMps velocity_enu;
    if (!oneq::coordinate::TryEcefToEnuVelocity(input.platform_velocity_mps, reference->origin_lla,
                                                &velocity_enu)) {
      if (status != nullptr) {
        *status = ArCoordinateStatus::kCoordinateTransformFail;
      }
      return false;
    }
    platform_pose->velocity_mps =
        RotateEnuVelocityToLocal(velocity_enu, reference->frame_attitude_deg);
  }

  platform_pose->attitude_deg = ToFoundationEuler(input.platform_attitude_deg);
  return true;
}

bool TryMakeArTargetFromExternalKinematics(const ArExternalTargetInput& target_input,
                                           const oneq::coordinate::LocalFrameReference& reference,
                                           oneq::foundation::Vector3f radar_local_velocity_mps,
                                           ArSceneTarget* target, ArCoordinateStatus* status) {
  if (status != nullptr) {
    *status = ArCoordinateStatus::kOk;
  }

  if (target == nullptr) {
    if (status != nullptr) {
      *status = ArCoordinateStatus::kNullOutput;
    }
    return false;
  }

  if (!oneq::coordinate::IsFinite(target_input.kinematics.velocity_mps) ||
      !IsFiniteVector3f(radar_local_velocity_mps)) {
    if (status != nullptr) {
      *status = ArCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }

  oneq::coordinate::EnuPositionM target_position_enu;
  switch (target_input.kinematics.position_frame) {
    case oneq::coordinate::PositionFrame::kEcef:
      if (!oneq::coordinate::TryEcefToEnu(target_input.kinematics.position_ecef_m,
                                          reference.origin_lla, &target_position_enu)) {
        if (status != nullptr) {
          *status = ArCoordinateStatus::kCoordinateTransformFail;
        }
        return false;
      }
      break;
    case oneq::coordinate::PositionFrame::kLla:
      if (!oneq::coordinate::TryLlaToEnu(target_input.kinematics.position_lla_deg_m,
                                         reference.origin_lla, &target_position_enu)) {
        if (status != nullptr) {
          *status = ArCoordinateStatus::kCoordinateTransformFail;
        }
        return false;
      }
      break;
    default:
      if (status != nullptr) {
        *status = ArCoordinateStatus::kCoordinateTransformFail;
      }
      return false;
  }
  oneq::foundation::Vector3f target_position_local =
      RotateEnuPositionToLocal(target_position_enu, reference.frame_attitude_deg);

  // 速度固定为 ECEF，转换为雷达局部坐标系后扣除平台速度得到相对速度
  oneq::coordinate::EnuVelocityMps velocity_enu;
  if (!oneq::coordinate::TryEcefToEnuVelocity(target_input.kinematics.velocity_mps,
                                              reference.origin_lla, &velocity_enu)) {
    if (status != nullptr) {
      *status = ArCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }
  oneq::foundation::Vector3f target_velocity_local =
      RotateEnuVelocityToLocal(velocity_enu, reference.frame_attitude_deg);
  target_velocity_local.x -= radar_local_velocity_mps.x;
  target_velocity_local.y -= radar_local_velocity_mps.y;
  target_velocity_local.z -= radar_local_velocity_mps.z;

  const float range = std::sqrt(target_position_local.x * target_position_local.x +
                                target_position_local.y * target_position_local.y +
                                target_position_local.z * target_position_local.z);

  target->external_target_id = target_input.target_id;
  target->target_name = target_input.target_name;
  target->velocity_x = target_velocity_local.x;
  target->velocity_y = target_velocity_local.y;
  target->velocity_z = target_velocity_local.z;
  target->rcs = target_input.rcs;
  target->range_m = range;
  target->position_x = target_position_local.x;
  target->position_y = target_position_local.y;
  target->position_z = target_position_local.z;
  target->target_swerling_type = target_input.swerling_type;
  return true;
}

}  // namespace session
}  // namespace airborne_radar
