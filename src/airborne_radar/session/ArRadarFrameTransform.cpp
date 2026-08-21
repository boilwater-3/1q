#include "1q/airborne_radar/session/ArRadarFrameTransform.h"

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
using oneq::common::validation::IsFinite;

bool IsFiniteVector3f(const oneq::foundation::Vector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

}  // namespace

oneq::coordinate::EulerAnglesDeg ComposeRadarAttitudeDeg(
    const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
    const oneq::coordinate::EulerAnglesDeg& mount_angles_deg) {
  return oneq::coordinate::ComposeAttitudeDeg(platform_attitude_deg, mount_angles_deg);
}

bool TryMakeArPoseFromPlatform(const ArPlatformInput& input,
                               const oneq::coordinate::EulerAnglesDeg& mount_angles_deg,
                               oneq::coordinate::LocalFrameReference* reference,
                               oneq::foundation::Vector3f* radar_local_velocity_mps,
                               ArCoordinateStatus* status) {
  if (status != nullptr) {
    *status = ArCoordinateStatus::kOk;
  }

  if (reference == nullptr || radar_local_velocity_mps == nullptr) {
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
      ComposeRadarAttitudeDeg(input.platform_attitude_deg, mount_angles_deg);

  if (!oneq::coordinate::IsFinite(input.platform_velocity_mps)) {
    *radar_local_velocity_mps = oneq::foundation::Vector3f{};
  } else {
    oneq::coordinate::EnuVelocityMps velocity_enu;
    if (!oneq::coordinate::TryEcefToEnuVelocity(input.platform_velocity_mps, reference->origin_lla,
                                                &velocity_enu)) {
      if (status != nullptr) {
        *status = ArCoordinateStatus::kCoordinateTransformFail;
      }
      return false;
    }
    *radar_local_velocity_mps =
        RotateEnuVelocityToLocal(velocity_enu, reference->frame_attitude_deg);
  }

  return true;
}

bool TryMakeArTargetFromEnu(const ArTargetInput& target_input,
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

  if (!IsFinite(target_input.position_x) || !IsFinite(target_input.position_y) ||
      !IsFinite(target_input.position_z) || !IsFinite(target_input.velocity_x) ||
      !IsFinite(target_input.velocity_y) || !IsFinite(target_input.velocity_z) ||
      !IsFiniteVector3f(radar_local_velocity_mps)) {
    if (status != nullptr) {
      *status = ArCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }

  const oneq::coordinate::EnuPositionM target_position_enu{
      static_cast<double>(target_input.position_x), static_cast<double>(target_input.position_y),
      static_cast<double>(target_input.position_z)};
  const oneq::foundation::Vector3f target_position_local =
      RotateEnuPositionToLocal(target_position_enu, reference.frame_attitude_deg);

  const oneq::coordinate::EnuVelocityMps velocity_enu{
      static_cast<double>(target_input.velocity_x), static_cast<double>(target_input.velocity_y),
      static_cast<double>(target_input.velocity_z)};
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
