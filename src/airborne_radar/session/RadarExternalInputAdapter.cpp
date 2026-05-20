#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"

#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace airborne_radar {
namespace session {

namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsFiniteVector3f(const oneq::foundation::Vector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

oneq::foundation::EulerAnglesDeg ToFoundationEuler(
    const oneq::coordinate::EulerAnglesDeg& attitude_deg) {
  oneq::foundation::EulerAnglesDeg output;
  output.yaw_deg = static_cast<float>(attitude_deg.yaw_deg);
  output.pitch_deg = static_cast<float>(attitude_deg.pitch_deg);
  output.roll_deg = static_cast<float>(attitude_deg.roll_deg);
  return output;
}

oneq::foundation::Vector3f ToFoundationVector(const oneq::coordinate::Vector3d& v) {
  oneq::foundation::Vector3f out;
  out.x = static_cast<float>(v.x);
  out.y = static_cast<float>(v.y);
  out.z = static_cast<float>(v.z);
  return out;
}

oneq::foundation::Vector3f RotateEnuPositionToLocal(
    const oneq::coordinate::EnuPositionM& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return ToFoundationVector(
      oneq::coordinate::RotateEnuToLocal(enu.east_m, enu.north_m, enu.up_m, local_attitude_deg));
}

oneq::foundation::Vector3f RotateEnuVelocityToLocal(
    const oneq::coordinate::EnuVelocityMps& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return ToFoundationVector(oneq::coordinate::RotateEnuToLocal(
      enu.east_mps, enu.north_mps, enu.up_mps, local_attitude_deg));
}

}  // namespace

oneq::coordinate::EulerAnglesDeg ComposeRadarAttitudeDeg(
    const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
    const oneq::coordinate::EulerAnglesDeg& radar_mount_angles_deg) {
  return oneq::coordinate::ComposeAttitudeDeg(platform_attitude_deg, radar_mount_angles_deg);
}

bool TryMakeRadarPoseFromExternalKinematics(
    const RadarExternalPoseInput& input,
    RadarLocalFrameReference* reference,
    oneq::foundation::PoseState* platform_pose,
    RadarCoordinateStatus* status) {
  if (status != nullptr) {
    *status = RadarCoordinateStatus::kOk;
  }

  if (reference == nullptr || platform_pose == nullptr) {
    if (status != nullptr) {
      *status = RadarCoordinateStatus::kNullOutput;
    }
    return false;
  }

  oneq::coordinate::LlaPositionDegM radar_lla;
  if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &radar_lla)) {
    if (status != nullptr) {
      *status = RadarCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }
  reference->origin_lla = radar_lla;
  reference->radar_attitude_deg =
      ComposeRadarAttitudeDeg(input.platform_attitude_deg, input.radar_mount_angles_deg);

  platform_pose->position_m = oneq::foundation::Vector3f{};

  if (!oneq::coordinate::IsFinite(input.platform_velocity_mps)) {
    platform_pose->velocity_mps = oneq::foundation::Vector3f{};
  } else {
    oneq::coordinate::EnuVelocityMps velocity_enu;
    if (!oneq::coordinate::TryEcefToEnuVelocity(
            input.platform_velocity_mps, reference->origin_lla, &velocity_enu)) {
      if (status != nullptr) {
        *status = RadarCoordinateStatus::kCoordinateTransformFail;
      }
      return false;
    }
    platform_pose->velocity_mps = RotateEnuVelocityToLocal(velocity_enu,
                                                           reference->radar_attitude_deg);
  }

  platform_pose->attitude_deg = ToFoundationEuler(input.platform_attitude_deg);
  return true;
}

bool TryMakeTargetFromExternalKinematics(
    std::uint64_t external_target_id,
    const TargetExternalKinematics& target_input,
    const RadarLocalFrameReference& reference,
    oneq::foundation::Vector3f radar_local_velocity_mps,
    RadarSceneTarget* target,
    RadarCoordinateStatus* status) {
  if (status != nullptr) {
    *status = RadarCoordinateStatus::kOk;
  }

  if (target == nullptr) {
    if (status != nullptr) {
      *status = RadarCoordinateStatus::kNullOutput;
    }
    return false;
  }

  if (!oneq::coordinate::IsFinite(target_input.target_velocity_mps) ||
      !IsFiniteVector3f(radar_local_velocity_mps)) {
    if (status != nullptr) {
      *status = RadarCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }

  oneq::coordinate::EnuPositionM target_position_enu;
  if (!oneq::coordinate::TryEcefToEnu(
          target_input.target_position_ecef_m, reference.origin_lla, &target_position_enu)) {
    if (status != nullptr) {
      *status = RadarCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }
  oneq::foundation::Vector3f target_position_local =
      RotateEnuPositionToLocal(target_position_enu, reference.radar_attitude_deg);

  // 速度固定为 ECEF，转换为雷达局部坐标系后扣除平台速度得到相对速度
  oneq::coordinate::EnuVelocityMps velocity_enu;
  if (!oneq::coordinate::TryEcefToEnuVelocity(
          target_input.target_velocity_mps, reference.origin_lla, &velocity_enu)) {
    if (status != nullptr) {
      *status = RadarCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }
  oneq::foundation::Vector3f target_velocity_local =
      RotateEnuVelocityToLocal(velocity_enu, reference.radar_attitude_deg);
  target_velocity_local.x -= radar_local_velocity_mps.x;
  target_velocity_local.y -= radar_local_velocity_mps.y;
  target_velocity_local.z -= radar_local_velocity_mps.z;

  const float range =
      std::sqrt(target_position_local.x * target_position_local.x +
                target_position_local.y * target_position_local.y +
                target_position_local.z * target_position_local.z);

  target->external_target_id = external_target_id;
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
