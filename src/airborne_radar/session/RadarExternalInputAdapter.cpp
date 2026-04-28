#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"

#include <algorithm>
#include <cmath>

#include "common/geometry/CoordinateConversion.h"
#include "common/geometry/GeometryTransform.h"

namespace airborne_radar {
namespace session {

namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsFiniteVector3f(const oneq::foundation::Vector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(
    const model::EulerAnglesDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

oneq::internal::geometry::EulerAnglesDeg ComposeGeometryEuler(
    const model::EulerAnglesDeg& platform_attitude_deg,
    const model::EulerAnglesDeg& mount_angles_deg) {
  const Eigen::Matrix3f platform_rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(platform_attitude_deg));
  const Eigen::Matrix3f mount_rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(mount_angles_deg));
  const Eigen::Matrix3f composed = platform_rotation * mount_rotation;

  const float r20 = composed(2, 0);
  const float clamped = std::max(-1.0f, std::min(1.0f, -r20));
  const float pitch_rad_internal = std::asin(clamped);
  const float cos_pitch = std::cos(pitch_rad_internal);

  float yaw_rad = 0.0f;
  float roll_rad = 0.0f;
  if (std::abs(cos_pitch) > 1.0e-6f) {
    yaw_rad = std::atan2(composed(1, 0), composed(0, 0));
    roll_rad = std::atan2(composed(2, 1), composed(2, 2));
  } else {
    yaw_rad = std::atan2(-composed(0, 1), composed(1, 1));
    roll_rad = 0.0f;
  }

  constexpr float kRadToDeg = 180.0f / 3.14159265358979f;
  oneq::internal::geometry::EulerAnglesDeg out;
  out.yaw_deg = yaw_rad * kRadToDeg;
  out.pitch_deg = -pitch_rad_internal * kRadToDeg;
  out.roll_deg = roll_rad * kRadToDeg;
  return out;
}

}  // namespace

oneq::foundation::EulerAnglesDeg ComposeRadarAttitudeDeg(
    const model::EulerAnglesDeg& platform_attitude_deg,
    const model::EulerAnglesDeg& radar_mount_angles_deg) {
  const oneq::internal::geometry::EulerAnglesDeg composed =
      ComposeGeometryEuler(platform_attitude_deg, radar_mount_angles_deg);

  oneq::foundation::EulerAnglesDeg output;
  output.yaw_deg = composed.yaw_deg;
  output.pitch_deg = composed.pitch_deg;
  output.roll_deg = composed.roll_deg;
  return output;
}

bool TryMakeRadarPoseFromExternalKinematics(
    const RadarExternalPoseInput& input,
    RadarLocalFrameReference* reference,
    oneq::foundation::PoseState* platform_pose) {
  if (reference == nullptr || platform_pose == nullptr) {
    return false;
  }

  oneq::foundation::LlaCoordinateDegM radar_lla;
  if (!oneq::foundation::TryEcefToLla(input.platform_position_ecef_m, &radar_lla)) {
    return false;
  }
  reference->origin_lla = radar_lla;
  reference->radar_attitude_deg =
      ComposeRadarAttitudeDeg(input.platform_attitude_deg, input.radar_mount_angles_deg);

  platform_pose->position_m = oneq::foundation::Vector3f{};

  if (!IsFiniteVector3f(input.platform_velocity_mps)) {
    platform_pose->velocity_mps = oneq::foundation::Vector3f{};
  } else {
    oneq::foundation::EnuCoordinateM velocity_enu;
    if (!oneq::internal::geometry::TryConvertEcefVelocityToEnu(
            input.platform_velocity_mps, reference->origin_lla, &velocity_enu)) {
      return false;
    }
    platform_pose->velocity_mps =
        oneq::internal::geometry::ConvertEnuToLocal(velocity_enu, reference->radar_attitude_deg);
  }

  platform_pose->attitude_deg = input.platform_attitude_deg;
  return true;
}

bool TryMakeTargetFromExternalKinematics(
    std::uint64_t external_target_id,
    const TargetExternalKinematics& target_input,
    const RadarLocalFrameReference& reference,
    oneq::foundation::Vector3f radar_local_velocity_mps,
    RadarSceneTarget* target) {
  if (target == nullptr || !IsFiniteVector3f(target_input.target_velocity_mps)) {
    return false;
  }

  oneq::internal::geometry::LocalFrameReference shared_ref;
  shared_ref.origin_lla = reference.origin_lla;
  shared_ref.frame_attitude_deg = reference.radar_attitude_deg;

  oneq::foundation::Vector3f target_position_local;
  if (!oneq::internal::geometry::TryConvertEcefPositionToLocal(
          target_input.target_position_ecef_m, shared_ref, &target_position_local)) {
    return false;
  }

  // 速度固定为 ECEF，转换为雷达局部坐标系后扣除平台速度得到相对速度
  oneq::foundation::EnuCoordinateM velocity_enu;
  if (!oneq::internal::geometry::TryConvertEcefVelocityToEnu(
          target_input.target_velocity_mps, reference.origin_lla, &velocity_enu)) {
    return false;
  }
  oneq::foundation::Vector3f target_velocity_local =
      oneq::internal::geometry::ConvertEnuToLocal(velocity_enu, reference.radar_attitude_deg);
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
