#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"

#include <algorithm>
#include <cmath>

#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "common/geometry/GeometryTransform.h"

namespace airborne_radar {
namespace session {

namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsFiniteVector3f(const oneq::foundation::Vector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(
    const oneq::foundation::EulerAnglesDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

oneq::foundation::Vector3f ConvertEnuToRadarLocal(
    const oneq::foundation::EnuCoordinateM& enu_position,
    const oneq::foundation::EulerAnglesDeg& attitude_deg) {
  oneq::internal::geometry::Vector3f enu_vector;
  enu_vector.x = static_cast<float>(enu_position.x_m);
  enu_vector.y = static_cast<float>(enu_position.y_m);
  enu_vector.z = static_cast<float>(enu_position.z_m);
  const oneq::internal::geometry::Vector3f local_vector =
      oneq::internal::geometry::RotateVectorToLocalFrame(enu_vector, ToGeometryEuler(attitude_deg));

  oneq::foundation::Vector3f local_position;
  local_position.x = local_vector.x;
  local_position.y = local_vector.y;
  local_position.z = local_vector.z;
  return local_position;
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

bool TryConvertEcefVelocityToEnu(const oneq::foundation::Vector3f& velocity_ecef_mps,
                                 const oneq::foundation::LlaCoordinateDegM& origin_lla,
                                 oneq::foundation::EnuCoordinateM* velocity_enu_mps) {
  if (velocity_enu_mps == nullptr || !oneq::foundation::IsValidLla(origin_lla) ||
      !IsFinite(velocity_ecef_mps.x) || !IsFinite(velocity_ecef_mps.y) ||
      !IsFinite(velocity_ecef_mps.z)) {
    return false;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double lat_rad = origin_lla.latitude_deg * kPi / 180.0;
  const double lon_rad = origin_lla.longitude_deg * kPi / 180.0;
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  const double vx = static_cast<double>(velocity_ecef_mps.x);
  const double vy = static_cast<double>(velocity_ecef_mps.y);
  const double vz = static_cast<double>(velocity_ecef_mps.z);

  velocity_enu_mps->x_m = -sin_lon * vx + cos_lon * vy;
  velocity_enu_mps->y_m = -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz;
  velocity_enu_mps->z_m = cos_lat * cos_lon * vx + cos_lat * sin_lon * vy + sin_lat * vz;
  return std::isfinite(velocity_enu_mps->x_m) != 0 && std::isfinite(velocity_enu_mps->y_m) != 0 &&
         std::isfinite(velocity_enu_mps->z_m) != 0;
}

oneq::foundation::EnuCoordinateM ToEnuFromNed(const oneq::foundation::Vector3f& ned_mps) {
  oneq::foundation::EnuCoordinateM enu;
  enu.x_m = static_cast<double>(ned_mps.y);
  enu.y_m = static_cast<double>(ned_mps.x);
  enu.z_m = static_cast<double>(-ned_mps.z);
  return enu;
}

oneq::foundation::EnuCoordinateM ToEnuFromEnuVector(const oneq::foundation::Vector3f& enu_mps) {
  oneq::foundation::EnuCoordinateM enu;
  enu.x_m = static_cast<double>(enu_mps.x);
  enu.y_m = static_cast<double>(enu_mps.y);
  enu.z_m = static_cast<double>(enu_mps.z);
  return enu;
}


bool TryConvertEcefToRadarLocalInternal(const oneq::foundation::EcefCoordinateM& position_ecef_m,
                                        const RadarLocalFrameReference& reference,
                                        oneq::foundation::Vector3f* position_local_m) {
  if (position_local_m == nullptr) {
    return false;
  }

  oneq::foundation::EnuCoordinateM enu_position;
  if (!oneq::foundation::TryEcefToEnu(position_ecef_m, reference.origin_lla, &enu_position)) {
    return false;
  }
  *position_local_m = ConvertEnuToRadarLocal(enu_position, reference.radar_attitude_deg);
  return true;
}

bool TryConvertEcefVelocityToRadarLocalInternal(const oneq::foundation::Vector3f& velocity_ecef_mps,
                                                const RadarLocalFrameReference& reference,
                                                oneq::foundation::Vector3f* velocity_local_mps) {
  if (velocity_local_mps == nullptr) {
    return false;
  }

  oneq::foundation::EnuCoordinateM velocity_enu_mps;
  if (!TryConvertEcefVelocityToEnu(velocity_ecef_mps, reference.origin_lla, &velocity_enu_mps)) {
    return false;
  }
  *velocity_local_mps = ConvertEnuToRadarLocal(velocity_enu_mps, reference.radar_attitude_deg);
  return true;
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

  if (!input.has_platform_velocity_ecef_mps || !IsFiniteVector3f(input.platform_velocity_mps)) {
    platform_pose->velocity_mps = oneq::foundation::Vector3f{};
  } else {
    switch (input.platform_velocity_frame) {
      case VelocityFrame::kRadarLocal:
        platform_pose->velocity_mps = input.platform_velocity_mps;
        break;
      case VelocityFrame::kEcef: {
        oneq::foundation::EnuCoordinateM velocity_enu;
        if (!TryConvertEcefVelocityToEnu(input.platform_velocity_mps, reference->origin_lla,
                                         &velocity_enu)) {
          return false;
        }
        platform_pose->velocity_mps =
            ConvertEnuToRadarLocal(velocity_enu, reference->radar_attitude_deg);
        break;
      }
      case VelocityFrame::kEnu: {
        oneq::foundation::EnuCoordinateM velocity_enu =
            ToEnuFromEnuVector(input.platform_velocity_mps);
        platform_pose->velocity_mps =
            ConvertEnuToRadarLocal(velocity_enu, reference->radar_attitude_deg);
        break;
      }
      case VelocityFrame::kNed: {
        oneq::foundation::EnuCoordinateM velocity_enu =
            ToEnuFromNed(input.platform_velocity_mps);
        platform_pose->velocity_mps =
            ConvertEnuToRadarLocal(velocity_enu, reference->radar_attitude_deg);
        break;
      }
    }
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

  oneq::foundation::Vector3f target_position_local;
  if (!TryConvertEcefToRadarLocalInternal(target_input.target_position_ecef_m, reference,
                                           &target_position_local)) {
    return false;
  }

  oneq::foundation::Vector3f target_velocity_local;
  switch (target_input.target_velocity_frame) {
    case VelocityFrame::kRadarLocal:
      target_velocity_local = target_input.target_velocity_mps;
      break;
    case VelocityFrame::kEcef: {
      if (!TryConvertEcefVelocityToRadarLocalInternal(
              target_input.target_velocity_mps, reference, &target_velocity_local)) {
        return false;
      }
      target_velocity_local.x -= radar_local_velocity_mps.x;
      target_velocity_local.y -= radar_local_velocity_mps.y;
      target_velocity_local.z -= radar_local_velocity_mps.z;
      break;
    }
    case VelocityFrame::kEnu: {
      oneq::foundation::EnuCoordinateM velocity_enu =
          ToEnuFromEnuVector(target_input.target_velocity_mps);
      target_velocity_local =
          ConvertEnuToRadarLocal(velocity_enu, reference.radar_attitude_deg);
      target_velocity_local.x -= radar_local_velocity_mps.x;
      target_velocity_local.y -= radar_local_velocity_mps.y;
      target_velocity_local.z -= radar_local_velocity_mps.z;
      break;
    }
    case VelocityFrame::kNed: {
      oneq::foundation::EnuCoordinateM velocity_enu =
          ToEnuFromNed(target_input.target_velocity_mps);
      target_velocity_local =
          ConvertEnuToRadarLocal(velocity_enu, reference.radar_attitude_deg);
      target_velocity_local.x -= radar_local_velocity_mps.x;
      target_velocity_local.y -= radar_local_velocity_mps.y;
      target_velocity_local.z -= radar_local_velocity_mps.z;
      break;
    }
  }

  const model::TargetFeature converted = model::MakeTargetFromCartesian(
      external_target_id, target_position_local.x, target_position_local.y,
      target_position_local.z, target_velocity_local.x, target_velocity_local.y,
      target_velocity_local.z, target_input.rcs, target_input.swerling_type);
  target->external_target_id = converted.external_target_id;
  target->current_track_velocity_x = converted.current_track_velocity_x;
  target->current_track_velocity_y = converted.current_track_velocity_y;
  target->current_track_velocity_z = converted.current_track_velocity_z;
  target->current_track_speed = converted.current_track_speed;
  target->current_track_rcs = converted.current_track_rcs;
  target->range_m = converted.range_m;
  target->has_cartesian_position = converted.has_cartesian_position;
  target->position_x = converted.position_x;
  target->position_y = converted.position_y;
  target->position_z = converted.position_z;
  target->target_swerling_type = converted.target_swerling_type;
  return true;
}

}  // namespace session
}  // namespace airborne_radar
