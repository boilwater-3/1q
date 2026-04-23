#include <cmath>

#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "common/geometry/GeometryTransform.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"

namespace electro_optical_sensor {
namespace session {

namespace {

constexpr float kNormFloor = 1.0e-6f;

void SetStatus(EosCoordinateStatus value, EosCoordinateStatus* status) {
  if (status != nullptr) {
    *status = value;
  }
}

oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(
    const oneq::foundation::EulerAnglesDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

oneq::foundation::Vector3f ConvertEnuToEosLocal(
    const oneq::foundation::EnuCoordinateM& enu,
    const oneq::foundation::EulerAnglesDeg& frame_attitude_deg) {
  oneq::internal::geometry::Vector3f enu_vector;
  enu_vector.x = static_cast<float>(enu.x_m);
  enu_vector.y = static_cast<float>(enu.y_m);
  enu_vector.z = static_cast<float>(enu.z_m);
  const oneq::internal::geometry::Vector3f local_vector =
      oneq::internal::geometry::RotateVectorToLocalFrame(enu_vector,
                                                         ToGeometryEuler(frame_attitude_deg));

  oneq::foundation::Vector3f local;
  local.x = local_vector.x;
  local.y = local_vector.y;
  local.z = local_vector.z;
  return local;
}

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsFiniteVector3f(const oneq::foundation::Vector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
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

bool TryConvertEcefVelocityToEnu(const oneq::foundation::Vector3f& velocity_ecef_mps,
                                 const oneq::foundation::LlaCoordinateDegM& origin_lla,
                                 oneq::foundation::EnuCoordinateM* velocity_enu_mps) {
  if (velocity_enu_mps == nullptr || !oneq::foundation::IsValidLla(origin_lla) ||
      !IsFiniteVector3f(velocity_ecef_mps)) {
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

bool TryConvertEcefToEosLocalInternal(const oneq::foundation::EcefCoordinateM& position_ecef_m,
                                      const EosCoordinateReference& reference,
                                      oneq::foundation::Vector3f* position_local_m,
                                      EosCoordinateStatus* status) {
  if (position_local_m == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::foundation::EnuCoordinateM enu;
  if (!oneq::foundation::TryEcefToEnu(position_ecef_m, reference.origin_lla, &enu)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }
  *position_local_m = ConvertEnuToEosLocal(enu, reference.frame_attitude_deg);
  return true;
}

bool TryConvertLlaToEosLocalInternal(const oneq::foundation::LlaCoordinateDegM& position_lla_deg_m,
                                     const EosCoordinateReference& reference,
                                     oneq::foundation::Vector3f* position_local_m,
                                     EosCoordinateStatus* status) {
  if (position_local_m == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::foundation::EnuCoordinateM enu;
  if (!oneq::foundation::TryLlaToEnu(position_lla_deg_m, reference.origin_lla, &enu)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }
  *position_local_m = ConvertEnuToEosLocal(enu, reference.frame_attitude_deg);
  return true;
}

bool ResolveTargetLookAngles(const oneq::foundation::Vector3f& relative_local,
                             const oneq::foundation::EulerAnglesDeg& platform_attitude_deg,
                             float* azimuth_deg, float* elevation_deg) {
  if (azimuth_deg == nullptr || elevation_deg == nullptr) {
    return false;
  }
  const float range_m =
      std::sqrt(relative_local.x * relative_local.x + relative_local.y * relative_local.y +
                relative_local.z * relative_local.z);
  if (range_m <= kNormFloor) {
    return false;
  }

  oneq::internal::geometry::Vector3f relative_vector;
  relative_vector.x = relative_local.x;
  relative_vector.y = relative_local.y;
  relative_vector.z = relative_local.z;
  const oneq::internal::geometry::Vector3f platform_frame_vector =
      oneq::internal::geometry::RotateVectorToLocalFrame(relative_vector,
                                                         ToGeometryEuler(platform_attitude_deg));
  const float horizontal_norm = std::sqrt(platform_frame_vector.x * platform_frame_vector.x +
                                          platform_frame_vector.y * platform_frame_vector.y);
  *azimuth_deg = std::atan2(platform_frame_vector.y, platform_frame_vector.x) * 180.0f /
                 foundation::constants::kPi;
  *elevation_deg =
      std::atan2(platform_frame_vector.z, horizontal_norm) * 180.0f / foundation::constants::kPi;
  return true;
}

bool FillTargetFromLocalPosition(std::uint64_t target_id,
                                 const oneq::foundation::Vector3f& target_local,
                                 const oneq::foundation::PoseState& platform_pose,
                                 const EosTargetAppearance& appearance, EosTargetState* target,
                                 EosCoordinateStatus* status) {
  if (target == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::foundation::Vector3f relative_local;
  relative_local.x = target_local.x - platform_pose.position_m.x;
  relative_local.y = target_local.y - platform_pose.position_m.y;
  relative_local.z = target_local.z - platform_pose.position_m.z;
  const float range_m =
      std::sqrt(relative_local.x * relative_local.x + relative_local.y * relative_local.y +
                relative_local.z * relative_local.z);
  if (range_m <= kNormFloor) {
    SetStatus(EosCoordinateStatus::kDegenerateGeometry, status);
    return false;
  }

  float azimuth_deg = 0.0f;
  float elevation_deg = 0.0f;
  if (!ResolveTargetLookAngles(relative_local, platform_pose.attitude_deg, &azimuth_deg,
                               &elevation_deg)) {
    SetStatus(EosCoordinateStatus::kDegenerateGeometry, status);
    return false;
  }

  target->target_id = target_id;
  target->range_m = range_m;
  target->azimuth_deg = azimuth_deg;
  target->elevation_deg = elevation_deg;
  target->apparent_temperature_k = appearance.apparent_temperature_k;
  target->emissivity = appearance.emissivity;
  target->reflectance = appearance.reflectance;
  target->projected_area_m2 = appearance.projected_area_m2;
  SetStatus(EosCoordinateStatus::kOk, status);
  return true;
}

}  // namespace

bool TryMakeEosPoseFromExternalKinematics(const EosExternalPoseInput& input,
                                          const EosCoordinateReference& reference,
                                          oneq::foundation::PoseState* pose,
                                          EosCoordinateStatus* status) {
  if (pose == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::foundation::Vector3f local_position;
  if (!TryConvertEcefToEosLocalInternal(input.platform_position_ecef_m, reference, &local_position,
                                        status)) {
    return false;
  }

  if (!IsFiniteVector3f(input.platform_velocity_mps)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  oneq::foundation::Vector3f velocity_local_mps;
  switch (input.platform_velocity_frame) {
    case EosVelocityFrame::kEosLocal:
      velocity_local_mps = input.platform_velocity_mps;
      break;
    case EosVelocityFrame::kEcef: {
      oneq::foundation::EnuCoordinateM velocity_enu;
      if (!TryConvertEcefVelocityToEnu(input.platform_velocity_mps, reference.origin_lla,
                                       &velocity_enu)) {
        SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
        return false;
      }
      velocity_local_mps = ConvertEnuToEosLocal(velocity_enu, reference.frame_attitude_deg);
      break;
    }
    case EosVelocityFrame::kEnu: {
      const oneq::foundation::EnuCoordinateM velocity_enu =
          ToEnuFromEnuVector(input.platform_velocity_mps);
      velocity_local_mps = ConvertEnuToEosLocal(velocity_enu, reference.frame_attitude_deg);
      break;
    }
    case EosVelocityFrame::kNed: {
      const oneq::foundation::EnuCoordinateM velocity_enu =
          ToEnuFromNed(input.platform_velocity_mps);
      velocity_local_mps = ConvertEnuToEosLocal(velocity_enu, reference.frame_attitude_deg);
      break;
    }
  }

  pose->position_m = local_position;
  pose->velocity_mps = velocity_local_mps;
  pose->attitude_deg = input.platform_attitude_deg;
  SetStatus(EosCoordinateStatus::kOk, status);
  return true;
}

bool TryMakeEosSceneTargetFromExternalInput(
    std::uint64_t target_id,
    const EosExternalTargetInput& input,
    const EosCoordinateReference& reference,
    const oneq::foundation::PoseState& platform_pose,
    EosSceneTarget* target,
    EosCoordinateStatus* status) {
  oneq::foundation::Vector3f target_local;
  switch (input.position_frame) {
    case EosTargetPositionFrame::kEcef:
      if (!TryConvertEcefToEosLocalInternal(input.target_position_ecef_m, reference, &target_local,
                                            status)) {
        return false;
      }
      break;
    case EosTargetPositionFrame::kLla:
      if (!TryConvertLlaToEosLocalInternal(input.target_position_lla_deg_m, reference, &target_local,
                                           status)) {
        return false;
      }
      break;
  }
  return FillTargetFromLocalPosition(target_id, target_local, platform_pose, input.appearance, target,
                                     status);
}

}  // namespace session
}  // namespace electro_optical_sensor
