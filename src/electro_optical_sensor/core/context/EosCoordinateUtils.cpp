#include "1q/electro_optical_sensor/core/context/EosCoordinateUtils.h"

#include <cmath>

#include <Eigen/Core>

#include "common/geometry/GeometryTransform.h"

namespace electro_optical_sensor {
namespace core {
namespace context {

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kNormFloor = 1.0e-6f;

oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(
    const oneq::common::EulerAnglesDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

oneq::common::Vector3f ConvertEnuToEosLocal(const oneq::common::EnuCoordinateM& enu,
                                            const oneq::common::EulerAnglesDeg& frame_attitude_deg) {
  const Eigen::Vector3f enu_vector(static_cast<float>(enu.x_m), static_cast<float>(enu.y_m),
                                   static_cast<float>(enu.z_m));
  const Eigen::Matrix3f rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(frame_attitude_deg));
  const Eigen::Vector3f local_vector = rotation.transpose() * enu_vector;

  oneq::common::Vector3f local;
  local.x = local_vector.x();
  local.y = local_vector.y();
  local.z = local_vector.z();
  return local;
}

bool ResolveTargetLookAngles(const oneq::common::Vector3f& relative_local,
                             const oneq::common::EulerAnglesDeg& platform_attitude_deg,
                             float* azimuth_deg, float* elevation_deg) {
  if (azimuth_deg == nullptr || elevation_deg == nullptr) {
    return false;
  }
  const Eigen::Vector3f relative_vector(relative_local.x, relative_local.y, relative_local.z);
  const float range_m = relative_vector.norm();
  if (range_m <= kNormFloor) {
    return false;
  }

  const Eigen::Matrix3f platform_rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(platform_attitude_deg));
  const Eigen::Vector3f platform_frame_vector = platform_rotation.transpose() * relative_vector;
  const float horizontal_norm =
      std::sqrt(platform_frame_vector.x() * platform_frame_vector.x() +
                platform_frame_vector.y() * platform_frame_vector.y());
  *azimuth_deg = std::atan2(platform_frame_vector.y(), platform_frame_vector.x()) * 180.0f / kPi;
  *elevation_deg = std::atan2(platform_frame_vector.z(), horizontal_norm) * 180.0f / kPi;
  return true;
}

bool FillTargetFromLocalPosition(std::uint64_t target_id, const oneq::common::Vector3f& target_local,
                                 const oneq::common::PoseState& platform_pose,
                                 const EosTargetAppearance& appearance, EosTargetState* target) {
  if (target == nullptr) {
    return false;
  }

  oneq::common::Vector3f relative_local;
  relative_local.x = target_local.x - platform_pose.position_m.x;
  relative_local.y = target_local.y - platform_pose.position_m.y;
  relative_local.z = target_local.z - platform_pose.position_m.z;
  const float range_m = std::sqrt(relative_local.x * relative_local.x +
                                  relative_local.y * relative_local.y +
                                  relative_local.z * relative_local.z);
  if (range_m <= kNormFloor) {
    return false;
  }

  float azimuth_deg = 0.0f;
  float elevation_deg = 0.0f;
  if (!ResolveTargetLookAngles(relative_local, platform_pose.attitude_deg, &azimuth_deg,
                               &elevation_deg)) {
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
  return true;
}

}  // namespace

bool TryConvertEcefToEosLocal(const oneq::common::EcefCoordinateM& position_ecef_m,
                              const EosCoordinateReference& reference,
                              oneq::common::Vector3f* position_local_m) {
  if (position_local_m == nullptr) {
    return false;
  }

  oneq::common::EnuCoordinateM enu;
  if (!oneq::common::TryEcefToEnu(position_ecef_m, reference.origin_lla, &enu)) {
    return false;
  }
  *position_local_m = ConvertEnuToEosLocal(enu, reference.frame_attitude_deg);
  return true;
}

bool TryConvertLlaToEosLocal(const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                             const EosCoordinateReference& reference,
                             oneq::common::Vector3f* position_local_m) {
  if (position_local_m == nullptr) {
    return false;
  }

  oneq::common::EnuCoordinateM enu;
  if (!oneq::common::TryLlaToEnu(position_lla_deg_m, reference.origin_lla, &enu)) {
    return false;
  }
  *position_local_m = ConvertEnuToEosLocal(enu, reference.frame_attitude_deg);
  return true;
}

bool TryMakeEosPoseFromEcef(const oneq::common::EcefCoordinateM& position_ecef_m,
                            const EosCoordinateReference& reference,
                            const oneq::common::Vector3f& velocity_local_mps,
                            const oneq::common::EulerAnglesDeg& attitude_deg,
                            oneq::common::PoseState* pose) {
  if (pose == nullptr) {
    return false;
  }

  oneq::common::Vector3f local_position;
  if (!TryConvertEcefToEosLocal(position_ecef_m, reference, &local_position)) {
    return false;
  }

  pose->position_m = local_position;
  pose->velocity_mps = velocity_local_mps;
  pose->attitude_deg = attitude_deg;
  return true;
}

bool TryMakeEosPoseFromLla(const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                           const EosCoordinateReference& reference,
                           const oneq::common::Vector3f& velocity_local_mps,
                           const oneq::common::EulerAnglesDeg& attitude_deg,
                           oneq::common::PoseState* pose) {
  if (pose == nullptr) {
    return false;
  }

  oneq::common::Vector3f local_position;
  if (!TryConvertLlaToEosLocal(position_lla_deg_m, reference, &local_position)) {
    return false;
  }

  pose->position_m = local_position;
  pose->velocity_mps = velocity_local_mps;
  pose->attitude_deg = attitude_deg;
  return true;
}

bool TryMakeEosTargetFromEcef(std::uint64_t target_id,
                              const oneq::common::EcefCoordinateM& target_ecef_m,
                              const EosCoordinateReference& reference,
                              const oneq::common::PoseState& platform_pose,
                              const EosTargetAppearance& appearance, EosTargetState* target) {
  oneq::common::Vector3f target_local;
  if (!TryConvertEcefToEosLocal(target_ecef_m, reference, &target_local)) {
    return false;
  }
  return FillTargetFromLocalPosition(target_id, target_local, platform_pose, appearance, target);
}

bool TryMakeEosTargetFromLla(std::uint64_t target_id,
                             const oneq::common::LlaCoordinateDegM& target_lla_deg_m,
                             const EosCoordinateReference& reference,
                             const oneq::common::PoseState& platform_pose,
                             const EosTargetAppearance& appearance, EosTargetState* target) {
  oneq::common::Vector3f target_local;
  if (!TryConvertLlaToEosLocal(target_lla_deg_m, reference, &target_local)) {
    return false;
  }
  return FillTargetFromLocalPosition(target_id, target_local, platform_pose, appearance, target);
}

}  // namespace context
}  // namespace core
}  // namespace electro_optical_sensor

