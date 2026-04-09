#include "1q/electro_optical_sensor/core/context/EosCoordinateUtils.h"

#include <cmath>

#include "common/geometry/GeometryTransform.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"

namespace electro_optical_sensor {
namespace core {
namespace context {

namespace {

constexpr float kNormFloor = 1.0e-6f;

void SetStatus(EosCoordinateStatus value, EosCoordinateStatus* status) {
  if (status != nullptr) {
    *status = value;
  }
}

oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(
    const oneq::common::EulerAnglesDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

oneq::common::Vector3f ConvertEnuToEosLocal(
    const oneq::common::EnuCoordinateM& enu,
    const oneq::common::EulerAnglesDeg& frame_attitude_deg) {
  oneq::internal::geometry::Vector3f enu_vector;
  enu_vector.x = static_cast<float>(enu.x_m);
  enu_vector.y = static_cast<float>(enu.y_m);
  enu_vector.z = static_cast<float>(enu.z_m);
  const oneq::internal::geometry::Vector3f local_vector =
      oneq::internal::geometry::RotateVectorToLocalFrame(enu_vector,
                                                         ToGeometryEuler(frame_attitude_deg));

  oneq::common::Vector3f local;
  local.x = local_vector.x;
  local.y = local_vector.y;
  local.z = local_vector.z;
  return local;
}

bool ResolveTargetLookAngles(const oneq::common::Vector3f& relative_local,
                             const oneq::common::EulerAnglesDeg& platform_attitude_deg,
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
                                 const oneq::common::Vector3f& target_local,
                                 const oneq::common::PoseState& platform_pose,
                                 const EosTargetAppearance& appearance, EosTargetState* target,
                                 EosCoordinateStatus* status) {
  if (target == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::common::Vector3f relative_local;
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

bool TryConvertEcefToEosLocal(const oneq::common::EcefCoordinateM& position_ecef_m,
                              const EosCoordinateReference& reference,
                              oneq::common::Vector3f* position_local_m,
                              EosCoordinateStatus* status) {
  if (position_local_m == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::common::EnuCoordinateM enu;
  if (!oneq::common::TryEcefToEnu(position_ecef_m, reference.origin_lla, &enu)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }
  *position_local_m = ConvertEnuToEosLocal(enu, reference.frame_attitude_deg);
  SetStatus(EosCoordinateStatus::kOk, status);
  return true;
}

bool TryConvertLlaToEosLocal(const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                             const EosCoordinateReference& reference,
                             oneq::common::Vector3f* position_local_m,
                             EosCoordinateStatus* status) {
  if (position_local_m == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::common::EnuCoordinateM enu;
  if (!oneq::common::TryLlaToEnu(position_lla_deg_m, reference.origin_lla, &enu)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }
  *position_local_m = ConvertEnuToEosLocal(enu, reference.frame_attitude_deg);
  SetStatus(EosCoordinateStatus::kOk, status);
  return true;
}

bool TryMakeEosPoseFromEcef(const oneq::common::EcefCoordinateM& position_ecef_m,
                            const EosCoordinateReference& reference,
                            const oneq::common::Vector3f& velocity_local_mps,
                            const oneq::common::EulerAnglesDeg& attitude_deg,
                            oneq::common::PoseState* pose, EosCoordinateStatus* status) {
  if (pose == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::common::Vector3f local_position;
  if (!TryConvertEcefToEosLocal(position_ecef_m, reference, &local_position, status)) {
    return false;
  }

  pose->position_m = local_position;
  pose->velocity_mps = velocity_local_mps;
  pose->attitude_deg = attitude_deg;
  SetStatus(EosCoordinateStatus::kOk, status);
  return true;
}

bool TryMakeEosPoseFromLla(const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                           const EosCoordinateReference& reference,
                           const oneq::common::Vector3f& velocity_local_mps,
                           const oneq::common::EulerAnglesDeg& attitude_deg,
                           oneq::common::PoseState* pose, EosCoordinateStatus* status) {
  if (pose == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::common::Vector3f local_position;
  if (!TryConvertLlaToEosLocal(position_lla_deg_m, reference, &local_position, status)) {
    return false;
  }

  pose->position_m = local_position;
  pose->velocity_mps = velocity_local_mps;
  pose->attitude_deg = attitude_deg;
  SetStatus(EosCoordinateStatus::kOk, status);
  return true;
}

bool TryMakeEosTargetFromEcef(std::uint64_t target_id,
                              const oneq::common::EcefCoordinateM& target_ecef_m,
                              const EosCoordinateReference& reference,
                              const oneq::common::PoseState& platform_pose,
                              const EosTargetAppearance& appearance, EosTargetState* target,
                              EosCoordinateStatus* status) {
  oneq::common::Vector3f target_local;
  if (!TryConvertEcefToEosLocal(target_ecef_m, reference, &target_local, status)) {
    return false;
  }
  return FillTargetFromLocalPosition(target_id, target_local, platform_pose, appearance, target,
                                     status);
}

bool TryMakeEosTargetFromLla(std::uint64_t target_id,
                             const oneq::common::LlaCoordinateDegM& target_lla_deg_m,
                             const EosCoordinateReference& reference,
                             const oneq::common::PoseState& platform_pose,
                             const EosTargetAppearance& appearance, EosTargetState* target,
                             EosCoordinateStatus* status) {
  oneq::common::Vector3f target_local;
  if (!TryConvertLlaToEosLocal(target_lla_deg_m, reference, &target_local, status)) {
    return false;
  }
  return FillTargetFromLocalPosition(target_id, target_local, platform_pose, appearance, target,
                                     status);
}

}  // namespace context
}  // namespace core
}  // namespace electro_optical_sensor
