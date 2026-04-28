#include <cmath>

#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
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

oneq::foundation::EulerAnglesDeg ToFoundationEuler(
    const oneq::coordinate::EulerAnglesDeg& attitude_deg) {
  oneq::foundation::EulerAnglesDeg output;
  output.yaw_deg = static_cast<float>(attitude_deg.yaw_deg);
  output.pitch_deg = static_cast<float>(attitude_deg.pitch_deg);
  output.roll_deg = static_cast<float>(attitude_deg.roll_deg);
  return output;
}

oneq::foundation::Vector3f RotateEnuVectorToLocal(
    double east,
    double north,
    double up,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  const oneq::coordinate::RotationMatrix3d inverse =
      oneq::coordinate::Inverse(oneq::coordinate::BuildRotationMatrix(local_attitude_deg));
  oneq::foundation::Vector3f local;
  local.x = static_cast<float>(inverse.m00 * east + inverse.m01 * north + inverse.m02 * up);
  local.y = static_cast<float>(inverse.m10 * east + inverse.m11 * north + inverse.m12 * up);
  local.z = static_cast<float>(inverse.m20 * east + inverse.m21 * north + inverse.m22 * up);
  return local;
}

oneq::foundation::Vector3f RotateEnuPositionToLocal(
    const oneq::coordinate::EnuPositionM& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return RotateEnuVectorToLocal(enu.east_m, enu.north_m, enu.up_m, local_attitude_deg);
}

oneq::foundation::Vector3f RotateEnuVelocityToLocal(
    const oneq::coordinate::EnuVelocityMps& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return RotateEnuVectorToLocal(enu.east_mps, enu.north_mps, enu.up_mps,
                                local_attitude_deg);
}

bool TryConvertEcefToEosLocalInternal(const oneq::coordinate::EcefPositionM& position_ecef_m,
                                      const EosCoordinateReference& reference,
                                      oneq::foundation::Vector3f* position_local_m,
                                      EosCoordinateStatus* status) {
  if (position_local_m == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }
  oneq::coordinate::EnuPositionM enu;
  if (!oneq::coordinate::TryEcefToEnu(position_ecef_m, reference.origin_lla, &enu)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }
  *position_local_m = RotateEnuPositionToLocal(enu, reference.frame_attitude_deg);
  return true;
}

bool TryConvertLlaToEosLocalInternal(const oneq::coordinate::LlaPositionDegM& position_lla_deg_m,
                                     const EosCoordinateReference& reference,
                                     oneq::foundation::Vector3f* position_local_m,
                                     EosCoordinateStatus* status) {
  if (position_local_m == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::coordinate::EnuPositionM enu;
  if (!oneq::coordinate::TryLlaToEnu(position_lla_deg_m, reference.origin_lla, &enu)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }
  *position_local_m = RotateEnuPositionToLocal(enu, reference.frame_attitude_deg);
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

  oneq::coordinate::EulerAnglesDeg platform_attitude;
  platform_attitude.yaw_deg = static_cast<double>(platform_attitude_deg.yaw_deg);
  platform_attitude.pitch_deg = static_cast<double>(platform_attitude_deg.pitch_deg);
  platform_attitude.roll_deg = static_cast<double>(platform_attitude_deg.roll_deg);
  const oneq::foundation::Vector3f platform_frame_vector = RotateEnuVectorToLocal(
      static_cast<double>(relative_local.x), static_cast<double>(relative_local.y),
      static_cast<double>(relative_local.z), platform_attitude);
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
                                 const EosTargetAppearance& appearance, EosSceneTarget* target,
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
  target->appearance = appearance;
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
  if (!TryConvertEcefToEosLocalInternal(input.platform_position_ecef_m, reference,
                                        &local_position, status)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  oneq::coordinate::EnuVelocityMps velocity_enu_mps;
  if (!oneq::coordinate::TryEcefToEnuVelocity(input.platform_velocity_mps,
                                              reference.origin_lla,
                                              &velocity_enu_mps)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }
  const oneq::foundation::Vector3f velocity_local_mps =
      RotateEnuVelocityToLocal(velocity_enu_mps, reference.frame_attitude_deg);

  pose->position_m = local_position;
  pose->velocity_mps = velocity_local_mps;
  pose->attitude_deg = ToFoundationEuler(input.platform_attitude_deg);
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
      if (!TryConvertEcefToEosLocalInternal(input.target_position_ecef_m, reference,
                                            &target_local, status)) {
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
