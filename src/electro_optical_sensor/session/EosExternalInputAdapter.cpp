#include <cmath>

#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "common/geometry/CoordinateConversion.h"
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

/// @brief 将 EosCoordinateReference 映射到内部共享 LocalFrameReference。
oneq::internal::geometry::LocalFrameReference ToSharedReference(
    const EosCoordinateReference& ref) {
  oneq::internal::geometry::LocalFrameReference shared;
  shared.origin_lla = ref.origin_lla;
  shared.frame_attitude_deg = ref.frame_attitude_deg;
  return shared;
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
  *position_local_m =
      oneq::internal::geometry::ConvertEnuToLocal(enu, reference.frame_attitude_deg);
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

  oneq::internal::geometry::EulerAnglesDeg geometry_attitude;
  geometry_attitude.yaw_deg = platform_attitude_deg.yaw_deg;
  geometry_attitude.pitch_deg = platform_attitude_deg.pitch_deg;
  geometry_attitude.roll_deg = platform_attitude_deg.roll_deg;

  const oneq::internal::geometry::Vector3f platform_frame_vector =
      oneq::internal::geometry::RotateVectorToLocalFrame(relative_vector, geometry_attitude);
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

  const auto shared_ref = ToSharedReference(reference);

  oneq::foundation::Vector3f local_position;
  if (!oneq::internal::geometry::TryConvertEcefPositionToLocal(
          input.platform_position_ecef_m, shared_ref, &local_position)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  oneq::foundation::Vector3f velocity_local_mps;
  if (!oneq::internal::geometry::TryConvertVelocityToLocal(
          input.platform_velocity_mps, oneq::internal::geometry::VelocityFrame::kEcef,
          shared_ref, &velocity_local_mps)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
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
  const auto shared_ref = ToSharedReference(reference);

  switch (input.position_frame) {
    case EosTargetPositionFrame::kEcef:
      if (!oneq::internal::geometry::TryConvertEcefPositionToLocal(
              input.target_position_ecef_m, shared_ref, &target_local)) {
        SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
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
