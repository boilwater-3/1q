#include "1q/airborne_radar/model/TargetFeatureUtils.h"

#include <cmath>

#include "common/geometry/GeometryTransform.h"

namespace airborne_radar {
namespace model {

namespace {

/**
 * @brief 计算三维向量的欧氏范数。
 * @param x x 分量。
 * @param y y 分量。
 * @param z z 分量。
 * @return 三维向量模长。
 */
float ComputeNorm3(float x, float y, float z) { return std::sqrt(x * x + y * y + z * z); }

/**
 * @brief 将公共姿态角转换为内部几何模块姿态角。
 */
oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(
    const oneq::foundation::EulerAnglesDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

/**
 * @brief 将 ENU 坐标转换到雷达局部坐标。
 */
oneq::foundation::Vector3f ConvertEnuToRadarLocal(const oneq::foundation::EnuCoordinateM& enu_position,
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

/**
 * @brief 判断目标是否携带可用的笛卡尔位置分量。
 * @param target 目标特征。
 * @return `has_cartesian_position` 为 `true` 时返回 `true`。
 */
bool HasCartesianPosition(const model::TargetFeature& target) {
  return target.has_cartesian_position;
}

/**
 * @brief 刷新目标中由速度向量派生出的标量字段。
 * @param[in,out] target 待更新的目标特征指针。
 */
void RefreshDerivedKinematics(model::TargetFeature* target) {
  if (target == nullptr) {
    return;
  }

  target->current_track_speed =
      ComputeNorm3(target->current_track_velocity_x, target->current_track_velocity_y,
                   target->current_track_velocity_z);
}

}  // namespace

model::TargetFeature MakeTargetFromCartesian(std::uint64_t external_target_id, float position_x,
                                             float position_y, float position_z, float velocity_x,
                                             float velocity_y, float velocity_z, float rcs,
                                             int swerling_type) {
  model::TargetFeature target(velocity_x, velocity_y, velocity_z, rcs, 0.0f, swerling_type,
                              external_target_id);
  target.position_x = position_x;
  target.position_y = position_y;
  target.position_z = position_z;
  target.has_cartesian_position = true;
  NormalizeTargetGeometry(&target);
  return target;
}

model::TargetFeature MakeGroundTarget(std::uint64_t external_target_id, float position_x,
                                      float position_y, float rcs, float velocity_x,
                                      float velocity_y, int swerling_type) {
  return MakeTargetFromCartesian(external_target_id, position_x, position_y, 0.0f, velocity_x,
                                 velocity_y, 0.0f, rcs, swerling_type);
}

model::TargetFeature MakeAirTarget(std::uint64_t external_target_id, float position_x,
                                   float position_y, float position_z, float velocity_x,
                                   float velocity_y, float velocity_z, float rcs,
                                   int swerling_type) {
  return MakeTargetFromCartesian(external_target_id, position_x, position_y, position_z, velocity_x,
                                 velocity_y, velocity_z, rcs, swerling_type);
}

bool TryConvertEcefToRadarLocal(const oneq::foundation::EcefCoordinateM& position_ecef_m,
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

bool TryConvertLlaToRadarLocal(const oneq::foundation::LlaCoordinateDegM& position_lla_deg_m,
                               const RadarLocalFrameReference& reference,
                               oneq::foundation::Vector3f* position_local_m) {
  if (position_local_m == nullptr) {
    return false;
  }

  oneq::foundation::EnuCoordinateM enu_position;
  if (!oneq::foundation::TryLlaToEnu(position_lla_deg_m, reference.origin_lla, &enu_position)) {
    return false;
  }
  *position_local_m = ConvertEnuToRadarLocal(enu_position, reference.radar_attitude_deg);
  return true;
}

bool TryMakeTargetFromEcef(std::uint64_t external_target_id,
                           const oneq::foundation::EcefCoordinateM& position_ecef_m,
                           const RadarLocalFrameReference& reference, float velocity_x,
                           float velocity_y, float velocity_z, float rcs, int swerling_type,
                           model::TargetFeature* target) {
  if (target == nullptr) {
    return false;
  }

  oneq::foundation::Vector3f local_position;
  if (!TryConvertEcefToRadarLocal(position_ecef_m, reference, &local_position)) {
    return false;
  }

  *target = MakeTargetFromCartesian(external_target_id, local_position.x, local_position.y,
                                    local_position.z, velocity_x, velocity_y, velocity_z, rcs,
                                    swerling_type);
  return true;
}

bool TryMakeTargetFromLla(std::uint64_t external_target_id,
                          const oneq::foundation::LlaCoordinateDegM& position_lla_deg_m,
                          const RadarLocalFrameReference& reference, float velocity_x,
                          float velocity_y, float velocity_z, float rcs, int swerling_type,
                          model::TargetFeature* target) {
  if (target == nullptr) {
    return false;
  }

  oneq::foundation::Vector3f local_position;
  if (!TryConvertLlaToRadarLocal(position_lla_deg_m, reference, &local_position)) {
    return false;
  }

  *target = MakeTargetFromCartesian(external_target_id, local_position.x, local_position.y,
                                    local_position.z, velocity_x, velocity_y, velocity_z, rcs,
                                    swerling_type);
  return true;
}

void NormalizeTargetGeometry(model::TargetFeature* target) {
  if (target == nullptr) {
    return;
  }

  RefreshDerivedKinematics(target);
  if (target->range_m > 0.0f || !HasCartesianPosition(*target)) {
    return;
  }

  target->range_m = ComputeNorm3(target->position_x, target->position_y, target->position_z);
}

void NormalizeTargetGeometry(model::TargetFeatureList* targets) {
  if (targets == nullptr) {
    return;
  }

  for (std::size_t i = 0; i < targets->size(); ++i) {
    NormalizeTargetGeometry(&(*targets)[i]);
  }
}

}  // namespace model
}  // namespace airborne_radar
