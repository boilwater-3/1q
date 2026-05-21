#include "common/geometry/GeometryTransform.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"

namespace oneq {
namespace internal {
namespace geometry {

namespace {

/**
 * @brief 圆周率常量。
 */
constexpr float kPi = 3.14159265358979f;
/**
 * @brief 向量范数保护下限。
 */
constexpr float kVectorNormFloor = 1.0e-6f;

/**
 * @brief 将角度从度转换为弧度。
 * @param[in] angle_deg 角度（度）。
 * @return 弧度值。
 */
float DegToRad(float angle_deg) { return angle_deg * kPi / 180.0f; }

/**
 * @brief 将角度从弧度转换为度。
 * @param[in] angle_rad 角度（弧度）。
 * @return 度值。
 */
float RadToDeg(float angle_rad) { return angle_rad * 180.0f / kPi; }

/**
 * @brief 对标量执行区间限幅。
 * @param[in] value 输入值。
 * @param[in] minimum 下界。
 * @param[in] maximum 上界。
 * @return 限幅后的结果。
 */

}  // namespace

AzimuthElevationDeg ClampAzimuthElevation(const AzimuthElevationDeg& angle,
                                          const AzimuthElevationLimitsDeg& limits) {
  AzimuthElevationDeg clamped;
  clamped.az_deg = oneq::internal::numerics::Clamp(angle.az_deg, limits.az_min_deg, limits.az_max_deg);
  clamped.el_deg = oneq::internal::numerics::Clamp(angle.el_deg, limits.el_min_deg, limits.el_max_deg);
  return clamped;
}

AzimuthElevationLimitsDeg IntersectScanLimits(const AzimuthElevationLimitsDeg& lhs,
                                              const AzimuthElevationLimitsDeg& rhs) {
  AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = lhs.az_min_deg > rhs.az_min_deg ? lhs.az_min_deg : rhs.az_min_deg;
  limits.az_max_deg = lhs.az_max_deg < rhs.az_max_deg ? lhs.az_max_deg : rhs.az_max_deg;
  limits.el_min_deg = lhs.el_min_deg > rhs.el_min_deg ? lhs.el_min_deg : rhs.el_min_deg;
  limits.el_max_deg = lhs.el_max_deg < rhs.el_max_deg ? lhs.el_max_deg : rhs.el_max_deg;

  if (limits.az_min_deg > limits.az_max_deg) {
    const float center = 0.5f * (limits.az_min_deg + limits.az_max_deg);
    limits.az_min_deg = center;
    limits.az_max_deg = center;
  }
  if (limits.el_min_deg > limits.el_max_deg) {
    const float center = 0.5f * (limits.el_min_deg + limits.el_max_deg);
    limits.el_min_deg = center;
    limits.el_max_deg = center;
  }
  return limits;
}

float ComputeAzimuthDifferenceDeg(float lhs_deg, float rhs_deg) {
  return std::fmod(lhs_deg - rhs_deg + 540.0f, 360.0f) - 180.0f;
}

Eigen::Vector3f AzimuthElevationToUnitVector(const AzimuthElevationDeg& pointing_deg) {
  const float az_rad = DegToRad(pointing_deg.az_deg);
  const float el_rad = DegToRad(pointing_deg.el_deg);
  const float cos_el = std::cos(el_rad);
  return Eigen::Vector3f(cos_el * std::cos(az_rad), cos_el * std::sin(az_rad), std::sin(el_rad));
}

AzimuthElevationDeg UnitVectorToAzimuthElevation(const Eigen::Vector3f& vector) {
  AzimuthElevationDeg pointing_deg;
  const float horizontal_norm = std::sqrt(vector.x() * vector.x() + vector.y() * vector.y());
  pointing_deg.az_deg = RadToDeg(std::atan2(vector.y(), vector.x()));
  pointing_deg.el_deg = RadToDeg(std::atan2(vector.z(), horizontal_norm));
  return pointing_deg;
}

Eigen::Matrix3f BuildRotationMatrix(const EulerAnglesDeg& euler_deg) {
  const float yaw_rad = DegToRad(euler_deg.yaw_deg);
  // 仓库内约定正 pitch 表示正仰角；在当前 z-up 右手系下，
  // 旋转矩阵需使用绕 y 轴的负角，才能与 az/el 语义保持一致。
  const float pitch_rad = DegToRad(-euler_deg.pitch_deg);
  const float roll_rad = DegToRad(euler_deg.roll_deg);

  const float cy = std::cos(yaw_rad);
  const float sy = std::sin(yaw_rad);
  const float cp = std::cos(pitch_rad);
  const float sp = std::sin(pitch_rad);
  const float cr = std::cos(roll_rad);
  const float sr = std::sin(roll_rad);

  Eigen::Matrix3f rotation;
  rotation << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp,
      sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr;
  return rotation;
}

Vector3f RotateVectorToLocalFrame(const Vector3f& world_vector,
                                  const EulerAnglesDeg& local_attitude_deg) {
  const Eigen::Matrix3f rotation = BuildRotationMatrix(local_attitude_deg);
  const Eigen::Vector3f input_vector(world_vector.x, world_vector.y, world_vector.z);
  const Eigen::Vector3f local_vector = rotation.transpose() * input_vector;

  Vector3f rotated;
  rotated.x = local_vector.x();
  rotated.y = local_vector.y();
  rotated.z = local_vector.z();
  return rotated;
}

AzimuthElevationDeg ComputeRelativeLineOfSightAzEl(const Vector3f& observer_position_m,
                                                   const EulerAnglesDeg& observer_attitude_deg,
                                                   const Vector3f& target_position_m) {
  Eigen::Vector3f line_of_sight_world(target_position_m.x - observer_position_m.x,
                                      target_position_m.y - observer_position_m.y,
                                      target_position_m.z - observer_position_m.z);
  const float norm = line_of_sight_world.norm();
  if (norm <= kVectorNormFloor) {
    return AzimuthElevationDeg();
  }
  line_of_sight_world /= norm;

  Vector3f normalized_world_vector;
  normalized_world_vector.x = line_of_sight_world.x();
  normalized_world_vector.y = line_of_sight_world.y();
  normalized_world_vector.z = line_of_sight_world.z();
  const Vector3f observer_frame_vector =
      RotateVectorToLocalFrame(normalized_world_vector, observer_attitude_deg);
  return UnitVectorToAzimuthElevation(
      Eigen::Vector3f(observer_frame_vector.x, observer_frame_vector.y, observer_frame_vector.z));
}

AzimuthElevationDeg ResolveStabilizedMountFramePointing(
    const AzimuthElevationDeg& desired_platform_pointing_deg,
    const EulerAnglesDeg& platform_attitude_deg, const EulerAnglesDeg& mount_angles_deg) {
  const Eigen::Vector3f platform_vector =
      AzimuthElevationToUnitVector(desired_platform_pointing_deg);
  const Eigen::Matrix3f platform_rotation = BuildRotationMatrix(platform_attitude_deg);
  const Eigen::Matrix3f mount_rotation = BuildRotationMatrix(mount_angles_deg);
  const Eigen::Vector3f body_vector = platform_rotation.transpose() * platform_vector;
  const Eigen::Vector3f mount_vector = mount_rotation.transpose() * body_vector;
  return UnitVectorToAzimuthElevation(mount_vector);
}

}  // namespace geometry
}  // namespace internal
}  // namespace oneq
