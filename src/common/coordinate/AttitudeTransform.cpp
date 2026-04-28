#include "1q/coordinate/attitude_transform.h"

#include <algorithm>
#include <cmath>

namespace oneq {
namespace coordinate {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNormFloor = 1.0e-9;

bool IsFiniteScalar(double value) { return std::isfinite(value) != 0; }
double DegToRad(double deg) { return deg * kPi / 180.0; }
double RadToDeg(double rad) { return rad * 180.0 / kPi; }

}  // namespace

bool IsFinite(const EulerAnglesDeg& attitude) {
  return IsFiniteScalar(attitude.yaw_deg) && IsFiniteScalar(attitude.pitch_deg) &&
         IsFiniteScalar(attitude.roll_deg);
}

bool IsFinite(const RotationMatrix3d& rotation) {
  return IsFiniteScalar(rotation.m00) && IsFiniteScalar(rotation.m01) &&
         IsFiniteScalar(rotation.m02) && IsFiniteScalar(rotation.m10) &&
         IsFiniteScalar(rotation.m11) && IsFiniteScalar(rotation.m12) &&
         IsFiniteScalar(rotation.m20) && IsFiniteScalar(rotation.m21) &&
         IsFiniteScalar(rotation.m22);
}

RotationMatrix3d BuildRotationMatrix(const EulerAnglesDeg& attitude_deg) {
  const double yaw_rad = DegToRad(attitude_deg.yaw_deg);
  const double pitch_rad = DegToRad(-attitude_deg.pitch_deg);
  const double roll_rad = DegToRad(attitude_deg.roll_deg);

  const double cy = std::cos(yaw_rad);
  const double sy = std::sin(yaw_rad);
  const double cp = std::cos(pitch_rad);
  const double sp = std::sin(pitch_rad);
  const double cr = std::cos(roll_rad);
  const double sr = std::sin(roll_rad);

  RotationMatrix3d rotation;
  rotation.m00 = cy * cp;
  rotation.m01 = cy * sp * sr - sy * cr;
  rotation.m02 = cy * sp * cr + sy * sr;
  rotation.m10 = sy * cp;
  rotation.m11 = sy * sp * sr + cy * cr;
  rotation.m12 = sy * sp * cr - cy * sr;
  rotation.m20 = -sp;
  rotation.m21 = cp * sr;
  rotation.m22 = cp * cr;
  return rotation;
}

EulerAnglesDeg ToEulerAnglesDeg(const RotationMatrix3d& rotation) {
  const double r20 = std::max(-1.0, std::min(1.0, -rotation.m20));
  const double pitch_rad_internal = std::asin(r20);
  const double cos_pitch = std::cos(pitch_rad_internal);

  double yaw_rad = 0.0;
  double roll_rad = 0.0;
  if (std::abs(cos_pitch) > kNormFloor) {
    yaw_rad = std::atan2(rotation.m10, rotation.m00);
    roll_rad = std::atan2(rotation.m21, rotation.m22);
  } else {
    yaw_rad = std::atan2(-rotation.m01, rotation.m11);
    roll_rad = 0.0;
  }

  EulerAnglesDeg attitude;
  attitude.yaw_deg = RadToDeg(yaw_rad);
  attitude.pitch_deg = -RadToDeg(pitch_rad_internal);
  attitude.roll_deg = RadToDeg(roll_rad);
  return attitude;
}

RotationMatrix3d Inverse(const RotationMatrix3d& rotation) {
  RotationMatrix3d inverse;
  inverse.m00 = rotation.m00;
  inverse.m01 = rotation.m10;
  inverse.m02 = rotation.m20;
  inverse.m10 = rotation.m01;
  inverse.m11 = rotation.m11;
  inverse.m12 = rotation.m21;
  inverse.m20 = rotation.m02;
  inverse.m21 = rotation.m12;
  inverse.m22 = rotation.m22;
  return inverse;
}

RotationMatrix3d Compose(const RotationMatrix3d& parent_to_child,
                         const RotationMatrix3d& child_to_grandchild) {
  RotationMatrix3d out;
  out.m00 = parent_to_child.m00 * child_to_grandchild.m00 +
            parent_to_child.m01 * child_to_grandchild.m10 +
            parent_to_child.m02 * child_to_grandchild.m20;
  out.m01 = parent_to_child.m00 * child_to_grandchild.m01 +
            parent_to_child.m01 * child_to_grandchild.m11 +
            parent_to_child.m02 * child_to_grandchild.m21;
  out.m02 = parent_to_child.m00 * child_to_grandchild.m02 +
            parent_to_child.m01 * child_to_grandchild.m12 +
            parent_to_child.m02 * child_to_grandchild.m22;
  out.m10 = parent_to_child.m10 * child_to_grandchild.m00 +
            parent_to_child.m11 * child_to_grandchild.m10 +
            parent_to_child.m12 * child_to_grandchild.m20;
  out.m11 = parent_to_child.m10 * child_to_grandchild.m01 +
            parent_to_child.m11 * child_to_grandchild.m11 +
            parent_to_child.m12 * child_to_grandchild.m21;
  out.m12 = parent_to_child.m10 * child_to_grandchild.m02 +
            parent_to_child.m11 * child_to_grandchild.m12 +
            parent_to_child.m12 * child_to_grandchild.m22;
  out.m20 = parent_to_child.m20 * child_to_grandchild.m00 +
            parent_to_child.m21 * child_to_grandchild.m10 +
            parent_to_child.m22 * child_to_grandchild.m20;
  out.m21 = parent_to_child.m20 * child_to_grandchild.m01 +
            parent_to_child.m21 * child_to_grandchild.m11 +
            parent_to_child.m22 * child_to_grandchild.m21;
  out.m22 = parent_to_child.m20 * child_to_grandchild.m02 +
            parent_to_child.m21 * child_to_grandchild.m12 +
            parent_to_child.m22 * child_to_grandchild.m22;
  return out;
}

EulerAnglesDeg ComposeAttitudeDeg(const EulerAnglesDeg& parent_to_child,
                                  const EulerAnglesDeg& child_to_grandchild) {
  return ToEulerAnglesDeg(
      Compose(BuildRotationMatrix(parent_to_child), BuildRotationMatrix(child_to_grandchild)));
}

namespace {

/// @brief ENU ↔ NED 固定坐标旋转矩阵：[[0,1,0],[1,0,0],[0,0,-1]]。
///        该矩阵是其自身的逆，因此双向转换共用。
RotationMatrix3d EnuNedFixedRotation() {
  RotationMatrix3d R;
  R.m00 = 0.0;  R.m01 = 1.0;  R.m02 = 0.0;
  R.m10 = 1.0;  R.m11 = 0.0;  R.m12 = 0.0;
  R.m20 = 0.0;  R.m21 = 0.0;  R.m22 = -1.0;
  return R;
}

}  // namespace

EulerAnglesDeg ToEnuAttitude(const EulerAnglesDeg& ned_attitude) {
  return ToEulerAnglesDeg(
      Compose(EnuNedFixedRotation(), BuildRotationMatrix(ned_attitude)));
}

EulerAnglesDeg ToNedAttitude(const EulerAnglesDeg& enu_attitude) {
  return ToEulerAnglesDeg(
      Compose(EnuNedFixedRotation(), BuildRotationMatrix(enu_attitude)));
}

Vector3d RotateEnuToLocal(double enu_east, double enu_north, double enu_up,
                          const EulerAnglesDeg& local_attitude_deg) {
  const RotationMatrix3d inverse = Inverse(BuildRotationMatrix(local_attitude_deg));
  Vector3d local;
  local.x = inverse.m00 * enu_east + inverse.m01 * enu_north + inverse.m02 * enu_up;
  local.y = inverse.m10 * enu_east + inverse.m11 * enu_north + inverse.m12 * enu_up;
  local.z = inverse.m20 * enu_east + inverse.m21 * enu_north + inverse.m22 * enu_up;
  return local;
}

}  // namespace coordinate
}  // namespace oneq
