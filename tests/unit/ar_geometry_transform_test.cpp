/**
 * @file geometry_transform_test.cpp
 * @brief 验证共享几何变换模块的姿态旋转、视线角解析与限位规则。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "common/geometry/GeometryTransform.h"

namespace oneq {
namespace internal {
namespace geometry {
namespace {

TEST(GeometryTransformTest, BuildRotationMatrixRotatesBodyXAxisByYaw) {
  EulerAnglesDeg euler_deg;
  euler_deg.yaw_deg = 90.0f;

  const Eigen::Matrix3f rotation = BuildRotationMatrix(euler_deg);
  const Eigen::Vector3f rotated = rotation * Eigen::Vector3f(1.0f, 0.0f, 0.0f);

  EXPECT_NEAR(rotated.x(), 0.0f, 1.0e-5f);
  EXPECT_NEAR(rotated.y(), 1.0f, 1.0e-5f);
  EXPECT_NEAR(rotated.z(), 0.0f, 1.0e-5f);
}

TEST(GeometryTransformTest, RotateVectorToLocalFrameUsesInverseFrameRotation) {
  Vector3f world_vector;
  world_vector.x = 0.0f;
  world_vector.y = 10.0f;
  world_vector.z = 0.0f;
  EulerAnglesDeg local_attitude_deg;
  local_attitude_deg.yaw_deg = 90.0f;

  const Vector3f local_vector = RotateVectorToLocalFrame(world_vector, local_attitude_deg);

  EXPECT_NEAR(local_vector.x, 10.0f, 1.0e-5f);
  EXPECT_NEAR(local_vector.y, 0.0f, 1.0e-5f);
  EXPECT_NEAR(local_vector.z, 0.0f, 1.0e-5f);
}

TEST(GeometryTransformTest, AzimuthElevationRoundTripPreservesAngles) {
  AzimuthElevationDeg input;
  input.az_deg = 35.0f;
  input.el_deg = -12.0f;

  const Eigen::Vector3f unit_vector = AzimuthElevationToUnitVector(input);
  const AzimuthElevationDeg recovered = UnitVectorToAzimuthElevation(unit_vector);

  EXPECT_NEAR(recovered.az_deg, input.az_deg, 1.0e-5f);
  EXPECT_NEAR(recovered.el_deg, input.el_deg, 1.0e-5f);
}

TEST(GeometryTransformTest, ComputeAzimuthDifferenceNormalizesToSignedRange) {
  EXPECT_FLOAT_EQ(ComputeAzimuthDifferenceDeg(170.0f, -170.0f), -20.0f);
  EXPECT_FLOAT_EQ(ComputeAzimuthDifferenceDeg(-170.0f, 170.0f), 20.0f);
}

TEST(GeometryTransformTest, IntersectAndClampFollowLegacyWindowRules) {
  AzimuthElevationLimitsDeg lhs;
  lhs.az_min_deg = -60.0f;
  lhs.az_max_deg = 20.0f;
  lhs.el_min_deg = -30.0f;
  lhs.el_max_deg = 30.0f;

  AzimuthElevationLimitsDeg rhs;
  rhs.az_min_deg = -15.0f;
  rhs.az_max_deg = 10.0f;
  rhs.el_min_deg = -5.0f;
  rhs.el_max_deg = 15.0f;

  const AzimuthElevationLimitsDeg limits = IntersectScanLimits(lhs, rhs);
  AzimuthElevationDeg angle;
  angle.az_deg = 25.0f;
  angle.el_deg = -20.0f;
  const AzimuthElevationDeg clamped = ClampAzimuthElevation(angle, limits);

  EXPECT_FLOAT_EQ(limits.az_min_deg, -15.0f);
  EXPECT_FLOAT_EQ(limits.az_max_deg, 10.0f);
  EXPECT_FLOAT_EQ(limits.el_min_deg, -5.0f);
  EXPECT_FLOAT_EQ(limits.el_max_deg, 15.0f);
  EXPECT_FLOAT_EQ(clamped.az_deg, 10.0f);
  EXPECT_FLOAT_EQ(clamped.el_deg, -5.0f);
}

TEST(GeometryTransformTest, ZeroAttitudeLineOfSightMatchesWorldFrameAngles) {
  Vector3f observer_position_m;
  observer_position_m.x = 0.0f;
  observer_position_m.y = 0.0f;
  observer_position_m.z = 0.0f;
  Vector3f target_position_m;
  target_position_m.x = 100.0f;
  target_position_m.y = 100.0f;
  target_position_m.z = 100.0f;

  const AzimuthElevationDeg look_angles =
      ComputeRelativeLineOfSightAzEl(observer_position_m, EulerAnglesDeg(), target_position_m);
  const float horizontal_range = std::sqrt(100.0f * 100.0f + 100.0f * 100.0f);
  const float expected_el_deg =
      static_cast<float>(std::atan2(100.0f, horizontal_range) * 180.0 / 3.14159265358979);

  EXPECT_NEAR(look_angles.az_deg, 45.0f, 1.0e-5f);
  EXPECT_NEAR(look_angles.el_deg, expected_el_deg, 1.0e-5f);
}

TEST(GeometryTransformTest, NonZeroYawChangesRelativeLineOfSightFrame) {
  Vector3f observer_position_m;
  observer_position_m.x = 0.0f;
  observer_position_m.y = 0.0f;
  observer_position_m.z = 0.0f;
  EulerAnglesDeg observer_attitude_deg;
  observer_attitude_deg.yaw_deg = 90.0f;
  Vector3f target_position_m;
  target_position_m.x = 0.0f;
  target_position_m.y = 100.0f;
  target_position_m.z = 0.0f;

  const AzimuthElevationDeg look_angles =
      ComputeRelativeLineOfSightAzEl(observer_position_m, observer_attitude_deg, target_position_m);

  EXPECT_NEAR(look_angles.az_deg, 0.0f, 1.0e-5f);
  EXPECT_NEAR(look_angles.el_deg, 0.0f, 1.0e-5f);
}

TEST(GeometryTransformTest, ResolveStabilizedMountFramePointingPreservesLegacyInverseRotation) {
  AzimuthElevationDeg desired_platform_pointing_deg;
  desired_platform_pointing_deg.az_deg = 0.0f;
  desired_platform_pointing_deg.el_deg = 0.0f;
  EulerAnglesDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 90.0f;

  const AzimuthElevationDeg mount_pointing = ResolveStabilizedMountFramePointing(
      desired_platform_pointing_deg, platform_attitude_deg, EulerAnglesDeg());

  EXPECT_NEAR(mount_pointing.az_deg, -90.0f, 1.0e-5f);
  EXPECT_NEAR(mount_pointing.el_deg, 0.0f, 1.0e-5f);
}

}  // namespace
}  // namespace geometry
}  // namespace internal
}  // namespace oneq
