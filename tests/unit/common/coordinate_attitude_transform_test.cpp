/**
 * @file coordinate_attitude_transform_test.cpp
 * @brief 验证 oneq::coordinate 姿态转换工具。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/coordinate/attitude_transform.h"

namespace oneq {
namespace coordinate {
namespace {

double MatrixAt(const RotationMatrix3d& matrix, int row, int col) {
  const double values[3][3] = {
      {matrix.m00, matrix.m01, matrix.m02},
      {matrix.m10, matrix.m11, matrix.m12},
      {matrix.m20, matrix.m21, matrix.m22},
  };
  return values[row][col];
}

constexpr double kTolerance = 1.0e-9;

TEST(CoordinateAttitudeTransformTest, BuildRotationMatrixRotatesXAxisByYaw) {
  EulerAnglesDeg attitude;
  attitude.yaw_deg = 90.0;

  const RotationMatrix3d rotation = BuildRotationMatrix(attitude);
  const double x = rotation.m00;
  const double y = rotation.m10;
  const double z = rotation.m20;

  EXPECT_NEAR(x, 0.0, 1.0e-12);
  EXPECT_NEAR(y, 1.0, 1.0e-12);
  EXPECT_NEAR(z, 0.0, 1.0e-12);
}

TEST(CoordinateAttitudeTransformTest, ComposeMatchesMatrixMultiplication) {
  EulerAnglesDeg platform;
  platform.yaw_deg = 40.0;
  platform.pitch_deg = 12.0;
  platform.roll_deg = -18.0;

  EulerAnglesDeg mount;
  mount.yaw_deg = 3.0;
  mount.pitch_deg = -2.5;
  mount.roll_deg = 1.5;

  const EulerAnglesDeg composed = ComposeAttitudeDeg(platform, mount);
  const RotationMatrix3d expected =
      Compose(BuildRotationMatrix(platform), BuildRotationMatrix(mount));
  const RotationMatrix3d actual = BuildRotationMatrix(composed);

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(MatrixAt(actual, row, col), MatrixAt(expected, row, col), 1.0e-12);
    }
  }
}

TEST(CoordinateAttitudeTransformTest, InverseIsTransposeForRotationMatrix) {
  EulerAnglesDeg attitude;
  attitude.yaw_deg = 15.0;
  attitude.pitch_deg = -5.0;
  attitude.roll_deg = 8.0;

  const RotationMatrix3d rotation = BuildRotationMatrix(attitude);
  const RotationMatrix3d inverse = Inverse(rotation);

  EXPECT_DOUBLE_EQ(inverse.m01, rotation.m10);
  EXPECT_DOUBLE_EQ(inverse.m02, rotation.m20);
  EXPECT_DOUBLE_EQ(inverse.m10, rotation.m01);
  EXPECT_DOUBLE_EQ(inverse.m12, rotation.m21);
  EXPECT_DOUBLE_EQ(inverse.m20, rotation.m02);
  EXPECT_DOUBLE_EQ(inverse.m21, rotation.m12);
}

// =============================================================================
// IsFinite — 有限性校验（含 NaN/Inf 拒绝分支）
// =============================================================================

TEST(CoordinateAttitudeTransformTest, IsFiniteAcceptsNormalEuler) {
  EulerAnglesDeg attitude;
  attitude.yaw_deg = 1.0;
  attitude.pitch_deg = -2.0;
  attitude.roll_deg = 3.0;
  EXPECT_TRUE(IsFinite(attitude));
}

TEST(CoordinateAttitudeTransformTest, IsFiniteRejectsNanEuler) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EulerAnglesDeg attitude;
  attitude.yaw_deg = 1.0;
  attitude.pitch_deg = nan;
  attitude.roll_deg = 3.0;
  EXPECT_FALSE(IsFinite(attitude));
}

TEST(CoordinateAttitudeTransformTest, IsFiniteRejectsInfEuler) {
  const double inf = std::numeric_limits<double>::infinity();
  EulerAnglesDeg attitude;
  attitude.yaw_deg = inf;
  EXPECT_FALSE(IsFinite(attitude));
}

TEST(CoordinateAttitudeTransformTest, IsFiniteAcceptsNormalMatrix) {
  EulerAnglesDeg attitude;
  attitude.yaw_deg = 10.0;
  attitude.pitch_deg = 20.0;
  attitude.roll_deg = 30.0;
  EXPECT_TRUE(IsFinite(BuildRotationMatrix(attitude)));
}

TEST(CoordinateAttitudeTransformTest, IsFiniteRejectsNanMatrix) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  RotationMatrix3d rotation;
  rotation.m00 = nan;
  EXPECT_FALSE(IsFinite(rotation));
}

// =============================================================================
// ToEulerAnglesDeg — 旋转矩阵反解（含 gimbal-lock 分支）
// =============================================================================

TEST(CoordinateAttitudeTransformTest, ToEulerRoundTripPreservesAngles) {
  EulerAnglesDeg input;
  input.yaw_deg = 45.0;
  input.pitch_deg = -30.0;
  input.roll_deg = 60.0;

  const RotationMatrix3d rotation = BuildRotationMatrix(input);
  const EulerAnglesDeg recovered = ToEulerAnglesDeg(rotation);

  EXPECT_NEAR(recovered.yaw_deg, input.yaw_deg, kTolerance);
  EXPECT_NEAR(recovered.pitch_deg, input.pitch_deg, kTolerance);
  EXPECT_NEAR(recovered.roll_deg, input.roll_deg, kTolerance);
}

TEST(CoordinateAttitudeTransformTest, ToEulerRoundTripNegativeYaw) {
  EulerAnglesDeg input;
  input.yaw_deg = -120.0;
  input.pitch_deg = 15.0;
  input.roll_deg = -25.0;

  const EulerAnglesDeg recovered = ToEulerAnglesDeg(BuildRotationMatrix(input));

  EXPECT_NEAR(recovered.yaw_deg, input.yaw_deg, kTolerance);
  EXPECT_NEAR(recovered.pitch_deg, input.pitch_deg, kTolerance);
  EXPECT_NEAR(recovered.roll_deg, input.roll_deg, kTolerance);
}

TEST(CoordinateAttitudeTransformTest, ToEulerHandlesGimbalLockAtPositivePitch90) {
  // pitch = ±90° 时 cos_pitch ≈ 0，触发 gimbal-lock 分支（else 路径）。
  EulerAnglesDeg input;
  input.yaw_deg = 0.0;
  input.pitch_deg = 90.0;
  input.roll_deg = 0.0;

  const EulerAnglesDeg recovered = ToEulerAnglesDeg(BuildRotationMatrix(input));
  // gimbal-lock 下 roll 归零、pitch 应恢复 ±90°
  EXPECT_NEAR(recovered.pitch_deg, input.pitch_deg, 1.0e-6);
  EXPECT_NEAR(recovered.roll_deg, 0.0, kTolerance);
}

TEST(CoordinateAttitudeTransformTest, ToEulerHandlesGimbalLockAtNegativePitch90) {
  EulerAnglesDeg input;
  input.yaw_deg = 0.0;
  input.pitch_deg = -90.0;
  input.roll_deg = 0.0;

  const EulerAnglesDeg recovered = ToEulerAnglesDeg(BuildRotationMatrix(input));
  EXPECT_NEAR(recovered.pitch_deg, input.pitch_deg, 1.0e-6);
  EXPECT_NEAR(recovered.roll_deg, 0.0, kTolerance);
}

// =============================================================================
// ToEnuAttitude / ToNedAttitude — ENU↔NED 姿态对偶
// =============================================================================

TEST(CoordinateAttitudeTransformTest, ToEnuAttitudeFromZeroNedMapsToAxisSwap) {
  // NED→ENU 固定旋转 [[0,1,0],[1,0,0],[0,0,-1]] 将 X/Y 轴互换、Z 反向。
  // 因此零 NED 姿态（机头朝北=Z 向下）映射到 ENU 后为 yaw=90°, roll=180°。
  EulerAnglesDeg ned;
  ned.yaw_deg = 0.0;
  ned.pitch_deg = 0.0;
  ned.roll_deg = 0.0;

  const EulerAnglesDeg enu = ToEnuAttitude(ned);
  EXPECT_NEAR(enu.yaw_deg, 90.0, kTolerance);
  EXPECT_NEAR(enu.pitch_deg, 0.0, kTolerance);
  EXPECT_NEAR(enu.roll_deg, 180.0, kTolerance);
}

TEST(CoordinateAttitudeTransformTest, ToNedAttitudeInvertsToEnuAttitude) {
  EulerAnglesDeg original;
  original.yaw_deg = 45.0;
  original.pitch_deg = -10.0;
  original.roll_deg = 5.0;

  // ENU→NED→ENU 应恢复原值（固定旋转矩阵是其自身的逆）
  const EulerAnglesDeg ned = ToNedAttitude(original);
  const EulerAnglesDeg back = ToEnuAttitude(ned);

  EXPECT_NEAR(back.yaw_deg, original.yaw_deg, 1.0e-6);
  EXPECT_NEAR(back.pitch_deg, original.pitch_deg, 1.0e-6);
  EXPECT_NEAR(back.roll_deg, original.roll_deg, 1.0e-6);
}

// =============================================================================
// RotateEnuToLocal / RotateLocalToEnu — 互逆向量旋转
// =============================================================================

TEST(CoordinateAttitudeTransformTest, RotateEnuToLocalWithZeroAttitudeIsIdentity) {
  EulerAnglesDeg attitude;  // 全零 → 单位矩阵

  const Vector3d local = RotateEnuToLocal(1.0, 2.0, 3.0, attitude);
  EXPECT_NEAR(local.x, 1.0, kTolerance);
  EXPECT_NEAR(local.y, 2.0, kTolerance);
  EXPECT_NEAR(local.z, 3.0, kTolerance);
}

TEST(CoordinateAttitudeTransformTest, RotateLocalToEnuInvertsRotateEnuToLocal) {
  EulerAnglesDeg attitude;
  attitude.yaw_deg = 30.0;
  attitude.pitch_deg = -15.0;
  attitude.roll_deg = 20.0;

  const Vector3d local = RotateEnuToLocal(10.0, 20.0, 30.0, attitude);
  const Vector3d enu = RotateLocalToEnu(local.x, local.y, local.z, attitude);

  EXPECT_NEAR(enu.x, 10.0, 1.0e-9);
  EXPECT_NEAR(enu.y, 20.0, 1.0e-9);
  EXPECT_NEAR(enu.z, 30.0, 1.0e-9);
}

}  // namespace
}  // namespace coordinate
}  // namespace oneq
