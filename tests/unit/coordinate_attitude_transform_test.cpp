/**
 * @file coordinate_attitude_transform_test.cpp
 * @brief 验证 oneq::coordinate 姿态转换工具。
 */

#include <gtest/gtest.h>

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

}  // namespace
}  // namespace coordinate
}  // namespace oneq
