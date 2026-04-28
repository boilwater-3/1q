/**
 * @file coordinate_velocity_transform_test.cpp
 * @brief 验证 oneq::coordinate 速度转换工具。
 */

#include <gtest/gtest.h>

#include "1q/coordinate/velocity_transform.h"

namespace oneq {
namespace coordinate {
namespace {

TEST(CoordinateVelocityTransformTest, EcefVelocityToEnuAtEquatorPrimeMeridian) {
  LlaPositionDegM origin;
  origin.latitude_deg = 0.0;
  origin.longitude_deg = 0.0;
  origin.altitude_m = 0.0;

  EcefVelocityMps ecef;
  ecef.x_mps = 3.0;
  ecef.y_mps = 1.0;
  ecef.z_mps = 2.0;

  EnuVelocityMps enu;
  ASSERT_TRUE(TryEcefToEnuVelocity(ecef, origin, &enu));
  EXPECT_DOUBLE_EQ(enu.east_mps, 1.0);
  EXPECT_DOUBLE_EQ(enu.north_mps, 2.0);
  EXPECT_DOUBLE_EQ(enu.up_mps, 3.0);
}

TEST(CoordinateVelocityTransformTest, LocalVelocityAxisRoundTrip) {
  EnuVelocityMps enu;
  enu.east_mps = 10.0;
  enu.north_mps = 20.0;
  enu.up_mps = 30.0;

  const NedVelocityMps ned = ToNedVelocity(enu);
  EXPECT_DOUBLE_EQ(ned.north_mps, 20.0);
  EXPECT_DOUBLE_EQ(ned.east_mps, 10.0);
  EXPECT_DOUBLE_EQ(ned.down_mps, -30.0);

  const NueVelocityMps nue = ToNueVelocity(ned);
  EXPECT_DOUBLE_EQ(nue.north_mps, 20.0);
  EXPECT_DOUBLE_EQ(nue.up_mps, 30.0);
  EXPECT_DOUBLE_EQ(nue.east_mps, 10.0);

  const EnuVelocityMps back = ToEnuVelocity(nue);
  EXPECT_DOUBLE_EQ(back.east_mps, enu.east_mps);
  EXPECT_DOUBLE_EQ(back.north_mps, enu.north_mps);
  EXPECT_DOUBLE_EQ(back.up_mps, enu.up_mps);
}

TEST(CoordinateVelocityTransformTest, RejectsInvalidInputs) {
  EcefVelocityMps ecef;
  ecef.x_mps = 1.0;
  ecef.y_mps = 2.0;
  ecef.z_mps = 3.0;
  LlaPositionDegM invalid_origin;
  invalid_origin.latitude_deg = 120.0;
  EnuVelocityMps enu;

  EXPECT_FALSE(TryEcefToEnuVelocity(ecef, invalid_origin, &enu));
  EXPECT_FALSE(TryEcefToEnuVelocity(ecef, LlaPositionDegM(), nullptr));
}

}  // namespace
}  // namespace coordinate
}  // namespace oneq
