/**
 * @file coordinate_transform_test.cpp
 * @brief 验证公共 LLA/ECEF/ENU 坐标转换工具的数值正确性与输入校验行为。
 */

#include <gtest/gtest.h>

#include "1q/foundation/coordinate_transform.h"

namespace oneq {
namespace foundation {
namespace {

TEST(CoordinateTransformTest, LlaToEcefAtEquatorPrimeMeridianMatchesWgs84Axis) {
  LlaCoordinateDegM lla;
  lla.latitude_deg = 0.0;
  lla.longitude_deg = 0.0;
  lla.altitude_m = 0.0;

  EcefCoordinateM ecef;
  ASSERT_TRUE(TryLlaToEcef(lla, &ecef));
  EXPECT_NEAR(ecef.x_m, 6378137.0, 1.0e-3);
  EXPECT_NEAR(ecef.y_m, 0.0, 1.0e-6);
  EXPECT_NEAR(ecef.z_m, 0.0, 1.0e-6);
}

TEST(CoordinateTransformTest, LlaEcefRoundTripPreservesCoordinates) {
  LlaCoordinateDegM input;
  input.latitude_deg = 31.2304;
  input.longitude_deg = 121.4737;
  input.altitude_m = 1234.5;

  EcefCoordinateM ecef;
  ASSERT_TRUE(TryLlaToEcef(input, &ecef));

  LlaCoordinateDegM recovered;
  ASSERT_TRUE(TryEcefToLla(ecef, &recovered));
  EXPECT_NEAR(recovered.latitude_deg, input.latitude_deg, 1.0e-6);
  EXPECT_NEAR(recovered.longitude_deg, input.longitude_deg, 1.0e-6);
  EXPECT_NEAR(recovered.altitude_m, input.altitude_m, 1.0e-3);
}

TEST(CoordinateTransformTest, EcefToEnuAtOriginReturnsZero) {
  LlaCoordinateDegM origin_lla;
  origin_lla.latitude_deg = 34.2;
  origin_lla.longitude_deg = 108.9;
  origin_lla.altitude_m = 500.0;
  EcefCoordinateM origin_ecef;
  ASSERT_TRUE(TryLlaToEcef(origin_lla, &origin_ecef));

  EnuCoordinateM enu;
  ASSERT_TRUE(TryEcefToEnu(origin_ecef, origin_lla, &enu));
  EXPECT_NEAR(enu.x_m, 0.0, 1.0e-6);
  EXPECT_NEAR(enu.y_m, 0.0, 1.0e-6);
  EXPECT_NEAR(enu.z_m, 0.0, 1.0e-6);
}

TEST(CoordinateTransformTest, LlaToEnuCapturesEastwardOffsetAtEquator) {
  LlaCoordinateDegM origin_lla;
  origin_lla.latitude_deg = 0.0;
  origin_lla.longitude_deg = 0.0;
  origin_lla.altitude_m = 0.0;

  LlaCoordinateDegM east_point_lla;
  east_point_lla.latitude_deg = 0.0;
  east_point_lla.longitude_deg = 0.001;
  east_point_lla.altitude_m = 0.0;

  EnuCoordinateM enu;
  ASSERT_TRUE(TryLlaToEnu(east_point_lla, origin_lla, &enu));

  EXPECT_GT(enu.x_m, 100.0);
  EXPECT_NEAR(enu.y_m, 0.0, 1.0e-3);
  EXPECT_NEAR(enu.z_m, 0.0, 1.0e-3);
}

TEST(CoordinateTransformTest, InvalidInputReturnsFalse) {
  LlaCoordinateDegM invalid_lla;
  invalid_lla.latitude_deg = 95.0;
  invalid_lla.longitude_deg = 0.0;
  invalid_lla.altitude_m = 0.0;

  EcefCoordinateM ecef;
  EXPECT_FALSE(TryLlaToEcef(invalid_lla, &ecef));
  EXPECT_FALSE(TryLlaToEcef(invalid_lla, nullptr));

  LlaCoordinateDegM lla;
  EXPECT_FALSE(TryEcefToLla(EcefCoordinateM(), nullptr));
  EXPECT_FALSE(TryEcefToLla(EcefCoordinateM(), &lla));
}

TEST(CoordinateTransformTest, ToVector3fPreservesEnuAxisMapping) {
  EnuCoordinateM enu;
  enu.x_m = 12.25;
  enu.y_m = -4.5;
  enu.z_m = 99.0;

  const Vector3f vector = ToVector3f(enu);
  EXPECT_FLOAT_EQ(vector.x, 12.25f);
  EXPECT_FLOAT_EQ(vector.y, -4.5f);
  EXPECT_FLOAT_EQ(vector.z, 99.0f);
}

TEST(CoordinateTransformTest, NueMappingMatchesEnuAxisReorder) {
  LlaCoordinateDegM origin_lla;
  origin_lla.latitude_deg = 0.0;
  origin_lla.longitude_deg = 0.0;
  origin_lla.altitude_m = 0.0;

  LlaCoordinateDegM east_point_lla;
  east_point_lla.latitude_deg = 0.0;
  east_point_lla.longitude_deg = 0.001;
  east_point_lla.altitude_m = 0.0;

  EnuCoordinateM enu;
  ASSERT_TRUE(TryLlaToEnu(east_point_lla, origin_lla, &enu));
  NueCoordinateM nue;
  ASSERT_TRUE(TryLlaToNue(east_point_lla, origin_lla, &nue));

  EXPECT_NEAR(nue.x_m, enu.y_m, 1.0e-6);
  EXPECT_NEAR(nue.y_m, enu.z_m, 1.0e-6);
  EXPECT_NEAR(nue.z_m, enu.x_m, 1.0e-6);
}

TEST(CoordinateTransformTest, ToVector3fPreservesNueAxisMapping) {
  NueCoordinateM nue;
  nue.x_m = -4.5;
  nue.y_m = 99.0;
  nue.z_m = 12.25;

  const Vector3f vector = ToVector3f(nue);
  EXPECT_FLOAT_EQ(vector.x, -4.5f);
  EXPECT_FLOAT_EQ(vector.y, 99.0f);
  EXPECT_FLOAT_EQ(vector.z, 12.25f);
}

}  // namespace
}  // namespace foundation
}  // namespace oneq
