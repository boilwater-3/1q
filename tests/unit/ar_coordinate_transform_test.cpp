/**
 * @file coordinate_position_transform_legacy_test.cpp
 * @brief 验证公共 LLA/ECEF/ENU 坐标转换工具的数值正确性与输入校验行为。
 */

#include <gtest/gtest.h>

#include "1q/coordinate/position_transform.h"

namespace oneq {
namespace coordinate {
namespace {

TEST(CoordinateTransformTest, LlaToEcefAtEquatorPrimeMeridianMatchesWgs84Axis) {
  LlaPositionDegM lla;
  lla.latitude_deg = 0.0;
  lla.longitude_deg = 0.0;
  lla.altitude_m = 0.0;

  EcefPositionM ecef;
  ASSERT_TRUE(TryLlaToEcef(lla, &ecef));
  EXPECT_NEAR(ecef.x_m, 6378137.0, 1.0e-3);
  EXPECT_NEAR(ecef.y_m, 0.0, 1.0e-6);
  EXPECT_NEAR(ecef.z_m, 0.0, 1.0e-6);
}

TEST(CoordinateTransformTest, LlaEcefRoundTripPreservesCoordinates) {
  LlaPositionDegM input;
  input.latitude_deg = 31.2304;
  input.longitude_deg = 121.4737;
  input.altitude_m = 1234.5;

  EcefPositionM ecef;
  ASSERT_TRUE(TryLlaToEcef(input, &ecef));

  LlaPositionDegM recovered;
  ASSERT_TRUE(TryEcefToLla(ecef, &recovered));
  EXPECT_NEAR(recovered.latitude_deg, input.latitude_deg, 1.0e-6);
  EXPECT_NEAR(recovered.longitude_deg, input.longitude_deg, 1.0e-6);
  EXPECT_NEAR(recovered.altitude_m, input.altitude_m, 1.0e-3);
}

TEST(CoordinateTransformTest, EcefToEnuAtOriginReturnsZero) {
  LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 34.2;
  origin_lla.longitude_deg = 108.9;
  origin_lla.altitude_m = 500.0;
  EcefPositionM origin_ecef;
  ASSERT_TRUE(TryLlaToEcef(origin_lla, &origin_ecef));

  EnuPositionM enu;
  ASSERT_TRUE(TryEcefToEnu(origin_ecef, origin_lla, &enu));
  EXPECT_NEAR(enu.east_m, 0.0, 1.0e-6);
  EXPECT_NEAR(enu.north_m, 0.0, 1.0e-6);
  EXPECT_NEAR(enu.up_m, 0.0, 1.0e-6);
}

TEST(CoordinateTransformTest, LlaToEnuCapturesEastwardOffsetAtEquator) {
  LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 0.0;
  origin_lla.longitude_deg = 0.0;
  origin_lla.altitude_m = 0.0;

  LlaPositionDegM east_point_lla;
  east_point_lla.latitude_deg = 0.0;
  east_point_lla.longitude_deg = 0.001;
  east_point_lla.altitude_m = 0.0;

  EnuPositionM enu;
  ASSERT_TRUE(TryLlaToEnu(east_point_lla, origin_lla, &enu));

  EXPECT_GT(enu.east_m, 100.0);
  EXPECT_NEAR(enu.north_m, 0.0, 1.0e-3);
  EXPECT_NEAR(enu.up_m, 0.0, 1.0e-3);
}

TEST(CoordinateTransformTest, InvalidInputReturnsFalse) {
  LlaPositionDegM invalid_lla;
  invalid_lla.latitude_deg = 95.0;
  invalid_lla.longitude_deg = 0.0;
  invalid_lla.altitude_m = 0.0;

  EcefPositionM ecef;
  EXPECT_FALSE(TryLlaToEcef(invalid_lla, &ecef));
  EXPECT_FALSE(TryLlaToEcef(invalid_lla, nullptr));

  LlaPositionDegM lla;
  EXPECT_FALSE(TryEcefToLla(EcefPositionM(), nullptr));
  EXPECT_FALSE(TryEcefToLla(EcefPositionM(), &lla));
}

TEST(CoordinateTransformTest, EnuUsesSemanticAxisNames) {
  EnuPositionM enu;
  enu.east_m = 12.25;
  enu.north_m = -4.5;
  enu.up_m = 99.0;

  EXPECT_DOUBLE_EQ(enu.east_m, 12.25);
  EXPECT_DOUBLE_EQ(enu.north_m, -4.5);
  EXPECT_DOUBLE_EQ(enu.up_m, 99.0);
}

TEST(CoordinateTransformTest, NueMappingMatchesEnuAxisReorder) {
  LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 0.0;
  origin_lla.longitude_deg = 0.0;
  origin_lla.altitude_m = 0.0;

  LlaPositionDegM east_point_lla;
  east_point_lla.latitude_deg = 0.0;
  east_point_lla.longitude_deg = 0.001;
  east_point_lla.altitude_m = 0.0;

  EnuPositionM enu;
  ASSERT_TRUE(TryLlaToEnu(east_point_lla, origin_lla, &enu));
  NuePositionM nue;
  ASSERT_TRUE(TryLlaToNue(east_point_lla, origin_lla, &nue));

  EXPECT_NEAR(nue.north_m, enu.north_m, 1.0e-6);
  EXPECT_NEAR(nue.up_m, enu.up_m, 1.0e-6);
  EXPECT_NEAR(nue.east_m, enu.east_m, 1.0e-6);
}

TEST(CoordinateTransformTest, NueUsesSemanticAxisNames) {
  NuePositionM nue;
  nue.north_m = -4.5;
  nue.up_m = 99.0;
  nue.east_m = 12.25;

  EXPECT_DOUBLE_EQ(nue.north_m, -4.5);
  EXPECT_DOUBLE_EQ(nue.up_m, 99.0);
  EXPECT_DOUBLE_EQ(nue.east_m, 12.25);
}

}  // namespace
}  // namespace coordinate
}  // namespace oneq
