/**
 * @file coordinate_position_transform_test.cpp
 * @brief 验证 oneq::coordinate 位置转换数值正确性与往返一致性。
 */

#include <gtest/gtest.h>

#include "1q/coordinate/position_transform.h"

namespace oneq {
namespace coordinate {
namespace {

// =============================================================================
// LLA <-> ECEF
// =============================================================================

TEST(CoordinateTest, LlaToEcefAtEquatorPrimeMeridian) {
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

TEST(CoordinateTest, LlaEcefRoundTrip) {
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

TEST(CoordinateTest, InvalidLlaRejected) {
  LlaPositionDegM bad;
  bad.latitude_deg = 100.0;
  EXPECT_FALSE(IsValid(bad));
  EXPECT_FALSE(TryLlaToEcef(bad, nullptr));
}

TEST(CoordinateTest, NullOutputRejected) {
  LlaPositionDegM lla;
  EXPECT_FALSE(TryLlaToEcef(lla, nullptr));
  EcefPositionM ecef;
  EXPECT_FALSE(TryEcefToLla(ecef, nullptr));
}

// =============================================================================
// ECEF -> 局部坐标系
// =============================================================================

TEST(CoordinateTest, EcefToEnuAtOriginReturnsZero) {
  LlaPositionDegM origin;
  origin.latitude_deg = 34.2;
  origin.longitude_deg = 108.9;
  origin.altitude_m = 500.0;
  EcefPositionM origin_ecef;
  ASSERT_TRUE(TryLlaToEcef(origin, &origin_ecef));

  EnuPositionM enu;
  ASSERT_TRUE(TryEcefToEnu(origin_ecef, origin, &enu));
  EXPECT_NEAR(enu.east_m, 0.0, 1.0e-6);
  EXPECT_NEAR(enu.north_m, 0.0, 1.0e-6);
  EXPECT_NEAR(enu.up_m, 0.0, 1.0e-6);
}

TEST(CoordinateTest, EcefToEnuPositiveEast) {
  LlaPositionDegM origin;
  origin.latitude_deg = 0.0;
  origin.longitude_deg = 0.0;
  origin.altitude_m = 0.0;

  LlaPositionDegM target = origin;
  target.longitude_deg = 0.001;

  EcefPositionM target_ecef;
  ASSERT_TRUE(TryLlaToEcef(target, &target_ecef));

  EnuPositionM enu;
  ASSERT_TRUE(TryEcefToEnu(target_ecef, origin, &enu));
  EXPECT_GT(enu.east_m, 100.0);
  EXPECT_NEAR(enu.north_m, 0.0, 1.0e-1);
}

TEST(CoordinateTest, EcefToNedPositiveNorth) {
  LlaPositionDegM origin;
  origin.latitude_deg = 0.0;
  origin.longitude_deg = 0.0;
  origin.altitude_m = 0.0;

  LlaPositionDegM target = origin;
  target.latitude_deg = 0.001;

  EcefPositionM target_ecef;
  ASSERT_TRUE(TryLlaToEcef(target, &target_ecef));

  NedPositionM ned;
  ASSERT_TRUE(TryEcefToNed(target_ecef, origin, &ned));
  EXPECT_GT(ned.north_m, 100.0);
}

// =============================================================================
// LLA -> 局部坐标系
// =============================================================================

TEST(CoordinateTest, LlaToEnuEquivalentToEcefPath) {
  LlaPositionDegM origin;
  origin.latitude_deg = 31.0;
  origin.longitude_deg = 121.0;
  origin.altitude_m = 100.0;

  LlaPositionDegM target = origin;
  target.longitude_deg += 0.001;

  EnuPositionM enu_via_lla;
  ASSERT_TRUE(TryLlaToEnu(target, origin, &enu_via_lla));

  EcefPositionM target_ecef;
  ASSERT_TRUE(TryLlaToEcef(target, &target_ecef));
  EnuPositionM enu_via_ecef;
  ASSERT_TRUE(TryEcefToEnu(target_ecef, origin, &enu_via_ecef));

  EXPECT_NEAR(enu_via_lla.east_m, enu_via_ecef.east_m, 1.0e-6);
  EXPECT_NEAR(enu_via_lla.north_m, enu_via_ecef.north_m, 1.0e-6);
  EXPECT_NEAR(enu_via_lla.up_m, enu_via_ecef.up_m, 1.0e-6);
}

// =============================================================================
// 局部坐标系互转
// =============================================================================

TEST(CoordinateTest, EnuNedRoundTrip) {
  EnuPositionM enu;
  enu.east_m = 100.0;
  enu.north_m = 200.0;
  enu.up_m = 300.0;

  const NedPositionM ned = ToNed(enu);
  EXPECT_DOUBLE_EQ(ned.north_m, 200.0);
  EXPECT_DOUBLE_EQ(ned.east_m, 100.0);
  EXPECT_DOUBLE_EQ(ned.down_m, -300.0);

  const EnuPositionM back = ToEnu(ned);
  EXPECT_DOUBLE_EQ(back.east_m, enu.east_m);
  EXPECT_DOUBLE_EQ(back.north_m, enu.north_m);
  EXPECT_DOUBLE_EQ(back.up_m, enu.up_m);
}

TEST(CoordinateTest, EnuNueRoundTrip) {
  EnuPositionM enu;
  enu.east_m = 1.0;
  enu.north_m = 2.0;
  enu.up_m = 3.0;

  const NuePositionM nue = ToNue(enu);
  EXPECT_DOUBLE_EQ(nue.north_m, 2.0);
  EXPECT_DOUBLE_EQ(nue.up_m, 3.0);
  EXPECT_DOUBLE_EQ(nue.east_m, 1.0);

  const EnuPositionM back = ToEnu(nue);
  EXPECT_DOUBLE_EQ(back.east_m, enu.east_m);
  EXPECT_DOUBLE_EQ(back.north_m, enu.north_m);
  EXPECT_DOUBLE_EQ(back.up_m, enu.up_m);
}

TEST(CoordinateTest, NedNueRoundTrip) {
  NedPositionM ned;
  ned.north_m = 10.0;
  ned.east_m = 20.0;
  ned.down_m = 30.0;

  const NuePositionM nue = ToNue(ned);
  const NedPositionM back = ToNed(nue);
  EXPECT_DOUBLE_EQ(back.north_m, ned.north_m);
  EXPECT_DOUBLE_EQ(back.east_m, ned.east_m);
  EXPECT_DOUBLE_EQ(back.down_m, ned.down_m);
}

}  // namespace
}  // namespace coordinate
}  // namespace oneq
