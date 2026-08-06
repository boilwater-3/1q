/**
 * @file coordinate_position_transform_test.cpp
 * @brief 验证 oneq::coordinate 位置转换数值正确性与往返一致性。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/coordinate/position_transform.h"

namespace oneq {
namespace coordinate {
namespace {

constexpr double kTolerance = 1.0e-9;

LlaPositionDegM MakeValidOrigin(double lat_deg, double lon_deg) {
  LlaPositionDegM origin;
  origin.latitude_deg = lat_deg;
  origin.longitude_deg = lon_deg;
  origin.altitude_m = 0.0;
  return origin;
}

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

// =============================================================================
// IsFinite — 各位置类型的有限性校验
// =============================================================================

TEST(CoordinateTest, IsFiniteAcceptsNormalEcefPosition) {
  EcefPositionM ecef;
  ecef.x_m = 1.0;
  ecef.y_m = 2.0;
  ecef.z_m = 3.0;
  EXPECT_TRUE(IsFinite(ecef));
}

TEST(CoordinateTest, IsFiniteRejectsNanEcefPosition) {
  EcefPositionM ecef;
  ecef.x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(IsFinite(ecef));
}

TEST(CoordinateTest, IsFiniteAcceptsNormalEnuPosition) {
  EnuPositionM enu;
  enu.east_m = 1.0;
  enu.north_m = 2.0;
  enu.up_m = 3.0;
  EXPECT_TRUE(IsFinite(enu));
}

TEST(CoordinateTest, IsFiniteRejectsInfNedPosition) {
  NedPositionM ned;
  ned.down_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(IsFinite(ned));
}

TEST(CoordinateTest, IsFiniteRejectsNanNuePosition) {
  NuePositionM nue;
  nue.up_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(IsFinite(nue));
}

// =============================================================================
// IsValid — LLA 边界校验的完整分支
// =============================================================================

TEST(CoordinateTest, IsValidRejectsLatitudeOutOfRange) {
  LlaPositionDegM too_high;
  too_high.latitude_deg = 90.001;
  EXPECT_FALSE(IsValid(too_high));

  LlaPositionDegM too_low;
  too_low.latitude_deg = -90.001;
  EXPECT_FALSE(IsValid(too_low));
}

TEST(CoordinateTest, IsValidRejectsLongitudeOutOfRange) {
  LlaPositionDegM too_far_east;
  too_far_east.longitude_deg = 180.001;
  EXPECT_FALSE(IsValid(too_far_east));

  LlaPositionDegM too_far_west;
  too_far_west.longitude_deg = -180.001;
  EXPECT_FALSE(IsValid(too_far_west));
}

TEST(CoordinateTest, IsValidRejectsNanCoordinates) {
  LlaPositionDegM lla;
  lla.latitude_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(IsValid(lla));
}

TEST(CoordinateTest, IsValidAcceptsBoundaryValues) {
  LlaPositionDegM at_pole;
  at_pole.latitude_deg = 90.0;
  at_pole.longitude_deg = 180.0;
  EXPECT_TRUE(IsValid(at_pole));

  LlaPositionDegM at_antipode;
  at_antipode.latitude_deg = -90.0;
  at_antipode.longitude_deg = -180.0;
  EXPECT_TRUE(IsValid(at_antipode));
}

// =============================================================================
// ECEF → LLA 极点分支（cos_lat ≈ 0 走 alt_m 替代公式）
// =============================================================================

TEST(CoordinateTest, EcefToLlaNearPoleUsesAlternateAltitudePath) {
  // 北极点：lat=90° → cos_lat≈0，触发 alt_m 的 else 分支
  LlaPositionDegM north_pole;
  north_pole.latitude_deg = 90.0;
  north_pole.longitude_deg = 0.0;
  north_pole.altitude_m = 0.0;

  EcefPositionM ecef;
  ASSERT_TRUE(TryLlaToEcef(north_pole, &ecef));

  LlaPositionDegM recovered;
  ASSERT_TRUE(TryEcefToLla(ecef, &recovered));
  EXPECT_NEAR(recovered.latitude_deg, 90.0, 1.0e-6);
  EXPECT_NEAR(recovered.altitude_m, 0.0, 1.0e-3);
}

TEST(CoordinateTest, EcefToLlaRejectsNullOutput) {
  EcefPositionM ecef;
  ecef.x_m = 6378137.0;
  EXPECT_FALSE(TryEcefToLla(ecef, nullptr));
}

TEST(CoordinateTest, EcefToLlaRejectsZeroNorm) {
  // 原点 (0,0,0) 的 norm=0 <= kNormFloor → 返回 false
  EcefPositionM zero;
  zero.x_m = 0.0;
  zero.y_m = 0.0;
  zero.z_m = 0.0;
  LlaPositionDegM lla;
  EXPECT_FALSE(TryEcefToLla(zero, &lla));
}

// =============================================================================
// TryEcefToNue / TryLlaToNed / TryLlaToNue — 派生路径
// =============================================================================

TEST(CoordinateTest, EcefToNueMatchesEcefToEnuThenToNue) {
  const LlaPositionDegM origin = MakeValidOrigin(31.0, 121.0);
  LlaPositionDegM target = origin;
  target.latitude_deg += 0.001;

  EcefPositionM target_ecef;
  ASSERT_TRUE(TryLlaToEcef(target, &target_ecef));

  EnuPositionM enu;
  ASSERT_TRUE(TryEcefToEnu(target_ecef, origin, &enu));
  const NuePositionM nue_expected = ToNue(enu);

  NuePositionM nue;
  ASSERT_TRUE(TryEcefToNue(target_ecef, origin, &nue));
  EXPECT_NEAR(nue.north_m, nue_expected.north_m, kTolerance);
  EXPECT_NEAR(nue.up_m, nue_expected.up_m, kTolerance);
  EXPECT_NEAR(nue.east_m, nue_expected.east_m, kTolerance);
}

TEST(CoordinateTest, LlaToNedMatchesLlaToEnuThenToNed) {
  const LlaPositionDegM origin = MakeValidOrigin(34.0, 108.0);
  LlaPositionDegM target = origin;
  target.longitude_deg += 0.001;

  EnuPositionM enu;
  ASSERT_TRUE(TryLlaToEnu(target, origin, &enu));
  const NedPositionM ned_expected = ToNed(enu);

  NedPositionM ned;
  ASSERT_TRUE(TryLlaToNed(target, origin, &ned));
  EXPECT_NEAR(ned.north_m, ned_expected.north_m, kTolerance);
  EXPECT_NEAR(ned.east_m, ned_expected.east_m, kTolerance);
  EXPECT_NEAR(ned.down_m, ned_expected.down_m, kTolerance);
}

TEST(CoordinateTest, LlaToNueMatchesLlaToEnuThenToNue) {
  const LlaPositionDegM origin = MakeValidOrigin(40.0, -74.0);
  LlaPositionDegM target = origin;
  target.altitude_m = 500.0;

  EnuPositionM enu;
  ASSERT_TRUE(TryLlaToEnu(target, origin, &enu));
  const NuePositionM nue_expected = ToNue(enu);

  NuePositionM nue;
  ASSERT_TRUE(TryLlaToNue(target, origin, &nue));
  EXPECT_NEAR(nue.north_m, nue_expected.north_m, kTolerance);
  EXPECT_NEAR(nue.up_m, nue_expected.up_m, kTolerance);
  EXPECT_NEAR(nue.east_m, nue_expected.east_m, kTolerance);
}

TEST(CoordinateTest, DerivedConversionsRejectNullOutput) {
  const LlaPositionDegM origin = MakeValidOrigin(0.0, 0.0);
  EcefPositionM ecef;
  ASSERT_TRUE(TryLlaToEcef(origin, &ecef));

  EXPECT_FALSE(TryEcefToNue(ecef, origin, nullptr));
  EXPECT_FALSE(TryLlaToNed(origin, origin, nullptr));
  EXPECT_FALSE(TryLlaToNue(origin, origin, nullptr));
}

// =============================================================================
// TryEnuToEcef — 反向位置转换 + 往返一致性
// =============================================================================

TEST(CoordinateTest, EnuToEcefRoundTripsWithEcefToEnu) {
  const LlaPositionDegM origin = MakeValidOrigin(31.2304, 121.4737);

  EcefPositionM origin_ecef;
  ASSERT_TRUE(TryLlaToEcef(origin, &origin_ecef));

  EnuPositionM offset;
  offset.east_m = 1000.0;
  offset.north_m = 2000.0;
  offset.up_m = 300.0;

  EcefPositionM target_ecef;
  ASSERT_TRUE(TryEnuToEcef(offset, origin, &target_ecef));

  EnuPositionM recovered;
  ASSERT_TRUE(TryEcefToEnu(target_ecef, origin, &recovered));
  EXPECT_NEAR(recovered.east_m, offset.east_m, 1.0e-6);
  EXPECT_NEAR(recovered.north_m, offset.north_m, 1.0e-6);
  EXPECT_NEAR(recovered.up_m, offset.up_m, 1.0e-6);
}

TEST(CoordinateTest, EnuToEcefRejectsInvalidInputs) {
  EnuPositionM enu;
  enu.east_m = 100.0;
  enu.north_m = 200.0;
  enu.up_m = 300.0;

  EXPECT_FALSE(TryEnuToEcef(enu, MakeValidOrigin(0.0, 0.0), nullptr));

  LlaPositionDegM bad_origin;
  bad_origin.latitude_deg = 200.0;
  EcefPositionM ecef;
  EXPECT_FALSE(TryEnuToEcef(enu, bad_origin, &ecef));

  EnuPositionM nan_enu;
  nan_enu.east_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(TryEnuToEcef(nan_enu, MakeValidOrigin(0.0, 0.0), &ecef));
}

// =============================================================================
// TryEnuToEcefDirection — 方向向量转换（含归一化与零向量拒绝）
// =============================================================================

TEST(CoordinateTest, EnuToEcefDirectionAtEquatorPrimeMeridian) {
  const LlaPositionDegM origin = MakeValidOrigin(0.0, 0.0);
  Vector3d enu_up;
  enu_up.x = 0.0;
  enu_up.y = 0.0;
  enu_up.z = 1.0;

  Vector3d ecef_dir;
  ASSERT_TRUE(TryEnuToEcefDirection(enu_up, origin, &ecef_dir));
  // 赤道+本初子午线下 Up 方向 → ECEF X 轴
  EXPECT_NEAR(ecef_dir.x, 1.0, kTolerance);
  EXPECT_NEAR(ecef_dir.y, 0.0, kTolerance);
  EXPECT_NEAR(ecef_dir.z, 0.0, kTolerance);
}

TEST(CoordinateTest, EnuToEcefDirectionIsNormalized) {
  const LlaPositionDegM origin = MakeValidOrigin(45.0, 30.0);
  Vector3d enu_dir;
  enu_dir.x = 3.0;
  enu_dir.y = 4.0;
  enu_dir.z = 12.0;  // 模长 = 13

  Vector3d ecef_dir;
  ASSERT_TRUE(TryEnuToEcefDirection(enu_dir, origin, &ecef_dir));
  const double norm = std::sqrt(ecef_dir.x * ecef_dir.x + ecef_dir.y * ecef_dir.y +
                                ecef_dir.z * ecef_dir.z);
  EXPECT_NEAR(norm, 1.0, 1.0e-12);
}

TEST(CoordinateTest, EnuToEcefDirectionRejectsNullAndInvalidOrigin) {
  Vector3d dir;
  dir.x = 1.0;
  EXPECT_FALSE(TryEnuToEcefDirection(dir, MakeValidOrigin(0.0, 0.0), nullptr));

  LlaPositionDegM bad;
  bad.latitude_deg = 200.0;
  Vector3d out;
  EXPECT_FALSE(TryEnuToEcefDirection(dir, bad, &out));
}

TEST(CoordinateTest, BearingNorthYieldsPureNorthOffset) {
  EnuPositionM offset;
  ASSERT_TRUE(TryBearingRangeToEnuOffset(0.0, 1000.0, &offset));
  EXPECT_NEAR(offset.east_m, 0.0, kTolerance);
  EXPECT_NEAR(offset.north_m, 1000.0, kTolerance);
  EXPECT_NEAR(offset.up_m, 0.0, kTolerance);
}

TEST(CoordinateTest, BearingEastYieldsPureEastOffset) {
  EnuPositionM offset;
  ASSERT_TRUE(TryBearingRangeToEnuOffset(90.0, 1000.0, &offset));
  EXPECT_NEAR(offset.east_m, 1000.0, kTolerance);
  EXPECT_NEAR(offset.north_m, 0.0, kTolerance);
  EXPECT_NEAR(offset.up_m, 0.0, kTolerance);
}

TEST(CoordinateTest, BearingSplitsOffsetAtFortyFiveDegrees) {
  EnuPositionM offset;
  ASSERT_TRUE(TryBearingRangeToEnuOffset(45.0, 1000.0, &offset));
  const double expected = 1000.0 * std::sqrt(0.5);
  EXPECT_NEAR(offset.east_m, expected, kTolerance);
  EXPECT_NEAR(offset.north_m, expected, kTolerance);
  EXPECT_NEAR(offset.up_m, 0.0, kTolerance);
}

TEST(CoordinateTest, BearingSouthFlipsNorthSign) {
  EnuPositionM offset;
  ASSERT_TRUE(TryBearingRangeToEnuOffset(180.0, 1000.0, &offset));
  EXPECT_NEAR(offset.east_m, 0.0, kTolerance);
  EXPECT_NEAR(offset.north_m, -1000.0, kTolerance);
  EXPECT_NEAR(offset.up_m, 0.0, kTolerance);
}

TEST(CoordinateTest, BearingRangeZeroYieldsZeroOffset) {
  EnuPositionM offset;
  ASSERT_TRUE(TryBearingRangeToEnuOffset(30.0, 0.0, &offset));
  EXPECT_NEAR(offset.east_m, 0.0, kTolerance);
  EXPECT_NEAR(offset.north_m, 0.0, kTolerance);
  EXPECT_NEAR(offset.up_m, 0.0, kTolerance);
}

TEST(CoordinateTest, BearingRangeRejectsInvalidInputs) {
  EnuPositionM offset;

  EXPECT_FALSE(TryBearingRangeToEnuOffset(0.0, 1000.0, nullptr));
  EXPECT_FALSE(TryBearingRangeToEnuOffset(0.0, -1000.0, &offset));
  EXPECT_FALSE(TryBearingRangeToEnuOffset(
      std::numeric_limits<double>::quiet_NaN(), 1000.0, &offset));
  EXPECT_FALSE(TryBearingRangeToEnuOffset(
      0.0, std::numeric_limits<double>::quiet_NaN(), &offset));
}

}  // namespace
}  // namespace coordinate
}  // namespace oneq
