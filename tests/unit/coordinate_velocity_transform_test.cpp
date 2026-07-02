/**
 * @file coordinate_velocity_transform_test.cpp
 * @brief 验证 oneq::coordinate 速度转换工具。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/coordinate/velocity_transform.h"

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

// =============================================================================
// IsFinite — 各速度类型的有限性校验
// =============================================================================

TEST(CoordinateVelocityTransformTest, IsFiniteAcceptsNormalEcefVelocity) {
  EcefVelocityMps v;
  v.x_mps = 1.0;
  v.y_mps = 2.0;
  v.z_mps = 3.0;
  EXPECT_TRUE(IsFinite(v));
}

TEST(CoordinateVelocityTransformTest, IsFiniteRejectsNanEcefVelocity) {
  EcefVelocityMps v;
  v.x_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(IsFinite(v));
}

TEST(CoordinateVelocityTransformTest, IsFiniteAcceptsNormalEnuVelocity) {
  EnuVelocityMps v;
  v.east_mps = 1.0;
  v.north_mps = 2.0;
  v.up_mps = 3.0;
  EXPECT_TRUE(IsFinite(v));
}

TEST(CoordinateVelocityTransformTest, IsFiniteRejectsInfNedVelocity) {
  NedVelocityMps v;
  v.down_mps = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(IsFinite(v));
}

TEST(CoordinateVelocityTransformTest, IsFiniteAcceptsNormalNueVelocity) {
  NueVelocityMps v;
  v.north_mps = 1.0;
  v.up_mps = 2.0;
  v.east_mps = 3.0;
  EXPECT_TRUE(IsFinite(v));
}

// =============================================================================
// ECEF → NED / NUE（经 ENU 中转的派生路径）
// =============================================================================

TEST(CoordinateVelocityTransformTest, EcefToNedVelocityMatchesEcefToEnuThenToNed) {
  const LlaPositionDegM origin = MakeValidOrigin(31.0, 121.0);
  EcefVelocityMps ecef;
  ecef.x_mps = 100.0;
  ecef.y_mps = 200.0;
  ecef.z_mps = 300.0;

  EnuVelocityMps enu;
  ASSERT_TRUE(TryEcefToEnuVelocity(ecef, origin, &enu));
  const NedVelocityMps ned_expected = ToNedVelocity(enu);

  NedVelocityMps ned;
  ASSERT_TRUE(TryEcefToNedVelocity(ecef, origin, &ned));
  EXPECT_NEAR(ned.north_mps, ned_expected.north_mps, kTolerance);
  EXPECT_NEAR(ned.east_mps, ned_expected.east_mps, kTolerance);
  EXPECT_NEAR(ned.down_mps, ned_expected.down_mps, kTolerance);
}

TEST(CoordinateVelocityTransformTest, EcefToNueVelocityMatchesEcefToEnuThenToNue) {
  const LlaPositionDegM origin = MakeValidOrigin(45.0, -70.0);
  EcefVelocityMps ecef;
  ecef.x_mps = 50.0;
  ecef.y_mps = -30.0;
  ecef.z_mps = 80.0;

  EnuVelocityMps enu;
  ASSERT_TRUE(TryEcefToEnuVelocity(ecef, origin, &enu));
  const NueVelocityMps nue_expected = ToNueVelocity(enu);

  NueVelocityMps nue;
  ASSERT_TRUE(TryEcefToNueVelocity(ecef, origin, &nue));
  EXPECT_NEAR(nue.north_mps, nue_expected.north_mps, kTolerance);
  EXPECT_NEAR(nue.up_mps, nue_expected.up_mps, kTolerance);
  EXPECT_NEAR(nue.east_mps, nue_expected.east_mps, kTolerance);
}

TEST(CoordinateVelocityTransformTest, EcefToNedVelocityRejectsNullOutput) {
  EcefVelocityMps ecef;
  const LlaPositionDegM origin = MakeValidOrigin(0.0, 0.0);
  EXPECT_FALSE(TryEcefToNedVelocity(ecef, origin, nullptr));
}

TEST(CoordinateVelocityTransformTest, EcefToNueVelocityRejectsNullOutput) {
  EcefVelocityMps ecef;
  const LlaPositionDegM origin = MakeValidOrigin(0.0, 0.0);
  EXPECT_FALSE(TryEcefToNueVelocity(ecef, origin, nullptr));
}

// =============================================================================
// ENU → ECEF（反向路径 + 往返一致性）
// =============================================================================

TEST(CoordinateVelocityTransformTest, EnuToEcefVelocityRoundTripsWithEcefToEnu) {
  const LlaPositionDegM origin = MakeValidOrigin(31.2304, 121.4737);
  EcefVelocityMps ecef_in;
  ecef_in.x_mps = 1000.0;
  ecef_in.y_mps = 2000.0;
  ecef_in.z_mps = 3000.0;

  EnuVelocityMps enu;
  ASSERT_TRUE(TryEcefToEnuVelocity(ecef_in, origin, &enu));

  EcefVelocityMps ecef_out;
  ASSERT_TRUE(TryEnuToEcefVelocity(enu, origin, &ecef_out));

  EXPECT_NEAR(ecef_out.x_mps, ecef_in.x_mps, 1.0e-6);
  EXPECT_NEAR(ecef_out.y_mps, ecef_in.y_mps, 1.0e-6);
  EXPECT_NEAR(ecef_out.z_mps, ecef_in.z_mps, 1.0e-6);
}

TEST(CoordinateVelocityTransformTest, EnuToEcefVelocityAtEquatorPrimeMeridian) {
  const LlaPositionDegM origin = MakeValidOrigin(0.0, 0.0);
  EnuVelocityMps enu;
  enu.east_mps = 1.0;
  enu.north_mps = 2.0;
  enu.up_mps = 3.0;

  EcefVelocityMps ecef;
  ASSERT_TRUE(TryEnuToEcefVelocity(enu, origin, &ecef));
  // 赤道+本初子午线下 ENU↔ECEF 各轴对齐：E→Y, N→Z, U→X
  EXPECT_NEAR(ecef.x_mps, 3.0, kTolerance);
  EXPECT_NEAR(ecef.y_mps, 1.0, kTolerance);
  EXPECT_NEAR(ecef.z_mps, 2.0, kTolerance);
}

TEST(CoordinateVelocityTransformTest, EnuToEcefVelocityRejectsInvalidInputs) {
  EnuVelocityMps enu;
  enu.east_mps = 1.0;
  enu.north_mps = 2.0;
  enu.up_mps = 3.0;

  // null 输出
  EXPECT_FALSE(TryEnuToEcefVelocity(enu, MakeValidOrigin(0.0, 0.0), nullptr));

  // 无效 origin
  LlaPositionDegM bad_origin;
  bad_origin.latitude_deg = 200.0;
  EcefVelocityMps ecef;
  EXPECT_FALSE(TryEnuToEcefVelocity(enu, bad_origin, &ecef));

  // NaN 速度
  EnuVelocityMps nan_enu;
  nan_enu.north_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(TryEnuToEcefVelocity(nan_enu, MakeValidOrigin(0.0, 0.0), &ecef));
}

}  // namespace
}  // namespace coordinate
}  // namespace oneq
