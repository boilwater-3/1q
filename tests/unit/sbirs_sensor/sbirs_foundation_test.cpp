#include <cmath>

#include <gtest/gtest.h>

#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"
#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"

namespace {

TEST(SbirsFoundationTest, ReceivedPowerScalesLinearlyWithRadiantIntensity) {
  // P = I_t · A_ap · τ_opt · τ_atm · η / d²：辐射强度加倍 → 接收功率加倍。
  const double dim_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      1.0e4, 1.0e6, 0.5, 0.8, 0.8, 0.7);
  const double bright_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      2.0e4, 1.0e6, 0.5, 0.8, 0.8, 0.7);
  EXPECT_NEAR(bright_power, 2.0 * dim_power, 1.0e-12);
}

TEST(SbirsFoundationTest, ReceivedPowerAndSnrDecreaseWithRange) {
  sbirs_sensor::config::SbirsHardwareConfig hardware;
  hardware.noise_equivalent_power_w = 1.0e-15f;
  const double near_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      1.0e4, 1.0e6, 0.5, 0.8, 0.8, 0.7);
  const double far_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      1.0e4, 2.0e6, 0.5, 0.8, 0.8, 0.7);
  EXPECT_GT(near_power, far_power);
  EXPECT_GT(sbirs_sensor::foundation::ComputeInfraredSnrLinear(near_power, hardware),
            sbirs_sensor::foundation::ComputeInfraredSnrLinear(far_power, hardware));
}

TEST(SbirsFoundationTest, NegativeRadiantIntensityYieldsZeroPower) {
  EXPECT_DOUBLE_EQ(sbirs_sensor::foundation::ComputeReceivedPowerW(-1.0, 1.0e6, 0.5, 0.8, 0.8, 0.7),
                   0.0);
}

TEST(SbirsEnvironmentModelTest, WorseWeatherReducesEffectiveTransmittance) {
  sbirs_sensor::config::SbirsEnvironmentConfig clear;
  sbirs_sensor::config::SbirsEnvironmentConfig fog;
  fog.weather_type = sbirs_sensor::config::SbirsWeatherType::kFog;
  fog.relative_humidity_percent = 90.0f;
  fog.visibility_km = 0.5f;
  EXPECT_GT(sbirs_sensor::environment::ResolveEffectiveTransmittance(clear),
            sbirs_sensor::environment::ResolveEffectiveTransmittance(fog));
}

TEST(SbirsEarthOccultationTest, FiniteSegmentBehindEarthIsOcculted) {
  sbirs_sensor::session::SbirsVector3M sat;
  sat.x = 7000000.0;
  sbirs_sensor::session::SbirsVector3M target;
  target.x = -7000000.0;
  EXPECT_TRUE(sbirs_sensor::foundation::IsEarthOcculted(sat, target, 6371000.0));
}

TEST(SbirsEarthOccultationTest, OutwardLineOfSightIsNotOcculted) {
  sbirs_sensor::session::SbirsVector3M sat;
  sat.x = 7000000.0;
  sbirs_sensor::session::SbirsVector3M target;
  target.x = 8000000.0;
  EXPECT_FALSE(sbirs_sensor::foundation::IsEarthOcculted(sat, target, 6371000.0));
}

TEST(SbirsEarthOccultationTest, MarginIsNegativeWhenOcculted) {
  // 对侧地表上方（6.5e6 m < 地球半径 6.371e6 m）：LOS 穿过地球 → 余量为负（遮挡深度）。
  sbirs_sensor::session::SbirsVector3M sat;
  sat.x = 7000000.0;
  sbirs_sensor::session::SbirsVector3M target;
  target.x = -6500000.0;
  EXPECT_LT(sbirs_sensor::foundation::ComputeEarthOccultationMarginM(sat, target, 6371000.0),
            0.0);
}

TEST(SbirsEarthOccultationTest, MarginIsPositiveWhenClear) {
  // 同侧视线：余量为正。
  sbirs_sensor::session::SbirsVector3M sat;
  sat.x = 7000000.0;
  sbirs_sensor::session::SbirsVector3M target;
  target.x = 8000000.0;
  EXPECT_GT(sbirs_sensor::foundation::ComputeEarthOccultationMarginM(sat, target, 6371000.0),
            0.0);
}

TEST(SbirsRaySphereIntersectionTest, DownwardRayHitsEarthSurface) {
  // 卫星沿 -z 打向地表：最近正根交点在球面近侧（z 分量 +r，即可见地面点）。
  sbirs_sensor::session::SbirsVector3M sat;
  sat.z = 7000000.0;
  sbirs_sensor::session::SbirsVector3M down;
  down.z = -1.0;
  sbirs_sensor::session::SbirsVector3M hit;
  ASSERT_TRUE(sbirs_sensor::foundation::TryIntersectRayWithSphere(sat, down, 6371000.0, &hit));
  EXPECT_NEAR(std::sqrt(hit.x * hit.x + hit.y * hit.y + hit.z * hit.z), 6371000.0, 1.0e-3);
  EXPECT_NEAR(hit.z, 6371000.0, 1.0e-3);
}

TEST(SbirsRaySphereIntersectionTest, UpwardRayMissesEarth) {
  // 背离地球的射线无正根交点。
  sbirs_sensor::session::SbirsVector3M sat;
  sat.z = 7000000.0;
  sbirs_sensor::session::SbirsVector3M up;
  up.z = 1.0;
  sbirs_sensor::session::SbirsVector3M hit;
  EXPECT_FALSE(sbirs_sensor::foundation::TryIntersectRayWithSphere(sat, up, 6371000.0, &hit));
}

TEST(SbirsRaySphereIntersectionTest, NonUnitDirectionGivesSameIntersection) {
  // 方向无需单位化：非单位方向与单位方向交点一致。
  sbirs_sensor::session::SbirsVector3M sat;
  sat.z = 7000000.0;
  sbirs_sensor::session::SbirsVector3M slow;
  slow.z = -0.37;
  sbirs_sensor::session::SbirsVector3M hit_slow;
  sbirs_sensor::session::SbirsVector3M hit_unit;
  ASSERT_TRUE(sbirs_sensor::foundation::TryIntersectRayWithSphere(sat, slow, 6371000.0, &hit_slow));
  ASSERT_TRUE(sbirs_sensor::foundation::TryIntersectRayWithSphere(
      sat, sbirs_sensor::foundation::Unit(slow), 6371000.0, &hit_unit));
  EXPECT_NEAR(hit_slow.x, hit_unit.x, 1.0e-6);
  EXPECT_NEAR(hit_slow.y, hit_unit.y, 1.0e-6);
  EXPECT_NEAR(hit_slow.z, hit_unit.z, 1.0e-6);
}

TEST(SbirsGeocentricLatLonTest, KnownAxesMapToExpectedAngles) {
  // x 轴：赤道 0°N 0°E；z 轴：北极点；y 轴：赤道 90°E。
  sbirs_sensor::session::SbirsVector3M on_greenwich;
  on_greenwich.x = 6371000.0;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  sbirs_sensor::foundation::ComputeGeocentricLatLonDeg(on_greenwich, &lat_deg, &lon_deg);
  EXPECT_NEAR(lat_deg, 0.0, 1.0e-9);
  EXPECT_NEAR(lon_deg, 0.0, 1.0e-9);

  sbirs_sensor::session::SbirsVector3M on_north_pole;
  on_north_pole.z = 6371000.0;
  sbirs_sensor::foundation::ComputeGeocentricLatLonDeg(on_north_pole, &lat_deg, &lon_deg);
  EXPECT_NEAR(lat_deg, 90.0, 1.0e-9);

  sbirs_sensor::session::SbirsVector3M on_east_equator;
  on_east_equator.y = 6371000.0;
  sbirs_sensor::foundation::ComputeGeocentricLatLonDeg(on_east_equator, &lat_deg, &lon_deg);
  EXPECT_NEAR(lat_deg, 0.0, 1.0e-9);
  EXPECT_NEAR(lon_deg, 90.0, 1.0e-9);
}

TEST(SbirsGroundIntersectionTest, SpacePointingDirectionReportsMiss) {
  // 指向太空的角无法交会 → false（调用方以 miss 标记该角）。
  sbirs_sensor::session::SbirsVector3M sat;
  sat.z = 7000000.0;
  sbirs_sensor::session::SbirsVector3M up;
  up.z = 1.0;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  EXPECT_FALSE(sbirs_sensor::foundation::TryComputeGroundIntersectionLatLonDeg(
      sat, up, 6371000.0, 0.0, &lat_deg, &lon_deg));
}

TEST(SbirsGroundIntersectionTest, NadirIntersectionAtZeroGmstIsLatLonOfPoint) {
  // GMST=0 时 ECI 与 ECEF 重合：+x 轴卫星指向地心的交点即其星下点 (0°, 0°)。
  sbirs_sensor::session::SbirsVector3M sat;
  sat.x = 7000000.0;
  sbirs_sensor::session::SbirsVector3M down;
  down.x = -1.0;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  ASSERT_TRUE(sbirs_sensor::foundation::TryComputeGroundIntersectionLatLonDeg(
      sat, down, 6371000.0, 0.0, &lat_deg, &lon_deg));
  EXPECT_NEAR(lat_deg, 0.0, 1.0e-9);
  EXPECT_NEAR(lon_deg, 0.0, 1.0e-9);
}

TEST(SbirsFocalPlaneOffsetTest, ZeroAngularOffsetMapsToFocalCenter) {
  sbirs_sensor::foundation::SbirsFocalPlaneOffset offset;
  ASSERT_TRUE(
      sbirs_sensor::foundation::ComputeFocalPlaneOffset(2.0, 30.0e-6, 0.0f, 0.0f, &offset));
  EXPECT_DOUBLE_EQ(offset.x_m, 0.0);
  EXPECT_DOUBLE_EQ(offset.y_m, 0.0);
  EXPECT_DOUBLE_EQ(offset.x_pixels, 0.0);
  EXPECT_DOUBLE_EQ(offset.y_pixels, 0.0);
}

TEST(SbirsFocalPlaneOffsetTest, OffsetFollowsFocalLengthTanMapping) {
  // f=2 m、Δaz=0.01°：x = 2·tan(0.01°) ≈ 3.4907e-4 m ≈ 11.636 像素（30 μm 间距）。
  // 期望值按 float 入参的 double 化精确计算（避免 float 舍入差）。
  const double kDegToRad = 3.14159265358979323846 / 180.0;
  sbirs_sensor::foundation::SbirsFocalPlaneOffset offset;
  ASSERT_TRUE(
      sbirs_sensor::foundation::ComputeFocalPlaneOffset(2.0, 30.0e-6, 0.01f, -0.02f, &offset));
  EXPECT_NEAR(offset.x_m, 2.0 * std::tan(static_cast<double>(0.01f) * kDegToRad), 1.0e-15);
  EXPECT_NEAR(offset.y_m, 2.0 * std::tan(static_cast<double>(-0.02f) * kDegToRad), 1.0e-15);
  EXPECT_NEAR(offset.x_pixels, offset.x_m / 30.0e-6, 1.0e-9);
  EXPECT_NEAR(offset.y_pixels, offset.y_m / 30.0e-6, 1.0e-9);
  EXPECT_LT(offset.y_m, 0.0);
}

TEST(SbirsFocalPlaneOffsetTest, NonPositiveFocalParametersAreRejected) {
  sbirs_sensor::foundation::SbirsFocalPlaneOffset offset;
  EXPECT_FALSE(
      sbirs_sensor::foundation::ComputeFocalPlaneOffset(0.0, 30.0e-6, 0.0f, 0.0f, &offset));
  EXPECT_FALSE(sbirs_sensor::foundation::ComputeFocalPlaneOffset(2.0, 0.0, 0.0f, 0.0f, &offset));
}

}  // namespace
