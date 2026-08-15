/**
 * @file sbirs_eci_transform_test.cpp
 * @brief ECI 输出参考系（2026-08 正式变更）单元测试：
 *        - 共享坐标域 GMST/ECEF→ECI 转换（1q/coordinate/inertial_transform.h）；
 *        - pipeline 周期入口 ECI 旋转的端到端效果（az 平移 GMST、el 不变）；
 *        - 输出弧度约定 az∈[0, 2π)、el∈[-π/2, π/2]；
 *        - scan_start_az 合法域 [0, 360) 的配置校验。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/coordinate/inertial_transform.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "sbirs_sensor/pipeline/SbirsPipeline.h"
#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
// GMST≈0（1.3e-7 deg 精度）：ECI≡ECEF，测试几何期望不变。
constexpr double kGmstZeroJulianDay = 2451544.2230698913;

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_sensor::session::SbirsSceneTarget HotTarget(std::uint64_t id, double y) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = id;
  target.target_name = "eci_target";
  target.position_ecef_m = Vector(8000000.0, y, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e8;
  return target;
}

sbirs_sensor::config::SbirsSessionConfig PipelineConfig() {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;
  return config;
}

sbirs_sensor::session::SbirsCycleInput InputAt(double utc_julian_day, std::uint32_t cycle_index) {
  return sbirs_sensor::session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithUtcJulianDay(utc_julian_day)
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .AddTarget(HotTarget(1U, 0.0))
      .Build();
}

}  // namespace

TEST(SbirsEciTransformTest, GmstMatchesValladoReferenceAtJ2000) {
  // Vallado 式 3-47：JD 2451545.0（J2000.0 正午）→ GMST = 280.46061837°。
  double gmst_rad = 0.0;
  ASSERT_TRUE(oneq::coordinate::TryComputeGmstRad(2451545.0, &gmst_rad));
  EXPECT_NEAR(gmst_rad, 280.46061837 * kDegToRad, 1.0e-9);
  // 范围 [0, 2π)。
  EXPECT_GE(gmst_rad, 0.0);
  EXPECT_LT(gmst_rad, 2.0 * kPi);
}

TEST(SbirsEciTransformTest, GmstRejectsInvalidJulianDay) {
  double gmst_rad = 0.0;
  EXPECT_FALSE(oneq::coordinate::TryComputeGmstRad(0.0, &gmst_rad));
  EXPECT_FALSE(oneq::coordinate::TryComputeGmstRad(-1.0, &gmst_rad));
  EXPECT_FALSE(oneq::coordinate::TryComputeGmstRad(std::numeric_limits<double>::quiet_NaN(),
                                                   &gmst_rad));
  EXPECT_FALSE(oneq::coordinate::TryComputeGmstRad(2451545.0, nullptr));
}

TEST(SbirsEciTransformTest, EcefToEciRotatesAboutZAxisPreservingNorm) {
  constexpr double kRadiusM = 6378137.0;
  const oneq::coordinate::EcefPositionM ecef(kRadiusM, 0.0, 1000.0);
  const double gmst_rad = 123.456789 * kDegToRad;

  oneq::coordinate::EciPositionM eci;
  ASSERT_TRUE(oneq::coordinate::TryEcefToEci(ecef, gmst_rad, &eci));

  // 绕 z 旋转：x' = x·cosθ − y·sinθ, y' = x·sinθ + y·cosθ, z' = z。
  EXPECT_NEAR(eci.x_m, kRadiusM * std::cos(gmst_rad), 1.0e-9);
  EXPECT_NEAR(eci.y_m, kRadiusM * std::sin(gmst_rad), 1.0e-9);
  EXPECT_DOUBLE_EQ(eci.z_m, ecef.z_m);
  // 模长保持。
  const double ecef_norm = std::sqrt(ecef.x_m * ecef.x_m + ecef.y_m * ecef.y_m + ecef.z_m * ecef.z_m);
  const double eci_norm = std::sqrt(eci.x_m * eci.x_m + eci.y_m * eci.y_m + eci.z_m * eci.z_m);
  EXPECT_DOUBLE_EQ(eci_norm, ecef_norm);

  // 零 GMST：两系重合。
  oneq::coordinate::EciPositionM eci_zero;
  ASSERT_TRUE(oneq::coordinate::TryEcefToEci(ecef, 0.0, &eci_zero));
  EXPECT_DOUBLE_EQ(eci_zero.x_m, ecef.x_m);
  EXPECT_DOUBLE_EQ(eci_zero.y_m, ecef.y_m);
  EXPECT_DOUBLE_EQ(eci_zero.z_m, ecef.z_m);
}

TEST(SbirsEciTransformTest, EciToEcefIsInverseOfEcefToEci) {
  const oneq::coordinate::EcefPositionM ecef(7000000.0, -250000.0, 1234567.0);
  const double gmst_rad = 77.7 * kDegToRad;

  oneq::coordinate::EciPositionM eci;
  oneq::coordinate::EcefPositionM round_trip;
  ASSERT_TRUE(oneq::coordinate::TryEcefToEci(ecef, gmst_rad, &eci));
  ASSERT_TRUE(oneq::coordinate::TryEciToEcef(eci, gmst_rad, &round_trip));
  EXPECT_DOUBLE_EQ(round_trip.x_m, ecef.x_m);
  EXPECT_DOUBLE_EQ(round_trip.y_m, ecef.y_m);
  EXPECT_DOUBLE_EQ(round_trip.z_m, ecef.z_m);
}

TEST(SbirsEciTransformTest, EcefVelocityToEciIncludesTransportTerm) {
  // 地面静止点（ECEF 速度 0）在 ECI 中具有 ω_e × r 的东向速度 ≈ 465 m/s。
  constexpr double kRadiusM = 6378137.0;
  const oneq::coordinate::EcefPositionM ecef_position(kRadiusM, 0.0, 0.0);
  const oneq::coordinate::EcefVelocityMps ecef_velocity(0.0, 0.0, 0.0);
  // GMST=0 时刻 ECI≡ECEF：输运项在原点位置 (R,0,0) 处沿 +y（东向）。
  const double gmst_rad = 0.0;

  oneq::coordinate::EciVelocityMps eci_velocity;
  ASSERT_TRUE(
      oneq::coordinate::TryEcefVelocityToEci(ecef_position, ecef_velocity, gmst_rad,
                                             &eci_velocity));
  // 在 GMST=0 参考系中：位置 (R,0,0)，ω×r = (−ω·y, ω·x, 0) = (0, ωR, 0)。
  const double omega = oneq::coordinate::kEarthRotationRateRadPerSec;
  EXPECT_NEAR(eci_velocity.x_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(eci_velocity.y_mps, omega * kRadiusM, 1.0e-6);
  EXPECT_NEAR(eci_velocity.y_mps, 465.1, 0.5);
  EXPECT_DOUBLE_EQ(eci_velocity.z_mps, 0.0);
}

TEST(SbirsEciTransformTest, PipelineRotatesSceneIntoEciAtCycleEntry) {
  // 同一 ECEF 场景（目标在卫星正东，ECEF az=0）在两个时刻各以新 pipeline 运行：
  // GMST≈0 时刻输出 az≈0；GMST≈77.7° 时刻输出 az≈GMST（绕 z 旋转仅平移方位）。
  sbirs_sensor::pipeline::SbirsPipeline base_pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(PipelineConfig()));
  const sbirs_sensor::pipeline::SbirsPipelineResult base =
      base_pipeline.RunCycle(InputAt(kGmstZeroJulianDay, 1U));
  ASSERT_EQ(base.detections.size(), 1U);
  EXPECT_NEAR(base.detections.front().record.azimuth_rad, 0.0, 1.0e-5);

  // GMST ≈ 77.7° 的 JD（d = 77.7/360.98564736629）。
  const double jd_shift = 2451545.0 + 77.7 / 360.98564736629;
  double gmst_rad = 0.0;
  ASSERT_TRUE(oneq::coordinate::TryComputeGmstRad(jd_shift, &gmst_rad));
  sbirs_sensor::pipeline::SbirsPipeline shifted_pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(PipelineConfig()));
  const sbirs_sensor::pipeline::SbirsPipelineResult shifted =
      shifted_pipeline.RunCycle(InputAt(jd_shift, 1U));
  ASSERT_EQ(shifted.detections.size(), 1U);
  EXPECT_NEAR(shifted.detections.front().record.azimuth_rad, gmst_rad, 1.0e-5);
  // el 对绕 z 旋转不变（目标在赤道面，el=0）。
  EXPECT_NEAR(shifted.detections.front().record.elevation_rad, 0.0, 1.0e-5);
}

TEST(SbirsEciTransformTest, OutputAnglesFollowEciRadConvention) {
  // 输出契约：az∈[0, 2π)、el∈[-π/2, π/2]；负方位折入 [0, 2π)。
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.scan_start_az_deg = 180.0f;  // 扫描中心对准正西目标（az 180°）
  config.mission.scan_span_deg = 360.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  // 目标在卫星正西（ECEF az = 180°→ 输出 π rad）。
  sbirs_sensor::session::SbirsSceneTarget west = HotTarget(1U, 0.0);
  west.position_ecef_m = Vector(6000000.0, 0.0, 0.0);
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(kGmstZeroJulianDay)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(west)
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_EQ(result.detections.size(), 1U);
  EXPECT_NEAR(result.detections.front().record.azimuth_rad, kPi, 1.0e-5);
  EXPECT_GE(result.detections.front().record.azimuth_rad, 0.0f);
  EXPECT_LT(result.detections.front().record.azimuth_rad, 2.0f * static_cast<float>(kPi));
  EXPECT_GE(result.detections.front().record.elevation_rad, -0.5f * static_cast<float>(kPi));
  EXPECT_LE(result.detections.front().record.elevation_rad, 0.5f * static_cast<float>(kPi));
}

TEST(SbirsEciTransformTest, ScanStartAzimuthValidationUsesEciRange) {
  // scan_start_az 为 ECI 方位，合法域 [0, 360)（2026-08 正式变更）。
  sbirs_sensor::config::SbirsSessionConfig valid = PipelineConfig();
  valid.mission.scan_start_az_deg = 359.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(valid).empty());

  sbirs_sensor::config::SbirsSessionConfig negative = valid;
  negative.mission.scan_start_az_deg = -1.0f;
  EXPECT_FALSE(sbirs_sensor::config::ValidateSbirsSessionConfig(negative).empty());

  sbirs_sensor::config::SbirsSessionConfig too_large = valid;
  too_large.mission.scan_start_az_deg = 360.0f;
  EXPECT_FALSE(sbirs_sensor::config::ValidateSbirsSessionConfig(too_large).empty());
}
