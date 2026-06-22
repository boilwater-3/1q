/**
 * @file sar_antenna_pattern_test.cpp
 * @brief 天线增益、方向图、合成孔径时间与分辨率的单元测试。
 */

#include "sar/geometry/SarAntenna.h"

#include <gtest/gtest.h>

#include <cmath>

namespace sar {
namespace geometry {
namespace {

AntennaParams MakeXbandAntenna() {
  AntennaParams a;
  a.length_m = 2.0;               // 2 m 方位天线
  a.width_m = 0.3;                // 0.3 m 距离天线
  a.peak_gain_linear = 1.0;
  a.beam_width_azimuth_rad = 0.015;  // ≈ 0.86° (λ=0.03/L=2)
  a.beam_width_range_rad = 0.1;      // ≈ 5.7°
  a.boresight_azimuth_rad = 0.0;
  return a;
}

}  // namespace

// ── AntennaGain ──────────────────────────────────────────────

TEST(SarAntennaPatternTest, GainMatchesPhysicalAperture) {
  const AntennaParams a = MakeXbandAntenna();
  const double wavelength_m = 0.03;
  const double gain = AntennaGain(a, wavelength_m);

  // G = 4π·L·W/λ² = 4π·2·0.3/0.0009 ≈ 12.566·0.6/0.0009 ≈ 8377
  const double expected = 4.0 * 3.141592653589793 * a.length_m * a.width_m /
                          (wavelength_m * wavelength_m);
  EXPECT_NEAR(gain, expected, 1.0e-6);
  EXPECT_GT(gain, 1000.0);
}

TEST(SarAntennaPatternTest, GainRejectsInvalidWavelength) {
  const AntennaParams a = MakeXbandAntenna();
  EXPECT_NEAR(AntennaGain(a, 0.0), 0.0, 1.0e-15);
  EXPECT_NEAR(AntennaGain(a, -0.03), 0.0, 1.0e-15);
}

// ── AzimuthPattern ───────────────────────────────────────────

TEST(SarAntennaPatternTest, AzimuthPatternPeakAtBoresight) {
  const AntennaParams a = MakeXbandAntenna();
  const double p0 = AzimuthPattern(a, 0.03, 0.0);
  EXPECT_NEAR(p0, 1.0, 1.0e-12);

  // 离轴应下降
  const double p_off = AzimuthPattern(a, 0.03, 0.01);
  EXPECT_LT(p_off, p0);
}

TEST(SarAntennaPatternTest, AzimuthPattern3dbWidth) {
  const AntennaParams a = MakeXbandAntenna();
  const double lambda = 0.03;
  // θ_3dB ≈ 0.886λ/L = 0.886·0.03/2 = 0.01329 rad
  const double expected_3db = 0.886 * lambda / a.length_m;

  // 在 θ=expected_3db/2 ≈ 0.006645 处, 方向图应 ≈ 0.5
  const double half_3db = expected_3db * 0.5;
  const double p_half = AzimuthPattern(a, lambda, half_3db);

  // sinc² 在 x=0.443 处 ≈ 0.5
  EXPECT_NEAR(p_half, 0.5, 0.05);
}

// ── SincPattern ──────────────────────────────────────────────

TEST(SarAntennaPatternTest, SincPatternPeakAtZero) {
  EXPECT_NEAR(SincPattern(1.0, 0.03, 0.0), 1.0, 1.0e-12);
}

TEST(SarAntennaPatternTest, SincPatternFirstNull) {
  // first null at sin(θ) = λ/L → θ ≈ λ/L for small θ
  const double theta_null = 0.03;  // λ/L = 0.03/1 = 0.03
  EXPECT_NEAR(SincPattern(1.0, 0.03, theta_null), 0.0, 1.0e-6);
}

// ── SyntheticApertureTime ────────────────────────────────────

TEST(SarAntennaPatternTest, SyntheticApertureTimeFormula) {
  const AntennaParams a = MakeXbandAntenna();
  // T_synth = R0 · θ_bw / v
  const double sat = SyntheticApertureTime(a, 1000.0, 100.0);
  // beam_width = 0.015 rad → T = 1000·0.015/100 = 0.15 s
  EXPECT_NEAR(sat, 0.15, 1.0e-12);
}

TEST(SarAntennaPatternTest, SyntheticApertureTimeRejectsInvalid) {
  const AntennaParams a = MakeXbandAntenna();
  EXPECT_NEAR(SyntheticApertureTime(a, 0.0, 100.0), 0.0, 1.0e-15);
  EXPECT_NEAR(SyntheticApertureTime(a, 1000.0, -100.0), 0.0, 1.0e-15);
}

// ── AntennaResolution ────────────────────────────────────────

TEST(SarAntennaPatternTest, SyntheticApertureResolutionIsHalfLength) {
  const AntennaParams a = MakeXbandAntenna();
  const double res = AntennaResolution(a, 1000.0, 0.03, true);
  EXPECT_NEAR(res, a.length_m * 0.5, 1.0e-12);
  EXPECT_NEAR(res, 1.0, 1.0e-12);  // L/2 = 1 m
}

TEST(SarAntennaPatternTest, RealApertureResolutionDependsOnRange) {
  const AntennaParams a = MakeXbandAntenna();
  // ρ = R·λ/L
  const double res_near = AntennaResolution(a, 500.0, 0.03, false);
  const double res_far = AntennaResolution(a, 5000.0, 0.03, false);
  // 远距分辨率更差
  EXPECT_GT(res_far, res_near);
  EXPECT_NEAR(res_near, 500.0 * 0.03 / 2.0, 1.0e-12);
}

}  // namespace geometry
}  // namespace sar
