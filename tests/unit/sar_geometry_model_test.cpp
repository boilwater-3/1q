/**
 * @file sar_geometry_model_test.cpp
 * @brief 斜距模型、多普勒模型、Sinc 与 DeterministicGaussianSampler 的单元测试。
 */

#include "sar/geometry/SarGeometry.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace sar {
namespace geometry {
namespace {

PlatformPulseState MakePlatformAt(double x_m, double y_m, double z_m, double vx_mps = 100.0) {
  PlatformPulseState p;
  p.pulse_id = 0U;
  p.time_s = 0.0;
  p.position_m = LocalPoint{x_m, y_m, z_m};
  p.velocity_x_mps = vx_mps;
  return p;
}

LocalPoint MakeTarget(double x_m, double y_m, double z_m) {
  return LocalPoint{x_m, y_m, z_m};
}

}  // namespace

// ── Sinc ──────────────────────────────────────────────────────

TEST(SarGeometryModelTest, SincAtZeroIsOne) {
  EXPECT_NEAR(Sinc(0.0), 1.0, 1.0e-15);
  EXPECT_NEAR(Sinc(1.0e-14), 1.0, 1.0e-15);
}

TEST(SarGeometryModelTest, SincAtOneIsZero) {
  EXPECT_NEAR(Sinc(1.0), 0.0, 1.0e-15);
  EXPECT_NEAR(Sinc(2.0), 0.0, 1.0e-15);
}

TEST(SarGeometryModelTest, SincIsSymmetric) {
  EXPECT_NEAR(Sinc(0.5), Sinc(-0.5), 1.0e-15);
  EXPECT_NEAR(Sinc(1.7), Sinc(-1.7), 1.0e-15);
}

// ── 斜距模型 ──────────────────────────────────────────────────

TEST(SarGeometryModelTest, ExactSlantRangeMatchesDistance) {
  const PlatformPulseState p = MakePlatformAt(0.0, 0.0, 0.0);
  const LocalPoint t = MakeTarget(3.0, 4.0, 0.0);
  EXPECT_NEAR(ExactSlantRange(p, t), 5.0, 1.0e-15);
}

TEST(SarGeometryModelTest, ClosestSlantRangeReturnsMin) {
  std::vector<PlatformPulseState> track;
  track.push_back(MakePlatformAt(0.0, 0.0, 0.0));
  track.push_back(MakePlatformAt(10.0, 0.0, 0.0));
  track.push_back(MakePlatformAt(20.0, 0.0, 0.0));
  const LocalPoint t = MakeTarget(9.0, 5.0, 0.0);

  // 距离: 到 [0,0,0] = sqrt(81+25) ≈ 9.22; 到 [10,0,0] = sqrt(1+25) ≈ 5.10; 到 [20,0,0] = sqrt(121+25) ≈ 12.08
  const double closest = ClosestSlantRange(track, t);
  EXPECT_NEAR(closest, std::sqrt(26.0), 1.0e-12);
}

TEST(SarGeometryModelTest, ClosestSlantRangeEmptyReturnsZero) {
  std::vector<PlatformPulseState> empty;
  EXPECT_NEAR(ClosestSlantRange(empty, MakeTarget(1.0, 2.0, 3.0)), 0.0, 1.0e-15);
}

TEST(SarGeometryModelTest, QuadraticApproxNearBroadsideIsClose) {
  QuadraticRangeApprox approx;
  approx.reference_range_m = 1000.0;
  approx.broadside_time_s = 5.0;
  approx.platform_velocity_mps = 100.0;

  // R(t) ≈ R0 + 0.5·(v²/R0)·(t-t0)²
  // t=5.0 → R=1000.0
  EXPECT_NEAR(QuadraticApproxRange(approx, 5.0), 1000.0, 1.0e-12);
  // t=5.1 → R ≈ 1000 + 0.5*(10000/1000)*0.01 = 1000 + 0.05 = 1000.05
  EXPECT_NEAR(QuadraticApproxRange(approx, 5.1), 1000.0 + 0.5 * 10.0 * 0.01, 1.0e-12);
}

TEST(SarGeometryModelTest, RangeRateMatchesFiniteDifference) {
  const PlatformPulseState p = MakePlatformAt(0.0, 1000.0, 0.0, 100.0);
  const LocalPoint t = MakeTarget(0.0, 0.0, 0.0);
  // 平台沿 x 飞行, 目标在原点 y=0。视线方向: 从平台指向目标 = (0, -1000, 0)
  // 速度 = (100, 0, 0), 两者点积 = 0 → 瞬时距离变化率接近 0
  EXPECT_NEAR(RangeRate(p, t), 0.0, 1.0e-12);

  // 目标在 [1000, 1000, 0], 平台在 [0, 1000, 0], v=(100,0,0)
  // (platform-target)·v = (-1000, 0, 0)·(100, 0, 0) / 1000 = -100 m/s (闭合)
  const LocalPoint t2 = MakeTarget(1000.0, 1000.0, 0.0);
  const double rr = RangeRate(p, t2);
  EXPECT_LT(rr, 0.0);   // 闭合: 距离率 < 0
  EXPECT_NEAR(rr, -100.0, 1.0e-12);
  EXPECT_GT(rr, -101.0);
}

// ── 多普勒模型 ────────────────────────────────────────────────

DopplerComputationInput MakeBroadsideInput() {
  DopplerComputationInput in;
  in.wavelength_m = 0.03;          // X-band
  in.platform_velocity_mps = 100.0;
  in.reference_slant_range_m = 1000.0;
  in.squint_angle_rad = 0.0;       // broadside
  in.real_aperture_length_m = 2.0; // 2 m antenna
  return in;
}

TEST(SarGeometryModelTest, ComputeDopplerParamsBroadside) {
  DopplerParams p;
  ASSERT_TRUE(ComputeDopplerParams(MakeBroadsideInput(), &p));

  // broadside → fd_central = 0
  EXPECT_NEAR(p.fd_central_hz, 0.0, 1.0e-15);

  // fd_rate = 2·v²/(λ·R0) = 2*10000/(0.03*1000) = 20000/30 ≈ 666.67
  const double expected_rate = 2.0 * 100.0 * 100.0 / (0.03 * 1000.0);
  EXPECT_NEAR(p.fd_rate_hz_per_s, expected_rate, 1.0e-6);

  // 合成孔径时间 = R0·θ_bw/v, θ_bw = λ/L
  const double beam_width_rad = 0.03 / 2.0;
  const double expected_sat = 1000.0 * beam_width_rad / 100.0;
  EXPECT_NEAR(p.synthetic_aperture_time_s, expected_sat, 1.0e-12);

  // doppler_bandwidth = |fd_rate| * synthetic_aperture_time
  EXPECT_NEAR(p.doppler_bandwidth_hz,
              std::abs(p.fd_rate_hz_per_s) * p.synthetic_aperture_time_s, 1.0e-12);
}

TEST(SarGeometryModelTest, ComputeDopplerParamsRejectsInvalid) {
  DopplerParams p;
  DopplerComputationInput bad;

  bad = MakeBroadsideInput();
  bad.wavelength_m = 0.0;
  EXPECT_FALSE(ComputeDopplerParams(bad, &p));

  bad = MakeBroadsideInput();
  bad.platform_velocity_mps = 0.0;
  EXPECT_FALSE(ComputeDopplerParams(bad, &p));

  bad = MakeBroadsideInput();
  bad.reference_slant_range_m = 0.0;
  EXPECT_FALSE(ComputeDopplerParams(bad, &p));

  EXPECT_FALSE(ComputeDopplerParams(MakeBroadsideInput(), nullptr));
}

TEST(SarGeometryModelTest, DopplerFrequencyAtIsLinear) {
  DopplerParams p;
  ASSERT_TRUE(ComputeDopplerParams(MakeBroadsideInput(), &p));

  // fd(t) = fd_central + fd_rate * t
  EXPECT_NEAR(DopplerFrequencyAt(p, 0.0), p.fd_central_hz, 1.0e-15);
  EXPECT_NEAR(DopplerFrequencyAt(p, 0.1), p.fd_central_hz + p.fd_rate_hz_per_s * 0.1, 1.0e-12);
}

TEST(SarGeometryModelTest, AzimuthResolutionFormula) {
  DopplerParams p;
  ASSERT_TRUE(ComputeDopplerParams(MakeBroadsideInput(), &p));

  const double res = AzimuthResolution(p, 100.0);
  EXPECT_NEAR(res, 100.0 / p.doppler_bandwidth_hz, 1.0e-12);
  EXPECT_GT(res, 0.0);
}

TEST(SarGeometryModelTest, AzimuthResolutionZeroBandwidthReturnsZero) {
  DopplerParams p{};
  p.doppler_bandwidth_hz = 0.0;
  EXPECT_NEAR(AzimuthResolution(p, 100.0), 0.0, 1.0e-15);
}

TEST(SarGeometryModelTest, DopplerBinFrequencySymmetry) {
  const double prf = 1000.0;
  const std::size_t n = 256U;

  // index 0 → 0 Hz
  EXPECT_NEAR(DopplerBinFrequency(0U, n, prf), 0.0, 1.0e-12);
  // index N/2 → PRF/2
  EXPECT_NEAR(DopplerBinFrequency(n / 2U, n, prf), 500.0, 1.0e-12);
  // index N/2+1 → -PRF/2 (negative side)
  EXPECT_LT(DopplerBinFrequency(n / 2U + 1U, n, prf), 0.0);
  // symmetry: f(k) = -f(N-k) for k>0
  const double f1 = DopplerBinFrequency(10U, n, prf);
  const double f2 = DopplerBinFrequency(n - 10U, n, prf);
  EXPECT_NEAR(f1, -f2, 1.0e-12);
}

TEST(SarGeometryModelTest, DopplerBinFrequencyRejectsInvalid) {
  EXPECT_NEAR(DopplerBinFrequency(0U, 0U, 1000.0), 0.0, 1.0e-15);
}

// ── DeterministicGaussianSampler ──────────────────────────────

TEST(SarGeometryModelTest, GaussianSamplerReproducible) {
  DeterministicGaussianSampler s1(2026U);
  DeterministicGaussianSampler s2(2026U);

  for (int i = 0; i < 100; ++i) {
    EXPECT_NEAR(s1.Sample(), s2.Sample(), 1.0e-15);
  }
}

TEST(SarGeometryModelTest, GaussianSamplerDifferentSeedsDiverge) {
  DeterministicGaussianSampler s1(42U);
  DeterministicGaussianSampler s2(99U);

  bool diverged = false;
  for (int i = 0; i < 100; ++i) {
    if (std::abs(s1.Sample() - s2.Sample()) > 1.0e-12) {
      diverged = true;
      break;
    }
  }
  EXPECT_TRUE(diverged);
}

TEST(SarGeometryModelTest, GaussianSamplerFiniteAll) {
  DeterministicGaussianSampler s(2026U);
  for (int i = 0; i < 1000; ++i) {
    const double v = s.Sample();
    EXPECT_TRUE(std::isfinite(v));
  }
}

}  // namespace geometry
}  // namespace sar
