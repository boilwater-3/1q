// 验收旁路 Sinclair S：纯共极化、纯交叉、缺 has_*、φ_vv=90°。

#include "remote_identification_radar/runtime/PolarizationAcceptanceS.h"

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace remote_identification_radar {
namespace runtime {
namespace {

session::RirPolarizationRcsSample MakeSample(float az_deg, float el_deg, float hh_dbsm,
                                             float vv_dbsm, bool has_cross, float cross_dbsm,
                                             bool has_phase, float phase_deg) {
  session::RirPolarizationRcsSample sample;
  sample.aspect_az_deg = az_deg;
  sample.aspect_el_deg = el_deg;
  sample.channel_1_rcs_dbsm = hh_dbsm;
  sample.channel_2_rcs_dbsm = vv_dbsm;
  sample.has_cross_pol = has_cross;
  sample.cross_rcs_dbsm = cross_dbsm;
  sample.has_phase_vv = has_phase;
  sample.phase_vv_rel_hh_deg = phase_deg;
  return sample;
}

TEST(RirPolarizationAcceptanceSTest, CopolarEqualNoCrossDepolZeroAbsDetEqualsSigma) {
  std::vector<session::RirPolarizationRcsSample> samples;
  // has_* 必须为真才能构 S；-4000 dBsm 在 double 下溢出为 0，表示无交叉功率。
  samples.push_back(MakeSample(0.0f, 0.0f, 0.0f, 0.0f, true, -4000.0f, true, 0.0f));
  PolarizationAcceptanceSResult result;
  ASSERT_TRUE(TryResolvePolarizationAcceptanceS(samples, 0.0f, 0.0f, &result));
  EXPECT_NEAR(result.span, 2.0, 1.0e-12);
  EXPECT_NEAR(result.abs_det, 1.0, 1.0e-12);
  EXPECT_NEAR(result.depolarization, 0.0, 1.0e-15);
  EXPECT_NEAR(result.psi_deg, 0.0, 1.0e-12);
  EXPECT_NEAR(result.tau_deg, 0.0, 1.0e-12);
}

TEST(RirPolarizationAcceptanceSTest, CrossOnlySpanIsTwiceSigmaHv) {
  std::vector<session::RirPolarizationRcsSample> samples;
  samples.push_back(MakeSample(10.0f, 5.0f, -4000.0f, -4000.0f, true, 0.0f, true, 0.0f));
  PolarizationAcceptanceSResult result;
  ASSERT_TRUE(TryResolvePolarizationAcceptanceS(samples, 10.0f, 5.0f, &result));
  EXPECT_NEAR(result.span, 2.0, 1.0e-12);
  EXPECT_NEAR(result.abs_det, 1.0, 1.0e-12);
  EXPECT_NEAR(result.depolarization, 1.0, 1.0e-12);
}

TEST(RirPolarizationAcceptanceSTest, MissingHasFlagsLeavesOutputUntouched) {
  PolarizationAcceptanceSResult result;
  result.span = 123.0;
  result.abs_det = 456.0;
  result.depolarization = 0.5;
  result.psi_deg = 12.0;
  result.tau_deg = 8.0;
  std::vector<session::RirPolarizationRcsSample> samples;
  samples.push_back(MakeSample(0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, false, 90.0f));
  EXPECT_FALSE(TryResolvePolarizationAcceptanceS(samples, 0.0f, 0.0f, &result));
  EXPECT_DOUBLE_EQ(result.span, 123.0);
  EXPECT_DOUBLE_EQ(result.abs_det, 456.0);
  EXPECT_DOUBLE_EQ(result.depolarization, 0.5);
  EXPECT_DOUBLE_EQ(result.psi_deg, 12.0);
  EXPECT_DOUBLE_EQ(result.tau_deg, 8.0);

  samples[0].has_cross_pol = true;
  EXPECT_FALSE(TryResolvePolarizationAcceptanceS(samples, 0.0f, 0.0f, &result));
  EXPECT_DOUBLE_EQ(result.span, 123.0);
  EXPECT_FALSE(TryResolvePolarizationAcceptanceS(samples, 0.0f, 0.0f, nullptr));
}

TEST(RirPolarizationAcceptanceSTest, NinetyDegreePhaseYieldsNonZeroTau) {
  std::vector<session::RirPolarizationRcsSample> samples;
  samples.push_back(MakeSample(0.0f, 0.0f, 0.0f, 0.0f, true, -3.0f, true, 90.0f));
  PolarizationAcceptanceSResult result;
  ASSERT_TRUE(TryResolvePolarizationAcceptanceS(samples, 1.0f, -1.0f, &result));
  EXPECT_NE(result.tau_deg, 0.0);
  EXPECT_TRUE(std::isfinite(result.tau_deg));
  EXPECT_GT(std::fabs(result.tau_deg), 1.0e-6);
}

}  // namespace
}  // namespace runtime
}  // namespace remote_identification_radar
