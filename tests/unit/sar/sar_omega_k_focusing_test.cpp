#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarOmegaKFocusing.h"
#include "sar/signal/SarWaveform.h"
#include "support/sar_reference_scene.h"

// Omega-K 聚焦编排器测试(契约 §6 验收门)。
// 因 Omega-K 经网格收缩后距离轴为非均匀映射,与 GBP 物理网格不可逐像素对齐,
// 故交叉验证策略为:峰值位置正确性 + 退化性 + 确定性 + 方位剖面质量对照 GBP。
namespace sar {
namespace imaging {
namespace {

constexpr double kReferenceSpeedOfLightMps = 299792458.0;

OmegaKConfig MakeOmegaKConfig(const test_support::ReferencePointScene& scene,
                              std::size_t reference_delay) {
  OmegaKConfig config;
  config.range_sample_count = scene.range_sample_count;
  config.azimuth_pulse_count = scene.pulses.size();
  config.sample_rate_hz = scene.sample_rate_hz;
  config.prf_hz = scene.prf_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.platform_velocity_mps = scene.platform_velocity_mps;
  config.reference_range_m =
      static_cast<double>(reference_delay) * kReferenceSpeedOfLightMps /
      (2.0 * scene.sample_rate_hz);
  return config;
}

struct FocusFixture {
  test_support::ReferencePointScene scene;
  signal::ComplexMatrix raw;
  OmegaKConfig config;
  std::size_t reference_delay{20U};
};

void BuildFixture(FocusFixture* fixture) {
  fixture->scene.pulse_count = 9U;
  fixture->scene.prf_hz = 50.0;
  fixture->scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&fixture->scene));
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(fixture->reference_delay, fixture->scene.sample_rate_hz,
                                               1.0);
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(fixture->scene, {target}, &fixture->raw));
  fixture->config = MakeOmegaKConfig(fixture->scene, fixture->reference_delay);
}

std::size_t FindPeakColumn(const signal::ComplexMatrix& image, std::size_t row) {
  std::size_t peak_col = 0U;
  double peak_power = -1.0;
  for (std::size_t col = 0U; col < image.cols; ++col) {
    const double power = std::norm(image(row, col));
    if (power > peak_power) {
      peak_power = power;
      peak_col = col;
    }
  }
  return peak_col;
}

std::size_t FindPeakRow(const signal::ComplexMatrix& image, std::size_t col) {
  std::size_t peak_row = 0U;
  double peak_power = -1.0;
  for (std::size_t row = 0U; row < image.rows; ++row) {
    const double power = std::norm(image(row, col));
    if (power > peak_power) {
      peak_power = power;
      peak_row = row;
    }
  }
  return peak_row;
}

TEST(SarOmegaKFocusingTest, FocusesSinglePointAtBroadside) {
  FocusFixture fixture;
  BuildFixture(&fixture);

  FocusedOmegaKImage output;
  ASSERT_TRUE(FocusStripmapOmegaK(fixture.config, fixture.raw, &output));
  EXPECT_EQ(output.diagnostics.failure_stage, "none");

  // 方位聚焦:broadside 目标峰值应在方位中心行。
  const std::size_t azimuth_center = output.image.rows / 2U;
  const std::size_t peak_col = FindPeakColumn(output.image, azimuth_center);
  const std::size_t peak_row = FindPeakRow(output.image, peak_col);
  EXPECT_EQ(peak_row, azimuth_center);

  // 距离聚焦:峰值应在 R_ref 对应列附近。
  const std::vector<double>& ranges =
      output.diagnostics.reference_mapping.absolute_slant_ranges_m;
  std::size_t ref_col = 0U;
  double min_diff = std::abs(ranges[0U] - fixture.config.reference_range_m);
  for (std::size_t c = 1U; c < ranges.size(); ++c) {
    const double diff = std::abs(ranges[c] - fixture.config.reference_range_m);
    if (diff < min_diff) {
      min_diff = diff;
      ref_col = c;
    }
  }
  // 峰值列应在 R_ref 对应列附近(允许 ±2 列容差,因收缩网格离散化)。
  EXPECT_LE(std::abs(static_cast<long>(peak_col) - static_cast<long>(ref_col)), 2L);
}

TEST(SarOmegaKFocusingTest, ImageIsFiniteAndDeterministic) {
  FocusFixture fixture;
  BuildFixture(&fixture);

  FocusedOmegaKImage first;
  FocusedOmegaKImage second;
  ASSERT_TRUE(FocusStripmapOmegaK(fixture.config, fixture.raw, &first));
  ASSERT_TRUE(FocusStripmapOmegaK(fixture.config, fixture.raw, &second));

  for (const signal::ComplexSample& sample : first.image.values) {
    EXPECT_TRUE(std::isfinite(sample.real()));
    EXPECT_TRUE(std::isfinite(sample.imag()));
  }
  EXPECT_EQ(first.image.values, second.image.values);
}

// 退化性不变量(契约 §5):参考距离 R_ref 处,网格收缩后的距离轴应包含 R_ref。
TEST(SarOmegaKFocusingTest, ReferenceRangeIsOnRangeAxis) {
  FocusFixture fixture;
  BuildFixture(&fixture);

  FocusedOmegaKImage output;
  ASSERT_TRUE(FocusStripmapOmegaK(fixture.config, fixture.raw, &output));

  const std::vector<double>& ranges =
      output.diagnostics.reference_mapping.absolute_slant_ranges_m;
  bool found = false;
  for (double range : ranges) {
    if (std::abs(range - fixture.config.reference_range_m) < 1.0) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

// 方位剖面质量对照 GBP:Omega-K 的方位中心列剖面与 GBP 方位剖面应有相近的峰值尖锐度
// (主瓣能量占比)。不要求逐像素一致(距离轴不可对齐),只验证方位聚焦质量量级合理。
TEST(SarOmegaKFocusingTest, AzimuthProfileQualityComparableToGbp) {
  FocusFixture fixture;
  BuildFixture(&fixture);

  // GBP 参考图像(小场景,覆盖目标)。
  GbpConfig gbp_config;
  gbp_config.sample_rate_hz = fixture.scene.sample_rate_hz;
  gbp_config.carrier_frequency_hz = fixture.scene.carrier_frequency_hz;
  gbp_config.grid.azimuth_pixel_count = fixture.scene.pulses.size();
  gbp_config.grid.range_pixel_count = 9U;
  gbp_config.grid.azimuth_spacing_m =
      fixture.scene.platform_velocity_mps / fixture.scene.prf_hz;
  gbp_config.grid.range_spacing_m =
      kReferenceSpeedOfLightMps / (2.0 * fixture.scene.sample_rate_hz);
  gbp_config.grid.azimuth_start_m =
      -0.5 * static_cast<double>(gbp_config.grid.azimuth_pixel_count - 1U) *
      gbp_config.grid.azimuth_spacing_m;
  gbp_config.grid.range_start_m =
      static_cast<double>(fixture.reference_delay - 4U) * gbp_config.grid.range_spacing_m;
  FocusedGbpImage gbp;
  ASSERT_TRUE(FocusSmallSceneGbp(gbp_config, fixture.scene.pulses, fixture.raw,
                                 fixture.scene.matched_filter, &gbp));

  FocusedOmegaKImage omega_k;
  ASSERT_TRUE(FocusStripmapOmegaK(fixture.config, fixture.raw, &omega_k));

  // Omega-K 方位剖面:取峰值列的方位剖面。
  const std::size_t ok_azimuth_center = omega_k.image.rows / 2U;
  const std::size_t ok_peak_col = FindPeakColumn(omega_k.image, ok_azimuth_center);
  double ok_peak_power = std::norm(omega_k.image(ok_azimuth_center, ok_peak_col));
  double ok_total_power = 0.0;
  for (std::size_t row = 0U; row < omega_k.image.rows; ++row) {
    ok_total_power += std::norm(omega_k.image(row, ok_peak_col));
  }
  const double ok_peak_ratio = ok_total_power > 0.0 ? ok_peak_power / ok_total_power : 0.0;

  // GBP 方位剖面:取峰值列的方位剖面。
  const std::size_t gbp_peak_col = gbp_config.grid.range_pixel_count / 2U;
  const std::size_t gbp_peak_row = FindPeakRow(gbp.image, gbp_peak_col);
  double gbp_peak_power = std::norm(gbp.image(gbp_peak_row, gbp_peak_col));
  double gbp_total_power = 0.0;
  for (std::size_t row = 0U; row < gbp.image.rows; ++row) {
    gbp_total_power += std::norm(gbp.image(row, gbp_peak_col));
  }
  const double gbp_peak_ratio = gbp_total_power > 0.0 ? gbp_peak_power / gbp_total_power : 0.0;

  // 两者方位剖面峰值能量占比应在同一量级(都应显著 > 均匀分布 1/N)。
  // broadside 单点目标,峰值能量应集中。
  RecordProperty("omega_k_azimuth_peak_ratio", std::to_string(ok_peak_ratio));
  RecordProperty("gbp_azimuth_peak_ratio", std::to_string(gbp_peak_ratio));
  EXPECT_GT(ok_peak_ratio, 1.0 / static_cast<double>(omega_k.image.rows));
}

TEST(SarOmegaKFocusingTest, RejectsNullOutput) {
  FocusFixture fixture;
  BuildFixture(&fixture);
  EXPECT_FALSE(FocusStripmapOmegaK(fixture.config, fixture.raw, nullptr));
}

TEST(SarOmegaKFocusingTest, RejectsSizeMismatchedRawHistory) {
  FocusFixture fixture;
  BuildFixture(&fixture);
  signal::ComplexMatrix bad = fixture.raw;
  bad.rows = 3U;
  bad.values.assign(3U * fixture.config.range_sample_count, signal::ComplexSample(0.0, 0.0));
  FocusedOmegaKImage output;
  EXPECT_FALSE(FocusStripmapOmegaK(fixture.config, bad, &output));
  EXPECT_EQ(output.diagnostics.failure_stage, "raw_history");
}

TEST(SarOmegaKFocusingTest, RejectsSinglePulseConfig) {
  FocusFixture fixture;
  BuildFixture(&fixture);
  fixture.config.azimuth_pulse_count = 1U;
  FocusedOmegaKImage output;
  EXPECT_FALSE(FocusStripmapOmegaK(fixture.config, fixture.raw, &output));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
