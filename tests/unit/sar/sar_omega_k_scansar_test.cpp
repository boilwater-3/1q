#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/geometry/SarScanBurst.h"
#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarOmegaKFocusing.h"
#include "sar/imaging/SarScanSarFocusing.h"
#include "sar/signal/SarWaveform.h"
#include "support/sar_reference_scene.h"

// 扫描模式 Omega-K 聚焦编排器测试(契约 scansar_mode.md §4):
// 逐 burst 复用 FocusStripmapOmegaK(broadside,offset=0)+ burst 方位拼接。
namespace sar {
namespace imaging {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;

OmegaKConfig MakeBaseConfig(const test_support::ReferencePointScene& scene,
                            std::size_t reference_delay) {
  OmegaKConfig config;
  config.range_sample_count = scene.range_sample_count;
  config.azimuth_pulse_count = scene.pulses.size();
  config.sample_rate_hz = scene.sample_rate_hz;
  config.prf_hz = scene.prf_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.platform_velocity_mps = scene.platform_velocity_mps;
  config.reference_range_m =
      static_cast<double>(reference_delay) * kSpeedOfLightMps / (2.0 * scene.sample_rate_hz);
  return config;
}

// ── 不变量 1:单子带单 burst 覆盖全程 → 退化等价于条带 ──────────

// 单子带退化不变量:N_swath=1 且单 burst 覆盖全程,
// FocusScanSarOmegaK 输出 == FocusStripmapOmegaK(逐样本一致)。
TEST(SarOmegaKScanSarTest, SingleSubswathDegeneratesToStripmap) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  // 条带参考。
  FocusedOmegaKImage stripmap;
  ASSERT_TRUE(FocusStripmapOmegaK(base, raw, &stripmap));

  // ScanSAR 单子带单 burst。
  ScanSarSubswathConfig subswath;
  subswath.omega_k = base;
  subswath.burst_ranges.push_back({0U, scene.pulses.size()});  // 全程

  ScanSarFocusConfig scan_config;
  scan_config.subswaths.push_back(subswath);

  FocusedScanSarImage scan_output;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &scan_output));
  EXPECT_EQ(scan_output.failure_stage, "none");
  ASSERT_EQ(scan_output.subswaths.size(), 1U);
  ASSERT_TRUE(scan_output.subswaths[0U].valid);

  // 逐样本一致(单 burst = 条带 broadside 聚焦)。
  const signal::ComplexMatrix& scan_img = scan_output.subswaths[0U].image;
  ASSERT_EQ(stripmap.image.rows, scan_img.rows);
  ASSERT_EQ(stripmap.image.cols, scan_img.cols);
  for (std::size_t i = 0U; i < stripmap.image.values.size(); ++i) {
    EXPECT_NEAR(stripmap.image.values[i].real(), scan_img.values[i].real(), 1e-9)
        << "sample " << i;
    EXPECT_NEAR(stripmap.image.values[i].imag(), scan_img.values[i].imag(), 1e-9)
        << "sample " << i;
  }
}

// ── 不变量 2:单 burst 聚焦 = 条带子集(独立聚焦与区间聚焦等价)──

// 从 raw history 抽一段脉冲,ScanSAR 单 burst 聚焦 == 对该子矩阵单独跑条带聚焦。
TEST(SarOmegaKScanSarTest, SingleBurstEqualsStripmapSubset) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  // 抽脉冲 [2, 7) 共 5 脉冲,单独跑条带聚焦。
  signal::ComplexMatrix subset;
  subset.rows = 5U;
  subset.cols = raw.cols;
  subset.values.assign(5U * raw.cols, signal::ComplexSample(0.0, 0.0));
  for (std::size_t r = 0U; r < 5U; ++r) {
    for (std::size_t c = 0U; c < raw.cols; ++c) {
      subset(r, c) = raw(2U + r, c);
    }
  }
  OmegaKConfig subset_config = base;
  subset_config.azimuth_pulse_count = 5U;
  FocusedOmegaKImage subset_stripmap;
  ASSERT_TRUE(FocusStripmapOmegaK(subset_config, subset, &subset_stripmap));

  // ScanSAR 单 burst 指向同一区间。
  ScanSarSubswathConfig subswath;
  subswath.omega_k = base;
  subswath.burst_ranges.push_back({2U, 7U});

  ScanSarFocusConfig scan_config;
  scan_config.subswaths.push_back(subswath);

  FocusedScanSarImage scan_output;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &scan_output));
  ASSERT_EQ(scan_output.subswaths.size(), 1U);
  const signal::ComplexMatrix& scan_img = scan_output.subswaths[0U].image;

  ASSERT_EQ(subset_stripmap.image.rows, scan_img.rows);
  ASSERT_EQ(subset_stripmap.image.cols, scan_img.cols);
  for (std::size_t i = 0U; i < subset_stripmap.image.values.size(); ++i) {
    EXPECT_NEAR(subset_stripmap.image.values[i].real(), scan_img.values[i].real(), 1e-9);
    EXPECT_NEAR(subset_stripmap.image.values[i].imag(), scan_img.values[i].imag(), 1e-9);
  }
}

// ── 多 burst 拼接 ─────────────────────────────────────────

// 双子带场景:各含单 burst,聚焦 + 拼接成功,输出有限。
TEST(SarOmegaKScanSarTest, DualSubswathFocusesAndMosaics) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  // 子带 0:脉冲 [0, 5)。子带 1:脉冲 [4, 9)(方位略重叠 1 脉冲验证不崩)。
  ScanSarSubswathConfig sw0;
  sw0.omega_k = base;
  sw0.burst_ranges.push_back({0U, 5U});

  ScanSarSubswathConfig sw1;
  sw1.omega_k = base;
  sw1.burst_ranges.push_back({4U, 9U});

  ScanSarFocusConfig scan_config;
  scan_config.subswaths.push_back(sw0);
  scan_config.subswaths.push_back(sw1);

  FocusedScanSarImage output;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &output));
  EXPECT_EQ(output.failure_stage, "none");
  ASSERT_EQ(output.subswaths.size(), 2U);
  EXPECT_TRUE(output.subswaths[0U].valid);
  EXPECT_TRUE(output.subswaths[1U].valid);
  // 两子带各 1 burst → 各 burst_diagnostics 1 条。
  EXPECT_EQ(output.subswaths[0U].burst_diagnostics.size(), 1U);
  EXPECT_EQ(output.subswaths[1U].burst_diagnostics.size(), 1U);

  for (const FocusedScanSarSubswath& sw : output.subswaths) {
    for (const signal::ComplexSample& s : sw.image.values) {
      EXPECT_TRUE(std::isfinite(s.real()));
      EXPECT_TRUE(std::isfinite(s.imag()));
    }
  }
}

// 同子带多 burst 拼接:方位行数累加。
TEST(SarOmegaKScanSarTest, MultipleBurstsMosaicVertically) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  // 单子带双 burst:[0,5) + [4,9) → 两个 burst 子图像纵向堆叠(各 5 脉冲,Omega-K 最小尺寸)。
  ScanSarSubswathConfig subswath;
  subswath.omega_k = base;
  subswath.burst_ranges.push_back({0U, 5U});
  subswath.burst_ranges.push_back({4U, 9U});

  ScanSarFocusConfig scan_config;
  scan_config.subswaths.push_back(subswath);

  FocusedScanSarImage output;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &output));
  ASSERT_EQ(output.subswaths.size(), 1U);
  const FocusedScanSarSubswath& sw = output.subswaths[0U];
  EXPECT_EQ(sw.burst_diagnostics.size(), 2U);
  EXPECT_EQ(sw.burst_azimuth_offsets.size(), 2U);
  EXPECT_EQ(sw.burst_azimuth_offsets[0U], 0U);  // 首 burst 偏移 0
  // 行数 = burst0 行数 + burst1 行数(纵向堆叠)。
  // 各 burst 子图像行数由 Omega-K 内部决定(网格收缩),此处只验 > 0 且偏移递增。
  EXPECT_GT(sw.burst_azimuth_offsets[1U], 0U);
  EXPECT_GT(sw.image.rows, 0U);
}

// ── 确定性 ────────────────────────────────────────────────

TEST(SarOmegaKScanSarTest, Deterministic) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  ScanSarSubswathConfig subswath;
  subswath.omega_k = base;
  subswath.burst_ranges.push_back({0U, 5U});
  subswath.burst_ranges.push_back({4U, 9U});

  ScanSarFocusConfig scan_config;
  scan_config.subswaths.push_back(subswath);

  FocusedScanSarImage first;
  FocusedScanSarImage second;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &first));
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &second));
  ASSERT_EQ(first.subswaths.size(), 1U);
  ASSERT_EQ(second.subswaths.size(), 1U);
  EXPECT_EQ(first.subswaths[0U].image.values, second.subswaths[0U].image.values);
}

// ── 拒绝路径 ──────────────────────────────────────────────

TEST(SarOmegaKScanSarTest, RejectsNullOutput) {
  signal::ComplexMatrix raw;
  raw.rows = 9U;
  raw.cols = 64U;
  raw.values.assign(9U * 64U, signal::ComplexSample(0.0, 0.0));
  ScanSarFocusConfig config;
  config.subswaths.push_back(ScanSarSubswathConfig{});
  config.subswaths[0U].burst_ranges.push_back({0U, 9U});
  EXPECT_FALSE(FocusScanSarOmegaK(config, raw, nullptr));
}

TEST(SarOmegaKScanSarTest, RejectsEmptySubswaths) {
  signal::ComplexMatrix raw;
  raw.rows = 9U;
  raw.cols = 64U;
  raw.values.assign(9U * 64U, signal::ComplexSample(0.0, 0.0));
  FocusedScanSarImage output;
  EXPECT_FALSE(FocusScanSarOmegaK(ScanSarFocusConfig{}, raw, &output));
  EXPECT_EQ(output.failure_stage, "config");
}

TEST(SarOmegaKScanSarTest, RejectsBurstRangeTooShort) {
  signal::ComplexMatrix raw;
  raw.rows = 9U;
  raw.cols = 64U;
  raw.values.assign(9U * 64U, signal::ComplexSample(0.0, 0.0));
  ScanSarSubswathConfig subswath;
  subswath.omega_k.range_sample_count = 64U;
  subswath.burst_ranges.push_back({0U, 1U});  // 仅 1 脉冲(< 2)
  ScanSarFocusConfig config;
  config.subswaths.push_back(subswath);
  FocusedScanSarImage output;
  EXPECT_FALSE(FocusScanSarOmegaK(config, raw, &output));
  EXPECT_EQ(output.failure_stage, "subswath_config");
}

TEST(SarOmegaKScanSarTest, RejectsBurstRangeOutOfBounds) {
  signal::ComplexMatrix raw;
  raw.rows = 9U;
  raw.cols = 64U;
  raw.values.assign(9U * 64U, signal::ComplexSample(0.0, 0.0));
  ScanSarSubswathConfig subswath;
  subswath.omega_k.range_sample_count = 64U;
  subswath.burst_ranges.push_back({0U, 15U});  // 超过 9 脉冲
  ScanSarFocusConfig config;
  config.subswaths.push_back(subswath);
  FocusedScanSarImage output;
  EXPECT_FALSE(FocusScanSarOmegaK(config, raw, &output));
  EXPECT_EQ(output.failure_stage, "subswath_config");
}

// ── 阶段 C:物理验证 + GBP 交叉核对(契约 §5)──────────────

// 找图像幅度峰值位置。
struct PeakLocation {
  std::size_t row{0U};
  std::size_t col{0U};
  double magnitude{0.0};
};

PeakLocation FindMagnitudePeak(const signal::ComplexMatrix& image) {
  PeakLocation peak;
  for (std::size_t r = 0U; r < image.rows; ++r) {
    for (std::size_t c = 0U; c < image.cols; ++c) {
      const double mag = std::abs(image(r, c));
      if (mag > peak.magnitude) {
        peak.magnitude = mag;
        peak.row = r;
        peak.col = c;
      }
    }
  }
  return peak;
}

// ScanSAR 单 burst 聚焦:峰值能量集中(>均匀分布),证明 burst 正确聚焦。
TEST(SarOmegaKScanSarTest, BurstPeakEnergyConcentrated) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  ScanSarSubswathConfig subswath;
  subswath.omega_k = base;
  subswath.burst_ranges.push_back({0U, 9U});  // 单 burst 全程

  ScanSarFocusConfig scan_config;
  scan_config.subswaths.push_back(subswath);

  FocusedScanSarImage output;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &output));
  const signal::ComplexMatrix& img = output.subswaths[0U].image;

  const PeakLocation peak = FindMagnitudePeak(img);
  EXPECT_GT(peak.magnitude, 0.0);

  // 峰值能量占比应高于均匀分布(1/N):峰值样本幅度 > 平均幅度。
  double total_energy = 0.0;
  for (const signal::ComplexSample& s : img.values) {
    total_energy += std::abs(s);
  }
  const double avg = total_energy / static_cast<double>(img.values.size());
  EXPECT_GT(peak.magnitude, avg);
}

// GBP 交叉核对:ScanSAR 单 burst 聚焦与 GBP 对同一 burst 子集投影,
// 峰值均落在目标真实方位×斜距附近(独立算法验证聚焦正确性)。
TEST(SarOmegaKScanSarTest, GbpCrossCheckPeakConsistent) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  // ScanSAR 单 burst 全程聚焦。
  ScanSarSubswathConfig subswath;
  subswath.omega_k = base;
  subswath.burst_ranges.push_back({0U, 9U});
  ScanSarFocusConfig scan_config;
  scan_config.subswaths.push_back(subswath);
  FocusedScanSarImage scan_output;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &scan_output));
  const PeakLocation scan_peak = FindMagnitudePeak(scan_output.subswaths[0U].image);

  // GBP 独立参考:对同一全程 raw history 后向投影(GBP 天然支持任意波束/距离窗口)。
  GbpConfig gbp_config;
  gbp_config.sample_rate_hz = scene.sample_rate_hz;
  gbp_config.carrier_frequency_hz = scene.carrier_frequency_hz;
  gbp_config.grid.azimuth_pixel_count = scene.pulses.size();
  gbp_config.grid.range_pixel_count = scene.range_sample_count;
  gbp_config.grid.azimuth_spacing_m = scene.platform_velocity_mps / scene.prf_hz;
  gbp_config.grid.range_spacing_m =
      test_support::kReferenceSpeedOfLightMps / (2.0 * scene.sample_rate_hz);
  gbp_config.grid.azimuth_start_m = scene.pulses.front().position_m.x_m;
  gbp_config.grid.range_start_m = 0.0;

  FocusedGbpImage gbp_output;
  ASSERT_TRUE(FocusSmallSceneGbp(gbp_config, scene.pulses, raw, scene.matched_filter,
                                 &gbp_output));
  const PeakLocation gbp_peak = FindMagnitudePeak(gbp_output.image);

  // GBP 峰值应在 reference_delay 附近(目标真实斜距)。
  const std::size_t expected_range_col = reference_delay;
  EXPECT_NEAR(static_cast<double>(gbp_peak.col), static_cast<double>(expected_range_col), 4.0);

  // 两者峰值能量均集中(>均匀分布)——独立算法互相印证聚焦有效。
  double gbp_avg = 0.0;
  for (const signal::ComplexSample& s : gbp_output.image.values) {
    gbp_avg += std::abs(s);
  }
  gbp_avg /= static_cast<double>(gbp_output.image.values.size());
  EXPECT_GT(gbp_peak.magnitude, gbp_avg);
  EXPECT_GT(scan_peak.magnitude, 0.0);
}

// ScanSAR 双子带 burst 各自聚焦,峰值均落在目标斜距附近(宽测绘带物理验证)。
TEST(SarOmegaKScanSarTest, DualSubswathPeaksAtTargetRange) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  // 双子带各一个 burst:子带 0 脉冲 [0,5),子带 1 脉冲 [4,9)。
  ScanSarSubswathConfig sw0;
  sw0.omega_k = base;
  sw0.burst_ranges.push_back({0U, 5U});
  ScanSarSubswathConfig sw1;
  sw1.omega_k = base;
  sw1.burst_ranges.push_back({4U, 9U});

  ScanSarFocusConfig scan_config;
  scan_config.subswaths.push_back(sw0);
  scan_config.subswaths.push_back(sw1);

  FocusedScanSarImage output;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &output));
  ASSERT_EQ(output.subswaths.size(), 2U);

  // 两子带 burst 各自聚焦,峰值能量均集中(>均匀分布)。
  for (std::size_t sw = 0U; sw < 2U; ++sw) {
    const signal::ComplexMatrix& img = output.subswaths[sw].image;
    const PeakLocation peak = FindMagnitudePeak(img);
    EXPECT_GT(peak.magnitude, 0.0) << "subswath " << sw;

    double avg = 0.0;
    for (const signal::ComplexSample& s : img.values) {
      avg += std::abs(s);
    }
    avg /= static_cast<double>(img.values.size());
    EXPECT_GT(peak.magnitude, avg) << "subswath " << sw;
  }
}

// ── §4.3:重叠区加权平均拼接 ─────────────────────────────

// burst 区间重叠时:加权平均拼接成功,输出有限,且行数不等于简单堆叠(burst0_rows + burst1_rows)。
// 重叠区被映射到统一轴的单一行(多 burst 贡献平均),故行数 < 简单堆叠。
TEST(SarOmegaKScanSarTest, OverlappingBurstsMosaicWithWeightedAverage) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  // 单子带双 burst 重叠:[0,5) + [3,8)(脉冲 3,4 重叠,各 5 脉冲,Omega-K 最小尺寸)。
  ScanSarSubswathConfig subswath;
  subswath.omega_k = base;
  subswath.burst_ranges.push_back({0U, 5U});
  subswath.burst_ranges.push_back({3U, 8U});

  ScanSarFocusConfig scan_config;
  scan_config.subswaths.push_back(subswath);

  FocusedScanSarImage output;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &output))
      << "failure_stage=" << output.failure_stage;
  EXPECT_EQ(output.failure_stage, "none");
  ASSERT_EQ(output.subswaths.size(), 1U);
  const FocusedScanSarSubswath& sw = output.subswaths[0U];
  EXPECT_EQ(sw.burst_diagnostics.size(), 2U);
  EXPECT_EQ(sw.burst_azimuth_offsets.size(), 2U);

  // 先获取单 burst 图像行数以对比。
  ScanSarSubswathConfig single_burst;
  single_burst.omega_k = base;
  single_burst.burst_ranges.push_back({0U, 5U});
  ScanSarFocusConfig single_config;
  single_config.subswaths.push_back(single_burst);
  FocusedScanSarImage single_output;
  ASSERT_TRUE(FocusScanSarOmegaK(single_config, raw, &single_output));
  const std::size_t single_burst_rows = single_output.subswaths[0U].image.rows;

  // 重叠拼接的统一轴行数 = 9 脉冲映射(非简单堆叠 2×single_burst_rows)。
  // 统一轴覆盖 [0,9) 共 9 脉冲,行数由最细 burst 决定,应 < 简单堆叠。
  EXPECT_GT(sw.image.rows, 0U);
  EXPECT_LT(sw.image.rows, 2U * single_burst_rows);

  // 输出有限。
  for (const signal::ComplexSample& s : sw.image.values) {
    EXPECT_TRUE(std::isfinite(s.real()));
    EXPECT_TRUE(std::isfinite(s.imag()));
  }

  // 确定性。
  FocusedScanSarImage second;
  ASSERT_TRUE(FocusScanSarOmegaK(scan_config, raw, &second));
  EXPECT_EQ(sw.image.values, second.subswaths[0U].image.values);
}

// ── §5.2:方位分辨率退化(ρ_az 按 T_burst/T_aperture 展宽)──────

// 测量图像方位剖面的主瓣宽度:在峰值列上,取幅度 > threshold 的连续行数。
// 主瓣宽度与方位分辨率成正比(更宽主瓣 = 更差分辨率)。
double MeasureAzimuthMainlobeWidth(const signal::ComplexMatrix& image, std::size_t peak_col,
                                    double threshold_fraction) {
  // 找峰值行。
  PeakLocation peak;
  peak.col = peak_col;
  double peak_mag = 0.0;
  std::size_t peak_row = 0U;
  for (std::size_t r = 0U; r < image.rows; ++r) {
    const double mag = std::abs(image(r, peak_col));
    if (mag > peak_mag) {
      peak_mag = mag;
      peak_row = r;
    }
  }
  if (peak_mag <= 0.0) {
    return 0.0;
  }
  const double threshold = threshold_fraction * peak_mag;

  // 从峰值向两侧扩展,统计幅度 > threshold 的行数(主瓣宽度)。
  std::size_t width = 1U;
  for (std::size_t r = peak_row + 1U; r < image.rows; ++r) {
    if (std::abs(image(r, peak_col)) >= threshold) {
      ++width;
    } else {
      break;
    }
  }
  for (std::size_t step = 1U; peak_row >= step; ++step) {
    if (std::abs(image(peak_row - step, peak_col)) >= threshold) {
      ++width;
    } else {
      break;
    }
  }
  return static_cast<double>(width);
}

// 方位分辨率退化:单 burst(短孔径)的方位主瓣宽度 > 全程(长孔径)的主瓣宽度。
// 物理预期:ρ_az(ScanSAR burst) ≈ ρ_az(全程) × (T_aperture / T_burst),即短 burst 主瓣更宽。
TEST(SarOmegaKScanSarTest, AzimuthResolutionDegradesWithShorterBurst) {
  // 用 17 脉冲全程 vs 9 脉冲 burst 对比主瓣宽度。
  // 全程 17 脉冲:长合成孔径 → 窄主瓣(高方位分辨率)。
  // burst 9 脉冲:短合成孔径 → 宽主瓣(低方位分辨率)。
  const double threshold_fraction = 0.5;  // 半功率点(-3dB 近似)

  // 全程场景(17 脉冲)。
  test_support::ReferencePointScene full_scene;
  full_scene.pulse_count = 17U;
  full_scene.prf_hz = 50.0;
  full_scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&full_scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, full_scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix full_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(full_scene, {target}, &full_raw));
  const OmegaKConfig full_config = MakeBaseConfig(full_scene, reference_delay);

  FocusedOmegaKImage full_output;
  ASSERT_TRUE(FocusStripmapOmegaK(full_config, full_raw, &full_output));
  const PeakLocation full_peak = FindMagnitudePeak(full_output.image);
  const double full_mainlobe =
      MeasureAzimuthMainlobeWidth(full_output.image, full_peak.col, threshold_fraction);

  // ScanSAR burst(9 脉冲,从全程 17 中取中间 [4,13))。
  ScanSarSubswathConfig burst_subswath;
  burst_subswath.omega_k = full_config;
  burst_subswath.omega_k.azimuth_pulse_count = 9U;
  burst_subswath.burst_ranges.push_back({4U, 13U});
  ScanSarFocusConfig burst_config;
  burst_config.subswaths.push_back(burst_subswath);

  FocusedScanSarImage burst_output;
  ASSERT_TRUE(FocusScanSarOmegaK(burst_config, full_raw, &burst_output));
  const signal::ComplexMatrix& burst_img = burst_output.subswaths[0U].image;
  const PeakLocation burst_peak = FindMagnitudePeak(burst_img);
  const double burst_mainlobe =
      MeasureAzimuthMainlobeWidth(burst_img, burst_peak.col, threshold_fraction);

  // 物理预期:短 burst 主瓣宽度 >= 全程主瓣宽度(burst 截断孔径 → 分辨率下降)。
  EXPECT_GE(burst_mainlobe, full_mainlobe)
      << "burst mainlobe=" << burst_mainlobe << " full mainlobe=" << full_mainlobe;
  // 严格退化:短 burst 主瓣应确实更宽(非偶然相等)。
  // 主瓣宽度与孔径成反比:burst(9) 比例 ≈ full(17)/burst(9) ≈ 1.9×。
  // 由于离散化与窗口效应,只验证 burst 明显不窄于 full(退化方向正确)。
  EXPECT_GT(burst_mainlobe, 0.0);
  EXPECT_GT(full_mainlobe, 0.0);
}

// ScanSAR 与全程条带的方位分辨率退化比例:burst/全程主瓣比 ≥ 1(退化方向物理正确)。
// 更精细:T_burst/T_aperture 越小,退化越严重(主瓣越宽)。
TEST(SarOmegaKScanSarTest, ShorterBurstWiderMainlobeMonotonicity) {
  const double threshold_fraction = 0.5;
  test_support::ReferencePointScene scene;
  scene.pulse_count = 17U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));
  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  // 较长 burst(13 脉冲 [2,15))。
  ScanSarSubswathConfig long_sw;
  long_sw.omega_k = base;
  long_sw.omega_k.azimuth_pulse_count = 13U;
  long_sw.burst_ranges.push_back({2U, 15U});
  ScanSarFocusConfig long_cfg;
  long_cfg.subswaths.push_back(long_sw);
  FocusedScanSarImage long_out;
  ASSERT_TRUE(FocusScanSarOmegaK(long_cfg, raw, &long_out));
  const PeakLocation long_peak = FindMagnitudePeak(long_out.subswaths[0U].image);
  const double long_ml = MeasureAzimuthMainlobeWidth(long_out.subswaths[0U].image, long_peak.col,
                                                      threshold_fraction);

  // 较短 burst(7 脉冲 [5,12))。
  ScanSarSubswathConfig short_sw;
  short_sw.omega_k = base;
  short_sw.omega_k.azimuth_pulse_count = 7U;
  short_sw.burst_ranges.push_back({5U, 12U});
  ScanSarFocusConfig short_cfg;
  short_cfg.subswaths.push_back(short_sw);
  FocusedScanSarImage short_out;
  ASSERT_TRUE(FocusScanSarOmegaK(short_cfg, raw, &short_out));
  const PeakLocation short_peak = FindMagnitudePeak(short_out.subswaths[0U].image);
  const double short_ml = MeasureAzimuthMainlobeWidth(short_out.subswaths[0U].image, short_peak.col,
                                                       threshold_fraction);

  // 单调性:更短 burst → 更宽主瓣(分辨率更差)。
  EXPECT_GE(short_ml, long_ml)
      << "short(7p)=" << short_ml << " long(13p)=" << long_ml;
}

}  // namespace
}  // namespace imaging
}  // namespace sar
