/**
 * @file sar_window_function_test.cpp
 * @brief 窗函数、加窗匹配滤波/距离压缩与二维脉冲压缩的单元测试。
 */

#include "sar/signal/SarWaveform.h"

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

namespace sar {
namespace signal {
namespace {

LfmWaveform MakeReferenceWaveform() {
  LfmWaveformConfig config;
  config.bandwidth_hz = 10.0e6;
  config.time_bandwidth_product = 100.0;
  config.sample_rate_hz = 20.0e6;
  config.start_frequency_hz = -5.0e6;  // 基带 LFM,从 -B/2 扫到 +B/2
  LfmWaveform wf;
  EXPECT_TRUE(GenerateLfmWaveform(config, &wf));
  return wf;
}

ComplexVector MakeDelayedEcho(const ComplexVector& waveform, std::size_t delay_samples) {
  ComplexVector echo(delay_samples + waveform.size(), ComplexSample(0.0, 0.0));
  for (std::size_t i = 0; i < waveform.size(); ++i) {
    echo[delay_samples + i] = waveform[i];
  }
  return echo;
}

}  // namespace

TEST(SarWindowFunctionTest, GenerateWindowNoneIsFlat) {
  ComplexVector w;
  ASSERT_TRUE(GenerateWindow(WindowSpec{WindowType::kNone, 0.0}, 16, &w));
  ASSERT_EQ(w.size(), 16u);
  for (const auto& v : w) {
    EXPECT_NEAR(v.real(), 1.0, 1.0e-12);
    EXPECT_NEAR(v.imag(), 0.0, 1.0e-12);
  }
}

TEST(SarWindowFunctionTest, HammingWindowIsSymmetricAndPeaksNearCenter) {
  ComplexVector w;
  ASSERT_TRUE(GenerateWindow(WindowSpec{WindowType::kHamming, 0.0}, 32, &w));
  EXPECT_NEAR(w.front().real(), w.back().real(), 1.0e-12);
  // Hamming N=32 中点: 0.54 - 0.46·cos(32π/31) ≈ 0.9976
  EXPECT_NEAR(w[16].real(), 0.9976, 1.0e-4);    // 中点峰值
  EXPECT_NEAR(w.front().real(), 0.08, 1.0e-3);  // 边缘
}

TEST(SarWindowFunctionTest, KaiserWindowPeaksAtCenter) {
  ComplexVector w;
  ASSERT_TRUE(GenerateWindow(WindowSpec{WindowType::kKaiser, 8.6}, 64, &w));
  EXPECT_GT(w[32].real(), w[0].real());
  // Kaiser 窗归一化:分母 I0(β),中点处 numerator→I0(β),值 ≈ 1
  EXPECT_NEAR(w[32].real(), 0.9990, 2.0e-3);
}

TEST(SarWindowFunctionTest, WindowedMatchedFilterAppliesWindowThenConjugates) {
  const LfmWaveform wf = MakeReferenceWaveform();
  ComplexVector filter_ham;
  ASSERT_TRUE(BuildMatchedFilter(wf.samples, WindowSpec{WindowType::kHamming, 0.0}, &filter_ham));

  ComplexVector win;
  ASSERT_TRUE(GenerateWindow(WindowSpec{WindowType::kHamming, 0.0}, wf.samples.size(), &win));
  const std::size_t n = wf.samples.size();
  for (std::size_t i = 0; i < n; ++i) {
    const ComplexSample expected = std::conj(wf.samples[n - 1U - i]) * win[n - 1U - i];
    EXPECT_NEAR(filter_ham[i].real(), expected.real(), 1.0e-9);
    EXPECT_NEAR(filter_ham[i].imag(), expected.imag(), 1.0e-9);
  }
}

TEST(SarWindowFunctionTest, WindowedRangeCompressionLowersPeakSidelobes) {
  const LfmWaveform wf = MakeReferenceWaveform();
  ComplexVector matched;
  ASSERT_TRUE(BuildMatchedFilter(wf.samples, &matched));
  const ComplexVector echo = MakeDelayedEcho(wf.samples, 5U);

  RangeCompressionResult rc_none;
  RangeCompressionResult rc_ham;
  ASSERT_TRUE(RangeCompress(echo, matched, wf.config.sample_rate_hz, &rc_none));
  ASSERT_TRUE(RangeCompress(echo, matched, wf.config.sample_rate_hz,
                            WindowSpec{WindowType::kHamming, 0.0}, &rc_ham));

  PulseQualityMetrics m_none;
  PulseQualityMetrics m_ham;
  ASSERT_TRUE(EstimatePulseQuality(rc_none.range_aligned_output, &m_none));
  ASSERT_TRUE(EstimatePulseQuality(rc_ham.range_aligned_output, &m_ham));

  EXPECT_LT(m_ham.pslr_db, m_none.pslr_db);            // 加窗降低峰值旁瓣
  EXPECT_GE(m_ham.width_3db_bins, m_none.width_3db_bins);  // 加窗展宽主瓣
}

TEST(SarWindowFunctionTest, Compress2DRangeOnlyPlacesPeakAtExpectedColumn) {
  const LfmWaveform wf = MakeReferenceWaveform();
  ComplexVector matched;
  ASSERT_TRUE(BuildMatchedFilter(wf.samples, &matched));

  const std::size_t delay = 5U;
  const std::size_t cols = wf.samples.size() + delay;
  const std::size_t rows = 8U;
  ComplexMatrix history;
  history.rows = rows;
  history.cols = cols;
  history.values.assign(rows * cols, ComplexSample(0.0, 0.0));
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t i = 0; i < wf.samples.size(); ++i) {
      history(r, delay + i) = wf.samples[i];
    }
  }

  RangeAzimuthCompressionConfig config;
  config.sample_rate_hz = wf.config.sample_rate_hz;
  config.range_window = WindowSpec{WindowType::kNone, 0.0};

  ComplexMatrix out;
  ASSERT_TRUE(Compress2D(history, matched, config, &out));
  ASSERT_EQ(out.rows, rows);
  ASSERT_EQ(out.cols, cols);

  std::size_t peak_col = 0U;
  double peak_mag = -1.0;
  for (std::size_t r = 0; r < out.rows; ++r) {
    for (std::size_t c = 0; c < out.cols; ++c) {
      const double mag = std::abs(out(r, c));
      if (mag > peak_mag) {
        peak_mag = mag;
        peak_col = c;
      }
    }
  }
  EXPECT_EQ(peak_col, delay);
}

TEST(SarWindowFunctionTest, Compress2DWithAzimuthCompressionProducesFiniteOutput) {
  const LfmWaveform wf = MakeReferenceWaveform();
  ComplexVector matched;
  ASSERT_TRUE(BuildMatchedFilter(wf.samples, &matched));

  const std::size_t delay = 5U;
  const std::size_t cols = wf.samples.size() + delay;
  const std::size_t rows = 16U;
  ComplexMatrix history;
  history.rows = rows;
  history.cols = cols;
  history.values.assign(rows * cols, ComplexSample(0.0, 0.0));
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t i = 0; i < wf.samples.size(); ++i) {
      history(r, delay + i) = wf.samples[i];
    }
  }

  RangeAzimuthCompressionConfig config;
  config.sample_rate_hz = wf.config.sample_rate_hz;
  config.prf_hz = 200.0;
  config.azimuth_matched_filter_rate_hz_per_s = 100.0;  // 非零触发方位压缩
  config.azimuth_window = WindowSpec{WindowType::kHamming, 0.0};

  ComplexMatrix out;
  ASSERT_TRUE(Compress2D(history, matched, config, &out));
  ASSERT_EQ(out.rows, rows);
  ASSERT_EQ(out.cols, cols);
  for (const auto& v : out.values) {
    EXPECT_TRUE(std::isfinite(v.real()));
    EXPECT_TRUE(std::isfinite(v.imag()));
  }
}

}  // namespace signal
}  // namespace sar
