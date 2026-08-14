#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <limits>

#include "sar/signal/SarWaveform.h"

namespace sar {
namespace signal {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

void ExpectNear(const ComplexSample& actual, const ComplexSample& expected, double tolerance) {
  EXPECT_NEAR(actual.real(), expected.real(), tolerance);
  EXPECT_NEAR(actual.imag(), expected.imag(), tolerance);
}

TEST(SarSignalChainTest, LfmWaveformUsesContractedDurationSlopeAndSamples) {
  LfmWaveformConfig config;
  config.bandwidth_hz = 20.0;
  config.time_bandwidth_product = 4.0;
  config.sample_rate_hz = 100.0;
  config.start_frequency_hz = 5.0;

  LfmWaveform waveform;
  ASSERT_TRUE(GenerateLfmWaveform(config, &waveform));

  EXPECT_DOUBLE_EQ(waveform.pulse_width_s, 0.2);
  EXPECT_DOUBLE_EQ(waveform.chirp_rate_hz_per_s, 100.0);
  ASSERT_EQ(waveform.samples.size(), 20U);
  ExpectNear(waveform.samples.front(), ComplexSample(1.0, 0.0), 1.0e-12);

  const double t = 3.0 / config.sample_rate_hz;
  const double phase =
      2.0 * kPi * (config.start_frequency_hz * t + 0.5 * waveform.chirp_rate_hz_per_s * t * t);
  ExpectNear(waveform.samples[3U], ComplexSample(std::cos(phase), std::sin(phase)), 1.0e-12);
}

TEST(SarSignalChainTest, LfmInstantaneousFrequencyIsMonotonic) {
  LfmWaveformConfig config;
  config.bandwidth_hz = 40.0;
  config.time_bandwidth_product = 8.0;
  config.sample_rate_hz = 200.0;
  config.start_frequency_hz = -20.0;

  LfmWaveform waveform;
  ASSERT_TRUE(GenerateLfmWaveform(config, &waveform));

  double previous_frequency = -std::numeric_limits<double>::infinity();
  for (std::size_t n = 0; n < waveform.samples.size(); ++n) {
    const double t = static_cast<double>(n) / config.sample_rate_hz;
    const double frequency = config.start_frequency_hz + waveform.chirp_rate_hz_per_s * t;
    EXPECT_GE(frequency, previous_frequency);
    previous_frequency = frequency;
  }
}

TEST(SarSignalChainTest, MatchedFilterIsTimeReversedConjugateOnce) {
  const ComplexVector waveform{{1.0, 2.0}, {3.0, -4.0}, {-5.0, 6.0}};

  ComplexVector filter;
  ASSERT_TRUE(BuildMatchedFilter(waveform, &filter));

  ASSERT_EQ(filter.size(), waveform.size());
  ExpectNear(filter[0U], ComplexSample(-5.0, -6.0), 1.0e-12);
  ExpectNear(filter[1U], ComplexSample(3.0, 4.0), 1.0e-12);
  ExpectNear(filter[2U], ComplexSample(1.0, -2.0), 1.0e-12);
}

TEST(SarSignalChainTest, LinearConvolutionMatchesDirectReference) {
  const ComplexVector input{{1.0, 1.0}, {2.0, 0.0}, {-1.0, 0.5}};
  const ComplexVector filter{{0.5, 0.0}, {1.0, -1.0}};

  ComplexVector output;
  ASSERT_TRUE(LinearConvolveFft(input, filter, &output));

  ASSERT_EQ(output.size(), input.size() + filter.size() - 1U);
  for (std::size_t n = 0; n < output.size(); ++n) {
    ComplexSample expected(0.0, 0.0);
    for (std::size_t k = 0; k < input.size(); ++k) {
      if (n >= k && (n - k) < filter.size()) {
        expected += input[k] * filter[n - k];
      }
    }
    ExpectNear(output[n], expected, 1.0e-10);
  }
}

TEST(SarSignalChainTest, RangeCompressionPlacesPeakAtDelayedPulse) {
  LfmWaveformConfig config;
  config.bandwidth_hz = 20.0;
  config.time_bandwidth_product = 4.0;
  config.sample_rate_hz = 100.0;
  config.start_frequency_hz = 0.0;
  LfmWaveform waveform;
  ASSERT_TRUE(GenerateLfmWaveform(config, &waveform));

  ComplexVector matched_filter;
  ASSERT_TRUE(BuildMatchedFilter(waveform.samples, &matched_filter));

  const std::size_t delay = 7U;
  ComplexVector input(delay + waveform.samples.size() + 5U, ComplexSample(0.0, 0.0));
  for (std::size_t i = 0; i < waveform.samples.size(); ++i) {
    input[delay + i] = waveform.samples[i];
  }

  RangeCompressionResult result;
  ASSERT_TRUE(RangeCompress(input, matched_filter, config.sample_rate_hz, &result));

  EXPECT_EQ(result.full_peak_index, delay + waveform.samples.size() - 1U);
  EXPECT_EQ(result.aligned_peak_index, delay);
  ASSERT_LT(delay, result.range_aligned_output.size());
  EXPECT_NEAR(std::abs(result.range_aligned_output[delay]),
              static_cast<double>(waveform.samples.size()), 1.0e-8);
  EXPECT_NEAR(result.range_bin_spacing_m, 299792458.0 / (2.0 * config.sample_rate_hz), 1.0e-12);
}

TEST(SarSignalChainTest, RangeCompressRowsMatchesPerRowRangeCompressExactly) {
  LfmWaveformConfig config;
  config.bandwidth_hz = 20.0;
  config.time_bandwidth_product = 4.0;
  config.sample_rate_hz = 100.0;
  config.start_frequency_hz = 0.0;
  LfmWaveform waveform;
  ASSERT_TRUE(GenerateLfmWaveform(config, &waveform));
  ComplexVector matched_filter;
  ASSERT_TRUE(BuildMatchedFilter(waveform.samples, &matched_filter));

  const std::size_t rows = 6U;
  const std::size_t cols = 16U;
  ComplexMatrix history;
  history.rows = rows;
  history.cols = cols;
  history.values.resize(rows * cols);
  for (std::size_t row = 0U; row < rows; ++row) {
    for (std::size_t col = 0U; col < cols; ++col) {
      const double angle = 0.37 * static_cast<double>(row * cols + col);
      history(row, col) = ComplexSample(std::cos(angle), std::sin(angle)) *
                          (0.5 + 0.01 * static_cast<double>(col));
    }
  }

  ComplexMatrix batched;
  ASSERT_TRUE(RangeCompressRows(history, matched_filter, config.sample_rate_hz, &batched));
  ASSERT_EQ(batched.rows, rows);
  ASSERT_EQ(batched.cols, cols);

  for (std::size_t row = 0U; row < rows; ++row) {
    ComplexVector one_row(cols);
    for (std::size_t col = 0U; col < cols; ++col) {
      one_row[col] = history(row, col);
    }
    RangeCompressionResult per_row;
    ASSERT_TRUE(RangeCompress(one_row, matched_filter, config.sample_rate_hz, &per_row));
    ASSERT_EQ(per_row.range_aligned_output.size(), cols);
    for (std::size_t col = 0U; col < cols; ++col) {
      // 批量路径与逐行路径使用同一卷积/对齐公式,数值逐位一致。
      EXPECT_DOUBLE_EQ(batched(row, col).real(), per_row.range_aligned_output[col].real());
      EXPECT_DOUBLE_EQ(batched(row, col).imag(), per_row.range_aligned_output[col].imag());
    }
  }
}

TEST(SarSignalChainTest, RangeCompressRowsRejectsInvalidInputs) {
  const ComplexVector filter(3U, ComplexSample(1.0, 0.0));
  ComplexMatrix output;
  EXPECT_FALSE(RangeCompressRows(ComplexMatrix{}, filter, 1.0e6, &output));
  EXPECT_FALSE(RangeCompressRows(ComplexMatrix{}, filter, 1.0e6, nullptr));

  ComplexMatrix shape_mismatch;
  shape_mismatch.rows = 2U;
  shape_mismatch.cols = 3U;
  shape_mismatch.values.resize(5U);
  EXPECT_FALSE(RangeCompressRows(shape_mismatch, filter, 1.0e6, &output));

  ComplexMatrix valid;
  valid.rows = 2U;
  valid.cols = 3U;
  valid.values.resize(6U, ComplexSample(0.0, 0.0));
  EXPECT_FALSE(RangeCompressRows(valid, ComplexVector{}, 1.0e6, &output));
  EXPECT_FALSE(RangeCompressRows(valid, filter, 0.0, &output));
  EXPECT_FALSE(RangeCompressRows(valid, filter, -1.0, &output));
}

TEST(SarSignalChainTest, PulseQualityMetricsCapturePeakWidthAndSidelobes) {
  ComplexVector pulse{{0.1, 0.0}, {0.3, 0.0}, {1.0, 0.0}, {0.25, 0.0}, {0.05, 0.0}};

  PulseQualityMetrics metrics;
  ASSERT_TRUE(EstimatePulseQuality(pulse, &metrics));

  EXPECT_EQ(metrics.peak_index, 2U);
  EXPECT_DOUBLE_EQ(metrics.peak_magnitude, 1.0);
  EXPECT_EQ(metrics.main_lobe_start, 2U);
  EXPECT_EQ(metrics.main_lobe_end, 2U);
  EXPECT_DOUBLE_EQ(metrics.width_3db_bins, 1.0);
  EXPECT_DOUBLE_EQ(metrics.width_20db_bins, 4.0);
  EXPECT_LT(metrics.pslr_db, 0.0);
  EXPECT_LT(metrics.islr_db, 0.0);
}

TEST(SarSignalChainTest, RejectsInvalidSignalInputs) {
  LfmWaveform waveform;
  EXPECT_FALSE(GenerateLfmWaveform(LfmWaveformConfig{}, &waveform));
  EXPECT_FALSE(GenerateLfmWaveform({1.0, 1.0, 1.0, 0.0}, nullptr));

  ComplexVector filter;
  EXPECT_FALSE(BuildMatchedFilter({}, &filter));
  EXPECT_FALSE(BuildMatchedFilter(ComplexVector(1U), nullptr));

  ComplexVector convolution;
  EXPECT_FALSE(LinearConvolveFft({}, ComplexVector(1U), &convolution));
  EXPECT_FALSE(LinearConvolveFft(ComplexVector(1U), {}, &convolution));

  RangeCompressionResult compression;
  EXPECT_FALSE(RangeCompress(ComplexVector(1U), ComplexVector(1U), 0.0, &compression));

  PulseQualityMetrics metrics;
  EXPECT_FALSE(EstimatePulseQuality({}, &metrics));
}

TEST(SarSignalChainTest, GenerateLfmWaveformRejectsNanBandwidth) {
  LfmWaveformConfig config;
  config.bandwidth_hz = std::numeric_limits<double>::quiet_NaN();
  config.time_bandwidth_product = 4.0;
  config.sample_rate_hz = 100.0e6;
  LfmWaveform waveform;
  EXPECT_FALSE(GenerateLfmWaveform(config, &waveform));
}

TEST(SarSignalChainTest, GenerateLfmWaveformRejectsInfBandwidth) {
  LfmWaveformConfig config;
  config.bandwidth_hz = std::numeric_limits<double>::infinity();
  config.time_bandwidth_product = 4.0;
  config.sample_rate_hz = 100.0e6;
  LfmWaveform waveform;
  EXPECT_FALSE(GenerateLfmWaveform(config, &waveform));
}

TEST(SarSignalChainTest, GenerateLfmWaveformRejectsNanSampleRate) {
  LfmWaveformConfig config;
  config.bandwidth_hz = 20.0;
  config.time_bandwidth_product = 4.0;
  config.sample_rate_hz = std::numeric_limits<double>::quiet_NaN();
  LfmWaveform waveform;
  EXPECT_FALSE(GenerateLfmWaveform(config, &waveform));
}

TEST(SarSignalChainTest, GenerateLfmWaveformRejectsInfSampleRate) {
  LfmWaveformConfig config;
  config.bandwidth_hz = 20.0;
  config.time_bandwidth_product = 4.0;
  config.sample_rate_hz = std::numeric_limits<double>::infinity();
  LfmWaveform waveform;
  EXPECT_FALSE(GenerateLfmWaveform(config, &waveform));
}

TEST(SarSignalChainTest, GenerateLfmWaveformRejectsNanTimeBandwidthProduct) {
  LfmWaveformConfig config;
  config.bandwidth_hz = 20.0;
  config.time_bandwidth_product = std::numeric_limits<double>::quiet_NaN();
  config.sample_rate_hz = 100.0;
  LfmWaveform waveform;
  EXPECT_FALSE(GenerateLfmWaveform(config, &waveform));
}

TEST(SarSignalChainTest, GenerateLfmWaveformRejectsInfTimeBandwidthProduct) {
  LfmWaveformConfig config;
  config.bandwidth_hz = 20.0;
  config.time_bandwidth_product = std::numeric_limits<double>::infinity();
  config.sample_rate_hz = 100.0;
  LfmWaveform waveform;
  EXPECT_FALSE(GenerateLfmWaveform(config, &waveform));
}

TEST(SarSignalChainTest, RangeCompressRejectsNanSampleRate) {
  LfmWaveformConfig config;
  config.bandwidth_hz = 20.0;
  config.time_bandwidth_product = 4.0;
  config.sample_rate_hz = 100.0;
  LfmWaveform waveform;
  ASSERT_TRUE(GenerateLfmWaveform(config, &waveform));

  ComplexVector matched_filter;
  ASSERT_TRUE(BuildMatchedFilter(waveform.samples, &matched_filter));

  ComplexVector input(10U, ComplexSample(0.0, 0.0));
  RangeCompressionResult compression;
  EXPECT_FALSE(RangeCompress(input, matched_filter,
                              std::numeric_limits<double>::quiet_NaN(), &compression));
  EXPECT_FALSE(RangeCompress(input, matched_filter,
                              std::numeric_limits<double>::infinity(), &compression));
}

}  // namespace
}  // namespace signal
}  // namespace sar
