#include <gtest/gtest.h>

#include <cmath>
#include <complex>

#include "sar/imaging/SarOmegaKSpectrumFrontEnd.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {
namespace {

// 与现有 Omega-K 部件测试一致的基准几何配置(5x5)。
OmegaKGeometryConfig ValidConfig() {
  OmegaKGeometryConfig config;
  config.range_sample_count = 5U;
  config.azimuth_pulse_count = 5U;
  config.sample_rate_hz = 2.0e8;
  config.prf_hz = 100.0;
  config.carrier_frequency_hz = 1.0e9;
  config.platform_velocity_mps = 100.0;
  config.reference_range_m = 1000.0;
  return config;
}

signal::ComplexMatrix ConstantRawHistory(const OmegaKGeometryConfig& config,
                                          signal::ComplexSample value) {
  signal::ComplexMatrix matrix;
  matrix.rows = config.azimuth_pulse_count;
  matrix.cols = config.range_sample_count;
  matrix.values.assign(matrix.rows * matrix.cols, value);
  return matrix;
}

OmegaKSpectrumFrontEndRequest ValidRequest(const OmegaKGeometryConfig& config) {
  OmegaKSpectrumFrontEndRequest request;
  request.request_id = 1U;
  request.config = config;
  request.raw_pulse_history = ConstantRawHistory(config, signal::ComplexSample(1.0, 0.0));
  return request;
}

TEST(SarOmegaKSpectrumFrontEndTest, ProducesSpectrumWithExpectedShape) {
  const OmegaKGeometryConfig config = ValidConfig();
  const OmegaKSpectrumFrontEndResult result = ExecuteOmegaKSpectrumFrontEnd(ValidRequest(config));

  ASSERT_EQ(result.status, OmegaKSpectrumFrontEndStatus::kSucceeded);
  EXPECT_EQ(result.source_spectrum.rows, config.azimuth_pulse_count);
  EXPECT_EQ(result.source_spectrum.cols, config.range_sample_count);
  // front-end 不因几何越界(out_of_support)或色散无效点拒绝 —— H_bulk 在无效点相位=0
  // 自动退化。几何诊断由后续网格收缩阶段处理。
  EXPECT_EQ(result.source_spectrum.values.size(),
            config.azimuth_pulse_count * config.range_sample_count);
}

TEST(SarOmegaKSpectrumFrontEndTest, SpectrumIsFiniteAndDeterministic) {
  const OmegaKGeometryConfig config = ValidConfig();
  const OmegaKSpectrumFrontEndRequest request = ValidRequest(config);
  const OmegaKSpectrumFrontEndResult first = ExecuteOmegaKSpectrumFrontEnd(request);
  const OmegaKSpectrumFrontEndResult second = ExecuteOmegaKSpectrumFrontEnd(request);

  ASSERT_EQ(first.status, OmegaKSpectrumFrontEndStatus::kSucceeded);
  for (const signal::ComplexSample& sample : first.source_spectrum.values) {
    EXPECT_TRUE(std::isfinite(sample.real()));
    EXPECT_TRUE(std::isfinite(sample.imag()));
  }
  // 确定性:相同输入产生相同输出。
  EXPECT_EQ(first.source_spectrum.values, second.source_spectrum.values);
}

// bulk 参考函数退化性不变量:front-end 谱 = 纯 2D FFT × 手动 H_bulk。
// H_bulk(pixel) = exp(+j · R_ref · K_z(pixel))。手动构造 H_bulk 与 front-end 输出对照。
TEST(SarOmegaKSpectrumFrontEndTest, SpectrumEqualsPure2dFftTimesManualBulkReference) {
  const OmegaKGeometryConfig config = ValidConfig();
  const OmegaKSpectrumFrontEndRequest request = ValidRequest(config);
  const OmegaKSpectrumFrontEndResult front_end = ExecuteOmegaKSpectrumFrontEnd(request);
  ASSERT_EQ(front_end.status, OmegaKSpectrumFrontEndStatus::kSucceeded);

  // 手动做同样的纯 2D FFT。
  signal::ComplexMatrix range_spectrum;
  ASSERT_TRUE(signal::FftRows(request.raw_pulse_history, false, &range_spectrum));
  signal::ComplexMatrix pure_two_d;
  ASSERT_TRUE(signal::FftCols(range_spectrum, false, &pure_two_d));

  // 手动施加 H_bulk = exp(+j · R_ref · K_z)。
  constexpr double kPi = 3.141592653589793238462643383279502884;
  signal::ComplexMatrix expected = pure_two_d;
  for (std::size_t index = 0U; index < expected.values.size(); ++index) {
    const double phase =
        config.reference_range_m * front_end.geometry.propagation_wavenumbers_rad_per_m[index];
    const signal::ComplexSample rotation(std::cos(phase), std::sin(phase));
    expected.values[index] *= rotation;
  }

  ASSERT_EQ(front_end.source_spectrum.rows, expected.rows);
  ASSERT_EQ(front_end.source_spectrum.cols, expected.cols);
  for (std::size_t index = 0U; index < expected.values.size(); ++index) {
    EXPECT_NEAR(front_end.source_spectrum.values[index].real(), expected.values[index].real(),
                1.0e-6);
    EXPECT_NEAR(front_end.source_spectrum.values[index].imag(), expected.values[index].imag(),
                1.0e-6);
  }
}

// 最小非平凡多普勒退化性:azimuth_pulse_count=2 时 front-end 应成功且无崩溃。
// 单脉冲(az=1)的 FftCols 依赖 Eigen FFT 的 N=1 行为,不作为 front-end 契约;
// Omega-K 聚焦物理上要求合成孔径(多脉冲),单脉冲无聚焦意义。
TEST(SarOmegaKSpectrumFrontEndTest, MinimalDopplerSucceedsWithoutCrash) {
  OmegaKGeometryConfig config = ValidConfig();
  config.azimuth_pulse_count = 2U;
  const OmegaKSpectrumFrontEndResult result = ExecuteOmegaKSpectrumFrontEnd(ValidRequest(config));

  EXPECT_EQ(result.status, OmegaKSpectrumFrontEndStatus::kSucceeded);
  EXPECT_EQ(result.source_spectrum.rows, 2U);
  EXPECT_EQ(result.source_spectrum.cols, config.range_sample_count);
  // f_a=0 的行(center)K_z=K_r(数学参考 §8 验收 2):中心行的 Stolt shift 应为零。
  // unshifted 频率轴:N=2 时 bin0=0Hz(f_a=0), bin1=prf/2=50Hz。
  const std::size_t cols = config.range_sample_count;
  for (std::size_t col = 0U; col < cols; ++col) {
    EXPECT_NEAR(result.geometry.stolt_shifts_hz[0U * cols + col], 0.0, 1.0e-9);
  }
}

TEST(SarOmegaKSpectrumFrontEndTest, RejectsInvalidRequestId) {
  OmegaKSpectrumFrontEndRequest request = ValidRequest(ValidConfig());
  request.request_id = 0U;
  const OmegaKSpectrumFrontEndResult result = ExecuteOmegaKSpectrumFrontEnd(request);
  EXPECT_EQ(result.status, OmegaKSpectrumFrontEndStatus::kRejected);
  EXPECT_EQ(result.reason, OmegaKSpectrumFrontEndReason::kInvalidRequestId);
  EXPECT_TRUE(result.source_spectrum.values.empty());
}

TEST(SarOmegaKSpectrumFrontEndTest, RejectsSingleAzimuthPulse) {
  // 单脉冲(az=1)无法构成方位合成孔径,且 N=1 方位 FFT 在 Eigen 下行为未定义。
  // front-end 应优雅拒绝而非崩溃。
  OmegaKSpectrumFrontEndRequest request = ValidRequest(ValidConfig());
  request.config.azimuth_pulse_count = 1U;
  request.raw_pulse_history = ConstantRawHistory(request.config, signal::ComplexSample(1.0, 0.0));
  const OmegaKSpectrumFrontEndResult result = ExecuteOmegaKSpectrumFrontEnd(request);
  EXPECT_EQ(result.status, OmegaKSpectrumFrontEndStatus::kRejected);
  EXPECT_EQ(result.reason, OmegaKSpectrumFrontEndReason::kInvalidConfig);
}

TEST(SarOmegaKSpectrumFrontEndTest, RejectsZeroCarrierFrequency) {
  OmegaKSpectrumFrontEndRequest request = ValidRequest(ValidConfig());
  request.config.carrier_frequency_hz = 0.0;
  const OmegaKSpectrumFrontEndResult result = ExecuteOmegaKSpectrumFrontEnd(request);
  EXPECT_EQ(result.status, OmegaKSpectrumFrontEndStatus::kRejected);
  EXPECT_EQ(result.reason, OmegaKSpectrumFrontEndReason::kInvalidConfig);
}

TEST(SarOmegaKSpectrumFrontEndTest, RejectsSizeMismatchedRawHistory) {
  OmegaKSpectrumFrontEndRequest request = ValidRequest(ValidConfig());
  request.raw_pulse_history.rows = 3U;  // 与 azimuth_pulse_count=5 不符
  request.raw_pulse_history.values.assign(3U * request.config.range_sample_count,
                                          signal::ComplexSample(0.0, 0.0));
  const OmegaKSpectrumFrontEndResult result = ExecuteOmegaKSpectrumFrontEnd(request);
  EXPECT_EQ(result.status, OmegaKSpectrumFrontEndStatus::kRejected);
  EXPECT_EQ(result.reason, OmegaKSpectrumFrontEndReason::kInvalidRawHistory);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
