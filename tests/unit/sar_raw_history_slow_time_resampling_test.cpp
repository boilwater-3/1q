#include <gtest/gtest.h>

#include <complex>

#include "sar/imaging/SarSlowTimeResampling.h"

namespace sar {
namespace imaging {
namespace {

std::complex<double> ColumnSignal(std::size_t col, double time_s) {
  const double scale = static_cast<double>(col + 1U);
  return std::complex<double>(scale + (2.0 + scale) * time_s,
                              -scale + (0.25 + scale) * time_s);
}

signal::ComplexMatrix BuildHistory(const std::vector<double>& times, std::size_t cols) {
  signal::ComplexMatrix matrix;
  matrix.rows = times.size();
  matrix.cols = cols;
  matrix.values.resize(matrix.rows * matrix.cols);
  for (std::size_t row = 0U; row < times.size(); ++row) {
    for (std::size_t col = 0U; col < cols; ++col) {
      matrix(row, col) = ColumnSignal(col, times[row]);
    }
  }
  return matrix;
}

TEST(SarRawHistorySlowTimeResamplingTest, UniformMatrixStrictlyDegenerates) {
  const std::vector<double> times{0.0, 0.25, 0.5, 0.75, 1.0};
  const signal::ComplexMatrix input = BuildHistory(times, 3U);
  signal::ComplexMatrix output;
  SlowTimeResamplingDiagnostics diagnostics;
  ASSERT_TRUE(ResampleRawHistorySlowTimeLinear(times, input, &output, &diagnostics));
  EXPECT_TRUE(diagnostics.uniform_within_tolerance);
  EXPECT_EQ(output.rows, input.rows);
  EXPECT_EQ(output.cols, input.cols);
  EXPECT_EQ(output.values, input.values);
}

TEST(SarRawHistorySlowTimeResamplingTest, RecoversEachColumnsIndependentAffineTruth) {
  const std::vector<double> times{1.0, 1.1, 1.55, 1.82, 2.0};
  const signal::ComplexMatrix input = BuildHistory(times, 4U);
  signal::ComplexMatrix output;
  SlowTimeResamplingDiagnostics diagnostics;
  ASSERT_TRUE(ResampleRawHistorySlowTimeLinear(times, input, &output, &diagnostics));
  for (std::size_t row = 0U; row < output.rows; ++row) {
    for (std::size_t col = 0U; col < output.cols; ++col) {
      const std::complex<double> expected = ColumnSignal(col, diagnostics.nominal_times_s[row]);
      EXPECT_NEAR(output(row, col).real(), expected.real(), 1.0e-12);
      EXPECT_NEAR(output(row, col).imag(), expected.imag(), 1.0e-12);
    }
  }

  signal::ComplexMatrix modified = input;
  for (std::size_t row = 0U; row < modified.rows; ++row) {
    modified(row, 2U) += std::complex<double>(10.0, -3.0);
  }
  signal::ComplexMatrix modified_output;
  ASSERT_TRUE(
      ResampleRawHistorySlowTimeLinear(times, modified, &modified_output, &diagnostics));
  for (std::size_t row = 0U; row < output.rows; ++row) {
    EXPECT_EQ(modified_output(row, 0U), output(row, 0U));
    EXPECT_EQ(modified_output(row, 1U), output(row, 1U));
    EXPECT_EQ(modified_output(row, 3U), output(row, 3U));
  }
}

TEST(SarRawHistorySlowTimeResamplingTest, RejectsInvalidMatrixAndTimeCount) {
  const std::vector<double> times{0.0, 0.5, 1.0};
  signal::ComplexMatrix input = BuildHistory(times, 2U);
  signal::ComplexMatrix output;
  SlowTimeResamplingDiagnostics diagnostics;
  EXPECT_FALSE(ResampleRawHistorySlowTimeLinear({0.0, 1.0}, input, &output, &diagnostics));
  input.values.pop_back();
  EXPECT_FALSE(ResampleRawHistorySlowTimeLinear(times, input, &output, &diagnostics));
  EXPECT_FALSE(ResampleRawHistorySlowTimeLinear(times, input, nullptr, &diagnostics));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
