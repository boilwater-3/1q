#include <gtest/gtest.h>

#include <complex>
#include <limits>

#include "sar/imaging/SarSlowTimeResampling.h"

namespace sar {
namespace imaging {
namespace {

std::complex<double> AffineSignal(double time_s) {
  return std::complex<double>(2.0 + 3.0 * time_s, -1.0 + 0.5 * time_s);
}

TEST(SarSlowTimeResamplingTest, UniformInputStrictlyDegenerates) {
  const std::vector<double> times{2.0, 2.25, 2.5, 2.75, 3.0};
  std::vector<std::complex<double>> input;
  for (std::size_t index = 0U; index < times.size(); ++index) {
    input.push_back(AffineSignal(times[index]));
  }
  std::vector<std::complex<double>> output;
  SlowTimeResamplingDiagnostics diagnostics;
  ASSERT_TRUE(ResampleSlowTimeLinear(times, input, &output, &diagnostics));
  EXPECT_TRUE(diagnostics.valid);
  EXPECT_TRUE(diagnostics.uniform_within_tolerance);
  EXPECT_EQ(diagnostics.nominal_times_s, times);
  EXPECT_EQ(output, input);
}

TEST(SarSlowTimeResamplingTest, DiagnosesNonuniformAxisAndPreservesEndpoints) {
  const std::vector<double> times{0.0, 0.15, 0.55, 0.8, 1.0};
  std::vector<std::complex<double>> input;
  for (std::size_t index = 0U; index < times.size(); ++index) {
    input.push_back(AffineSignal(times[index]));
  }
  std::vector<std::complex<double>> output;
  SlowTimeResamplingDiagnostics diagnostics;
  ASSERT_TRUE(ResampleSlowTimeLinear(times, input, &output, &diagnostics));
  EXPECT_FALSE(diagnostics.uniform_within_tolerance);
  EXPECT_DOUBLE_EQ(diagnostics.nominal_interval_s, 0.25);
  EXPECT_DOUBLE_EQ(diagnostics.nominal_prf_hz, 4.0);
  EXPECT_DOUBLE_EQ(diagnostics.minimum_actual_interval_s, 0.15);
  EXPECT_DOUBLE_EQ(diagnostics.maximum_actual_interval_s, 0.4);
  EXPECT_GT(diagnostics.maximum_abs_time_axis_deviation_s, 0.0);
  EXPECT_EQ(output.front(), input.front());
  EXPECT_EQ(output.back(), input.back());
}

TEST(SarSlowTimeResamplingTest, RecoversComplexAffineAnalyticTruth) {
  const std::vector<double> times{1.0, 1.12, 1.55, 1.81, 2.0};
  std::vector<std::complex<double>> input;
  for (std::size_t index = 0U; index < times.size(); ++index) {
    input.push_back(AffineSignal(times[index]));
  }
  std::vector<std::complex<double>> output;
  SlowTimeResamplingDiagnostics diagnostics;
  ASSERT_TRUE(ResampleSlowTimeLinear(times, input, &output, &diagnostics));
  for (std::size_t index = 0U; index < output.size(); ++index) {
    EXPECT_NEAR(output[index].real(), AffineSignal(diagnostics.nominal_times_s[index]).real(),
                1.0e-12);
    EXPECT_NEAR(output[index].imag(), AffineSignal(diagnostics.nominal_times_s[index]).imag(),
                1.0e-12);
  }
}

TEST(SarSlowTimeResamplingTest, RejectsInvalidInputAndIsDeterministic) {
  const std::vector<double> times{0.0, 0.2, 0.6, 1.0};
  const std::vector<std::complex<double>> input{
      AffineSignal(0.0), AffineSignal(0.2), AffineSignal(0.6), AffineSignal(1.0)};
  std::vector<std::complex<double>> first;
  std::vector<std::complex<double>> second;
  SlowTimeResamplingDiagnostics first_diagnostics;
  SlowTimeResamplingDiagnostics second_diagnostics;
  ASSERT_TRUE(ResampleSlowTimeLinear(times, input, &first, &first_diagnostics));
  ASSERT_TRUE(ResampleSlowTimeLinear(times, input, &second, &second_diagnostics));
  EXPECT_EQ(first, second);
  EXPECT_EQ(first_diagnostics.nominal_times_s, second_diagnostics.nominal_times_s);

  std::vector<double> invalid_times{0.0, 0.2, 0.2, 1.0};
  EXPECT_FALSE(ResampleSlowTimeLinear(invalid_times, input, &first, &first_diagnostics));
  invalid_times = times;
  invalid_times[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ResampleSlowTimeLinear(invalid_times, input, &first, &first_diagnostics));
  EXPECT_FALSE(ResampleSlowTimeLinear(times, {}, &first, &first_diagnostics));
  EXPECT_FALSE(ResampleSlowTimeLinear(times, input, nullptr, &first_diagnostics));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
