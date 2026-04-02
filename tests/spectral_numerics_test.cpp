// Copyright 2026. All Rights Reserved.
//
// @file spectral_numerics_test.cpp
// @brief 验证频谱基础数值入口的可用性与一致性。

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <complex>
#include <vector>

#include "common/numerics/SpectralNumerics.h"

namespace oneq {
namespace tests {

namespace {

constexpr double kTolerance = 1.0e-6;

}  // namespace

TEST(SpectralNumericsTest, RfftForwardBackwardRoundTripMatchesInput) {
  common::numerics::RfftPlan plan;
  ASSERT_TRUE(common::numerics::RFFTI(8U, &plan));

  const std::vector<double> input = {1.0, 2.0, 0.0, -1.0, 0.5, 0.0, 0.0, 0.0};
  std::vector<std::complex<double>> spectrum;
  ASSERT_TRUE(common::numerics::RFFTF(plan, input, &spectrum));

  std::vector<double> reconstructed;
  ASSERT_TRUE(common::numerics::RFFTB(plan, spectrum, &reconstructed));
  ASSERT_EQ(reconstructed.size(), input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    EXPECT_NEAR(reconstructed[i], input[i], kTolerance);
  }
}

TEST(SpectralNumericsTest, ZfftMatchesRealFftPathForRealSequence) {
  const std::vector<double> input = {1.0, -1.0, 0.0, 0.5, 0.25, 0.0, 0.0, 0.0};
  common::numerics::RfftPlan plan;
  ASSERT_TRUE(common::numerics::RFFTI(input.size(), &plan));

  std::vector<std::complex<double>> real_fft_spectrum;
  ASSERT_TRUE(common::numerics::RFFTF(plan, input, &real_fft_spectrum));

  std::vector<std::complex<double>> complex_input(input.size(), std::complex<double>(0.0, 0.0));
  for (std::size_t i = 0; i < input.size(); ++i) {
    complex_input[i] = std::complex<double>(input[i], 0.0);
  }
  const std::vector<std::complex<double>> complex_fft_spectrum =
      common::numerics::ZFFT1D(complex_input, false);
  ASSERT_EQ(real_fft_spectrum.size(), complex_fft_spectrum.size());
  for (std::size_t i = 0; i < real_fft_spectrum.size(); ++i) {
    EXPECT_NEAR(real_fft_spectrum[i].real(), complex_fft_spectrum[i].real(), kTolerance);
    EXPECT_NEAR(real_fft_spectrum[i].imag(), complex_fft_spectrum[i].imag(), kTolerance);
  }
}

TEST(SpectralNumericsTest, LstsqsSolvesOverdeterminedStableSystem) {
  Eigen::MatrixXd a(4, 2);
  a << 1.0, 0.0, 1.0, 1.0, 1.0, 2.0, 1.0, 3.0;
  Eigen::VectorXd b(4);
  b << 1.0, 2.0, 3.0, 4.0;

  Eigen::VectorXd solution;
  ASSERT_TRUE(common::numerics::lstsqs(a, b, &solution));
  ASSERT_EQ(solution.size(), 2);
  EXPECT_NEAR(solution(0), 1.0, 1.0e-4);
  EXPECT_NEAR(solution(1), 1.0, 1.0e-4);
}

TEST(SpectralNumericsTest, LstsqsReturnsFiniteForNearSingularInput) {
  Eigen::MatrixXd a(3, 2);
  a << 1.0, 1.0, 2.0, 2.0 + 1.0e-9, 3.0, 3.0 + 2.0e-9;
  Eigen::VectorXd b(3);
  b << 2.0, 4.0, 6.0;

  Eigen::VectorXd solution;
  ASSERT_TRUE(common::numerics::lstsqs(a, b, &solution));
  EXPECT_TRUE(solution.allFinite());
}

TEST(SpectralNumericsTest, MarpleSpectReturnsNonNegativeAndDetectsPeak) {
  const std::size_t fft_length = 16U;
  std::vector<double> signal(fft_length, 0.0);
  for (std::size_t i = 0; i < fft_length; ++i) {
    signal[i] = std::sin(2.0 * 3.14159265358979323846 * 2.0 * static_cast<double>(i) /
                         static_cast<double>(fft_length));
  }

  std::vector<double> power_spectrum;
  ASSERT_TRUE(common::numerics::marple_spect(signal, fft_length, &power_spectrum));
  ASSERT_FALSE(power_spectrum.empty());
  for (std::size_t i = 0; i < power_spectrum.size(); ++i) {
    EXPECT_GE(power_spectrum[i], 0.0);
  }

  std::size_t peak_index = 0U;
  double peak_value = power_spectrum[0];
  for (std::size_t i = 1; i < power_spectrum.size(); ++i) {
    if (power_spectrum[i] > peak_value) {
      peak_value = power_spectrum[i];
      peak_index = i;
    }
  }
  EXPECT_EQ(peak_index, 2U);
}

}  // namespace tests
}  // namespace oneq
