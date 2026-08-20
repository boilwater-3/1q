#include <gtest/gtest.h>

#include <cmath>
#include <complex>

#include "sar/signal/SarFft.h"

namespace sar {
namespace signal {
namespace {

constexpr double kTolerance = 1.0e-9;
constexpr double kPi = 3.141592653589793238462643383279502884;

void ExpectNear(const ComplexSample& actual, const ComplexSample& expected,
                double tolerance = kTolerance) {
  EXPECT_NEAR(actual.real(), expected.real(), tolerance);
  EXPECT_NEAR(actual.imag(), expected.imag(), tolerance);
}

ComplexVector MakeSinusoid(std::size_t size, std::size_t bin) {
  ComplexVector signal(size);
  for (std::size_t n = 0; n < size; ++n) {
    const double angle = 2.0 * kPi * static_cast<double>(bin * n) / static_cast<double>(size);
    signal[n] = ComplexSample(std::cos(angle), std::sin(angle));
  }
  return signal;
}

TEST(SarFftBackendTest, ComplexRoundTripRestoresInput) {
  const ComplexVector input{{1.0, 0.5},  {-2.0, 3.0}, {0.25, -0.75}, {4.0, 0.0},
                            {-1.5, 2.5}, {0.0, -3.0}, {2.25, 1.0}};

  ComplexVector spectrum;
  ASSERT_TRUE(Fft1D(input, false, &spectrum));
  ComplexVector reconstructed;
  ASSERT_TRUE(Fft1D(spectrum, true, &reconstructed));

  ASSERT_EQ(reconstructed.size(), input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    ExpectNear(reconstructed[i], input[i]);
  }
}

TEST(SarFftBackendTest, DeltaPulseProducesFlatSpectrum) {
  ComplexVector input(8U, ComplexSample(0.0, 0.0));
  input[0] = ComplexSample(1.0, 0.0);

  ComplexVector spectrum;
  ASSERT_TRUE(Fft1D(input, false, &spectrum));

  ASSERT_EQ(spectrum.size(), input.size());
  for (const ComplexSample& value : spectrum) {
    ExpectNear(value, ComplexSample(1.0, 0.0));
  }
}

TEST(SarFftBackendTest, KnownSinusoidPeaksAtExpectedBin) {
  const std::size_t size = 16U;
  const std::size_t expected_bin = 3U;
  const ComplexVector input = MakeSinusoid(size, expected_bin);

  ComplexVector spectrum;
  ASSERT_TRUE(Fft1D(input, false, &spectrum));

  ASSERT_EQ(spectrum.size(), size);
  for (std::size_t bin = 0; bin < size; ++bin) {
    const double expected_magnitude = (bin == expected_bin) ? static_cast<double>(size) : 0.0;
    EXPECT_NEAR(std::abs(spectrum[bin]), expected_magnitude, 1.0e-8);
  }
}

TEST(SarFftBackendTest, RowTransformOnlyChangesRowAxis) {
  ComplexMatrix input;
  input.rows = 2U;
  input.cols = 4U;
  input.values = {
      {1.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
      {0.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
  };

  ComplexMatrix spectrum;
  ASSERT_TRUE(FftRows(input, false, &spectrum));

  ASSERT_EQ(spectrum.rows, 2U);
  ASSERT_EQ(spectrum.cols, 4U);
  for (std::size_t col = 0; col < 4U; ++col) {
    ExpectNear(spectrum(0U, col), ComplexSample(1.0, 0.0));
  }
  ExpectNear(spectrum(1U, 0U), ComplexSample(1.0, 0.0));
  ExpectNear(spectrum(1U, 1U), ComplexSample(0.0, -1.0));
  ExpectNear(spectrum(1U, 2U), ComplexSample(-1.0, 0.0));
  ExpectNear(spectrum(1U, 3U), ComplexSample(0.0, 1.0));
}

TEST(SarFftBackendTest, ColumnTransformOnlyChangesColumnAxis) {
  ComplexMatrix input;
  input.rows = 4U;
  input.cols = 2U;
  input.values = {
      {1.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0},
      {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
  };

  ComplexMatrix spectrum;
  ASSERT_TRUE(FftCols(input, false, &spectrum));

  ASSERT_EQ(spectrum.rows, 4U);
  ASSERT_EQ(spectrum.cols, 2U);
  for (std::size_t row = 0; row < 4U; ++row) {
    ExpectNear(spectrum(row, 0U), ComplexSample(1.0, 0.0));
  }
  ExpectNear(spectrum(0U, 1U), ComplexSample(1.0, 0.0));
  ExpectNear(spectrum(1U, 1U), ComplexSample(0.0, -1.0));
  ExpectNear(spectrum(2U, 1U), ComplexSample(-1.0, 0.0));
  ExpectNear(spectrum(3U, 1U), ComplexSample(0.0, 1.0));
}

TEST(SarFftBackendTest, NonPowerOfTwoLengthRoundTripIsSupported) {
  const ComplexVector input = MakeSinusoid(10U, 2U);

  ComplexVector spectrum;
  ASSERT_TRUE(Fft1D(input, false, &spectrum));
  ComplexVector reconstructed;
  ASSERT_TRUE(Fft1D(spectrum, true, &reconstructed));

  ASSERT_EQ(reconstructed.size(), input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    ExpectNear(reconstructed[i], input[i], 1.0e-8);
  }
}

TEST(SarFftBackendTest, RejectsInvalidInputs) {
  ComplexVector vector_output;
  EXPECT_FALSE(Fft1D({}, false, &vector_output));
  EXPECT_FALSE(Fft1D(ComplexVector(4U), false, nullptr));

  ComplexMatrix malformed;
  malformed.rows = 2U;
  malformed.cols = 2U;
  malformed.values.resize(3U);
  ComplexMatrix matrix_output;
  EXPECT_FALSE(FftRows(malformed, false, &matrix_output));
  EXPECT_FALSE(FftCols(malformed, false, &matrix_output));
}

}  // namespace
}  // namespace signal
}  // namespace sar
