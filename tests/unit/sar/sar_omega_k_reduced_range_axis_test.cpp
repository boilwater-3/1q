#include <gtest/gtest.h>

#include "sar/imaging/SarOmegaKReducedRangeAxis.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {
namespace {

TEST(SarOmegaKReducedRangeAxisTest, BuildsRelativeDelayAxisForUniformReducedGrid) {
  OmegaKReducedRangeAxisDiagnostics diagnostics;
  ASSERT_TRUE(DiagnoseOmegaKReducedRangeAxis({-40.0, -20.0, 0.0, 20.0}, &diagnostics));
  EXPECT_TRUE(diagnostics.valid);
  EXPECT_DOUBLE_EQ(diagnostics.frequency_spacing_hz, 20.0);
  EXPECT_DOUBLE_EQ(diagnostics.effective_bandwidth_hz, 60.0);
  EXPECT_DOUBLE_EQ(diagnostics.unambiguous_delay_window_s, 0.05);
  EXPECT_DOUBLE_EQ(diagnostics.relative_delay_spacing_s, 0.0125);
  ASSERT_EQ(diagnostics.relative_delays_s.size(), 4U);
  EXPECT_DOUBLE_EQ(diagnostics.relative_delays_s[3], 0.0375);
}

TEST(SarOmegaKReducedRangeAxisTest, MatchesFftFacadeInverseRoundTripConvention) {
  signal::ComplexVector spectrum{
      {1.0, 0.0}, {0.0, 1.0}, {-2.0, 0.5}, {3.0, -1.0}};
  signal::ComplexVector relative_delay_samples;
  signal::ComplexVector restored_spectrum;
  ASSERT_TRUE(signal::Fft1D(spectrum, true, &relative_delay_samples));
  ASSERT_TRUE(signal::Fft1D(relative_delay_samples, false, &restored_spectrum));
  ASSERT_EQ(restored_spectrum.size(), spectrum.size());
  for (std::size_t index = 0U; index < spectrum.size(); ++index) {
    EXPECT_NEAR(restored_spectrum[index].real(), spectrum[index].real(), 1.0e-12);
    EXPECT_NEAR(restored_spectrum[index].imag(), spectrum[index].imag(), 1.0e-12);
  }
}

TEST(SarOmegaKReducedRangeAxisTest, RejectsNonuniformAndInvalidAxes) {
  OmegaKReducedRangeAxisDiagnostics diagnostics;
  EXPECT_FALSE(DiagnoseOmegaKReducedRangeAxis({0.0}, &diagnostics));
  EXPECT_FALSE(DiagnoseOmegaKReducedRangeAxis({0.0, 20.0, 41.0}, &diagnostics));
  EXPECT_FALSE(DiagnoseOmegaKReducedRangeAxis({0.0, 20.0, 20.0}, &diagnostics));
  EXPECT_FALSE(DiagnoseOmegaKReducedRangeAxis({20.0, 0.0}, &diagnostics));
  EXPECT_FALSE(DiagnoseOmegaKReducedRangeAxis({0.0, 20.0}, nullptr));
}

TEST(SarOmegaKReducedRangeAxisTest, IsDeterministic) {
  OmegaKReducedRangeAxisDiagnostics first;
  OmegaKReducedRangeAxisDiagnostics second;
  ASSERT_TRUE(DiagnoseOmegaKReducedRangeAxis({-40.0, -20.0, 0.0, 20.0}, &first));
  ASSERT_TRUE(DiagnoseOmegaKReducedRangeAxis({-40.0, -20.0, 0.0, 20.0}, &second));
  EXPECT_EQ(first.relative_delays_s, second.relative_delays_s);
  EXPECT_DOUBLE_EQ(first.relative_range_spacing_m, second.relative_range_spacing_m);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
