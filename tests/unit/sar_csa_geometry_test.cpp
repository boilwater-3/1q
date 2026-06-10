#include <gtest/gtest.h>

#include <cmath>

#include "sar/imaging/SarCsaGeometry.h"

namespace sar {
namespace imaging {
namespace {

CsaGeometryConfig BaseConfig() {
  CsaGeometryConfig config;
  config.range_sample_count = 5U;
  config.azimuth_pulse_count = 5U;
  config.sample_rate_hz = 100.0;
  config.prf_hz = 100.0;
  config.carrier_frequency_hz = 1.0e9;
  config.platform_velocity_mps = 100.0;
  config.reference_range_m = 1000.0;
  return config;
}

TEST(SarCsaGeometryTest, BuildsOddAndEvenUnshiftedFrequencyAxes) {
  CsaGeometryConfig config = BaseConfig();
  CsaGeometryDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateCsaFrequencyGeometry(config, &diagnostics));
  ASSERT_EQ(diagnostics.range_frequencies_hz.size(), 5U);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[0], 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[1], 20.0);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[2], 40.0);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[3], -40.0);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[4], -20.0);

  config.range_sample_count = 4U;
  config.sample_rate_hz = 80.0;
  ASSERT_TRUE(EvaluateCsaFrequencyGeometry(config, &diagnostics));
  ASSERT_EQ(diagnostics.range_frequencies_hz.size(), 4U);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[0], 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[1], 20.0);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[2], 40.0);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[3], -20.0);
}

TEST(SarCsaGeometryTest, ComputesSymmetricDopplerFactorsAndZeroOrigin) {
  CsaGeometryDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateCsaFrequencyGeometry(BaseConfig(), &diagnostics));
  ASSERT_TRUE(diagnostics.valid);
  EXPECT_DOUBLE_EQ(diagnostics.doppler_factors[0], 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.chirp_scaling_factors[0], 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.doppler_factors[1], diagnostics.doppler_factors[4]);
  EXPECT_DOUBLE_EQ(diagnostics.doppler_factors[2], diagnostics.doppler_factors[3]);
  EXPECT_DOUBLE_EQ(diagnostics.chirp_scaling_factors[1],
                   diagnostics.chirp_scaling_factors[4]);
  EXPECT_GT(diagnostics.doppler_factors[1], diagnostics.doppler_factors[2]);
  EXPECT_LT(diagnostics.chirp_scaling_factors[1], diagnostics.chirp_scaling_factors[2]);
}

TEST(SarCsaGeometryTest, DiagnosesBinsOutsideApprovedDomainWithoutNonFiniteValues) {
  CsaGeometryConfig config = BaseConfig();
  config.platform_velocity_mps = 5.0;
  CsaGeometryDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateCsaFrequencyGeometry(config, &diagnostics));
  EXPECT_FALSE(diagnostics.valid);
  EXPECT_GT(diagnostics.invalid_doppler_bin_count, 0U);
  EXPECT_LT(diagnostics.doppler_validity_margin_hz, 0.0);
  for (std::size_t index = 0U; index < diagnostics.doppler_factors.size(); ++index) {
    EXPECT_TRUE(std::isfinite(diagnostics.doppler_factors[index]));
    EXPECT_TRUE(std::isfinite(diagnostics.chirp_scaling_factors[index]));
  }
}

TEST(SarCsaGeometryTest, RejectsInvalidInputAndIsDeterministic) {
  CsaGeometryConfig config = BaseConfig();
  CsaGeometryDiagnostics first;
  CsaGeometryDiagnostics second;
  ASSERT_TRUE(EvaluateCsaFrequencyGeometry(config, &first));
  ASSERT_TRUE(EvaluateCsaFrequencyGeometry(config, &second));
  EXPECT_EQ(first.range_frequencies_hz, second.range_frequencies_hz);
  EXPECT_EQ(first.azimuth_frequencies_hz, second.azimuth_frequencies_hz);
  EXPECT_EQ(first.doppler_factors, second.doppler_factors);
  EXPECT_EQ(first.chirp_scaling_factors, second.chirp_scaling_factors);

  config.reference_range_m = 0.0;
  EXPECT_FALSE(EvaluateCsaFrequencyGeometry(config, &first));
  EXPECT_FALSE(first.valid);
  EXPECT_TRUE(first.range_frequencies_hz.empty());
  EXPECT_FALSE(EvaluateCsaFrequencyGeometry(config, nullptr));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
