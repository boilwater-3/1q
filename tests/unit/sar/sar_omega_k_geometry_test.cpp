#include <gtest/gtest.h>

#include <cmath>

#include "sar/imaging/SarOmegaKGeometry.h"

namespace sar {
namespace imaging {
namespace {

OmegaKGeometryConfig BaseConfig() {
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

std::size_t Offset(std::size_t azimuth, std::size_t range, std::size_t range_count) {
  return azimuth * range_count + range;
}

TEST(SarOmegaKGeometryTest, UsesApprovedUnshiftedFrequencyAxes) {
  OmegaKGeometryDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(BaseConfig(), &diagnostics));
  ASSERT_EQ(diagnostics.range_frequencies_hz.size(), 5U);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[0], 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[1], 4.0e7);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[2], 8.0e7);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[3], -8.0e7);
  EXPECT_DOUBLE_EQ(diagnostics.range_frequencies_hz[4], -4.0e7);
  EXPECT_DOUBLE_EQ(diagnostics.azimuth_frequencies_hz[3], -40.0);
}

TEST(SarOmegaKGeometryTest, HasZeroStoltShiftAtZeroDoppler) {
  const OmegaKGeometryConfig config = BaseConfig();
  OmegaKGeometryDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(config, &diagnostics));
  for (std::size_t range = 0U; range < config.range_sample_count; ++range) {
    const std::size_t index = Offset(0U, range, config.range_sample_count);
    EXPECT_NEAR(diagnostics.propagation_wavenumbers_rad_per_m[index],
                diagnostics.range_wavenumbers_rad_per_m[range], 1.0e-12);
    EXPECT_NEAR(diagnostics.stolt_shifts_hz[index], 0.0, 1.0e-7);
  }
}

TEST(SarOmegaKGeometryTest, IsSymmetricAndShiftGrowsWithAbsoluteDoppler) {
  const OmegaKGeometryConfig config = BaseConfig();
  OmegaKGeometryDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(config, &diagnostics));
  const std::size_t range = 0U;
  const std::size_t positive_low = Offset(1U, range, config.range_sample_count);
  const std::size_t positive_high = Offset(2U, range, config.range_sample_count);
  const std::size_t negative_high = Offset(3U, range, config.range_sample_count);
  const std::size_t negative_low = Offset(4U, range, config.range_sample_count);
  EXPECT_DOUBLE_EQ(diagnostics.propagation_wavenumbers_rad_per_m[positive_low],
                   diagnostics.propagation_wavenumbers_rad_per_m[negative_low]);
  EXPECT_DOUBLE_EQ(diagnostics.stolt_shifts_hz[positive_high],
                   diagnostics.stolt_shifts_hz[negative_high]);
  EXPECT_GT(diagnostics.propagation_wavenumbers_rad_per_m[positive_low],
            diagnostics.propagation_wavenumbers_rad_per_m[positive_high]);
  EXPECT_LT(diagnostics.stolt_shifts_hz[positive_low],
            diagnostics.stolt_shifts_hz[positive_high]);
}

TEST(SarOmegaKGeometryTest, SeparatelyDiagnosesDispersionAndSupportFailures) {
  OmegaKGeometryConfig config = BaseConfig();
  config.platform_velocity_mps = 1.0;
  OmegaKGeometryDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(config, &diagnostics));
  EXPECT_FALSE(diagnostics.valid);
  EXPECT_GT(diagnostics.invalid_dispersion_point_count, 0U);
  EXPECT_GT(diagnostics.out_of_support_stolt_query_count, 0U);
  for (std::size_t index = 0U; index < diagnostics.stolt_shifts_hz.size(); ++index) {
    EXPECT_TRUE(std::isfinite(diagnostics.propagation_wavenumbers_rad_per_m[index]));
    EXPECT_TRUE(std::isfinite(diagnostics.source_range_frequency_queries_hz[index]));
    EXPECT_TRUE(std::isfinite(diagnostics.stolt_shifts_hz[index]));
  }
}

TEST(SarOmegaKGeometryTest, RejectsInvalidInputAndIsDeterministic) {
  OmegaKGeometryConfig config = BaseConfig();
  OmegaKGeometryDiagnostics first;
  OmegaKGeometryDiagnostics second;
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(config, &first));
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(config, &second));
  EXPECT_EQ(first.propagation_wavenumbers_rad_per_m,
            second.propagation_wavenumbers_rad_per_m);
  EXPECT_EQ(first.source_range_frequency_queries_hz,
            second.source_range_frequency_queries_hz);

  config.reference_range_m = 0.0;
  EXPECT_FALSE(EvaluateOmegaKStoltGeometry(config, &first));
  EXPECT_FALSE(first.valid);
  EXPECT_TRUE(first.range_frequencies_hz.empty());
  EXPECT_FALSE(EvaluateOmegaKStoltGeometry(config, nullptr));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
