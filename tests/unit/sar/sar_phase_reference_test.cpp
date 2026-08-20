#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "sar/imaging/SarPhaseReference.h"

namespace sar {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;

signal::ComplexMatrix MakeImage(std::size_t rows, std::size_t cols) {
  signal::ComplexMatrix image;
  image.rows = rows;
  image.cols = cols;
  image.values.assign(rows * cols, signal::ComplexSample(1.0, 0.0));
  return image;
}

TEST(SarPhaseReferenceTest, NativeModeDoesNotModifyImage) {
  signal::ComplexMatrix image = MakeImage(2U, 3U);
  image(1U, 2U) = signal::ComplexSample(0.5, -0.25);
  const signal::ComplexMatrix original = image;

  imaging::PhaseReferenceConfig config;
  config.mode = imaging::PhaseReferenceMode::kNative;
  imaging::PhaseReferenceDiagnostics diagnostics;
  ASSERT_TRUE(imaging::ApplyBroadsideCenterPhaseReference(config, &image, &diagnostics));

  EXPECT_FALSE(diagnostics.applied);
  EXPECT_EQ(image.values, original.values);
}

TEST(SarPhaseReferenceTest, BroadsideCenterReferenceMatchesContractedPhase) {
  signal::ComplexMatrix image = MakeImage(3U, 4U);
  image(2U, 3U) = signal::ComplexSample(2.0, 0.0);

  imaging::PhaseReferenceConfig config;
  config.mode = imaging::PhaseReferenceMode::kCenterBroadside;
  config.carrier_frequency_hz = 10.0e9;
  config.prf_hz = 1000.0;
  config.platform_velocity_mps = 120.0;
  config.range_bin_spacing_m = 1.5;

  imaging::PhaseReferenceDiagnostics diagnostics;
  ASSERT_TRUE(imaging::ApplyBroadsideCenterPhaseReference(config, &image, &diagnostics));

  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  const double center_row = 1.0;
  for (std::size_t row = 0U; row < image.rows; ++row) {
    const double slow_time_s = (static_cast<double>(row) - center_row) / config.prf_hz;
    const double x_m = config.platform_velocity_mps * slow_time_s;
    for (std::size_t col = 0U; col < image.cols; ++col) {
      const double range_m =
          std::max(static_cast<double>(col) * config.range_bin_spacing_m,
                   0.5 * config.range_bin_spacing_m);
      const double slant_m = std::sqrt(range_m * range_m + x_m * x_m);
      const double phase = 4.0 * kPi * slant_m / wavelength_m;
      const double expected_magnitude = (row == 2U && col == 3U) ? 2.0 : 1.0;
      EXPECT_NEAR(std::abs(image(row, col)), expected_magnitude, 1.0e-12);
      EXPECT_NEAR(std::real(image(row, col)), expected_magnitude * std::cos(phase), 1.0e-12);
      EXPECT_NEAR(std::imag(image(row, col)), expected_magnitude * std::sin(phase), 1.0e-12);
    }
  }
  EXPECT_TRUE(diagnostics.applied);
  EXPECT_EQ(diagnostics.mode, imaging::PhaseReferenceMode::kCenterBroadside);
  EXPECT_LE(diagnostics.min_phase_rad, diagnostics.max_phase_rad);
}

TEST(SarPhaseReferenceTest, RejectsInvalidBroadsideReferenceInputAtomically) {
  signal::ComplexMatrix image = MakeImage(2U, 2U);
  const signal::ComplexMatrix original = image;

  imaging::PhaseReferenceConfig config;
  config.mode = imaging::PhaseReferenceMode::kCenterBroadside;
  config.carrier_frequency_hz = 0.0;
  config.prf_hz = 1000.0;
  config.platform_velocity_mps = 120.0;
  config.range_bin_spacing_m = 1.5;

  imaging::PhaseReferenceDiagnostics diagnostics;
  EXPECT_FALSE(imaging::ApplyBroadsideCenterPhaseReference(config, &image, &diagnostics));
  EXPECT_EQ(image.values, original.values);
}

TEST(SarPhaseReferenceTest, EstimatesAndAppliesOnlyGlobalPhaseOffset) {
  signal::ComplexMatrix reference = MakeImage(1U, 3U);
  reference.values = {signal::ComplexSample(1.0, 0.0), signal::ComplexSample(0.0, 2.0),
                      signal::ComplexSample(-1.0, 0.5)};
  signal::ComplexMatrix candidate = reference;
  const double applied_phase = 0.6;
  const signal::ComplexSample rotation(std::cos(applied_phase), std::sin(applied_phase));
  for (signal::ComplexSample& sample : candidate.values) {
    sample *= rotation;
  }

  double phase_offset_rad = 0.0;
  ASSERT_TRUE(imaging::EstimateGlobalPhaseOffset(reference, candidate, &phase_offset_rad));
  EXPECT_NEAR(phase_offset_rad, -applied_phase, 1.0e-12);
  ASSERT_TRUE(imaging::ApplyGlobalPhaseOffset(phase_offset_rad, &candidate));
  for (std::size_t index = 0U; index < reference.values.size(); ++index) {
    EXPECT_NEAR(std::abs(reference.values[index] - candidate.values[index]), 0.0, 1.0e-12);
  }
}

}  // namespace
}  // namespace sar
