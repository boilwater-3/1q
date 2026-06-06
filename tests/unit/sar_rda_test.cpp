#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarRda.h"
#include "sar/signal/SarWaveform.h"
#include "support/sar_reference_scene.h"

namespace sar {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;

using RdaSceneFixture = test_support::ReferencePointScene;

echo::PointTarget MakeTargetAtDelay(std::size_t delay_sample, double sample_rate_hz,
                                    double desired_amplitude) {
  return test_support::MakeReferenceTargetAtDelay(delay_sample, sample_rate_hz, desired_amplitude);
}

RdaSceneFixture MakeFixture() {
  RdaSceneFixture fixture;
  EXPECT_TRUE(test_support::BuildReferencePointScene(&fixture));
  return fixture;
}

signal::ComplexMatrix BuildRawHistory(const RdaSceneFixture& fixture,
                                      const std::vector<echo::PointTarget>& targets) {
  signal::ComplexMatrix history;
  EXPECT_TRUE(test_support::BuildReferenceRawHistory(fixture, targets, &history));
  return history;
}

imaging::RdaConfig MakeRdaConfig(const RdaSceneFixture& fixture, std::size_t reference_delay) {
  return test_support::MakeReferenceRdaConfig(fixture, reference_delay);
}

TEST(SarRdaTest, SingleCenterPointFocusesNearExpectedPixel) {
  const RdaSceneFixture fixture = MakeFixture();
  const std::size_t expected_delay = 20U;
  const signal::ComplexMatrix raw_history =
      BuildRawHistory(fixture, {MakeTargetAtDelay(expected_delay, fixture.sample_rate_hz, 1.0)});

  imaging::FocusedSarImage focused;
  ASSERT_TRUE(imaging::FocusStripmapRda(MakeRdaConfig(fixture, expected_delay), raw_history,
                                        fixture.matched_filter, &focused));

  const std::size_t peak_index = imaging::FindPeakIndex(focused.image);
  const std::size_t peak_row = peak_index / focused.image.cols;
  const std::size_t peak_col = peak_index % focused.image.cols;
  EXPECT_EQ(peak_row, focused.image.rows / 2U);
  EXPECT_NEAR(static_cast<double>(peak_col), static_cast<double>(expected_delay), 1.0);
  EXPECT_GT(std::abs(focused.image(peak_row, peak_col)), 0.0);
  double off_peak_row_energy = 0.0;
  for (std::size_t row = 0U; row < focused.image.rows; ++row) {
    if (row != peak_row) {
      off_peak_row_energy += std::norm(focused.image(row, peak_col));
    }
  }
  EXPECT_GT(off_peak_row_energy, 0.0);
}

TEST(SarRdaTest, ThreeSeparatedTargetsRemainResolvableWithExpectedPeakOrder) {
  const RdaSceneFixture fixture = MakeFixture();
  const std::vector<std::size_t> expected_bins{12U, 24U, 36U};
  const signal::ComplexMatrix raw_history =
      BuildRawHistory(fixture, {MakeTargetAtDelay(expected_bins[0], fixture.sample_rate_hz, 3.0),
                                MakeTargetAtDelay(expected_bins[1], fixture.sample_rate_hz, 2.0),
                                MakeTargetAtDelay(expected_bins[2], fixture.sample_rate_hz, 1.0)});

  imaging::FocusedSarImage focused;
  ASSERT_TRUE(imaging::FocusStripmapRda(MakeRdaConfig(fixture, expected_bins[1]), raw_history,
                                        fixture.matched_filter, &focused));

  const std::size_t focused_row = imaging::FindPeakIndex(focused.image) / focused.image.cols;
  const double first = std::abs(focused.image(focused_row, expected_bins[0]));
  const double second = std::abs(focused.image(focused_row, expected_bins[1]));
  const double third = std::abs(focused.image(focused_row, expected_bins[2]));
  EXPECT_GT(first, second);
  EXPECT_GT(second, third);
  EXPECT_GT(third, 0.0);
}

TEST(SarRdaTest, DiagnosticsRecordRdaStagesAndReferenceParameters) {
  const RdaSceneFixture fixture = MakeFixture();
  const std::size_t expected_delay = 18U;
  const imaging::RdaConfig config = MakeRdaConfig(fixture, expected_delay);
  const signal::ComplexMatrix raw_history =
      BuildRawHistory(fixture, {MakeTargetAtDelay(expected_delay, fixture.sample_rate_hz, 1.0)});

  imaging::FocusedSarImage focused;
  ASSERT_TRUE(imaging::FocusStripmapRda(config, raw_history, fixture.matched_filter, &focused));

  EXPECT_TRUE(focused.diagnostics.range_compression_applied);
  EXPECT_TRUE(focused.diagnostics.azimuth_fft_applied);
  EXPECT_TRUE(focused.diagnostics.azimuth_matched_filter_applied);
  EXPECT_TRUE(focused.diagnostics.azimuth_ifft_applied);
  EXPECT_EQ(focused.diagnostics.rcmc_interpolation, "linear");
  EXPECT_DOUBLE_EQ(focused.diagnostics.reference_range_m, config.reference_range_m);
  EXPECT_NEAR(focused.diagnostics.range_bin_spacing_m,
              kSpeedOfLightMps / (2.0 * fixture.sample_rate_hz), 1.0e-12);
  EXPECT_GT(focused.diagnostics.doppler_rate_hz_per_s, 0.0);
  const double expected_spacing_m = config.platform_velocity_mps / config.prf_hz;
  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  const double expected_curvature_rad =
      4.0 * 3.14159265358979323846 * expected_spacing_m * expected_spacing_m /
      (wavelength_m * config.reference_range_m);
  const double aperture_edge_m =
      0.5 * static_cast<double>(raw_history.rows - 1U) * expected_spacing_m;
  const double expected_max_doppler_hz =
      2.0 * config.platform_velocity_mps * aperture_edge_m /
      (wavelength_m *
       std::sqrt(config.reference_range_m * config.reference_range_m +
                 aperture_edge_m * aperture_edge_m));
  EXPECT_DOUBLE_EQ(focused.diagnostics.azimuth_sample_spacing_m, expected_spacing_m);
  EXPECT_NEAR(focused.diagnostics.azimuth_phase_curvature_rad_per_pulse2,
              expected_curvature_rad, 1.0e-15);
  EXPECT_NEAR(focused.diagnostics.max_geometric_doppler_hz, expected_max_doppler_hz, 1.0e-15);
  EXPECT_NEAR(focused.diagnostics.doppler_nyquist_margin,
              0.5 * config.prf_hz / expected_max_doppler_hz, 1.0e-12);
  EXPECT_GT(focused.diagnostics.azimuth_width_3db_bins, 0.0);
  EXPECT_TRUE(std::isfinite(focused.diagnostics.image_entropy_nats));
  EXPECT_GE(focused.diagnostics.image_entropy_nats, 0.0);
  EXPECT_EQ(focused.image.rows, fixture.pulse_count);
  EXPECT_EQ(focused.image.cols, fixture.range_sample_count);
}

TEST(SarRdaTest, DiagnosticsPreserveEquivalentAzimuthSpacingAndPhaseCurvature) {
  RdaSceneFixture baseline = MakeFixture();
  const std::size_t expected_delay = 20U;
  const signal::ComplexMatrix baseline_raw =
      BuildRawHistory(baseline, {MakeTargetAtDelay(expected_delay, baseline.sample_rate_hz, 1.0)});
  imaging::FocusedSarImage baseline_focused;
  ASSERT_TRUE(imaging::FocusStripmapRda(MakeRdaConfig(baseline, expected_delay), baseline_raw,
                                        baseline.matched_filter, &baseline_focused));

  RdaSceneFixture equivalent = baseline;
  equivalent.platform_velocity_mps *= 2.0;
  equivalent.prf_hz *= 2.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&equivalent));
  const signal::ComplexMatrix equivalent_raw = BuildRawHistory(
      equivalent, {MakeTargetAtDelay(expected_delay, equivalent.sample_rate_hz, 1.0)});
  imaging::FocusedSarImage equivalent_focused;
  ASSERT_TRUE(imaging::FocusStripmapRda(MakeRdaConfig(equivalent, expected_delay), equivalent_raw,
                                        equivalent.matched_filter, &equivalent_focused));

  EXPECT_DOUBLE_EQ(baseline_focused.diagnostics.azimuth_sample_spacing_m,
                   equivalent_focused.diagnostics.azimuth_sample_spacing_m);
  EXPECT_DOUBLE_EQ(baseline_focused.diagnostics.azimuth_phase_curvature_rad_per_pulse2,
                   equivalent_focused.diagnostics.azimuth_phase_curvature_rad_per_pulse2);
}

TEST(SarRdaTest, SinglePulseDiagnosticsUseInfiniteNyquistMargin) {
  const RdaSceneFixture fixture = MakeFixture();
  imaging::RdaDiagnostics diagnostics;
  ASSERT_TRUE(imaging::ComputeRdaSamplingDiagnostics(MakeRdaConfig(fixture, 20U), 1U,
                                                     &diagnostics));

  EXPECT_DOUBLE_EQ(diagnostics.max_geometric_doppler_hz, 0.0);
  EXPECT_TRUE(std::isinf(diagnostics.doppler_nyquist_margin));
}

TEST(SarRdaTest, RejectsSinglePulseImagingAperture) {
  RdaSceneFixture fixture;
  fixture.pulse_count = 1U;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&fixture));
  const std::size_t expected_delay = 20U;
  const signal::ComplexMatrix raw_history =
      BuildRawHistory(fixture, {MakeTargetAtDelay(expected_delay, fixture.sample_rate_hz, 1.0)});
  imaging::FocusedSarImage focused;

  EXPECT_FALSE(imaging::FocusStripmapRda(MakeRdaConfig(fixture, expected_delay), raw_history,
                                         fixture.matched_filter, &focused));
}

TEST(SarRdaTest, AzimuthWidthUsesPeakRangeColumnAndContiguousHalfPowerRows) {
  signal::ComplexMatrix image;
  image.rows = 5U;
  image.cols = 2U;
  image.values.assign(image.rows * image.cols, signal::ComplexSample(0.0, 0.0));
  image(0U, 1U) = signal::ComplexSample(0.6, 0.0);
  image(1U, 1U) = signal::ComplexSample(0.8, 0.0);
  image(2U, 1U) = signal::ComplexSample(1.0, 0.0);
  image(3U, 1U) = signal::ComplexSample(0.75, 0.0);
  image(4U, 1U) = signal::ComplexSample(0.4, 0.0);
  image(0U, 0U) = signal::ComplexSample(0.9, 0.0);

  EXPECT_DOUBLE_EQ(imaging::EstimateAzimuthWidth3dbBins(image), 3.0);

  signal::ComplexMatrix empty;
  EXPECT_DOUBLE_EQ(imaging::EstimateAzimuthWidth3dbBins(empty), 0.0);
}

TEST(SarRdaTest, ImageEntropyUsesNormalizedPowerDistribution) {
  signal::ComplexMatrix single_peak;
  single_peak.rows = 1U;
  single_peak.cols = 4U;
  single_peak.values = {signal::ComplexSample(2.0, 0.0), signal::ComplexSample(0.0, 0.0),
                        signal::ComplexSample(0.0, 0.0), signal::ComplexSample(0.0, 0.0)};
  EXPECT_DOUBLE_EQ(imaging::EstimateImageEntropyNats(single_peak), 0.0);

  signal::ComplexMatrix uniform;
  uniform.rows = 1U;
  uniform.cols = 4U;
  uniform.values.assign(4U, signal::ComplexSample(1.0, 0.0));
  EXPECT_NEAR(imaging::EstimateImageEntropyNats(uniform), std::log(4.0), 1.0e-12);

  signal::ComplexMatrix empty;
  EXPECT_DOUBLE_EQ(imaging::EstimateImageEntropyNats(empty), 0.0);
}

TEST(SarRdaTest, SincRcmcImprovesFractionalBandlimitedSampleAccuracy) {
  signal::ComplexMatrix input;
  input.rows = 1U;
  input.cols = 64U;
  input.values.resize(input.cols);
  const double angular_frequency = 1.2;
  for (std::size_t col = 0U; col < input.cols; ++col) {
    const double phase = angular_frequency * static_cast<double>(col);
    input(0U, col) = signal::ComplexSample(std::cos(phase), std::sin(phase));
  }

  const double fractional_shift = 0.35;
  const std::vector<double> shifts(1U, fractional_shift);
  signal::ComplexMatrix linear;
  signal::ComplexMatrix sinc;
  std::size_t linear_out_of_bounds = 0U;
  std::size_t sinc_out_of_bounds = 0U;
  ASSERT_TRUE(imaging::ApplyRangeMigrationCorrection(
      input, shifts, imaging::RcmcInterpolation::kLinear, 4U, &linear, &linear_out_of_bounds));
  ASSERT_TRUE(imaging::ApplyRangeMigrationCorrection(
      input, shifts, imaging::RcmcInterpolation::kSinc, 4U, &sinc, &sinc_out_of_bounds));

  double linear_error = 0.0;
  double sinc_error = 0.0;
  for (std::size_t col = 8U; col < 56U; ++col) {
    const double phase = angular_frequency * (static_cast<double>(col) + fractional_shift);
    const signal::ComplexSample expected(std::cos(phase), std::sin(phase));
    linear_error += std::norm(linear(0U, col) - expected);
    sinc_error += std::norm(sinc(0U, col) - expected);
  }
  EXPECT_LT(sinc_error, linear_error);
  EXPECT_EQ(linear_out_of_bounds, 1U);
  EXPECT_GT(sinc_out_of_bounds, 0U);
}

TEST(SarRdaTest, SincRcmcRecordsDiagnosticsWithoutChangingLinearDefaultPath) {
  const RdaSceneFixture fixture = MakeFixture();
  const std::size_t expected_delay = 20U;
  const signal::ComplexMatrix raw_history =
      BuildRawHistory(fixture, {MakeTargetAtDelay(expected_delay, fixture.sample_rate_hz, 1.0)});
  imaging::RdaConfig config = MakeRdaConfig(fixture, expected_delay);
  config.rcmc_interpolation = imaging::RcmcInterpolation::kSinc;

  imaging::FocusedSarImage focused;
  ASSERT_TRUE(imaging::FocusStripmapRda(config, raw_history, fixture.matched_filter, &focused));
  EXPECT_EQ(focused.diagnostics.rcmc_interpolation, "sinc");
  EXPECT_NEAR(static_cast<double>(imaging::FindPeakIndex(focused.image) % focused.image.cols),
              static_cast<double>(expected_delay), 1.0);
}

TEST(SarRdaTest, RejectsInvalidInputs) {
  imaging::FocusedSarImage focused;
  EXPECT_FALSE(imaging::FocusStripmapRda(imaging::RdaConfig{}, signal::ComplexMatrix{},
                                         signal::ComplexVector{}, &focused));
  EXPECT_FALSE(imaging::FocusStripmapRda(imaging::RdaConfig{}, signal::ComplexMatrix{},
                                         signal::ComplexVector{}, nullptr));
  imaging::RdaConfig invalid_sinc = MakeRdaConfig(MakeFixture(), 20U);
  invalid_sinc.rcmc_interpolation = imaging::RcmcInterpolation::kSinc;
  invalid_sinc.sinc_half_width = 1U;
  EXPECT_FALSE(imaging::FocusStripmapRda(invalid_sinc, signal::ComplexMatrix{},
                                         signal::ComplexVector{}, &focused));
}

}  // namespace
}  // namespace sar
