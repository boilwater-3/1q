#include <gtest/gtest.h>

#include <string>

#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarRda.h"
#include "support/sar_reference_scene.h"

namespace sar {
namespace {

imaging::GbpConfig MakeGbpConfig(const test_support::ReferencePointScene& scene,
                                 std::size_t center_delay, std::size_t azimuth_pixels,
                                 std::size_t range_pixels) {
  imaging::GbpConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.grid.azimuth_pixel_count = azimuth_pixels;
  config.grid.range_pixel_count = range_pixels;
  config.grid.azimuth_spacing_m = scene.platform_velocity_mps / scene.prf_hz;
  config.grid.range_spacing_m =
      test_support::kReferenceSpeedOfLightMps / (2.0 * scene.sample_rate_hz);
  config.grid.azimuth_start_m =
      -0.5 * static_cast<double>(azimuth_pixels - 1U) * config.grid.azimuth_spacing_m;
  config.grid.range_start_m =
      static_cast<double>(center_delay - range_pixels / 2U) * config.grid.range_spacing_m;
  return config;
}

signal::ComplexMatrix CropRdaReferenceWindow(const signal::ComplexMatrix& rda,
                                             std::size_t center_range_col,
                                             std::size_t range_pixel_count) {
  signal::ComplexMatrix cropped;
  cropped.rows = rda.rows;
  cropped.cols = range_pixel_count;
  cropped.values.assign(cropped.rows * cropped.cols, signal::ComplexSample(0.0, 0.0));
  const std::size_t start_col = center_range_col - range_pixel_count / 2U;
  for (std::size_t row = 0U; row < cropped.rows; ++row) {
    for (std::size_t col = 0U; col < cropped.cols; ++col) {
      cropped(row, col) = rda(row, start_col + col);
    }
  }
  return cropped;
}

std::vector<geometry::PlatformPulseState> BuildTurningWaypointTrack(
    const test_support::ReferencePointScene& scene, double final_cross_range_m) {
  geometry::WaypointTrackConfig config;
  geometry::Waypoint start;
  start.time_s = scene.pulses.front().time_s;
  start.position_m = scene.pulses.front().position_m;
  geometry::Waypoint turn;
  turn.time_s = scene.pulses[scene.pulses.size() / 2U].time_s;
  turn.position_m = scene.pulses[scene.pulses.size() / 2U].position_m;
  geometry::Waypoint end;
  end.time_s = scene.pulses.back().time_s;
  end.position_m = scene.pulses.back().position_m;
  end.position_m.y_m = final_cross_range_m;
  config.waypoints = {start, turn, end};
  for (const geometry::PlatformPulseState& pulse : scene.pulses) {
    config.pulse_times_s.push_back(pulse.time_s);
  }
  std::vector<geometry::PlatformPulseState> pulses;
  if (!geometry::GenerateWaypointTrack(config, &pulses)) {
    return {};
  }
  return pulses;
}

TEST(SarGbpTest, FocusesSinglePointAtExpectedSmallScenePixel) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(
      scene, {test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0)},
      &raw_history));

  imaging::FocusedGbpImage focused;
  ASSERT_TRUE(imaging::FocusSmallSceneGbp(MakeGbpConfig(scene, target_delay, 9U, 9U), scene.pulses,
                                          raw_history, scene.matched_filter, &focused));

  const imaging::ImageQualityMetrics quality = imaging::EvaluateImageQuality(focused.image);
  ASSERT_TRUE(quality.valid);
  EXPECT_EQ(quality.peak_row, 4U);
  EXPECT_EQ(quality.peak_col, 4U);
  EXPECT_EQ(focused.diagnostics.evaluated_pixels, 81U);
  EXPECT_GT(focused.diagnostics.accumulated_samples, 0U);
}

TEST(SarGbpTest, RdaAndGbpLocateSameReferenceTargetRange) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(
      scene, {test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0)},
      &raw_history));

  imaging::FocusedSarImage rda;
  ASSERT_TRUE(imaging::FocusStripmapRda(test_support::MakeReferenceRdaConfig(scene, target_delay),
                                        raw_history, scene.matched_filter, &rda));
  imaging::FocusedGbpImage gbp;
  ASSERT_TRUE(imaging::FocusSmallSceneGbp(MakeGbpConfig(scene, target_delay, 9U, 9U), scene.pulses,
                                          raw_history, scene.matched_filter, &gbp));

  const imaging::ImageQualityMetrics rda_quality = imaging::EvaluateImageQuality(rda.image);
  const imaging::ImageQualityMetrics gbp_quality = imaging::EvaluateImageQuality(gbp.image);
  ASSERT_TRUE(rda_quality.valid);
  ASSERT_TRUE(gbp_quality.valid);
  EXPECT_NEAR(static_cast<double>(rda_quality.peak_col), static_cast<double>(target_delay), 1.0);
  EXPECT_EQ(gbp_quality.peak_col, 4U);
  EXPECT_GT(rda_quality.peak_magnitude, 0.0);
  EXPECT_GT(gbp_quality.peak_magnitude, 0.0);

  const signal::ComplexMatrix rda_window = CropRdaReferenceWindow(rda.image, target_delay, 9U);
  const imaging::ImageComparisonMetrics comparison =
      imaging::CompareImagesWithGlobalPhaseReference(rda_window, gbp.image);
  ASSERT_TRUE(comparison.valid);
  RecordProperty("global_phase_offset_rad", std::to_string(comparison.phase_offset_rad));
  RecordProperty("normalized_rms_error", std::to_string(comparison.normalized_rms_error));
  RecordProperty("coherent_correlation", std::to_string(comparison.coherent_correlation));
  EXPECT_LT(comparison.normalized_rms_error, 0.1);
  EXPECT_GT(comparison.coherent_correlation, 0.99);
  EXPECT_LE(comparison.coherent_correlation, 1.0);
}

TEST(SarGbpTest, ThreeSeparatedRangeTargetsPreserveExpectedPeakOrder) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(
      scene,
      {test_support::MakeReferenceTargetAtDelay(16U, scene.sample_rate_hz, 3.0),
       test_support::MakeReferenceTargetAtDelay(20U, scene.sample_rate_hz, 2.0),
       test_support::MakeReferenceTargetAtDelay(24U, scene.sample_rate_hz, 1.0)},
      &raw_history));

  imaging::FocusedGbpImage focused;
  ASSERT_TRUE(imaging::FocusSmallSceneGbp(MakeGbpConfig(scene, 20U, 9U, 9U), scene.pulses,
                                          raw_history, scene.matched_filter, &focused));

  const double first = std::abs(focused.image(4U, 0U));
  const double second = std::abs(focused.image(4U, 4U));
  const double third = std::abs(focused.image(4U, 8U));
  EXPECT_GT(first, second);
  EXPECT_GT(second, third);
  EXPECT_GT(third, 0.0);
}

TEST(SarGbpTest, LinearAndSincRdaAreMeasuredAgainstGbpReference) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(
      scene, {test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0)},
      &raw_history));

  imaging::RdaConfig linear_config = test_support::MakeReferenceRdaConfig(scene, target_delay);
  imaging::FocusedSarImage linear;
  ASSERT_TRUE(imaging::FocusStripmapRda(linear_config, raw_history, scene.matched_filter, &linear));
  imaging::RdaConfig sinc_config = linear_config;
  sinc_config.rcmc_interpolation = imaging::RcmcInterpolation::kSinc;
  imaging::FocusedSarImage sinc;
  ASSERT_TRUE(imaging::FocusStripmapRda(sinc_config, raw_history, scene.matched_filter, &sinc));
  imaging::FocusedGbpImage gbp;
  ASSERT_TRUE(imaging::FocusSmallSceneGbp(MakeGbpConfig(scene, target_delay, 9U, 9U), scene.pulses,
                                          raw_history, scene.matched_filter, &gbp));

  const imaging::ImageComparisonMetrics linear_comparison =
      imaging::CompareImagesWithGlobalPhaseReference(
          CropRdaReferenceWindow(linear.image, target_delay, 9U), gbp.image);
  const imaging::ImageComparisonMetrics sinc_comparison =
      imaging::CompareImagesWithGlobalPhaseReference(
          CropRdaReferenceWindow(sinc.image, target_delay, 9U), gbp.image);
  ASSERT_TRUE(linear_comparison.valid);
  ASSERT_TRUE(sinc_comparison.valid);
  RecordProperty("linear_nrms", std::to_string(linear_comparison.normalized_rms_error));
  RecordProperty("sinc_nrms", std::to_string(sinc_comparison.normalized_rms_error));
  RecordProperty("linear_correlation", std::to_string(linear_comparison.coherent_correlation));
  RecordProperty("sinc_correlation", std::to_string(sinc_comparison.coherent_correlation));
  EXPECT_TRUE(std::isfinite(linear_comparison.normalized_rms_error));
  EXPECT_TRUE(std::isfinite(sinc_comparison.normalized_rms_error));
}

TEST(SarGbpTest, L3WaypointRawEchoDegradesL1RdaWhileGbpUsesActualTrajectory) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  const std::vector<echo::PointTarget> targets = {
      test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0)};

  const std::vector<geometry::PlatformPulseState> l3_pulses = BuildTurningWaypointTrack(scene, 3.0);
  ASSERT_EQ(l3_pulses.size(), scene.pulses.size());
  signal::ComplexMatrix l1_raw;
  signal::ComplexMatrix l3_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, targets, &l1_raw));
  test_support::ReferencePointScene l3_scene = scene;
  l3_scene.pulses = l3_pulses;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(l3_scene, targets, &l3_raw));

  const imaging::RdaConfig rda_config = test_support::MakeReferenceRdaConfig(scene, target_delay);
  imaging::FocusedSarImage l1_rda;
  imaging::FocusedSarImage l3_rda;
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, l1_raw, scene.matched_filter, &l1_rda));
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, l3_raw, scene.matched_filter, &l3_rda));
  imaging::FocusedGbpImage l1_gbp;
  imaging::FocusedGbpImage l3_gbp;
  const imaging::GbpConfig gbp_config = MakeGbpConfig(scene, target_delay, 9U, 9U);
  ASSERT_TRUE(
      imaging::FocusSmallSceneGbp(gbp_config, scene.pulses, l1_raw, scene.matched_filter, &l1_gbp));
  ASSERT_TRUE(
      imaging::FocusSmallSceneGbp(gbp_config, l3_pulses, l3_raw, scene.matched_filter, &l3_gbp));

  const imaging::ImageComparisonMetrics l1_comparison =
      imaging::CompareImagesWithGlobalPhaseReference(
          CropRdaReferenceWindow(l1_rda.image, target_delay, 9U), l1_gbp.image);
  const imaging::ImageComparisonMetrics l3_comparison =
      imaging::CompareImagesWithGlobalPhaseReference(
          CropRdaReferenceWindow(l3_rda.image, target_delay, 9U), l3_gbp.image);
  const imaging::ImageQualityMetrics l3_gbp_quality = imaging::EvaluateImageQuality(l3_gbp.image);
  ASSERT_TRUE(l1_comparison.valid);
  ASSERT_TRUE(l3_comparison.valid);
  ASSERT_TRUE(l3_gbp_quality.valid);
  RecordProperty("l1_rda_vs_gbp_nrms", std::to_string(l1_comparison.normalized_rms_error));
  RecordProperty("l3_rda_vs_gbp_nrms", std::to_string(l3_comparison.normalized_rms_error));
  RecordProperty("l1_rda_vs_gbp_correlation", std::to_string(l1_comparison.coherent_correlation));
  RecordProperty("l3_rda_vs_gbp_correlation", std::to_string(l3_comparison.coherent_correlation));
  EXPECT_NEAR(static_cast<double>(l3_gbp_quality.peak_row), 4.0, 1.0);
  EXPECT_EQ(l3_gbp_quality.peak_col, 4U);
  EXPECT_LT(l1_comparison.normalized_rms_error, 0.1);
  EXPECT_GT(l1_comparison.coherent_correlation, 0.99);
  EXPECT_GT(l3_comparison.normalized_rms_error, 0.4);
  EXPECT_LT(l3_comparison.coherent_correlation, 0.9);
  EXPECT_GT(l3_comparison.normalized_rms_error, l1_comparison.normalized_rms_error);
  EXPECT_LT(l3_comparison.coherent_correlation, l1_comparison.coherent_correlation);
}

TEST(SarGbpTest, RejectsSceneBeyondApproved128SquareGate) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(
      scene, {test_support::MakeReferenceTargetAtDelay(20U, scene.sample_rate_hz, 1.0)},
      &raw_history));

  imaging::GbpConfig oversized = MakeGbpConfig(scene, 20U, 129U, 8U);
  imaging::FocusedGbpImage focused;
  EXPECT_FALSE(imaging::FocusSmallSceneGbp(oversized, scene.pulses, raw_history,
                                           scene.matched_filter, &focused));
}

}  // namespace
}  // namespace sar
