#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarMotionCompensation.h"
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

double MaxResidualRangeError(const std::vector<geometry::PlatformPulseState>& ideal,
                             const std::vector<geometry::PlatformPulseState>& actual,
                             const geometry::LocalPoint& reference,
                             const geometry::LocalPoint& target) {
  double maximum = 0.0;
  for (std::size_t index = 0U; index < ideal.size(); ++index) {
    const double reference_error = geometry::Distance(actual[index].position_m, reference) -
                                   geometry::Distance(ideal[index].position_m, reference);
    const double target_error = geometry::Distance(actual[index].position_m, target) -
                                geometry::Distance(ideal[index].position_m, target);
    maximum = std::max(maximum, std::abs(target_error - reference_error));
  }
  return maximum;
}

struct L3CompensationCaseMetrics {
  imaging::ImageComparisonMetrics uncompensated{};
  imaging::ImageComparisonMetrics compensated{};
  double max_range_error_m{0.0};
};

bool EvaluateL3CompensationCase(const test_support::ReferencePointScene& scene,
                                std::size_t target_delay, double final_cross_range_m,
                                L3CompensationCaseMetrics* metrics) {
  if (metrics == nullptr) {
    return false;
  }
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0);
  const std::vector<geometry::PlatformPulseState> l3_pulses =
      BuildTurningWaypointTrack(scene, final_cross_range_m);
  if (l3_pulses.size() != scene.pulses.size()) {
    return false;
  }
  test_support::ReferencePointScene l3_scene = scene;
  l3_scene.pulses = l3_pulses;
  signal::ComplexMatrix l3_raw;
  if (!test_support::BuildReferenceRawHistory(l3_scene, {target}, &l3_raw)) {
    return false;
  }

  imaging::FirstOrderMotionCompensationConfig compensation_config;
  compensation_config.sample_rate_hz = scene.sample_rate_hz;
  compensation_config.carrier_frequency_hz = scene.carrier_frequency_hz;
  compensation_config.reference_point_m = target.position_m;
  signal::ComplexMatrix compensated_raw;
  imaging::MotionCompensationDiagnostics compensation_diagnostics;
  if (!imaging::ApplyFirstOrderMotionCompensation(compensation_config, scene.pulses, l3_pulses,
                                                  l3_raw, &compensated_raw,
                                                  &compensation_diagnostics)) {
    return false;
  }

  const imaging::RdaConfig rda_config = test_support::MakeReferenceRdaConfig(scene, target_delay);
  imaging::FocusedSarImage uncompensated_rda;
  imaging::FocusedSarImage compensated_rda;
  if (!imaging::FocusStripmapRda(rda_config, l3_raw, scene.matched_filter, &uncompensated_rda) ||
      !imaging::FocusStripmapRda(rda_config, compensated_raw, scene.matched_filter,
                                 &compensated_rda)) {
    return false;
  }
  imaging::FocusedGbpImage l3_gbp;
  if (!imaging::FocusSmallSceneGbp(MakeGbpConfig(scene, target_delay, 9U, 9U), l3_pulses, l3_raw,
                                   scene.matched_filter, &l3_gbp)) {
    return false;
  }

  metrics->uncompensated = imaging::CompareImagesWithGlobalPhaseReference(
      CropRdaReferenceWindow(uncompensated_rda.image, target_delay, 9U), l3_gbp.image);
  metrics->compensated = imaging::CompareImagesWithGlobalPhaseReference(
      CropRdaReferenceWindow(compensated_rda.image, target_delay, 9U), l3_gbp.image);
  metrics->max_range_error_m = compensation_diagnostics.max_abs_range_error_m;
  return metrics->uncompensated.valid && metrics->compensated.valid;
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

TEST(SarGbpTest, FirstOrderCompensationImprovesL3ReferenceButLeavesSpatialResidual) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  const echo::PointTarget reference_target =
      test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0);
  const std::vector<geometry::PlatformPulseState> l3_pulses = BuildTurningWaypointTrack(scene, 3.0);
  ASSERT_EQ(l3_pulses.size(), scene.pulses.size());
  L3CompensationCaseMetrics metrics;
  ASSERT_TRUE(EvaluateL3CompensationCase(scene, target_delay, 3.0, &metrics));
  RecordProperty("uncompensated_nrms", std::to_string(metrics.uncompensated.normalized_rms_error));
  RecordProperty("compensated_nrms", std::to_string(metrics.compensated.normalized_rms_error));
  RecordProperty("uncompensated_correlation",
                 std::to_string(metrics.uncompensated.coherent_correlation));
  RecordProperty("compensated_correlation",
                 std::to_string(metrics.compensated.coherent_correlation));
  EXPECT_LT(metrics.compensated.normalized_rms_error, metrics.uncompensated.normalized_rms_error);
  EXPECT_GT(metrics.compensated.coherent_correlation, metrics.uncompensated.coherent_correlation);
  EXPECT_LT(metrics.compensated.normalized_rms_error, 0.25);
  EXPECT_GT(metrics.compensated.coherent_correlation, 0.97);

  const echo::PointTarget off_reference_target =
      test_support::MakeReferenceTargetAtDelay(16U, scene.sample_rate_hz, 1.0);
  const double residual_range_error_m = MaxResidualRangeError(
      scene.pulses, l3_pulses, reference_target.position_m, off_reference_target.position_m);
  RecordProperty("off_reference_max_residual_range_error_m",
                 std::to_string(residual_range_error_m));
  EXPECT_GT(residual_range_error_m, 0.0);
  EXPECT_LT(residual_range_error_m, 0.001);
  EXPECT_LT(residual_range_error_m, metrics.max_range_error_m);
}

TEST(SarGbpTest, L3FirstOrderCompensationApplicabilityMatrixFindsPassAndFailureRegions) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<double> cross_range_offsets_m = {0.0, 1.0, 3.0, 6.0, 12.0};
  std::vector<L3CompensationCaseMetrics> matrix;
  for (std::size_t index = 0U; index < cross_range_offsets_m.size(); ++index) {
    L3CompensationCaseMetrics metrics;
    ASSERT_TRUE(EvaluateL3CompensationCase(scene, 20U, cross_range_offsets_m[index], &metrics));
    matrix.push_back(metrics);
    RecordProperty("turn_" + std::to_string(index) + "_compensated_nrms",
                   std::to_string(metrics.compensated.normalized_rms_error));
    RecordProperty("turn_" + std::to_string(index) + "_compensated_correlation",
                   std::to_string(metrics.compensated.coherent_correlation));
  }

  EXPECT_LT(matrix[2U].compensated.normalized_rms_error, 0.25);
  EXPECT_GT(matrix[2U].compensated.coherent_correlation, 0.97);
  EXPECT_TRUE(matrix.back().compensated.normalized_rms_error >= 0.25 ||
              matrix.back().compensated.coherent_correlation <= 0.97);

  const std::vector<geometry::PlatformPulseState> l3_pulses =
      BuildTurningWaypointTrack(scene, 12.0);
  ASSERT_EQ(l3_pulses.size(), scene.pulses.size());
  const geometry::LocalPoint reference =
      test_support::MakeReferenceTargetAtDelay(20U, scene.sample_rate_hz, 1.0).position_m;
  const std::vector<std::size_t> target_delays = {20U, 18U, 16U, 12U};
  double previous_residual_m = -1.0;
  for (std::size_t index = 0U; index < target_delays.size(); ++index) {
    const geometry::LocalPoint target =
        test_support::MakeReferenceTargetAtDelay(target_delays[index], scene.sample_rate_hz, 1.0)
            .position_m;
    const double residual_m = MaxResidualRangeError(scene.pulses, l3_pulses, reference, target);
    RecordProperty("target_" + std::to_string(index) + "_residual_range_error_m",
                   std::to_string(residual_m));
    EXPECT_GE(residual_m, previous_residual_m);
    previous_residual_m = residual_m;
  }
  EXPECT_GT(previous_residual_m, 0.001);
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
