#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarMotionCompensation.h"
#include "sar/imaging/SarRda.h"
#include "support/sar_reference_scene.h"

namespace sar {
namespace {

constexpr std::size_t kMatrixCenterDelay = 20U;
constexpr std::size_t kMatrixPixels = 9U;
constexpr double kPi = 3.141592653589793238462643383279502884;

imaging::GbpConfig MakeMatrixGbpConfig(const test_support::ReferencePointScene& scene,
                                       std::size_t center_delay = kMatrixCenterDelay,
                                       std::size_t pixel_count = kMatrixPixels) {
  imaging::GbpConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.grid.azimuth_pixel_count = pixel_count;
  config.grid.range_pixel_count = pixel_count;
  config.grid.azimuth_spacing_m = scene.platform_velocity_mps / scene.prf_hz;
  config.grid.range_spacing_m =
      test_support::kReferenceSpeedOfLightMps / (2.0 * scene.sample_rate_hz);
  config.grid.azimuth_start_m =
      -0.5 * static_cast<double>(pixel_count - 1U) * config.grid.azimuth_spacing_m;
  config.grid.range_start_m =
      static_cast<double>(center_delay - pixel_count / 2U) * config.grid.range_spacing_m;
  return config;
}

signal::ComplexMatrix CropMatrixWindow(const signal::ComplexMatrix& image,
                                       std::size_t center_delay = kMatrixCenterDelay,
                                       std::size_t pixel_count = kMatrixPixels) {
  signal::ComplexMatrix cropped;
  cropped.rows = image.rows;
  cropped.cols = pixel_count;
  cropped.values.assign(cropped.rows * cropped.cols, signal::ComplexSample(0.0, 0.0));
  const std::size_t first_col = center_delay - pixel_count / 2U;
  for (std::size_t row = 0U; row < cropped.rows; ++row) {
    for (std::size_t col = 0U; col < cropped.cols; ++col) {
      cropped(row, col) = image(row, first_col + col);
    }
  }
  return cropped;
}

std::vector<geometry::PlatformPulseState> BuildTurningTrack(
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

struct MatrixFocusResult {
  imaging::FocusedSarImage rda{};
  imaging::FocusedGbpImage gbp{};
  imaging::FocusedGbpImage bp{};
  imaging::ImageComparisonMetrics rda_vs_gbp{};
};

bool FocusMatrix(const test_support::ReferencePointScene& scene,
                 const std::vector<geometry::PlatformPulseState>& pulses,
                 const signal::ComplexMatrix& raw, MatrixFocusResult* result,
                 std::size_t center_delay = kMatrixCenterDelay,
                 std::size_t pixel_count = kMatrixPixels) {
  if (result == nullptr || pulses.size() != scene.pulses.size()) {
    return false;
  }
  if (!imaging::FocusStripmapRda(test_support::MakeReferenceRdaConfig(scene, center_delay), raw,
                                 scene.matched_filter, &result->rda) ||
      !imaging::FocusSmallSceneGbp(MakeMatrixGbpConfig(scene, center_delay, pixel_count), pulses,
                                   raw, scene.matched_filter, &result->gbp) ||
      !imaging::FocusSmallSceneBp(MakeMatrixGbpConfig(scene, center_delay, pixel_count), pulses,
                                  raw, scene.matched_filter, &result->bp)) {
    return false;
  }
  result->rda_vs_gbp = imaging::CompareImagesWithGlobalPhaseReference(
      CropMatrixWindow(result->rda.image, center_delay, pixel_count), result->gbp.image);
  return result->rda_vs_gbp.valid;
}

bool FocusL1Matrix(const test_support::ReferencePointScene& scene,
                   const std::vector<echo::PointTarget>& targets, MatrixFocusResult* result) {
  signal::ComplexMatrix raw;
  return test_support::BuildReferenceRawHistory(scene, targets, &raw) &&
         FocusMatrix(scene, scene.pulses, raw, result);
}

void RecordComparison(const std::string& prefix, const imaging::ImageComparisonMetrics& metrics) {
  ::testing::Test::RecordProperty(prefix + "_phase_offset_rad",
                                  std::to_string(metrics.phase_offset_rad));
  ::testing::Test::RecordProperty(prefix + "_nrms", std::to_string(metrics.normalized_rms_error));
  ::testing::Test::RecordProperty(prefix + "_correlation",
                                  std::to_string(metrics.coherent_correlation));
}

void RecordQuality(const std::string& prefix, const imaging::ImageQualityMetrics& metrics) {
  ::testing::Test::RecordProperty(prefix + "_peak_row", std::to_string(metrics.peak_row));
  ::testing::Test::RecordProperty(prefix + "_peak_col", std::to_string(metrics.peak_col));
  ::testing::Test::RecordProperty(prefix + "_range_width_3db_bins",
                                  std::to_string(metrics.range_width_3db_bins));
  ::testing::Test::RecordProperty(prefix + "_azimuth_width_3db_bins",
                                  std::to_string(metrics.azimuth_width_3db_bins));
  ::testing::Test::RecordProperty(prefix + "_pslr_db", std::to_string(metrics.pslr_db));
  ::testing::Test::RecordProperty(prefix + "_islr_db", std::to_string(metrics.islr_db));
  ::testing::Test::RecordProperty(prefix + "_entropy_nats", std::to_string(metrics.entropy_nats));
}

double ComputeTargetOffsetNonlinearPhaseResidualRad(
    const test_support::ReferencePointScene& scene, double target_azimuth_m,
    std::size_t target_delay) {
  const double reference_range_m =
      static_cast<double>(target_delay) * test_support::kReferenceSpeedOfLightMps /
      (2.0 * scene.sample_rate_hz);
  const double wavelength_m =
      test_support::kReferenceSpeedOfLightMps / scene.carrier_frequency_hz;
  const double center_slant_m =
      std::sqrt(reference_range_m * reference_range_m + target_azimuth_m * target_azimuth_m);
  const double center_difference_m = center_slant_m - reference_range_m;
  const double center_slope = -target_azimuth_m / center_slant_m;
  double max_residual_m = 0.0;
  for (const geometry::PlatformPulseState& pulse : scene.pulses) {
    const double platform_azimuth_m = pulse.position_m.x_m;
    const double target_slant_m =
        std::sqrt(reference_range_m * reference_range_m +
                  (platform_azimuth_m - target_azimuth_m) *
                      (platform_azimuth_m - target_azimuth_m));
    const double broadside_slant_m =
        std::sqrt(reference_range_m * reference_range_m +
                  platform_azimuth_m * platform_azimuth_m);
    const double nonlinear_residual_m =
        target_slant_m - broadside_slant_m - center_difference_m -
        center_slope * platform_azimuth_m;
    max_residual_m = std::max(max_residual_m, std::abs(nonlinear_residual_m));
  }
  return 4.0 * kPi * max_residual_m / wavelength_m;
}

struct ApplicabilityCaseResult {
  test_support::ReferenceRawHistoryDiagnostics raw_diagnostics{};
  MatrixFocusResult focus{};
  imaging::ImageQualityMetrics quality{};
  std::string classification{};
};

bool RunApplicabilityCase(const test_support::ReferencePointScene& scene,
                          const echo::PointTarget& target, std::size_t center_delay,
                          ApplicabilityCaseResult* result) {
  if (result == nullptr) {
    return false;
  }
  signal::ComplexMatrix raw;
  if (!test_support::BuildReferenceRawHistory(scene, {target}, &raw, &result->raw_diagnostics) ||
      !FocusMatrix(scene, scene.pulses, raw, &result->focus, center_delay, kMatrixPixels)) {
    return false;
  }
  result->quality = imaging::EvaluateImageQuality(result->focus.gbp.image);
  if (!result->quality.valid) {
    result->classification = "invalid";
  } else if (result->raw_diagnostics.clipped_pulse_count > 0U) {
    result->classification = "echo_clipped";
  } else if (result->quality.peak_row == 0U ||
             result->quality.peak_row + 1U == result->focus.gbp.image.rows ||
             result->quality.peak_col == 0U ||
             result->quality.peak_col + 1U == result->focus.gbp.image.cols ||
             result->focus.rda_vs_gbp.normalized_rms_error >= 0.1 ||
             result->focus.rda_vs_gbp.coherent_correlation <= 0.99) {
    result->classification = "boundary_degraded";
  } else {
    result->classification = "interior_pass";
  }
  return true;
}

void RecordApplicabilityCase(const std::string& prefix, const ApplicabilityCaseResult& result) {
  ::testing::Test::RecordProperty(prefix + "_classification", result.classification);
  ::testing::Test::RecordProperty(prefix + "_clipped_pulses",
                                  std::to_string(result.raw_diagnostics.clipped_pulse_count));
  ::testing::Test::RecordProperty(prefix + "_clipped_samples",
                                  std::to_string(result.raw_diagnostics.clipped_sample_count));
  RecordComparison(prefix + "_rda_vs_gbp", result.focus.rda_vs_gbp);
  RecordQuality(prefix + "_gbp", result.quality);
}

TEST(SarReferenceScenarioMatrixTest, M1CenterSinglePointPreservesApprovedBaseline) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  MatrixFocusResult result;
  ASSERT_TRUE(FocusL1Matrix(
      scene,
      {test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0)},
      &result));
  MatrixFocusResult repeated;
  ASSERT_TRUE(FocusL1Matrix(
      scene,
      {test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0)},
      &repeated));

  RecordComparison("m1_rda_vs_gbp", result.rda_vs_gbp);
  const imaging::ImageQualityMetrics quality = imaging::EvaluateImageQuality(result.gbp.image);
  ASSERT_TRUE(quality.valid);
  RecordQuality("m1_gbp", quality);
  EXPECT_EQ(result.rda.image.values, repeated.rda.image.values);
  EXPECT_EQ(result.gbp.image.values, repeated.gbp.image.values);
  EXPECT_EQ(result.bp.image.values, result.gbp.image.values);
  EXPECT_LT(result.rda_vs_gbp.normalized_rms_error, 0.1);
  EXPECT_GT(result.rda_vs_gbp.coherent_correlation, 0.99);
}

TEST(SarReferenceScenarioMatrixTest, M2RangeOffsetSinglePointLocatesExpectedPixel) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  MatrixFocusResult result;
  ASSERT_TRUE(FocusL1Matrix(
      scene, {test_support::MakeReferenceTargetAtDelay(18U, scene.sample_rate_hz, 1.0)}, &result));

  const imaging::ImageQualityMetrics gbp_quality = imaging::EvaluateImageQuality(result.gbp.image);
  ASSERT_TRUE(gbp_quality.valid);
  RecordComparison("m2_rda_vs_gbp", result.rda_vs_gbp);
  RecordQuality("m2_gbp", gbp_quality);
  EXPECT_EQ(result.bp.image.values, result.gbp.image.values);
  EXPECT_EQ(gbp_quality.peak_row, 4U);
  EXPECT_EQ(gbp_quality.peak_col, 2U);
  EXPECT_LT(result.rda_vs_gbp.normalized_rms_error, 0.1);
  EXPECT_GT(result.rda_vs_gbp.coherent_correlation, 0.99);
}

TEST(SarReferenceScenarioMatrixTest, M3AzimuthOffsetSinglePointLocatesExpectedPixel) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  MatrixFocusResult result;
  ASSERT_TRUE(FocusL1Matrix(scene,
                            {test_support::MakeReferenceTargetAtPosition(
                                0.2, kMatrixCenterDelay, scene.sample_rate_hz, 1.0)},
                            &result));

  const imaging::ImageQualityMetrics gbp_quality = imaging::EvaluateImageQuality(result.gbp.image);
  ASSERT_TRUE(gbp_quality.valid);
  RecordComparison("m3_rda_vs_gbp", result.rda_vs_gbp);
  RecordQuality("m3_gbp", gbp_quality);
  EXPECT_EQ(result.bp.image.values, result.gbp.image.values);
  EXPECT_EQ(gbp_quality.peak_row, 6U);
  EXPECT_EQ(gbp_quality.peak_col, 4U);
  EXPECT_LT(result.rda_vs_gbp.normalized_rms_error, 0.1);
  EXPECT_GT(result.rda_vs_gbp.coherent_correlation, 0.99);
}

TEST(SarReferenceScenarioMatrixTest, M4TwoDimensionalTargetsPreserveLocationsAndAmplitudeOrder) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<echo::PointTarget> targets = {
      test_support::MakeReferenceTargetAtPosition(-0.2, 18U, scene.sample_rate_hz, 3.0),
      test_support::MakeReferenceTargetAtPosition(0.0, 20U, scene.sample_rate_hz, 2.0),
      test_support::MakeReferenceTargetAtPosition(0.2, 22U, scene.sample_rate_hz, 1.0)};
  MatrixFocusResult result;
  ASSERT_TRUE(FocusL1Matrix(scene, targets, &result));

  RecordComparison("m4_rda_vs_gbp", result.rda_vs_gbp);
  const imaging::ImageQualityMetrics quality = imaging::EvaluateImageQuality(result.gbp.image);
  ASSERT_TRUE(quality.valid);
  RecordQuality("m4_gbp", quality);
  EXPECT_EQ(result.bp.image.values, result.gbp.image.values);
  const double first = std::abs(result.gbp.image(2U, 2U));
  const double second = std::abs(result.gbp.image(4U, 4U));
  const double third = std::abs(result.gbp.image(6U, 6U));
  EXPECT_GT(first, second);
  EXPECT_GT(second, third);
  EXPECT_GT(third, 0.0);
  EXPECT_LT(result.rda_vs_gbp.normalized_rms_error, 0.1);
  EXPECT_GT(result.rda_vs_gbp.coherent_correlation, 0.99);
}

TEST(SarReferenceScenarioMatrixTest, M5L2ZeroAndSeededPerturbationPreserveCompensationContract) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 33U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<echo::PointTarget> targets = {
      test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0),
      test_support::MakeReferenceTargetAtPosition(0.4, 18U, scene.sample_rate_hz, 0.5)};

  signal::ComplexMatrix ideal_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, targets, &ideal_raw));
  imaging::FirstOrderMotionCompensationConfig compensation_config;
  compensation_config.sample_rate_hz = scene.sample_rate_hz;
  compensation_config.carrier_frequency_hz = scene.carrier_frequency_hz;
  compensation_config.reference_point_m = targets.front().position_m;
  signal::ComplexMatrix zero_compensated;
  imaging::MotionCompensationDiagnostics zero_diagnostics;
  ASSERT_TRUE(imaging::ApplyFirstOrderMotionCompensation(compensation_config, scene.pulses,
                                                         scene.pulses, ideal_raw, &zero_compensated,
                                                         &zero_diagnostics));
  EXPECT_EQ(zero_compensated.values, ideal_raw.values);

  geometry::PerturbedStripmapTrackConfig l2_config;
  l2_config.ideal.start_position_m = scene.pulses.front().position_m;
  l2_config.ideal.velocity_x_mps = scene.platform_velocity_mps;
  l2_config.ideal.prf_hz = scene.prf_hz;
  l2_config.ideal.pulse_count = scene.pulse_count;
  l2_config.velocity_error_stddev_y_mps = 30.0;
  l2_config.velocity_error_stddev_z_mps = 10.0;
  l2_config.random_seed = 2026U;
  std::vector<geometry::PlatformPulseState> actual_pulses;
  geometry::TrajectoryErrorDiagnostics trajectory_diagnostics;
  ASSERT_TRUE(
      geometry::GeneratePerturbedStripmapTrack(l2_config, &actual_pulses, &trajectory_diagnostics));
  test_support::ReferencePointScene actual_scene = scene;
  actual_scene.pulses = actual_pulses;
  signal::ComplexMatrix actual_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(actual_scene, targets, &actual_raw));
  signal::ComplexMatrix compensated_raw;
  imaging::MotionCompensationDiagnostics diagnostics;
  ASSERT_TRUE(imaging::ApplyFirstOrderMotionCompensation(compensation_config, scene.pulses,
                                                         actual_pulses, actual_raw,
                                                         &compensated_raw, &diagnostics));

  imaging::FocusedSarImage ideal;
  imaging::FocusedSarImage uncompensated;
  imaging::FocusedSarImage compensated;
  const imaging::RdaConfig rda_config =
      test_support::MakeReferenceRdaConfig(scene, kMatrixCenterDelay);
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, ideal_raw, scene.matched_filter, &ideal));
  ASSERT_TRUE(
      imaging::FocusStripmapRda(rda_config, actual_raw, scene.matched_filter, &uncompensated));
  ASSERT_TRUE(
      imaging::FocusStripmapRda(rda_config, compensated_raw, scene.matched_filter, &compensated));
  const imaging::ImageComparisonMetrics uncompensated_comparison =
      imaging::CompareImagesWithGlobalPhaseReference(ideal.image, uncompensated.image);
  const imaging::ImageComparisonMetrics compensated_comparison =
      imaging::CompareImagesWithGlobalPhaseReference(ideal.image, compensated.image);
  ASSERT_TRUE(uncompensated_comparison.valid);
  ASSERT_TRUE(compensated_comparison.valid);
  RecordComparison("m5_uncompensated", uncompensated_comparison);
  RecordComparison("m5_compensated", compensated_comparison);
  const imaging::ImageQualityMetrics compensated_quality =
      imaging::EvaluateImageQuality(compensated.image);
  ASSERT_TRUE(compensated_quality.valid);
  RecordQuality("m5_compensated", compensated_quality);
  EXPECT_LT(compensated_comparison.normalized_rms_error,
            uncompensated_comparison.normalized_rms_error);
  EXPECT_GT(compensated_comparison.coherent_correlation,
            uncompensated_comparison.coherent_correlation);
  EXPECT_LT(compensated_comparison.normalized_rms_error, 0.3);
  EXPECT_GT(compensated_comparison.coherent_correlation, 0.95);
}

void VerifyL3MatrixCase(double final_cross_range_m, bool expect_compensation_pass) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<echo::PointTarget> targets = {
      test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0),
      test_support::MakeReferenceTargetAtPosition(0.2, 18U, scene.sample_rate_hz, 0.5)};
  const std::vector<geometry::PlatformPulseState> l3_pulses =
      BuildTurningTrack(scene, final_cross_range_m);
  ASSERT_EQ(l3_pulses.size(), scene.pulses.size());
  test_support::ReferencePointScene l3_scene = scene;
  l3_scene.pulses = l3_pulses;
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(l3_scene, targets, &raw));

  imaging::FirstOrderMotionCompensationConfig compensation_config;
  compensation_config.sample_rate_hz = scene.sample_rate_hz;
  compensation_config.carrier_frequency_hz = scene.carrier_frequency_hz;
  compensation_config.reference_point_m = targets.front().position_m;
  signal::ComplexMatrix compensated_raw;
  imaging::MotionCompensationDiagnostics diagnostics;
  ASSERT_TRUE(imaging::ApplyFirstOrderMotionCompensation(
      compensation_config, scene.pulses, l3_pulses, raw, &compensated_raw, &diagnostics));
  MatrixFocusResult uncompensated;
  MatrixFocusResult compensated;
  ASSERT_TRUE(FocusMatrix(scene, l3_pulses, raw, &uncompensated));
  ASSERT_TRUE(FocusMatrix(scene, l3_pulses, compensated_raw, &compensated));
  RecordComparison("l3_uncompensated", uncompensated.rda_vs_gbp);
  RecordComparison("l3_compensated", compensated.rda_vs_gbp);
  const imaging::ImageQualityMetrics gbp_quality =
      imaging::EvaluateImageQuality(uncompensated.gbp.image);
  const imaging::ImageQualityMetrics compensated_quality =
      imaging::EvaluateImageQuality(compensated.rda.image);
  ASSERT_TRUE(gbp_quality.valid);
  ASSERT_TRUE(compensated_quality.valid);
  RecordQuality("l3_gbp", gbp_quality);
  RecordQuality("l3_compensated_rda", compensated_quality);
  EXPECT_EQ(uncompensated.bp.image.values, uncompensated.gbp.image.values);
  EXPECT_LT(compensated.rda_vs_gbp.normalized_rms_error,
            uncompensated.rda_vs_gbp.normalized_rms_error);
  EXPECT_GT(compensated.rda_vs_gbp.coherent_correlation,
            uncompensated.rda_vs_gbp.coherent_correlation);
  if (expect_compensation_pass) {
    EXPECT_LT(compensated.rda_vs_gbp.normalized_rms_error, 0.25);
    EXPECT_GT(compensated.rda_vs_gbp.coherent_correlation, 0.97);
  } else {
    EXPECT_TRUE(compensated.rda_vs_gbp.normalized_rms_error >= 0.25 ||
                compensated.rda_vs_gbp.coherent_correlation <= 0.97);
  }
}

TEST(SarReferenceScenarioMatrixTest, M6L3PassRegionRetainsCompensationImprovement) {
  VerifyL3MatrixCase(3.0, true);
}

TEST(SarReferenceScenarioMatrixTest, M7L3FailureRegionKeepsBpReferenceAndCompensationFailure) {
  VerifyL3MatrixCase(12.0, false);
}

TEST(SarBoundaryParameterMatrixTest, B1ToB3SeparateInteriorImageEdgeAndEchoClipping) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  struct RangeBoundaryCase {
    const char* name;
    std::size_t target_delay;
    std::size_t center_delay;
    const char* expected_classification;
  };
  const std::vector<RangeBoundaryCase> cases = {
      {"b1_center", 20U, 20U, "interior_pass"},
      {"b2_lower_interior", 18U, 20U, "interior_pass"},
      {"b2_lower_image_edge", 16U, 20U, "boundary_degraded"},
      {"b3_upper_interior", 22U, 20U, "interior_pass"},
      {"b3_upper_image_edge", 24U, 20U, "boundary_degraded"},
      {"b3_upper_echo_clip", 56U, 56U, "echo_clipped"}};
  for (const RangeBoundaryCase& boundary_case : cases) {
    ApplicabilityCaseResult result;
    ASSERT_TRUE(RunApplicabilityCase(scene,
                                     test_support::MakeReferenceTargetAtDelay(
                                         boundary_case.target_delay, scene.sample_rate_hz, 1.0),
                                     boundary_case.center_delay, &result));
    RecordApplicabilityCase(boundary_case.name, result);
    EXPECT_EQ(result.focus.bp.image.values, result.focus.gbp.image.values);
    EXPECT_EQ(result.classification, boundary_case.expected_classification);
  }
}

TEST(SarBoundaryParameterMatrixTest, B4SeparatesInteriorAndAzimuthGridEdge) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  struct AzimuthBoundaryCase {
    const char* name;
    double azimuth_m;
    const char* expected_classification;
  };
  const std::vector<AzimuthBoundaryCase> cases = {{"b4_center", 0.0, "interior_pass"},
                                                  {"b4_interior", 0.2, "interior_pass"},
                                                  {"b4_grid_edge", 0.4, "boundary_degraded"}};
  for (const AzimuthBoundaryCase& boundary_case : cases) {
    ApplicabilityCaseResult result;
    ASSERT_TRUE(RunApplicabilityCase(
        scene,
        test_support::MakeReferenceTargetAtPosition(boundary_case.azimuth_m, kMatrixCenterDelay,
                                                    scene.sample_rate_hz, 1.0),
        kMatrixCenterDelay, &result));
    RecordApplicabilityCase(boundary_case.name, result);
    EXPECT_EQ(result.focus.bp.image.values, result.focus.gbp.image.values);
    EXPECT_EQ(result.classification, boundary_case.expected_classification);
  }
}

void VerifyParameterCase(const std::string& name, test_support::ReferencePointScene scene,
                         const std::string& expected_classification) {
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  ApplicabilityCaseResult result;
  ASSERT_TRUE(RunApplicabilityCase(
      scene,
      test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0),
      kMatrixCenterDelay, &result));
  RecordApplicabilityCase(name, result);
  EXPECT_EQ(result.focus.bp.image.values, result.focus.gbp.image.values);
  EXPECT_EQ(result.classification, expected_classification);
}

TEST(SarBoundaryParameterMatrixTest, P1SampleRateApplicability) {
  for (const double sample_rate_hz : {80.0e6, 100.0e6, 120.0e6}) {
    test_support::ReferencePointScene scene;
    scene.sample_rate_hz = sample_rate_hz;
    VerifyParameterCase("p1_" + std::to_string(static_cast<int>(sample_rate_hz / 1.0e6)), scene,
                        "interior_pass");
  }
}

TEST(SarBoundaryParameterMatrixTest, P2CarrierFrequencyApplicability) {
  for (const double carrier_frequency_hz : {0.8e9, 1.0e9, 1.2e9}) {
    test_support::ReferencePointScene scene;
    scene.carrier_frequency_hz = carrier_frequency_hz;
    VerifyParameterCase("p2_" + std::to_string(static_cast<int>(carrier_frequency_hz / 1.0e6)),
                        scene, "interior_pass");
  }
}

TEST(SarBoundaryParameterMatrixTest, P3PrfApplicability) {
  const std::vector<std::pair<double, std::string>> cases = {
      {10.0, "boundary_degraded"}, {20.0, "interior_pass"}, {40.0, "interior_pass"}};
  for (const std::pair<double, std::string>& parameter_case : cases) {
    test_support::ReferencePointScene scene;
    scene.prf_hz = parameter_case.first;
    VerifyParameterCase("p3_" + std::to_string(static_cast<int>(parameter_case.first)), scene,
                        parameter_case.second);
  }
}

TEST(SarBoundaryParameterMatrixTest, P4PlatformVelocityApplicability) {
  const std::vector<std::pair<double, std::string>> cases = {
      {1.0, "interior_pass"}, {2.0, "interior_pass"}, {4.0, "boundary_degraded"}};
  for (const std::pair<double, std::string>& parameter_case : cases) {
    test_support::ReferencePointScene scene;
    scene.platform_velocity_mps = parameter_case.first;
    VerifyParameterCase("p4_" + std::to_string(static_cast<int>(parameter_case.first)), scene,
                        parameter_case.second);
  }
}

TEST(SarAzimuthSamplingAuditTest, SpacingSweepDegradesBeforeGeometricDopplerNyquistFailure) {
  const std::vector<double> spacings_m = {0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2};
  double previous_nrms = -1.0;
  for (std::size_t case_index = 0U; case_index < spacings_m.size(); ++case_index) {
    test_support::ReferencePointScene scene;
    scene.prf_hz = scene.platform_velocity_mps / spacings_m[case_index];
    ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
    const echo::PointTarget target =
        test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0);
    ApplicabilityCaseResult result;
    ASSERT_TRUE(RunApplicabilityCase(scene, target, kMatrixCenterDelay, &result));

    const double wavelength_m =
        test_support::kReferenceSpeedOfLightMps / scene.carrier_frequency_hz;
    const double edge_x_m = std::abs(scene.pulses.front().position_m.x_m);
    const double edge_range_m =
        geometry::Distance(scene.pulses.front().position_m, target.position_m);
    const double max_geometric_doppler_hz =
        2.0 * scene.platform_velocity_mps * edge_x_m / (wavelength_m * edge_range_m);
    const double nyquist_margin = 0.5 * scene.prf_hz / max_geometric_doppler_hz;
    double max_phase_step_rad = 0.0;
    for (std::size_t pulse = 1U; pulse < scene.pulses.size(); ++pulse) {
      const double previous_range_m =
          geometry::Distance(scene.pulses[pulse - 1U].position_m, target.position_m);
      const double current_range_m =
          geometry::Distance(scene.pulses[pulse].position_m, target.position_m);
      max_phase_step_rad = std::max(
          max_phase_step_rad, 4.0 * 3.14159265358979323846 *
                                  std::abs(current_range_m - previous_range_m) / wavelength_m);
    }

    const std::string prefix = "spacing_" + std::to_string(case_index);
    RecordApplicabilityCase(prefix, result);
    ::testing::Test::RecordProperty(prefix + "_meters", std::to_string(spacings_m[case_index]));
    ::testing::Test::RecordProperty(prefix + "_nyquist_margin", std::to_string(nyquist_margin));
    ::testing::Test::RecordProperty(prefix + "_max_phase_step_rad",
                                    std::to_string(max_phase_step_rad));
    EXPECT_NEAR(result.focus.rda.diagnostics.azimuth_sample_spacing_m, spacings_m[case_index],
                1.0e-12);
    EXPECT_NEAR(result.focus.rda.diagnostics.max_geometric_doppler_hz,
                max_geometric_doppler_hz, 1.0e-12);
    EXPECT_NEAR(result.focus.rda.diagnostics.doppler_nyquist_margin, nyquist_margin, 1.0e-12);
    EXPECT_GT(nyquist_margin, 1.0);
    EXPECT_LT(max_phase_step_rad, 3.14159265358979323846);
    EXPECT_GE(result.focus.rda_vs_gbp.normalized_rms_error, previous_nrms);
    previous_nrms = result.focus.rda_vs_gbp.normalized_rms_error;
  }
}

TEST(SarAzimuthSamplingAuditTest, RcmcInterpolationDoesNotExplainCoarseSpacingFailure) {
  test_support::ReferencePointScene scene;
  scene.prf_hz = 10.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  imaging::FocusedGbpImage gbp;
  ASSERT_TRUE(imaging::FocusSmallSceneGbp(MakeMatrixGbpConfig(scene), scene.pulses, raw,
                                          scene.matched_filter, &gbp));
  std::vector<imaging::ImageComparisonMetrics> comparisons;
  for (const imaging::RcmcInterpolation interpolation :
       {imaging::RcmcInterpolation::kNone, imaging::RcmcInterpolation::kLinear,
        imaging::RcmcInterpolation::kSinc}) {
    imaging::RdaConfig config = test_support::MakeReferenceRdaConfig(scene, kMatrixCenterDelay);
    config.rcmc_interpolation = interpolation;
    imaging::FocusedSarImage rda;
    ASSERT_TRUE(imaging::FocusStripmapRda(config, raw, scene.matched_filter, &rda));
    const imaging::ImageComparisonMetrics comparison =
        imaging::CompareImagesWithGlobalPhaseReference(CropMatrixWindow(rda.image), gbp.image);
    ASSERT_TRUE(comparison.valid);
    comparisons.push_back(comparison);
  }

  RecordComparison("coarse_spacing_none", comparisons[0U]);
  RecordComparison("coarse_spacing_linear", comparisons[1U]);
  RecordComparison("coarse_spacing_sinc", comparisons[2U]);
  EXPECT_GT(comparisons[0U].normalized_rms_error, 0.1);
  EXPECT_GT(comparisons[1U].normalized_rms_error, 0.1);
  EXPECT_GT(comparisons[2U].normalized_rms_error, 0.1);
}

struct PhaseCurvatureCase {
  double phase_curvature_rad{0.0};
  imaging::ImageComparisonMetrics comparison{};
};

PhaseCurvatureCase EvaluatePhaseCurvatureCase(test_support::ReferencePointScene scene) {
  PhaseCurvatureCase result;
  if (!test_support::BuildReferencePointScene(&scene)) {
    return result;
  }
  ApplicabilityCaseResult applicability;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0);
  if (!RunApplicabilityCase(scene, target, kMatrixCenterDelay, &applicability)) {
    return result;
  }
  result.phase_curvature_rad =
      applicability.focus.rda.diagnostics.azimuth_phase_curvature_rad_per_pulse2;
  result.comparison = applicability.focus.rda_vs_gbp;
  return result;
}

TEST(SarAzimuthSamplingAuditTest, PhaseCurvatureCollapsesEquivalentCarrierAndRangeCases) {
  test_support::ReferencePointScene lower_sample_rate;
  lower_sample_rate.sample_rate_hz = 80.0e6;
  const PhaseCurvatureCase range_case = EvaluatePhaseCurvatureCase(lower_sample_rate);
  test_support::ReferencePointScene lower_carrier;
  lower_carrier.carrier_frequency_hz = 0.8e9;
  const PhaseCurvatureCase carrier_case = EvaluatePhaseCurvatureCase(lower_carrier);

  test_support::ReferencePointScene higher_sample_rate;
  higher_sample_rate.sample_rate_hz = 120.0e6;
  const PhaseCurvatureCase near_range_case = EvaluatePhaseCurvatureCase(higher_sample_rate);
  test_support::ReferencePointScene higher_carrier;
  higher_carrier.carrier_frequency_hz = 1.2e9;
  const PhaseCurvatureCase high_carrier_case = EvaluatePhaseCurvatureCase(higher_carrier);

  ASSERT_TRUE(range_case.comparison.valid);
  ASSERT_TRUE(carrier_case.comparison.valid);
  ASSERT_TRUE(near_range_case.comparison.valid);
  ASSERT_TRUE(high_carrier_case.comparison.valid);
  ::testing::Test::RecordProperty("lower_curvature_rad",
                                  std::to_string(range_case.phase_curvature_rad));
  ::testing::Test::RecordProperty("lower_range_nrms",
                                  std::to_string(range_case.comparison.normalized_rms_error));
  ::testing::Test::RecordProperty("lower_carrier_nrms",
                                  std::to_string(carrier_case.comparison.normalized_rms_error));
  ::testing::Test::RecordProperty("higher_curvature_rad",
                                  std::to_string(near_range_case.phase_curvature_rad));
  ::testing::Test::RecordProperty("near_range_nrms",
                                  std::to_string(near_range_case.comparison.normalized_rms_error));
  ::testing::Test::RecordProperty(
      "higher_carrier_nrms", std::to_string(high_carrier_case.comparison.normalized_rms_error));
  EXPECT_NEAR(range_case.phase_curvature_rad, carrier_case.phase_curvature_rad, 1.0e-12);
  EXPECT_NEAR(range_case.comparison.normalized_rms_error,
              carrier_case.comparison.normalized_rms_error, 0.001);
  EXPECT_NEAR(near_range_case.phase_curvature_rad, high_carrier_case.phase_curvature_rad, 1.0e-12);
  EXPECT_NEAR(near_range_case.comparison.normalized_rms_error,
              high_carrier_case.comparison.normalized_rms_error, 0.001);
  EXPECT_GT(near_range_case.phase_curvature_rad, range_case.phase_curvature_rad);
  EXPECT_GT(near_range_case.comparison.normalized_rms_error,
            range_case.comparison.normalized_rms_error);
}

TEST(SarRdaDiagnosticDecisionTest, ApertureAndAzimuthOffsetMatrixPreservesDiagnosticDefinitions) {
  struct DecisionCaseResult {
    double quadratic_phase_span_rad{0.0};
    imaging::ImageComparisonMetrics comparison{};
  };
  std::vector<DecisionCaseResult> results;
  const std::vector<std::uint32_t> pulse_counts = {5U, 9U, 17U, 33U};
  const std::vector<double> spacings_m = {0.1, 0.2};
  for (const std::uint32_t pulse_count : pulse_counts) {
    for (const double spacing_m : spacings_m) {
      for (const double target_azimuth_m : {0.0, spacing_m}) {
        test_support::ReferencePointScene scene;
        scene.pulse_count = pulse_count;
        scene.prf_hz = scene.platform_velocity_mps / spacing_m;
        ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
        const echo::PointTarget target = test_support::MakeReferenceTargetAtPosition(
            target_azimuth_m, kMatrixCenterDelay, scene.sample_rate_hz, 1.0);
        signal::ComplexMatrix raw;
        test_support::ReferenceRawHistoryDiagnostics raw_diagnostics;
        ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw, &raw_diagnostics));
        MatrixFocusResult focus;
        ASSERT_TRUE(FocusMatrix(scene, scene.pulses, raw, &focus, kMatrixCenterDelay, pulse_count));
        ASSERT_TRUE(focus.rda_vs_gbp.valid);
        const std::string prefix =
            "pulses_" + std::to_string(pulse_count) + "_spacing_" +
            std::to_string(static_cast<int>(spacing_m * 1000.0)) + "_offset_" +
            std::to_string(static_cast<int>(target_azimuth_m * 1000.0));
        RecordComparison(prefix, focus.rda_vs_gbp);
        ::testing::Test::RecordProperty(
            prefix + "_curvature_rad_per_pulse2",
            std::to_string(focus.rda.diagnostics.azimuth_phase_curvature_rad_per_pulse2));
        ::testing::Test::RecordProperty(
            prefix + "_nyquist_margin",
            std::to_string(focus.rda.diagnostics.doppler_nyquist_margin));
        DecisionCaseResult decision_result;
        decision_result.quadratic_phase_span_rad =
            focus.rda.diagnostics.azimuth_quadratic_phase_span_rad;
        decision_result.comparison = focus.rda_vs_gbp;
        results.push_back(decision_result);
        ::testing::Test::RecordProperty(prefix + "_quadratic_phase_span_rad",
                                        std::to_string(decision_result.quadratic_phase_span_rad));
        EXPECT_EQ(raw_diagnostics.clipped_pulse_count, 0U);
        EXPECT_EQ(focus.bp.image.values, focus.gbp.image.values);
        EXPECT_NEAR(focus.rda.diagnostics.azimuth_sample_spacing_m, spacing_m, 1.0e-12);
        EXPECT_GT(focus.rda.diagnostics.doppler_nyquist_margin, 1.0);
      }
    }
  }

  ASSERT_EQ(results.size(), 16U);
  for (std::size_t center_index = 0U; center_index < results.size(); center_index += 2U) {
    EXPECT_GE(results[center_index + 1U].comparison.normalized_rms_error,
              results[center_index].comparison.normalized_rms_error);
  }
  for (const std::pair<std::size_t, std::size_t> equivalent_pair :
       {std::make_pair(2U, 4U), std::make_pair(6U, 8U), std::make_pair(10U, 12U)}) {
    EXPECT_NEAR(results[equivalent_pair.first].quadratic_phase_span_rad,
                results[equivalent_pair.second].quadratic_phase_span_rad, 1.0e-12);
    EXPECT_NEAR(results[equivalent_pair.first].comparison.normalized_rms_error,
                results[equivalent_pair.second].comparison.normalized_rms_error, 0.03);
  }
}

TEST(SarRdaTargetOffsetDecisionTest, EqualPhysicalAperturesSeparateOffsetGeometryFromSampling) {
  struct OffsetCaseResult {
    double nonlinear_phase_residual_rad{0.0};
    imaging::ImageComparisonMetrics comparison{};
  };
  std::vector<OffsetCaseResult> results;
  const std::vector<std::pair<std::uint32_t, double>> equal_aperture_cases = {
      std::make_pair(5U, 0.2), std::make_pair(9U, 0.1),
      std::make_pair(9U, 0.2), std::make_pair(17U, 0.1),
      std::make_pair(17U, 0.2), std::make_pair(33U, 0.1)};
  const std::vector<double> normalized_offsets = {0.0, 0.25, 0.5};
  for (const std::pair<std::uint32_t, double>& aperture_case : equal_aperture_cases) {
    test_support::ReferencePointScene scene;
    scene.pulse_count = aperture_case.first;
    scene.prf_hz = scene.platform_velocity_mps / aperture_case.second;
    ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
    const double aperture_half_width_m =
        0.5 * static_cast<double>(scene.pulse_count - 1U) * aperture_case.second;
    for (const double normalized_offset : normalized_offsets) {
      const double target_azimuth_m = normalized_offset * aperture_half_width_m;
      const echo::PointTarget target = test_support::MakeReferenceTargetAtPosition(
          target_azimuth_m, kMatrixCenterDelay, scene.sample_rate_hz, 1.0);
      signal::ComplexMatrix raw;
      test_support::ReferenceRawHistoryDiagnostics raw_diagnostics;
      ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw, &raw_diagnostics));
      MatrixFocusResult focus;
      ASSERT_TRUE(FocusMatrix(scene, scene.pulses, raw, &focus, kMatrixCenterDelay,
                              scene.pulse_count));
      ASSERT_TRUE(focus.rda_vs_gbp.valid);
      const std::string prefix =
          "pulses_" + std::to_string(scene.pulse_count) + "_spacing_" +
          std::to_string(static_cast<int>(aperture_case.second * 1000.0)) + "_normalized_offset_" +
          std::to_string(static_cast<int>(normalized_offset * 100.0));
      const double nonlinear_phase_residual_rad = ComputeTargetOffsetNonlinearPhaseResidualRad(
          scene, target_azimuth_m, kMatrixCenterDelay);
      RecordComparison(prefix, focus.rda_vs_gbp);
      ::testing::Test::RecordProperty(prefix + "_aperture_half_width_m",
                                      std::to_string(aperture_half_width_m));
      ::testing::Test::RecordProperty(prefix + "_target_azimuth_m",
                                      std::to_string(target_azimuth_m));
      ::testing::Test::RecordProperty(prefix + "_nonlinear_phase_residual_rad",
                                      std::to_string(nonlinear_phase_residual_rad));
      results.push_back({nonlinear_phase_residual_rad, focus.rda_vs_gbp});
      EXPECT_EQ(raw_diagnostics.clipped_pulse_count, 0U);
      EXPECT_EQ(focus.bp.image.values, focus.gbp.image.values);
    }
  }

  ASSERT_EQ(results.size(), 18U);
  for (std::size_t aperture_index = 0U; aperture_index < equal_aperture_cases.size();
       ++aperture_index) {
    const std::size_t first = aperture_index * normalized_offsets.size();
    EXPECT_NEAR(results[first].nonlinear_phase_residual_rad, 0.0, 1.0e-12);
    EXPECT_LT(results[first].nonlinear_phase_residual_rad,
              results[first + 1U].nonlinear_phase_residual_rad);
    EXPECT_LT(results[first + 1U].nonlinear_phase_residual_rad,
              results[first + 2U].nonlinear_phase_residual_rad);
    EXPECT_LE(results[first].comparison.normalized_rms_error,
              results[first + 1U].comparison.normalized_rms_error);
    EXPECT_LE(results[first + 1U].comparison.normalized_rms_error,
              results[first + 2U].comparison.normalized_rms_error);
  }
  for (std::size_t pair_index = 0U; pair_index < equal_aperture_cases.size(); pair_index += 2U) {
    for (std::size_t offset_index = 0U; offset_index < normalized_offsets.size(); ++offset_index) {
      const OffsetCaseResult& first =
          results[pair_index * normalized_offsets.size() + offset_index];
      const OffsetCaseResult& second =
          results[(pair_index + 1U) * normalized_offsets.size() + offset_index];
      EXPECT_NEAR(first.nonlinear_phase_residual_rad, second.nonlinear_phase_residual_rad, 1.0e-12);
    }
  }
}

TEST(SarRdaTargetOffsetDecisionTest, SymmetricOffsetsPreserveErrorMagnitude) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 17U;
  scene.prf_hz = scene.platform_velocity_mps / 0.1;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  std::vector<double> nrms_values;
  std::vector<double> nonlinear_phase_residuals_rad;
  for (const double target_azimuth_m : {-0.4, -0.2, 0.0, 0.2, 0.4}) {
    const echo::PointTarget target = test_support::MakeReferenceTargetAtPosition(
        target_azimuth_m, kMatrixCenterDelay, scene.sample_rate_hz, 1.0);
    signal::ComplexMatrix raw;
    test_support::ReferenceRawHistoryDiagnostics raw_diagnostics;
    ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw, &raw_diagnostics));
    MatrixFocusResult focus;
    ASSERT_TRUE(
        FocusMatrix(scene, scene.pulses, raw, &focus, kMatrixCenterDelay, scene.pulse_count));
    ASSERT_TRUE(focus.rda_vs_gbp.valid);
    const std::string prefix =
        "signed_offset_" + std::to_string(static_cast<int>(target_azimuth_m * 1000.0));
    RecordComparison(prefix, focus.rda_vs_gbp);
    nrms_values.push_back(focus.rda_vs_gbp.normalized_rms_error);
    nonlinear_phase_residuals_rad.push_back(
        ComputeTargetOffsetNonlinearPhaseResidualRad(scene, target_azimuth_m, kMatrixCenterDelay));
    ::testing::Test::RecordProperty(prefix + "_nonlinear_phase_residual_rad",
                                    std::to_string(nonlinear_phase_residuals_rad.back()));
    EXPECT_EQ(raw_diagnostics.clipped_pulse_count, 0U);
    EXPECT_EQ(focus.bp.image.values, focus.gbp.image.values);
  }

  ASSERT_EQ(nrms_values.size(), 5U);
  EXPECT_NEAR(nrms_values[0], nrms_values[4], 1.0e-12);
  EXPECT_NEAR(nrms_values[1], nrms_values[3], 1.0e-12);
  EXPECT_NEAR(nonlinear_phase_residuals_rad[0], nonlinear_phase_residuals_rad[4], 1.0e-12);
  EXPECT_NEAR(nonlinear_phase_residuals_rad[1], nonlinear_phase_residuals_rad[3], 1.0e-12);
}

TEST(SarReferenceNoiseTest, FixedSeedAndExactEnergyScalingAreDeterministic) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  signal::ComplexMatrix clean;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(
      scene,
      {test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0)},
      &clean));
  signal::ComplexMatrix first;
  signal::ComplexMatrix repeated;
  signal::ComplexMatrix different_seed;
  test_support::ReferenceNoiseDiagnostics first_diagnostics;
  test_support::ReferenceNoiseDiagnostics repeated_diagnostics;
  test_support::ReferenceNoiseDiagnostics different_seed_diagnostics;
  ASSERT_TRUE(test_support::AddDeterministicComplexGaussianNoise(clean, 20.0, 17U, &first,
                                                                &first_diagnostics));
  ASSERT_TRUE(test_support::AddDeterministicComplexGaussianNoise(clean, 20.0, 17U, &repeated,
                                                                &repeated_diagnostics));
  ASSERT_TRUE(test_support::AddDeterministicComplexGaussianNoise(
      clean, 20.0, 29U, &different_seed, &different_seed_diagnostics));

  EXPECT_EQ(first.values, repeated.values);
  EXPECT_NE(first.values, different_seed.values);
  EXPECT_NEAR(first_diagnostics.realized_snr_db, 20.0, 1.0e-12);
  EXPECT_NEAR(repeated_diagnostics.noise_energy, first_diagnostics.noise_energy, 1.0e-12);
  EXPECT_NEAR(different_seed_diagnostics.realized_snr_db, 20.0, 1.0e-12);
  signal::ComplexMatrix invalid;
  test_support::ReferenceNoiseDiagnostics invalid_diagnostics;
  EXPECT_FALSE(test_support::AddDeterministicComplexGaussianNoise(
      signal::ComplexMatrix{}, 20.0, 17U, &invalid, &invalid_diagnostics));
  EXPECT_FALSE(test_support::AddDeterministicComplexGaussianNoise(
      clean, std::numeric_limits<double>::infinity(), 17U, &invalid, &invalid_diagnostics));
}

TEST(SarReferenceClutterTest, FixedGridSeedAndExactScrAreDeterministic) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  signal::ComplexMatrix target_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(
      scene,
      {test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0)},
      &target_raw));

  test_support::ReferenceClutterGridConfig config;
  config.azimuth_count = 3U;
  config.range_count = 3U;
  config.azimuth_start_m = -0.3;
  config.azimuth_spacing_m = 0.3;
  config.range_start_sample = 16U;
  config.range_spacing_samples = 4U;
  config.requested_scr_db = 20.0;
  config.seed = 17U;

  signal::ComplexMatrix first_mixed;
  signal::ComplexMatrix first_clutter;
  signal::ComplexMatrix repeated_mixed;
  signal::ComplexMatrix repeated_clutter;
  signal::ComplexMatrix different_seed_mixed;
  signal::ComplexMatrix different_seed_clutter;
  test_support::ReferenceClutterDiagnostics first_diagnostics;
  test_support::ReferenceClutterDiagnostics repeated_diagnostics;
  test_support::ReferenceClutterDiagnostics different_seed_diagnostics;
  ASSERT_TRUE(test_support::BuildDeterministicDistributedClutter(
      scene, target_raw, config, &first_mixed, &first_clutter, &first_diagnostics));
  ASSERT_TRUE(test_support::BuildDeterministicDistributedClutter(
      scene, target_raw, config, &repeated_mixed, &repeated_clutter, &repeated_diagnostics));
  config.seed = 29U;
  ASSERT_TRUE(test_support::BuildDeterministicDistributedClutter(
      scene, target_raw, config, &different_seed_mixed, &different_seed_clutter,
      &different_seed_diagnostics));

  EXPECT_EQ(first_clutter.values, repeated_clutter.values);
  EXPECT_EQ(first_mixed.values, repeated_mixed.values);
  EXPECT_NE(first_clutter.values, different_seed_clutter.values);
  EXPECT_NE(first_mixed.values, different_seed_mixed.values);
  EXPECT_NEAR(first_diagnostics.realized_scr_db, 20.0, 1.0e-12);
  EXPECT_NEAR(repeated_diagnostics.clutter_energy, first_diagnostics.clutter_energy, 1.0e-12);
  EXPECT_NEAR(different_seed_diagnostics.realized_scr_db, 20.0, 1.0e-12);
  EXPECT_EQ(first_diagnostics.scatterer_count, 9U);
  EXPECT_EQ(first_diagnostics.grid_rows, 3U);
  EXPECT_EQ(first_diagnostics.grid_cols, 3U);
  EXPECT_EQ(first_diagnostics.raw_diagnostics.clipped_pulse_count, 0U);
  EXPECT_EQ(first_diagnostics.raw_diagnostics.clipped_target_count, 0U);
  EXPECT_EQ(first_diagnostics.raw_diagnostics.clipped_sample_count, 0U);

  MatrixFocusResult focus;
  ASSERT_TRUE(FocusMatrix(scene, scene.pulses, first_mixed, &focus));
  EXPECT_EQ(focus.bp.image.values, focus.gbp.image.values);
}

TEST(SarReferenceClutterMatrixTest, M1AndM4RecordDeterministicScrAndDensityTrends) {
  struct Scenario {
    const char* name;
    std::vector<echo::PointTarget> targets;
  };
  struct Grid {
    const char* name;
    std::size_t count;
    double azimuth_start_m;
    double azimuth_spacing_m;
    std::size_t range_start_sample;
    std::size_t range_spacing_samples;
  };
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<Scenario> scenarios = {
      {"m1",
       {test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0)}},
      {"m4",
       {test_support::MakeReferenceTargetAtPosition(-0.2, 18U, scene.sample_rate_hz, 3.0),
        test_support::MakeReferenceTargetAtPosition(0.0, 20U, scene.sample_rate_hz, 2.0),
        test_support::MakeReferenceTargetAtPosition(0.2, 22U, scene.sample_rate_hz, 1.0)}}};
  const std::vector<Grid> grids = {{"sparse", 3U, -0.3, 0.3, 15U, 3U},
                                   {"dense", 5U, -0.4, 0.2, 13U, 2U}};
  const std::vector<double> scr_values_db = {30.0, 20.0, 10.0, 0.0};
  const std::vector<std::uint64_t> seeds = {17U, 29U};
  for (const Scenario& scenario : scenarios) {
    signal::ComplexMatrix target_raw;
    ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, scenario.targets, &target_raw));
    MatrixFocusResult clean_focus;
    ASSERT_TRUE(FocusMatrix(scene, scene.pulses, target_raw, &clean_focus));
    for (const Grid& grid : grids) {
      for (const std::uint64_t seed : seeds) {
        double first_rda_clean_nrms = 0.0;
        double first_gbp_clean_nrms = 0.0;
        double last_rda_clean_nrms = 0.0;
        double last_gbp_clean_nrms = 0.0;
        for (std::size_t scr_index = 0U; scr_index < scr_values_db.size(); ++scr_index) {
          test_support::ReferenceClutterGridConfig config;
          config.azimuth_count = grid.count;
          config.range_count = grid.count;
          config.azimuth_start_m = grid.azimuth_start_m;
          config.azimuth_spacing_m = grid.azimuth_spacing_m;
          config.range_start_sample = grid.range_start_sample;
          config.range_spacing_samples = grid.range_spacing_samples;
          config.requested_scr_db = scr_values_db[scr_index];
          config.seed = seed;
          signal::ComplexMatrix mixed_raw;
          signal::ComplexMatrix clutter_raw;
          test_support::ReferenceClutterDiagnostics diagnostics;
          ASSERT_TRUE(test_support::BuildDeterministicDistributedClutter(
              scene, target_raw, config, &mixed_raw, &clutter_raw, &diagnostics));
          ASSERT_NEAR(diagnostics.realized_scr_db, config.requested_scr_db, 1.0e-12);
          ASSERT_EQ(diagnostics.raw_diagnostics.clipped_sample_count, 0U);
          MatrixFocusResult focus;
          ASSERT_TRUE(FocusMatrix(scene, scene.pulses, mixed_raw, &focus));
          ASSERT_EQ(focus.bp.image.values, focus.gbp.image.values);
          const imaging::ImageComparisonMetrics rda_to_clean =
              imaging::CompareImagesWithGlobalPhaseReference(
                  CropMatrixWindow(focus.rda.image), CropMatrixWindow(clean_focus.rda.image));
          const imaging::ImageComparisonMetrics gbp_to_clean =
              imaging::CompareImagesWithGlobalPhaseReference(focus.gbp.image,
                                                             clean_focus.gbp.image);
          ASSERT_TRUE(rda_to_clean.valid);
          ASSERT_TRUE(gbp_to_clean.valid);
          const std::string prefix =
              std::string(scenario.name) + "_" + grid.name + "_seed_" + std::to_string(seed) +
              "_scr_" + std::to_string(static_cast<int>(config.requested_scr_db));
          RecordComparison(prefix + "_rda_clean", rda_to_clean);
          RecordComparison(prefix + "_gbp_clean", gbp_to_clean);
          RecordComparison(prefix + "_rda_gbp", focus.rda_vs_gbp);
          ::testing::Test::RecordProperty(prefix + "_realized_scr_db",
                                          std::to_string(diagnostics.realized_scr_db));
          if (scr_index == 0U) {
            first_rda_clean_nrms = rda_to_clean.normalized_rms_error;
            first_gbp_clean_nrms = gbp_to_clean.normalized_rms_error;
          }
          last_rda_clean_nrms = rda_to_clean.normalized_rms_error;
          last_gbp_clean_nrms = gbp_to_clean.normalized_rms_error;
        }
        EXPECT_GT(last_rda_clean_nrms, first_rda_clean_nrms);
        EXPECT_GT(last_gbp_clean_nrms, first_gbp_clean_nrms);
      }
    }
  }
}

TEST(SarReferenceSnrMatrixTest, M1AndM4PreserveDeterminismAndRecordQualityTrends) {
  struct Scenario {
    const char* name;
    std::vector<echo::PointTarget> targets;
  };
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<Scenario> scenarios = {
      {"m1",
       {test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0)}},
      {"m4",
       {test_support::MakeReferenceTargetAtPosition(-0.2, 18U, scene.sample_rate_hz, 3.0),
        test_support::MakeReferenceTargetAtPosition(0.0, 20U, scene.sample_rate_hz, 2.0),
        test_support::MakeReferenceTargetAtPosition(0.2, 22U, scene.sample_rate_hz, 1.0)}}};
  const std::vector<double> snr_values_db = {30.0, 20.0, 10.0, 0.0};
  const std::vector<std::uint64_t> seeds = {17U, 29U};
  for (const Scenario& scenario : scenarios) {
    signal::ComplexMatrix clean_raw;
    ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, scenario.targets, &clean_raw));
    MatrixFocusResult clean_focus;
    ASSERT_TRUE(FocusMatrix(scene, scene.pulses, clean_raw, &clean_focus));
    for (const std::uint64_t seed : seeds) {
      double previous_rda_clean_nrms = -1.0;
      double previous_gbp_clean_nrms = -1.0;
      for (const double snr_db : snr_values_db) {
        signal::ComplexMatrix noisy_raw;
        signal::ComplexMatrix repeated_raw;
        test_support::ReferenceNoiseDiagnostics noise_diagnostics;
        test_support::ReferenceNoiseDiagnostics repeated_diagnostics;
        ASSERT_TRUE(test_support::AddDeterministicComplexGaussianNoise(
            clean_raw, snr_db, seed, &noisy_raw, &noise_diagnostics));
        ASSERT_TRUE(test_support::AddDeterministicComplexGaussianNoise(
            clean_raw, snr_db, seed, &repeated_raw, &repeated_diagnostics));
        ASSERT_EQ(noisy_raw.values, repeated_raw.values);
        ASSERT_NEAR(noise_diagnostics.realized_snr_db, snr_db, 1.0e-12);
        MatrixFocusResult noisy_focus;
        ASSERT_TRUE(FocusMatrix(scene, scene.pulses, noisy_raw, &noisy_focus));
        ASSERT_EQ(noisy_focus.bp.image.values, noisy_focus.gbp.image.values);
        const imaging::ImageComparisonMetrics rda_to_clean =
            imaging::CompareImagesWithGlobalPhaseReference(
                CropMatrixWindow(noisy_focus.rda.image), CropMatrixWindow(clean_focus.rda.image));
        const imaging::ImageComparisonMetrics gbp_to_clean =
            imaging::CompareImagesWithGlobalPhaseReference(noisy_focus.gbp.image,
                                                           clean_focus.gbp.image);
        ASSERT_TRUE(rda_to_clean.valid);
        ASSERT_TRUE(gbp_to_clean.valid);
        const std::string prefix =
            std::string(scenario.name) + "_seed_" + std::to_string(seed) + "_snr_" +
            std::to_string(static_cast<int>(snr_db));
        RecordComparison(prefix + "_rda_clean", rda_to_clean);
        RecordComparison(prefix + "_gbp_clean", gbp_to_clean);
        RecordComparison(prefix + "_rda_gbp", noisy_focus.rda_vs_gbp);
        ::testing::Test::RecordProperty(prefix + "_realized_snr_db",
                                        std::to_string(noise_diagnostics.realized_snr_db));
        EXPECT_GT(rda_to_clean.normalized_rms_error, previous_rda_clean_nrms);
        EXPECT_GT(gbp_to_clean.normalized_rms_error, previous_gbp_clean_nrms);
        previous_rda_clean_nrms = rda_to_clean.normalized_rms_error;
        previous_gbp_clean_nrms = gbp_to_clean.normalized_rms_error;
      }
    }
  }
}

TEST(SarReferenceSnrScrMatrixTest, ComponentsAreDeterministicIndependentAndOrderInvariant) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  signal::ComplexMatrix target_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(
      scene,
      {test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0)},
      &target_raw));

  test_support::ReferenceJointInterferenceConfig config;
  config.enable_noise = true;
  config.requested_snr_db = 20.0;
  config.noise_seed = 17U;
  config.enable_clutter = true;
  config.clutter.azimuth_count = 3U;
  config.clutter.range_count = 3U;
  config.clutter.azimuth_start_m = -0.3;
  config.clutter.azimuth_spacing_m = 0.3;
  config.clutter.range_start_sample = 15U;
  config.clutter.range_spacing_samples = 3U;
  config.clutter.requested_scr_db = 20.0;
  config.clutter.seed = 29U;

  signal::ComplexMatrix joint;
  signal::ComplexMatrix noise;
  signal::ComplexMatrix clutter;
  signal::ComplexMatrix repeated_joint;
  signal::ComplexMatrix repeated_noise;
  signal::ComplexMatrix repeated_clutter;
  test_support::ReferenceJointInterferenceDiagnostics diagnostics;
  test_support::ReferenceJointInterferenceDiagnostics repeated_diagnostics;
  ASSERT_TRUE(test_support::BuildDeterministicJointInterference(
      scene, target_raw, config, &joint, &noise, &clutter, &diagnostics));
  ASSERT_TRUE(test_support::BuildDeterministicJointInterference(
      scene, target_raw, config, &repeated_joint, &repeated_noise, &repeated_clutter,
      &repeated_diagnostics));
  EXPECT_EQ(joint.values, repeated_joint.values);
  EXPECT_EQ(noise.values, repeated_noise.values);
  EXPECT_EQ(clutter.values, repeated_clutter.values);
  EXPECT_NEAR(diagnostics.noise.realized_snr_db, config.requested_snr_db, 1.0e-12);
  EXPECT_NEAR(diagnostics.clutter.realized_scr_db, config.clutter.requested_scr_db, 1.0e-12);
  EXPECT_NEAR(diagnostics.noise.signal_energy, diagnostics.target_energy, 1.0e-12);
  EXPECT_NEAR(diagnostics.clutter.target_energy, diagnostics.target_energy, 1.0e-12);

  signal::ComplexMatrix reverse_order = target_raw;
  for (std::size_t index = 0U; index < reverse_order.values.size(); ++index) {
    reverse_order.values[index] += clutter.values[index];
    reverse_order.values[index] += noise.values[index];
  }
  ASSERT_EQ(joint.values.size(), reverse_order.values.size());
  for (std::size_t index = 0U; index < joint.values.size(); ++index) {
    EXPECT_NEAR(std::abs(joint.values[index] - reverse_order.values[index]), 0.0, 1.0e-15);
  }

  test_support::ReferenceJointInterferenceConfig different_noise_config = config;
  different_noise_config.noise_seed = 31U;
  signal::ComplexMatrix different_noise_joint;
  signal::ComplexMatrix different_noise;
  signal::ComplexMatrix same_clutter;
  test_support::ReferenceJointInterferenceDiagnostics different_noise_diagnostics;
  ASSERT_TRUE(test_support::BuildDeterministicJointInterference(
      scene, target_raw, different_noise_config, &different_noise_joint, &different_noise,
      &same_clutter, &different_noise_diagnostics));
  EXPECT_NE(noise.values, different_noise.values);
  EXPECT_EQ(clutter.values, same_clutter.values);

  test_support::ReferenceJointInterferenceConfig different_clutter_config = config;
  different_clutter_config.clutter.seed = 37U;
  signal::ComplexMatrix different_clutter_joint;
  signal::ComplexMatrix same_noise;
  signal::ComplexMatrix different_clutter;
  test_support::ReferenceJointInterferenceDiagnostics different_clutter_diagnostics;
  ASSERT_TRUE(test_support::BuildDeterministicJointInterference(
      scene, target_raw, different_clutter_config, &different_clutter_joint, &same_noise,
      &different_clutter, &different_clutter_diagnostics));
  EXPECT_EQ(noise.values, same_noise.values);
  EXPECT_NE(clutter.values, different_clutter.values);
}

TEST(SarReferenceSnrScrMatrixTest, M1CompleteAndM4SentinelMatricesRecordJointTrends) {
  struct Scenario {
    const char* name;
    std::vector<echo::PointTarget> targets;
    std::size_t grid_count;
    double azimuth_start_m;
    double azimuth_spacing_m;
    std::size_t range_start_sample;
    std::size_t range_spacing_samples;
    bool complete_matrix;
  };
  struct Level {
    bool enabled;
    double db;
    const char* name;
  };
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<Scenario> scenarios = {
      {"m1",
       {test_support::MakeReferenceTargetAtDelay(kMatrixCenterDelay, scene.sample_rate_hz, 1.0)},
       3U, -0.3, 0.3, 15U, 3U, true},
      {"m4",
       {test_support::MakeReferenceTargetAtPosition(-0.2, 18U, scene.sample_rate_hz, 3.0),
        test_support::MakeReferenceTargetAtPosition(0.0, 20U, scene.sample_rate_hz, 2.0),
        test_support::MakeReferenceTargetAtPosition(0.2, 22U, scene.sample_rate_hz, 1.0)},
       5U, -0.4, 0.2, 13U, 2U, false}};
  const std::vector<Level> levels = {
      {false, 0.0, "none"}, {true, 20.0, "20"}, {true, 0.0, "0"}};
  const std::vector<std::pair<std::uint64_t, std::uint64_t>> seed_pairs = {
      {17U, 29U}, {31U, 37U}};

  for (const Scenario& scenario : scenarios) {
    signal::ComplexMatrix target_raw;
    ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, scenario.targets, &target_raw));
    MatrixFocusResult clean_focus;
    ASSERT_TRUE(FocusMatrix(scene, scene.pulses, target_raw, &clean_focus));
    for (const std::pair<std::uint64_t, std::uint64_t>& seeds : seed_pairs) {
      for (std::size_t snr_index = 0U; snr_index < levels.size(); ++snr_index) {
        for (std::size_t scr_index = 0U; scr_index < levels.size(); ++scr_index) {
          const Level& snr = levels[snr_index];
          const Level& scr = levels[scr_index];
          if (!scenario.complete_matrix && snr_index != scr_index) {
            continue;
          }
          test_support::ReferenceJointInterferenceConfig config;
          config.enable_noise = snr.enabled;
          config.requested_snr_db = snr.db;
          config.noise_seed = seeds.first;
          config.enable_clutter = scr.enabled;
          config.clutter.azimuth_count = scenario.grid_count;
          config.clutter.range_count = scenario.grid_count;
          config.clutter.azimuth_start_m = scenario.azimuth_start_m;
          config.clutter.azimuth_spacing_m = scenario.azimuth_spacing_m;
          config.clutter.range_start_sample = scenario.range_start_sample;
          config.clutter.range_spacing_samples = scenario.range_spacing_samples;
          config.clutter.requested_scr_db = scr.db;
          config.clutter.seed = seeds.second;
          signal::ComplexMatrix joint_raw;
          signal::ComplexMatrix noise_raw;
          signal::ComplexMatrix clutter_raw;
          test_support::ReferenceJointInterferenceDiagnostics diagnostics;
          ASSERT_TRUE(test_support::BuildDeterministicJointInterference(
              scene, target_raw, config, &joint_raw, &noise_raw, &clutter_raw, &diagnostics));
          if (!snr.enabled && !scr.enabled) {
            ASSERT_EQ(joint_raw.values, target_raw.values);
          }
          if (snr.enabled) {
            ASSERT_NEAR(diagnostics.noise.realized_snr_db, snr.db, 1.0e-12);
          }
          if (scr.enabled) {
            ASSERT_NEAR(diagnostics.clutter.realized_scr_db, scr.db, 1.0e-12);
            ASSERT_EQ(diagnostics.clutter.raw_diagnostics.clipped_sample_count, 0U);
          }
          MatrixFocusResult joint_focus;
          ASSERT_TRUE(FocusMatrix(scene, scene.pulses, joint_raw, &joint_focus));
          ASSERT_EQ(joint_focus.bp.image.values, joint_focus.gbp.image.values);
          const imaging::ImageComparisonMetrics rda_to_clean =
              imaging::CompareImagesWithGlobalPhaseReference(
                  CropMatrixWindow(joint_focus.rda.image), CropMatrixWindow(clean_focus.rda.image));
          const imaging::ImageComparisonMetrics gbp_to_clean =
              imaging::CompareImagesWithGlobalPhaseReference(joint_focus.gbp.image,
                                                             clean_focus.gbp.image);
          ASSERT_TRUE(rda_to_clean.valid);
          ASSERT_TRUE(gbp_to_clean.valid);
          const std::string prefix =
              std::string(scenario.name) + "_noise_seed_" + std::to_string(seeds.first) +
              "_clutter_seed_" + std::to_string(seeds.second) + "_snr_" + snr.name + "_scr_" +
              scr.name;
          RecordComparison(prefix + "_rda_clean", rda_to_clean);
          RecordComparison(prefix + "_gbp_clean", gbp_to_clean);
          RecordComparison(prefix + "_rda_gbp", joint_focus.rda_vs_gbp);
        }
      }
    }
  }
}

}  // namespace
}  // namespace sar
