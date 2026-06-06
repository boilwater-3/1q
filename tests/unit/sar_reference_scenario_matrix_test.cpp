#include <gtest/gtest.h>

#include <cmath>
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

}  // namespace
}  // namespace sar
