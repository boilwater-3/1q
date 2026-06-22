#include <gtest/gtest.h>

#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarMotionCompensation.h"
#include "sar/imaging/SarRda.h"
#include "support/sar_reference_scene.h"

namespace sar {
namespace {

TEST(SarMotionCompensationTest, ZeroTrajectoryErrorLeavesRawHistoryUnchanged) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw_history));

  imaging::FirstOrderMotionCompensationConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.reference_point_m = target.position_m;
  signal::ComplexMatrix compensated;
  imaging::MotionCompensationDiagnostics diagnostics;
  ASSERT_TRUE(imaging::ApplyFirstOrderMotionCompensation(config, scene.pulses, scene.pulses,
                                                         raw_history, &compensated, &diagnostics));

  EXPECT_EQ(compensated.values, raw_history.values);
  EXPECT_DOUBLE_EQ(diagnostics.max_abs_range_error_m, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.rms_range_error_m, 0.0);
  EXPECT_EQ(diagnostics.compensated_pulses, scene.pulses.size());
}

TEST(SarMotionCompensationTest, ImprovesL2RdaAgainstIdealReference) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 33U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0);

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

  signal::ComplexMatrix ideal_raw;
  signal::ComplexMatrix actual_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &ideal_raw));
  test_support::ReferencePointScene actual_scene = scene;
  actual_scene.pulses = actual_pulses;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(actual_scene, {target}, &actual_raw));

  imaging::FirstOrderMotionCompensationConfig compensation_config;
  compensation_config.sample_rate_hz = scene.sample_rate_hz;
  compensation_config.carrier_frequency_hz = scene.carrier_frequency_hz;
  compensation_config.reference_point_m = target.position_m;
  signal::ComplexMatrix compensated_raw;
  imaging::MotionCompensationDiagnostics compensation_diagnostics;
  ASSERT_TRUE(imaging::ApplyFirstOrderMotionCompensation(
      compensation_config, scene.pulses, actual_pulses, actual_raw, &compensated_raw,
      &compensation_diagnostics));

  const imaging::RdaConfig rda_config = test_support::MakeReferenceRdaConfig(scene, target_delay);
  imaging::FocusedSarImage ideal_image;
  imaging::FocusedSarImage uncompensated_image;
  imaging::FocusedSarImage compensated_image;
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, ideal_raw, scene.matched_filter, &ideal_image));
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, actual_raw, scene.matched_filter,
                                        &uncompensated_image));
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, compensated_raw, scene.matched_filter,
                                        &compensated_image));

  const imaging::ImageComparisonMetrics uncompensated =
      imaging::CompareImagesWithGlobalPhaseReference(ideal_image.image, uncompensated_image.image);
  const imaging::ImageComparisonMetrics compensated =
      imaging::CompareImagesWithGlobalPhaseReference(ideal_image.image, compensated_image.image);
  ASSERT_TRUE(uncompensated.valid);
  ASSERT_TRUE(compensated.valid);
  RecordProperty("uncompensated_nrms", std::to_string(uncompensated.normalized_rms_error));
  RecordProperty("compensated_nrms", std::to_string(compensated.normalized_rms_error));
  RecordProperty("uncompensated_correlation", std::to_string(uncompensated.coherent_correlation));
  RecordProperty("compensated_correlation", std::to_string(compensated.coherent_correlation));
  RecordProperty("max_range_error_m",
                 std::to_string(compensation_diagnostics.max_abs_range_error_m));
  EXPECT_GT(compensation_diagnostics.max_abs_range_error_m, 0.0);
  EXPECT_LT(compensated.normalized_rms_error, uncompensated.normalized_rms_error);
  EXPECT_GT(compensated.coherent_correlation, uncompensated.coherent_correlation);
  EXPECT_LT(compensated.normalized_rms_error, 0.3);
  EXPECT_GT(compensated.coherent_correlation, 0.95);
}

TEST(SarMotionCompensationTest, RejectsNullCompensatedPointer) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw_history));

  imaging::FirstOrderMotionCompensationConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.reference_point_m = target.position_m;
  imaging::MotionCompensationDiagnostics diagnostics;
  EXPECT_FALSE(imaging::ApplyFirstOrderMotionCompensation(config, scene.pulses, scene.pulses,
                                                         raw_history, nullptr, &diagnostics));
}

TEST(SarMotionCompensationTest, RejectsNullDiagnosticsPointer) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw_history));

  imaging::FirstOrderMotionCompensationConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.reference_point_m = target.position_m;
  signal::ComplexMatrix compensated;
  EXPECT_FALSE(imaging::ApplyFirstOrderMotionCompensation(config, scene.pulses, scene.pulses,
                                                         raw_history, &compensated, nullptr));
}

TEST(SarMotionCompensationTest, RejectsBothNullOutputs) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw_history));

  imaging::FirstOrderMotionCompensationConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.reference_point_m = target.position_m;
  EXPECT_FALSE(imaging::ApplyFirstOrderMotionCompensation(config, scene.pulses, scene.pulses,
                                                         raw_history, nullptr, nullptr));
}

}  // namespace
}  // namespace sar
