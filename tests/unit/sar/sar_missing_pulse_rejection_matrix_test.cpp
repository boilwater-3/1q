#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "sar/imaging/SarRda.h"
#include "sar/imaging/SarSlowTimeResampling.h"
#include "support/sar_reference_scene.h"

namespace sar {
namespace imaging {
namespace {

struct RejectionCaseResult {
  bool resampling_succeeded{false};
  bool rda_attempted{false};
  bool rda_succeeded{false};
  SlowTimeGapDiagnostics gap{};
  signal::ComplexMatrix output{};
};

signal::ComplexMatrix RemoveRows(const signal::ComplexMatrix& input,
                                 const std::vector<std::size_t>& removed_rows) {
  signal::ComplexMatrix output;
  output.cols = input.cols;
  output.rows = input.rows - removed_rows.size();
  output.values.reserve(output.rows * output.cols);
  for (std::size_t row = 0U; row < input.rows; ++row) {
    if (std::find(removed_rows.begin(), removed_rows.end(), row) != removed_rows.end()) {
      continue;
    }
    for (std::size_t col = 0U; col < input.cols; ++col) {
      output.values.push_back(input(row, col));
    }
  }
  return output;
}

std::vector<double> RemoveTimes(const std::vector<geometry::PlatformPulseState>& pulses,
                                const std::vector<std::size_t>& removed_rows) {
  std::vector<double> times;
  for (std::size_t row = 0U; row < pulses.size(); ++row) {
    if (std::find(removed_rows.begin(), removed_rows.end(), row) == removed_rows.end()) {
      times.push_back(pulses[row].time_s);
    }
  }
  return times;
}

bool EvaluateRemovedRows(const test_support::ReferencePointScene& scene,
                         const signal::ComplexMatrix& raw,
                         const std::vector<std::size_t>& removed_rows,
                         RejectionCaseResult* result) {
  if (result == nullptr) {
    return false;
  }
  *result = RejectionCaseResult{};
  const signal::ComplexMatrix reduced_raw = RemoveRows(raw, removed_rows);
  const std::vector<double> reduced_times = RemoveTimes(scene.pulses, removed_rows);
  SlowTimeResamplingDiagnostics resampling_diagnostics;
  result->resampling_succeeded = ResampleRawHistorySlowTimeLinearGuarded(
      reduced_times, 1.0 / scene.prf_hz, reduced_raw, &result->output, &result->gap,
      &resampling_diagnostics);
  if (!result->resampling_succeeded) {
    return true;
  }
  result->rda_attempted = true;
  FocusedSarImage image;
  result->rda_succeeded =
      FocusStripmapRda(test_support::MakeReferenceRdaConfig(scene, 28U), result->output,
                       scene.matched_filter, &image);
  return true;
}

TEST(SarMissingPulseRejectionMatrixTest, AllowsBaselineAndSmallJitterToReachRda) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<echo::PointTarget> targets{
      test_support::MakeReferenceTargetAtDelay(28U, scene.sample_rate_hz, 1.0)};
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, targets, &raw));

  RejectionCaseResult baseline;
  ASSERT_TRUE(EvaluateRemovedRows(scene, raw, {}, &baseline));
  EXPECT_TRUE(baseline.gap.resampling_allowed);
  EXPECT_TRUE(baseline.resampling_succeeded);
  EXPECT_TRUE(baseline.rda_attempted);
  EXPECT_TRUE(baseline.rda_succeeded);

  std::vector<double> jittered_times;
  for (std::size_t index = 0U; index < scene.pulses.size(); ++index) {
    const double jitter_s =
        index == 0U || index + 1U == scene.pulses.size()
            ? 0.0
            : 0.05 / scene.prf_hz *
                  std::sin(2.0 * 3.14159265358979323846 * static_cast<double>(index) /
                           static_cast<double>(scene.pulses.size() - 1U));
    jittered_times.push_back(scene.pulses[index].time_s + jitter_s);
  }
  signal::ComplexMatrix output;
  SlowTimeGapDiagnostics gap;
  SlowTimeResamplingDiagnostics resampling;
  EXPECT_TRUE(ResampleRawHistorySlowTimeLinearGuarded(
      jittered_times, 1.0 / scene.prf_hz, raw, &output, &gap, &resampling));
  EXPECT_TRUE(gap.resampling_allowed);
  FocusedSarImage jittered_image;
  EXPECT_TRUE(FocusStripmapRda(test_support::MakeReferenceRdaConfig(scene, 28U), output,
                               scene.matched_filter, &jittered_image));
}

TEST(SarMissingPulseRejectionMatrixTest, RejectsSingleAndAdjacentMissingBeforeRda) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<echo::PointTarget> targets{
      test_support::MakeReferenceTargetAtDelay(28U, scene.sample_rate_hz, 1.0)};
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, targets, &raw));

  RejectionCaseResult single;
  ASSERT_TRUE(EvaluateRemovedRows(scene, raw, {4U}, &single));
  EXPECT_FALSE(single.resampling_succeeded);
  EXPECT_FALSE(single.rda_attempted);
  EXPECT_TRUE(single.output.values.empty());
  EXPECT_EQ(single.gap.rejected_gap_count, 1U);
  EXPECT_EQ(single.gap.suspected_missing_pulse_count, 1U);
  EXPECT_EQ(single.gap.first_rejected_gap_index, 3U);

  RejectionCaseResult adjacent;
  ASSERT_TRUE(EvaluateRemovedRows(scene, raw, {3U, 4U}, &adjacent));
  EXPECT_FALSE(adjacent.resampling_succeeded);
  EXPECT_FALSE(adjacent.rda_attempted);
  EXPECT_TRUE(adjacent.output.values.empty());
  EXPECT_EQ(adjacent.gap.rejected_gap_count, 1U);
  EXPECT_EQ(adjacent.gap.suspected_missing_pulse_count, 2U);
}

TEST(SarMissingPulseRejectionMatrixTest, RejectsSeparateMissingAndBoundaryGap) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::vector<echo::PointTarget> targets{
      test_support::MakeReferenceTargetAtDelay(28U, scene.sample_rate_hz, 1.0)};
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, targets, &raw));

  RejectionCaseResult separate;
  ASSERT_TRUE(EvaluateRemovedRows(scene, raw, {2U, 6U}, &separate));
  EXPECT_FALSE(separate.resampling_succeeded);
  EXPECT_FALSE(separate.rda_attempted);
  EXPECT_TRUE(separate.output.values.empty());
  EXPECT_EQ(separate.gap.rejected_gap_count, 2U);
  EXPECT_EQ(separate.gap.suspected_missing_pulse_count, 2U);
  EXPECT_EQ(separate.gap.first_rejected_gap_index, 1U);

  signal::ComplexMatrix boundary_raw;
  boundary_raw.rows = 3U;
  boundary_raw.cols = 1U;
  boundary_raw.values = {signal::ComplexSample(1.0, 0.0), signal::ComplexSample(2.0, 0.0),
                         signal::ComplexSample(3.0, 0.0)};
  signal::ComplexMatrix boundary_output;
  SlowTimeGapDiagnostics boundary_gap;
  SlowTimeResamplingDiagnostics boundary_resampling;
  EXPECT_FALSE(ResampleRawHistorySlowTimeLinearGuarded(
      {0.0, 0.375, 0.625}, 0.25, boundary_raw, &boundary_output, &boundary_gap,
      &boundary_resampling));
  EXPECT_DOUBLE_EQ(boundary_gap.maximum_gap_ratio, 1.5);
  EXPECT_TRUE(boundary_output.values.empty());
}

}  // namespace
}  // namespace imaging
}  // namespace sar
