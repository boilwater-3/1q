#include <gtest/gtest.h>

#include <limits>

#include "sar/imaging/SarSlowTimeResampling.h"

namespace sar {
namespace imaging {
namespace {

TEST(SarMissingPulseGapDiagnosticsTest, AllowsUniformAndSmallJitterAxes) {
  SlowTimeGapDiagnostics diagnostics;
  ASSERT_TRUE(DiagnoseSlowTimeGaps({0.0, 0.25, 0.5, 0.75, 1.0}, 0.25, &diagnostics));
  EXPECT_TRUE(diagnostics.resampling_allowed);
  EXPECT_DOUBLE_EQ(diagnostics.maximum_gap_ratio, 1.0);
  EXPECT_EQ(diagnostics.suspected_missing_pulse_count, 0U);
  EXPECT_EQ(diagnostics.first_rejected_gap_index, static_cast<std::size_t>(-1));

  ASSERT_TRUE(DiagnoseSlowTimeGaps({0.0, 0.2, 0.52, 0.76, 1.0}, 0.25, &diagnostics));
  EXPECT_TRUE(diagnostics.resampling_allowed);
  EXPECT_LT(diagnostics.maximum_gap_ratio, 1.5);
}

TEST(SarMissingPulseGapDiagnosticsTest, RejectsBoundaryAndCountsMissingPulses) {
  SlowTimeGapDiagnostics diagnostics;
  ASSERT_TRUE(DiagnoseSlowTimeGaps({0.0, 0.25, 0.625, 0.875}, 0.25, &diagnostics));
  EXPECT_FALSE(diagnostics.resampling_allowed);
  EXPECT_DOUBLE_EQ(diagnostics.maximum_gap_ratio, 1.5);
  EXPECT_EQ(diagnostics.rejected_gap_count, 1U);
  EXPECT_EQ(diagnostics.suspected_missing_pulse_count, 1U);
  EXPECT_EQ(diagnostics.first_rejected_gap_index, 1U);

  ASSERT_TRUE(DiagnoseSlowTimeGaps({0.0, 0.25, 1.0, 1.25}, 0.25, &diagnostics));
  EXPECT_EQ(diagnostics.suspected_missing_pulse_count, 2U);
}

TEST(SarMissingPulseGapDiagnosticsTest, AccumulatesMultipleRejectedGaps) {
  SlowTimeGapDiagnostics diagnostics;
  ASSERT_TRUE(DiagnoseSlowTimeGaps({0.0, 0.5, 0.75, 1.5}, 0.25, &diagnostics));
  EXPECT_FALSE(diagnostics.resampling_allowed);
  EXPECT_EQ(diagnostics.rejected_gap_count, 2U);
  EXPECT_EQ(diagnostics.suspected_missing_pulse_count, 3U);
  EXPECT_EQ(diagnostics.first_rejected_gap_index, 0U);
}

TEST(SarMissingPulseGapDiagnosticsTest, GuardedMatrixResamplingRejectsGap) {
  signal::ComplexMatrix input;
  input.rows = 3U;
  input.cols = 1U;
  input.values = {signal::ComplexSample(1.0, 0.0), signal::ComplexSample(2.0, 0.0),
                  signal::ComplexSample(3.0, 0.0)};
  signal::ComplexMatrix output;
  SlowTimeGapDiagnostics gap_diagnostics;
  SlowTimeResamplingDiagnostics resampling_diagnostics;
  EXPECT_FALSE(ResampleRawHistorySlowTimeLinearGuarded(
      {0.0, 0.25, 0.75}, 0.25, input, &output, &gap_diagnostics, &resampling_diagnostics));
  EXPECT_FALSE(gap_diagnostics.resampling_allowed);
  EXPECT_TRUE(output.values.empty());

  EXPECT_TRUE(ResampleRawHistorySlowTimeLinearGuarded(
      {0.0, 0.25, 0.5}, 0.25, input, &output, &gap_diagnostics, &resampling_diagnostics));
  EXPECT_EQ(output.values, input.values);
}

TEST(SarMissingPulseGapDiagnosticsTest, RejectsInvalidInputDeterministically) {
  SlowTimeGapDiagnostics diagnostics;
  EXPECT_FALSE(DiagnoseSlowTimeGaps({0.0}, 0.25, &diagnostics));
  EXPECT_FALSE(DiagnoseSlowTimeGaps({0.0, 0.0}, 0.25, &diagnostics));
  EXPECT_FALSE(DiagnoseSlowTimeGaps({0.0, 0.25}, 0.0, &diagnostics));
  EXPECT_FALSE(DiagnoseSlowTimeGaps(
      {0.0, std::numeric_limits<double>::infinity()}, 0.25, &diagnostics));
  EXPECT_FALSE(DiagnoseSlowTimeGaps({0.0, 0.25}, 0.25, nullptr));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
