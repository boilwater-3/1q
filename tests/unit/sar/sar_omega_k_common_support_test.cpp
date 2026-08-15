#include <gtest/gtest.h>

#include "sar/imaging/SarOmegaKCommonSupport.h"

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

TEST(SarOmegaKCommonSupportTest, FindsCommonWindowAcrossFullDopplerGrid) {
  OmegaKGeometryDiagnostics geometry;
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(BaseConfig(), &geometry));
  ASSERT_FALSE(geometry.valid);

  OmegaKCommonSupportDiagnostics diagnostics;
  ASSERT_TRUE(DiagnoseOmegaKCommonStoltSupport(geometry, &diagnostics));
  EXPECT_TRUE(diagnostics.valid);
  EXPECT_TRUE(diagnostics.usable_for_interpolation);
  EXPECT_LT(diagnostics.common_valid_column_count, diagnostics.target_range_bin_count);
  EXPECT_GT(diagnostics.discarded_column_count, 0U);
  EXPECT_EQ(diagnostics.common_valid_column_count + diagnostics.discarded_column_count,
            diagnostics.target_range_bin_count);
  EXPECT_EQ(diagnostics.largest_contiguous_original_column_indices.size(),
            diagnostics.largest_contiguous_column_count);
}

TEST(SarOmegaKCommonSupportTest, ZeroDopplerSingleRowKeepsEveryColumn) {
  OmegaKGeometryConfig config = BaseConfig();
  config.azimuth_pulse_count = 1U;
  OmegaKGeometryDiagnostics geometry;
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(config, &geometry));
  ASSERT_TRUE(geometry.valid);

  OmegaKCommonSupportDiagnostics diagnostics;
  ASSERT_TRUE(DiagnoseOmegaKCommonStoltSupport(geometry, &diagnostics));
  EXPECT_EQ(diagnostics.common_valid_column_count, config.range_sample_count);
  EXPECT_EQ(diagnostics.discarded_column_count, 0U);
  EXPECT_DOUBLE_EQ(diagnostics.common_valid_ratio, 1.0);
  EXPECT_EQ(diagnostics.largest_contiguous_column_count, config.range_sample_count);
  EXPECT_TRUE(diagnostics.usable_for_interpolation);
}

TEST(SarOmegaKCommonSupportTest, IncludesClosedSupportBoundaryAndRejectsBeyondIt) {
  OmegaKGeometryDiagnostics geometry;
  geometry.range_frequency_bin_count = 3U;
  geometry.azimuth_frequency_bin_count = 2U;
  geometry.range_frequencies_hz = {0.0, 1.0, -1.0};
  geometry.source_range_frequency_queries_hz = {-1.0, 0.0, 1.0, -1.0, 0.0, 1.01};
  geometry.stolt_shifts_hz.assign(6U, 0.0);

  OmegaKCommonSupportDiagnostics diagnostics;
  ASSERT_TRUE(DiagnoseOmegaKCommonStoltSupport(geometry, &diagnostics));
  EXPECT_TRUE(diagnostics.common_valid_column_mask[0]);
  EXPECT_TRUE(diagnostics.common_valid_column_mask[1]);
  EXPECT_FALSE(diagnostics.common_valid_column_mask[2]);
  EXPECT_EQ(diagnostics.out_of_support_query_count_per_column[2], 1U);
}

TEST(SarOmegaKCommonSupportTest, RejectsInvalidShapeAndIsDeterministic) {
  OmegaKGeometryDiagnostics geometry;
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(BaseConfig(), &geometry));
  OmegaKCommonSupportDiagnostics first;
  OmegaKCommonSupportDiagnostics second;
  ASSERT_TRUE(DiagnoseOmegaKCommonStoltSupport(geometry, &first));
  ASSERT_TRUE(DiagnoseOmegaKCommonStoltSupport(geometry, &second));
  EXPECT_EQ(first.common_valid_column_mask, second.common_valid_column_mask);
  EXPECT_EQ(first.largest_contiguous_original_column_indices,
            second.largest_contiguous_original_column_indices);

  geometry.source_range_frequency_queries_hz.pop_back();
  EXPECT_FALSE(DiagnoseOmegaKCommonStoltSupport(geometry, &first));
  EXPECT_FALSE(first.valid);
  EXPECT_FALSE(DiagnoseOmegaKCommonStoltSupport(geometry, nullptr));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
