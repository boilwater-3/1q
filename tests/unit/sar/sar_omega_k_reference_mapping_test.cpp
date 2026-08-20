#include <gtest/gtest.h>

#include "sar/imaging/SarOmegaKReferenceMapping.h"

namespace sar {
namespace imaging {
namespace {

OmegaKReferenceMappingRequest ValidRequest() {
  OmegaKReferenceMappingRequest request;
  request.request_id = 61U;
  request.propagation_speed_mps = 300.0;
  request.reference_slant_range_m = 1000.0;
  request.delay_sign = OmegaKDelaySign::kPositiveIncreasesRange;
  request.reference_phase_sign = OmegaKReferencePhaseSign::kIdentity;
  request.transform_normalization = 1.0;
  request.relative_delays_s = {0.0, 0.2, 0.4};
  request.azimuth_coordinates = {-1.0, 1.0};
  request.relative_delay_domain.rows = 2U;
  request.relative_delay_domain.cols = 3U;
  request.relative_delay_domain.values = {
      {1.0, 0.0}, {2.0, 1.0}, {3.0, -1.0},
      {4.0, 0.5}, {5.0, -0.5}, {6.0, 0.0},
  };
  return request;
}

TEST(SarOmegaKReferenceMappingTest, MapsRelativeDelayToAbsoluteSlantRange) {
  const OmegaKReferenceMappingRequest request = ValidRequest();
  const OmegaKReferenceMappingResult result = ExecuteOmegaKReferenceMapping(request);
  ASSERT_EQ(result.status, OmegaKReferenceMappingStatus::kSucceeded);
  ASSERT_EQ(result.absolute_slant_ranges_m.size(), 3U);
  EXPECT_DOUBLE_EQ(result.absolute_slant_ranges_m[0], 1000.0);
  EXPECT_DOUBLE_EQ(result.absolute_slant_ranges_m[1], 1030.0);
  EXPECT_DOUBLE_EQ(result.absolute_slant_ranges_m[2], 1060.0);
  EXPECT_EQ(result.referenced_intermediate.values, request.relative_delay_domain.values);
}

TEST(SarOmegaKReferenceMappingTest, SupportsExplicitDecreasingRangeConvention) {
  OmegaKReferenceMappingRequest request = ValidRequest();
  request.delay_sign = OmegaKDelaySign::kPositiveDecreasesRange;
  const OmegaKReferenceMappingResult result = ExecuteOmegaKReferenceMapping(request);
  ASSERT_EQ(result.status, OmegaKReferenceMappingStatus::kSucceeded);
  EXPECT_DOUBLE_EQ(result.absolute_slant_ranges_m[2], 940.0);
}

TEST(SarOmegaKReferenceMappingTest, RejectsInvalidRequestsAtomically) {
  OmegaKReferenceMappingRequest request = ValidRequest();
  request.reference_phase_sign = OmegaKReferencePhaseSign::kUnspecified;
  EXPECT_EQ(ExecuteOmegaKReferenceMapping(request).reason,
            OmegaKReferenceMappingReason::kInvalidConvention);

  request = ValidRequest();
  request.relative_delay_domain.values.pop_back();
  const OmegaKReferenceMappingResult result = ExecuteOmegaKReferenceMapping(request);
  EXPECT_EQ(result.reason, OmegaKReferenceMappingReason::kInvalidMatrix);
  EXPECT_TRUE(result.absolute_slant_ranges_m.empty());
  EXPECT_TRUE(result.referenced_intermediate.values.empty());

  request = ValidRequest();
  request.delay_sign = OmegaKDelaySign::kPositiveDecreasesRange;
  request.reference_slant_range_m = 10.0;
  EXPECT_EQ(ExecuteOmegaKReferenceMapping(request).reason,
            OmegaKReferenceMappingReason::kInvalidAbsoluteRange);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
