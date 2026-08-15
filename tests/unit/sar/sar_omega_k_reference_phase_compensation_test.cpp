#include <gtest/gtest.h>

#include <cmath>

#include "sar/imaging/SarOmegaKReferencePhaseCompensation.h"

namespace sar {
namespace imaging {
namespace {

OmegaKPhaseCompensationRequest ValidRequest() {
  OmegaKPhaseCompensationRequest request;
  request.request_id = 71U;
  request.sign = OmegaKPhaseApplicationSign::kPositive;
  request.absolute_slant_ranges_m = {1000.0, 1010.0};
  request.azimuth_coordinates = {-1.0, 1.0};
  request.range_phase_radians = {0.0, std::acos(-1.0) / 2.0};
  request.referenced_intermediate.rows = 2U;
  request.referenced_intermediate.cols = 2U;
  request.referenced_intermediate.values = {
      {1.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 1.0},
  };
  return request;
}

TEST(SarOmegaKReferencePhaseCompensationTest, AppliesExplicitPhaseByRangeColumn) {
  const OmegaKPhaseCompensationRequest request = ValidRequest();
  const OmegaKPhaseCompensationResult result =
      ExecuteOmegaKReferencePhaseCompensation(request);
  ASSERT_EQ(result.status, OmegaKPhaseCompensationStatus::kSucceeded);
  EXPECT_NEAR(result.compensated_intermediate.values[0].real(), 1.0, 1.0e-12);
  EXPECT_NEAR(result.compensated_intermediate.values[1].imag(), 1.0, 1.0e-12);
  EXPECT_NEAR(result.compensated_intermediate.values[2].imag(), 1.0, 1.0e-12);
  EXPECT_NEAR(result.compensated_intermediate.values[3].real(), -1.0, 1.0e-12);
  EXPECT_EQ(result.absolute_slant_ranges_m, request.absolute_slant_ranges_m);
}

TEST(SarOmegaKReferencePhaseCompensationTest, NegativeSignConjugatesFactor) {
  OmegaKPhaseCompensationRequest request = ValidRequest();
  request.sign = OmegaKPhaseApplicationSign::kNegative;
  const OmegaKPhaseCompensationResult result =
      ExecuteOmegaKReferencePhaseCompensation(request);
  ASSERT_EQ(result.status, OmegaKPhaseCompensationStatus::kSucceeded);
  EXPECT_NEAR(result.compensated_intermediate.values[1].imag(), -1.0, 1.0e-12);
}

TEST(SarOmegaKReferencePhaseCompensationTest, RejectsMalformedRequestAtomically) {
  OmegaKPhaseCompensationRequest request = ValidRequest();
  request.sign = OmegaKPhaseApplicationSign::kUnspecified;
  EXPECT_EQ(ExecuteOmegaKReferencePhaseCompensation(request).reason,
            OmegaKPhaseCompensationReason::kInvalidSign);

  request = ValidRequest();
  request.range_phase_radians.pop_back();
  const OmegaKPhaseCompensationResult result =
      ExecuteOmegaKReferencePhaseCompensation(request);
  EXPECT_EQ(result.reason, OmegaKPhaseCompensationReason::kInvalidPhase);
  EXPECT_TRUE(result.compensated_intermediate.values.empty());
  EXPECT_TRUE(result.absolute_slant_ranges_m.empty());
}

}  // namespace
}  // namespace imaging
}  // namespace sar
