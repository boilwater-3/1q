#include <gtest/gtest.h>

#include "sar/imaging/SarOmegaKPointTargetAcceptance.h"

namespace sar {
namespace imaging {
namespace {

OmegaKPointTargetAcceptanceRequest ValidRequest() {
  OmegaKPointTargetAcceptanceRequest request;
  request.request_id = 91U;
  request.absolute_slant_ranges_m = {990.0, 1000.0, 1010.0};
  request.azimuth_coordinates = {-1.0, 0.0, 1.0};
  request.numerical_image_candidate.rows = 3U;
  request.numerical_image_candidate.cols = 3U;
  request.numerical_image_candidate.values = {
      {0.01, 0.0}, {0.1, 0.0}, {0.01, 0.0},
      {0.1, 0.0}, {1.0, 0.0}, {0.1, 0.0},
      {0.01, 0.0}, {0.1, 0.0}, {0.01, 0.0},
  };
  request.truth.independently_generated = true;
  request.truth.inside_common_support = true;
  request.truth.absolute_slant_range_m = 1000.0;
  request.truth.azimuth_coordinate = 0.0;
  request.truth.peak_phase_rad = 0.0;
  request.truth.peak_magnitude = 1.0;
  request.tolerances.maximum_range_error_m = 0.0;
  request.tolerances.maximum_azimuth_error = 0.0;
  request.tolerances.maximum_abs_phase_error_rad = 1.0e-12;
  request.tolerances.maximum_relative_magnitude_error = 1.0e-12;
  request.tolerances.maximum_range_pslr_db = -19.0;
  request.tolerances.maximum_azimuth_pslr_db = -19.0;
  request.tolerances.maximum_range_islr_db = -16.0;
  request.tolerances.maximum_azimuth_islr_db = -16.0;
  return request;
}

TEST(SarOmegaKPointTargetAcceptanceTest, AcceptsIndependentFocusedPointTarget) {
  const OmegaKPointTargetAcceptanceResult result =
      EvaluateOmegaKPointTargetCandidate(ValidRequest());
  EXPECT_EQ(result.status, OmegaKPointTargetAcceptanceStatus::kPassed);
  EXPECT_EQ(result.peak_row, 1U);
  EXPECT_EQ(result.peak_col, 1U);
  EXPECT_NEAR(result.range_pslr_db, -20.0, 1.0e-12);
  EXPECT_NEAR(result.azimuth_pslr_db, -20.0, 1.0e-12);
}

TEST(SarOmegaKPointTargetAcceptanceTest, ReportsQualityFailureWithoutRejecting) {
  OmegaKPointTargetAcceptanceRequest request = ValidRequest();
  request.truth.absolute_slant_range_m = 1001.0;
  const OmegaKPointTargetAcceptanceResult result =
      EvaluateOmegaKPointTargetCandidate(request);
  EXPECT_EQ(result.status, OmegaKPointTargetAcceptanceStatus::kFailed);
  EXPECT_DOUBLE_EQ(result.range_error_m, 1.0);
  EXPECT_EQ(result.reason, OmegaKPointTargetAcceptanceReason::kNone);
}

TEST(SarOmegaKPointTargetAcceptanceTest, RejectsNonIndependentOrUnsupportedTruth) {
  OmegaKPointTargetAcceptanceRequest request = ValidRequest();
  request.truth.independently_generated = false;
  EXPECT_EQ(EvaluateOmegaKPointTargetCandidate(request).reason,
            OmegaKPointTargetAcceptanceReason::kTruthNotIndependent);

  request = ValidRequest();
  request.truth.inside_common_support = false;
  const OmegaKPointTargetAcceptanceResult result =
      EvaluateOmegaKPointTargetCandidate(request);
  EXPECT_EQ(result.reason, OmegaKPointTargetAcceptanceReason::kOutsideCommonSupport);
  EXPECT_DOUBLE_EQ(result.range_pslr_db, 0.0);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
