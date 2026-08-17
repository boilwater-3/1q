#include <gtest/gtest.h>

#include <cmath>

#include "sar/imaging/SarPgaGradientTruthComparison.h"

namespace sar {
namespace imaging {
namespace {

PgaGradientComparisonRequest ValidRequest() {
  PgaGradientComparisonRequest request;
  request.request_id = 161U;
  request.estimated_wrapped_gradient_rad = {0.1, 100.0, -3.13};
  request.estimator_valid_pair_mask = {1U, 0U, 1U};
  request.truth_wrapped_gradient_rad = {0.1, -100.0, 3.13};
  request.truth_valid_pair_mask = {1U, 1U, 1U};
  request.minimum_jointly_valid_pair_count = 2U;
  request.maximum_abs_wrapped_error_tolerance_rad = 0.03;
  request.rms_wrapped_error_tolerance_rad = 0.02;
  return request;
}

TEST(SarPgaGradientTruthComparisonTest, ComparesOnlyJointlyValidWrappedErrors) {
  const PgaGradientComparisonResult result = ComparePgaGradientTruth(ValidRequest());
  ASSERT_EQ(result.status, PgaGradientComparisonStatus::kPassed);
  EXPECT_EQ(result.jointly_valid_pair_mask, (std::vector<std::uint8_t>{1U, 0U, 1U}));
  EXPECT_EQ(result.jointly_valid_pair_count, 2U);
  EXPECT_LT(result.maximum_abs_wrapped_error_rad, 0.024);
  EXPECT_LT(result.rms_wrapped_error_rad, 0.017);
}

TEST(SarPgaGradientTruthComparisonTest, ReportsValidQualityFailure) {
  PgaGradientComparisonRequest request = ValidRequest();
  request.maximum_abs_wrapped_error_tolerance_rad = 0.001;
  request.rms_wrapped_error_tolerance_rad = 0.001;
  const PgaGradientComparisonResult result = ComparePgaGradientTruth(request);
  EXPECT_EQ(result.status, PgaGradientComparisonStatus::kFailed);
  EXPECT_EQ(result.reason, PgaGradientComparisonReason::kNone);
}

TEST(SarPgaGradientTruthComparisonTest, RejectsInsufficientPairsAtomically) {
  PgaGradientComparisonRequest request = ValidRequest();
  request.minimum_jointly_valid_pair_count = 3U;
  const PgaGradientComparisonResult result = ComparePgaGradientTruth(request);
  EXPECT_EQ(result.reason, PgaGradientComparisonReason::kInsufficientJointlyValidPairs);
  EXPECT_TRUE(result.jointly_valid_pair_mask.empty());
  EXPECT_EQ(result.jointly_valid_pair_count, 0U);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
