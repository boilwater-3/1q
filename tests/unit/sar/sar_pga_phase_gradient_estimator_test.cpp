#include <gtest/gtest.h>

#include <cmath>

#include "sar/imaging/SarPgaPhaseGradientEstimator.h"

namespace sar {
namespace imaging {
namespace {

PgaPhaseGradientEstimatorRequest ValidRequest() {
  PgaPhaseGradientEstimatorRequest request;
  request.request_id = 151U;
  request.aperture_profile = {
      std::polar(1.0, 0.0), std::polar(2.0, 0.5),
      std::polar(3.0, 1.0), std::polar(4.0, 2.0),
  };
  request.support_mask = {1U, 1U, 1U, 1U};
  request.minimum_valid_pair_count = 3U;
  return request;
}

TEST(SarPgaPhaseGradientEstimatorTest, EstimatesAdjacentConjugateProductGradient) {
  const PgaPhaseGradientEstimatorResult result = EstimatePgaPhaseGradient(ValidRequest());
  ASSERT_EQ(result.status, PgaPhaseGradientEstimatorStatus::kSucceeded);
  EXPECT_EQ(result.valid_pair_count, 3U);
  EXPECT_EQ(result.valid_pair_mask, (std::vector<std::uint8_t>{1U, 1U, 1U}));
  EXPECT_NEAR(result.wrapped_gradient_rad[0], 0.5, 1.0e-12);
  EXPECT_NEAR(result.wrapped_gradient_rad[1], 0.5, 1.0e-12);
  EXPECT_NEAR(result.wrapped_gradient_rad[2], 1.0, 1.0e-12);
}

TEST(SarPgaPhaseGradientEstimatorTest, DoesNotBridgeUnsupportedGap) {
  PgaPhaseGradientEstimatorRequest request = ValidRequest();
  request.support_mask = {1U, 1U, 0U, 1U};
  request.minimum_valid_pair_count = 1U;
  const PgaPhaseGradientEstimatorResult result = EstimatePgaPhaseGradient(request);
  ASSERT_EQ(result.status, PgaPhaseGradientEstimatorStatus::kSucceeded);
  EXPECT_EQ(result.valid_pair_mask, (std::vector<std::uint8_t>{1U, 0U, 0U}));
  EXPECT_DOUBLE_EQ(result.wrapped_gradient_rad[1], 0.0);
  EXPECT_DOUBLE_EQ(result.wrapped_gradient_rad[2], 0.0);
}

TEST(SarPgaPhaseGradientEstimatorTest, RejectsInvalidOrInsufficientPairsAtomically) {
  PgaPhaseGradientEstimatorRequest request = ValidRequest();
  request.aperture_profile[1] = signal::ComplexSample(0.0, 0.0);
  request.minimum_valid_pair_count = 3U;
  const PgaPhaseGradientEstimatorResult insufficient = EstimatePgaPhaseGradient(request);
  EXPECT_EQ(insufficient.reason, PgaPhaseGradientEstimatorReason::kInsufficientValidPairs);
  EXPECT_TRUE(insufficient.valid_pair_mask.empty());
  EXPECT_TRUE(insufficient.wrapped_gradient_rad.empty());

  request = ValidRequest();
  request.support_mask[0] = 2U;
  EXPECT_EQ(EstimatePgaPhaseGradient(request).reason,
            PgaPhaseGradientEstimatorReason::kInvalidSupportMask);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
