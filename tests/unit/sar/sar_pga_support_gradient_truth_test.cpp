#include <gtest/gtest.h>

#include <cmath>

#include "sar/imaging/SarPgaSupportGradientTruth.h"

namespace sar {
namespace imaging {
namespace {

PgaSupportGradientTruthRequest ValidRequest() {
  PgaSupportGradientTruthRequest request;
  request.request_id = 141U;
  request.aperture_profile = {{1.0, 0.0}, {4.0, 0.0}, {4.0, 0.0}, {0.5, 0.0}};
  request.injected_phase_rad = {5.0, 5.5, 6.0, 7.0};
  request.peak_relative_threshold = 0.25;
  request.minimum_supported_samples = 3U;
  return request;
}

TEST(SarPgaSupportGradientTruthTest, SelectsSupportWithFirstPeakTieBreaking) {
  const PgaSupportGradientTruthResult result =
      ExecutePgaSupportGradientTruth(ValidRequest());
  ASSERT_EQ(result.status, PgaSupportGradientTruthStatus::kSucceeded);
  EXPECT_EQ(result.peak_index, 1U);
  EXPECT_EQ(result.supported_sample_count, 3U);
  EXPECT_EQ(result.support_mask, (std::vector<std::uint8_t>{1U, 1U, 1U, 0U}));
}

TEST(SarPgaSupportGradientTruthTest, BuildsWrappedGradientIndependentOfConstantPhase) {
  PgaSupportGradientTruthRequest request = ValidRequest();
  const PgaSupportGradientTruthResult first = ExecutePgaSupportGradientTruth(request);
  for (double& phase : request.injected_phase_rad) {
    phase += 100.0;
  }
  const PgaSupportGradientTruthResult shifted = ExecutePgaSupportGradientTruth(request);
  ASSERT_EQ(first.wrapped_forward_gradient_rad.size(), 3U);
  for (std::size_t index = 0U; index < first.wrapped_forward_gradient_rad.size(); ++index) {
    EXPECT_NEAR(first.wrapped_forward_gradient_rad[index],
                shifted.wrapped_forward_gradient_rad[index], 1.0e-12);
  }

  request = ValidRequest();
  request.injected_phase_rad = {0.0, 4.0, 4.0, 4.0};
  const double pi = std::acos(-1.0);
  EXPECT_NEAR(ExecutePgaSupportGradientTruth(request).wrapped_forward_gradient_rad[0],
              4.0 - 2.0 * pi, 1.0e-12);
}

TEST(SarPgaSupportGradientTruthTest, RejectsInvalidOrInsufficientSupportAtomically) {
  PgaSupportGradientTruthRequest request = ValidRequest();
  request.minimum_supported_samples = 4U;
  const PgaSupportGradientTruthResult insufficient =
      ExecutePgaSupportGradientTruth(request);
  EXPECT_EQ(insufficient.reason, PgaSupportGradientTruthReason::kInsufficientSupport);
  EXPECT_TRUE(insufficient.support_mask.empty());

  request = ValidRequest();
  request.aperture_profile.assign(4U, signal::ComplexSample(0.0, 0.0));
  EXPECT_EQ(ExecutePgaSupportGradientTruth(request).reason,
            PgaSupportGradientTruthReason::kZeroEnergy);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
