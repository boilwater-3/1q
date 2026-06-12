#include <gtest/gtest.h>

#include "sar/imaging/SarOmegaKRelativeDelayTransform.h"

namespace sar {
namespace imaging {
namespace {

OmegaKRelativeDelayRequest ValidRequest() {
  OmegaKRelativeDelayRequest request;
  request.request_id = 51U;
  request.reduced_range_frequencies_hz = {-40.0, -20.0, 0.0, 20.0};
  request.reduced_spectrum.rows = 2U;
  request.reduced_spectrum.cols = 4U;
  request.reduced_spectrum.values = {
      {1.0, 0.0}, {0.0, 1.0}, {-2.0, 0.5}, {3.0, -1.0},
      {0.5, 0.5}, {-1.0, 2.0}, {4.0, 0.0}, {-0.5, -0.25},
  };
  return request;
}

TEST(SarOmegaKRelativeDelayTransformTest, TransformsRowsAndRoundTripsSpectrum) {
  const OmegaKRelativeDelayRequest request = ValidRequest();
  const OmegaKRelativeDelayResult result = ExecuteOmegaKRelativeDelayTransform(request);
  ASSERT_EQ(result.status, OmegaKRelativeDelayStatus::kSucceeded);
  EXPECT_EQ(result.relative_delay_domain.rows, request.reduced_spectrum.rows);
  EXPECT_EQ(result.relative_delay_domain.cols, request.reduced_spectrum.cols);
  EXPECT_EQ(result.axis_diagnostics.relative_delays_s.size(), request.reduced_spectrum.cols);

  signal::ComplexMatrix restored;
  ASSERT_TRUE(signal::FftRows(result.relative_delay_domain, false, &restored));
  for (std::size_t index = 0U; index < restored.values.size(); ++index) {
    EXPECT_NEAR(restored.values[index].real(), request.reduced_spectrum.values[index].real(),
                1.0e-12);
    EXPECT_NEAR(restored.values[index].imag(), request.reduced_spectrum.values[index].imag(),
                1.0e-12);
  }
}

TEST(SarOmegaKRelativeDelayTransformTest, RejectsInvalidRequestsAtomically) {
  OmegaKRelativeDelayRequest request = ValidRequest();
  request.request_id = 0U;
  EXPECT_EQ(ExecuteOmegaKRelativeDelayTransform(request).reason,
            OmegaKRelativeDelayReason::kInvalidRequestId);

  request = ValidRequest();
  request.reduced_range_frequencies_hz[2] = 1.0;
  EXPECT_EQ(ExecuteOmegaKRelativeDelayTransform(request).reason,
            OmegaKRelativeDelayReason::kInvalidFrequencyAxis);

  request = ValidRequest();
  request.reduced_spectrum.values.pop_back();
  const OmegaKRelativeDelayResult result = ExecuteOmegaKRelativeDelayTransform(request);
  EXPECT_EQ(result.reason, OmegaKRelativeDelayReason::kInvalidSpectrum);
  EXPECT_TRUE(result.relative_delay_domain.values.empty());
  EXPECT_TRUE(result.axis_diagnostics.relative_delays_s.empty());
}

TEST(SarOmegaKRelativeDelayTransformTest, IsDeterministicAndPreservesInput) {
  const OmegaKRelativeDelayRequest request = ValidRequest();
  const OmegaKRelativeDelayRequest before = request;
  const OmegaKRelativeDelayResult first = ExecuteOmegaKRelativeDelayTransform(request);
  const OmegaKRelativeDelayResult second = ExecuteOmegaKRelativeDelayTransform(request);
  EXPECT_EQ(first.relative_delay_domain.values, second.relative_delay_domain.values);
  EXPECT_EQ(request.reduced_spectrum.values, before.reduced_spectrum.values);
  EXPECT_EQ(request.reduced_range_frequencies_hz, before.reduced_range_frequencies_hz);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
