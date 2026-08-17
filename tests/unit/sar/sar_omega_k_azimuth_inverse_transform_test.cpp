#include <gtest/gtest.h>

#include "sar/imaging/SarOmegaKAzimuthInverseTransform.h"

namespace sar {
namespace imaging {
namespace {

OmegaKAzimuthInverseRequest ValidRequest() {
  OmegaKAzimuthInverseRequest request;
  request.request_id = 81U;
  request.absolute_slant_ranges_m = {1000.0, 1010.0};
  request.output_azimuth_coordinates = {-2.0, 2.0};
  request.additional_normalization = 2.0;
  request.compensated_intermediate.rows = 2U;
  request.compensated_intermediate.cols = 2U;
  request.compensated_intermediate.values = {
      {1.0, 0.0}, {1.0, 0.0},
      {1.0, 0.0}, {-1.0, 0.0},
  };
  return request;
}

TEST(SarOmegaKAzimuthInverseTransformTest, TransformsColumnsAndAppliesNormalization) {
  const OmegaKAzimuthInverseRequest request = ValidRequest();
  const OmegaKAzimuthInverseResult result = ExecuteOmegaKAzimuthInverseTransform(request);
  ASSERT_EQ(result.status, OmegaKAzimuthInverseStatus::kSucceeded);
  ASSERT_EQ(result.numerical_image_candidate.values.size(), 4U);
  EXPECT_NEAR(result.numerical_image_candidate.values[0].real(), 2.0, 1.0e-12);
  EXPECT_NEAR(result.numerical_image_candidate.values[1].real(), 0.0, 1.0e-12);
  EXPECT_NEAR(result.numerical_image_candidate.values[2].real(), 0.0, 1.0e-12);
  EXPECT_NEAR(result.numerical_image_candidate.values[3].real(), 2.0, 1.0e-12);
  EXPECT_EQ(result.absolute_slant_ranges_m, request.absolute_slant_ranges_m);
  EXPECT_EQ(result.output_azimuth_coordinates, request.output_azimuth_coordinates);
}

TEST(SarOmegaKAzimuthInverseTransformTest, RoundTripsWithDeclaredNormalization) {
  OmegaKAzimuthInverseRequest request = ValidRequest();
  request.additional_normalization = 1.0;
  const OmegaKAzimuthInverseResult result = ExecuteOmegaKAzimuthInverseTransform(request);
  ASSERT_EQ(result.status, OmegaKAzimuthInverseStatus::kSucceeded);
  signal::ComplexMatrix restored;
  ASSERT_TRUE(signal::FftCols(result.numerical_image_candidate, false, &restored));
  for (std::size_t index = 0U; index < restored.values.size(); ++index) {
    EXPECT_NEAR(restored.values[index].real(),
                request.compensated_intermediate.values[index].real(), 1.0e-12);
    EXPECT_NEAR(restored.values[index].imag(),
                request.compensated_intermediate.values[index].imag(), 1.0e-12);
  }
}

TEST(SarOmegaKAzimuthInverseTransformTest, RejectsMalformedRequestAtomically) {
  OmegaKAzimuthInverseRequest request = ValidRequest();
  request.additional_normalization = 0.0;
  EXPECT_EQ(ExecuteOmegaKAzimuthInverseTransform(request).reason,
            OmegaKAzimuthInverseReason::kInvalidNormalization);

  request = ValidRequest();
  request.output_azimuth_coordinates.pop_back();
  const OmegaKAzimuthInverseResult result = ExecuteOmegaKAzimuthInverseTransform(request);
  EXPECT_EQ(result.reason, OmegaKAzimuthInverseReason::kInvalidMatrix);
  EXPECT_TRUE(result.numerical_image_candidate.values.empty());
  EXPECT_TRUE(result.absolute_slant_ranges_m.empty());
}

}  // namespace
}  // namespace imaging
}  // namespace sar
