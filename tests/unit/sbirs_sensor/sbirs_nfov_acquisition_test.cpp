#include <gtest/gtest.h>

#include "sbirs_sensor/pipeline/SbirsNfovAcquisition.h"

namespace {

sbirs_sensor::pipeline::SbirsNfovAcquisitionRequest BaselineRequest() {
  sbirs_sensor::pipeline::SbirsNfovAcquisitionRequest request;
  request.field_of_view_azimuth_deg = 4.0f;
  request.field_of_view_elevation_deg = 4.0f;
  request.snr = 10.0;
  request.minimum_snr_linear = 5.0f;
  return request;
}

TEST(SbirsNfovAcquisitionTest, AcceptsDelayedTruthInsideCuedFieldOfView) {
  const sbirs_sensor::pipeline::SbirsNfovAcquisitionRequest request = BaselineRequest();

  EXPECT_TRUE(sbirs_sensor::pipeline::IsNfovAcquisitionEligible(request));
}

TEST(SbirsNfovAcquisitionTest, RejectsDelayedTruthOutsideCuedFieldOfView) {
  sbirs_sensor::pipeline::SbirsNfovAcquisitionRequest request = BaselineRequest();
  request.predicted_azimuth_deg = 2.1f;

  EXPECT_FALSE(sbirs_sensor::pipeline::IsNfovAcquisitionEligible(request));
}

TEST(SbirsNfovAcquisitionTest, AppliesSettleErrorAndSnrThreshold) {
  sbirs_sensor::pipeline::SbirsNfovAcquisitionRequest request = BaselineRequest();
  request.measured_azimuth_deg = 1.0f;
  request.pointing_settle_error_deg = 1.0f;
  request.predicted_azimuth_deg = 0.0f;
  request.snr = 4.9;

  EXPECT_FALSE(sbirs_sensor::pipeline::IsNfovAcquisitionEligible(request));
}

}  // namespace
