#include <gtest/gtest.h>

#include "1q/sbirs_sensor/session/SbirsCycleOutputAdapter.h"

namespace {

TEST(SbirsCycleOutputBuilderTest, NativeFrameHelperAcceptsSbirsDetectionShape) {
  sbirs_sensor::session::SbirsOutputFrame frame;
  sbirs_sensor::output::SbirsDetectionRecord detection;
  detection.detection_id = 1U;
  detection.azimuth_deg = 2.0f;
  detection.elevation_deg = 3.0f;
  detection.infrared_snr_linear = 4.0f;
  detection.observation_stage = sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch;
  detection.detected = true;
  frame.detections.push_back(detection);

  EXPECT_TRUE(sbirs_sensor::session::SbirsOutputFrameContainsOnlyNativeFields(frame));
}

}  // namespace
