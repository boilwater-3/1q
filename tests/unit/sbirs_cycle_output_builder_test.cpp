#include <gtest/gtest.h>

#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleOutputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"

namespace {

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_sensor::session::SbirsSceneTarget Target(std::uint64_t id, const char* name) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = id;
  target.target_name = name;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  return target;
}

sbirs_sensor::session::SbirsCycleInput InputWithTarget(std::uint32_t cycle_index) {
  return sbirs_sensor::session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .AddTarget(Target(7U, "boost"))
      .Build();
}

sbirs_sensor::session::SbirsCycleResult ResultForTarget(std::uint32_t cycle_index, bool detected) {
  sbirs_sensor::session::SbirsCycleResult result;
  result.input_cycle_index = cycle_index;
  result.output_frame.cycle_index = cycle_index;
  result.executed_this_cycle = true;
  sbirs_sensor::output::SbirsDetectionRecord detection;
  detection.detection_id = 11U;
  detection.azimuth_deg = 2.0f;
  detection.elevation_deg = 3.0f;
  detection.infrared_snr_linear = 4.0f;
  detection.observation_stage = sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack;
  detection.detected = detected;
  result.output_frame.detections.push_back(detection);

  sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = detection.detection_id;
  attribution.target_id = 7U;
  attribution.target_name = "boost";
  attribution.estimated_range_m = 1000000.0f;
  attribution.used_truth_assist = true;
  result.detection_attributions.push_back(attribution);
  return result;
}

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

TEST(SbirsCycleOutputBuilderTest, DebugViewMapsDetectionAttributionBackToInputTarget) {
  const sbirs_sensor::session::SbirsCycleInput input = InputWithTarget(1U);
  const sbirs_sensor::session::SbirsCycleResult result = ResultForTarget(1U, true);

  const sbirs_sensor::session::SbirsOutputDebugView view =
      sbirs_sensor::session::SbirsOutputDebugViewBuilder::Build(input, result);

  ASSERT_EQ(view.targets.size(), 1U);
  EXPECT_EQ(view.targets[0].target_id, 7U);
  EXPECT_EQ(view.targets[0].target_name, "boost");
  EXPECT_TRUE(view.targets[0].has_raw_output_record);
  EXPECT_TRUE(view.targets[0].used_truth_assist);
  EXPECT_FLOAT_EQ(view.targets[0].estimated_range_m, 1000000.0f);
  EXPECT_EQ(view.targets[0].status, sbirs_sensor::session::SbirsDebugTargetStatus::kDetected);
}

TEST(SbirsCycleOutputBuilderTest, LifecycleRecorderTracksFoundUpdatedLostAndOptionalNotDetected) {
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;
  const sbirs_sensor::session::SbirsCycleInput input1 = InputWithTarget(1U);

  std::vector<sbirs_sensor::session::SbirsDetectionLifecycleEvent> events =
      recorder.Update(input1, ResultForTarget(1U, true));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind,
            sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kFirstDetected);
  EXPECT_TRUE(events[0].used_truth_assist);

  events = recorder.Update(InputWithTarget(2U), ResultForTarget(2U, true));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kUpdated);

  sbirs_sensor::session::SbirsCycleInput empty_input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(3U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .Build();
  sbirs_sensor::session::SbirsCycleResult empty_result;
  empty_result.input_cycle_index = 3U;
  empty_result.output_frame.cycle_index = 3U;
  empty_result.executed_this_cycle = true;
  events = recorder.Update(empty_input, empty_result);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kLost);
  EXPECT_EQ(events[0].reason,
            sbirs_sensor::session::SbirsDetectionLifecycleReason::kTargetMissingFromInput);

  sbirs_sensor::session::SbirsDetectionLifecycleRecorder diagnose_recorder(
      sbirs_sensor::session::SbirsDetectionLifecycleRecorderConfig{true});
  events = diagnose_recorder.Update(InputWithTarget(4U), sbirs_sensor::session::SbirsCycleResult{});
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kNotDetected);
}

}  // namespace
