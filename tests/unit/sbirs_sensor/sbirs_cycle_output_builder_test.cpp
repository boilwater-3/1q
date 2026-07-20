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
  attribution.tracking_source =
      sbirs_sensor::attribution::SbirsTrackingSource::kStrictTruthAssisted;
  attribution.has_estimation_nis = true;
  attribution.estimation_nis = 1.5f;
  attribution.estimation_nis_gate_exceeded = false;
  result.detection_attributions.push_back(attribution);
  return result;
}

sbirs_sensor::session::SbirsCycleResult RejectedResult(std::uint32_t cycle_index) {
  sbirs_sensor::session::SbirsCycleResult result;
  result.input_cycle_index = cycle_index;
  result.has_validation_error = true;
  result.abort_reason = sbirs_sensor::session::SbirsPipelineAbortReason::kValidationRejected;
  return result;
}

sbirs_sensor::session::SbirsCycleResult NisLossResultForTarget(std::uint32_t cycle_index) {
  sbirs_sensor::session::SbirsCycleResult result;
  result.input_cycle_index = cycle_index;
  result.output_frame.cycle_index = cycle_index;
  result.executed_this_cycle = true;

  sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = 12U;
  attribution.target_id = 7U;
  attribution.target_name = "boost";
  attribution.estimated_range_m = 1000000.0f;
  attribution.tracking_source = sbirs_sensor::attribution::SbirsTrackingSource::kEstimated;
  attribution.capture_failure_reason =
      sbirs_sensor::attribution::SbirsCaptureFailureReason::kEstimationNisGateLost;
  attribution.has_estimation_nis = true;
  attribution.estimation_nis = 12.5f;
  attribution.estimation_nis_gate_exceeded = true;
  result.detection_attributions.push_back(attribution);
  return result;
}

sbirs_sensor::session::SbirsCycleResult PointingTimeoutResultForTarget(std::uint32_t cycle_index) {
  sbirs_sensor::session::SbirsCycleResult result;
  result.input_cycle_index = cycle_index;
  result.output_frame.cycle_index = cycle_index;
  result.executed_this_cycle = true;

  sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = 13U;
  attribution.target_id = 7U;
  attribution.target_name = "boost";
  attribution.estimated_range_m = 1000000.0f;
  attribution.tracking_source =
      sbirs_sensor::attribution::SbirsTrackingSource::kNotApplicable;
  attribution.nfov_channel_id = 2;
  attribution.capture_failure_reason =
      sbirs_sensor::attribution::SbirsCaptureFailureReason::kNfovPointingTimeout;
  result.detection_attributions.push_back(attribution);
  return result;
}

sbirs_sensor::session::SbirsCycleResult CoastingResultForTarget(std::uint32_t cycle_index) {
  sbirs_sensor::session::SbirsCycleResult result;
  result.input_cycle_index = cycle_index;
  result.output_frame.cycle_index = cycle_index;
  result.executed_this_cycle = true;
  sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = 14U;
  attribution.target_id = 7U;
  attribution.target_name = "boost";
  attribution.nfov_channel_id = 1;
  attribution.has_nfov_tracking_diagnostics = true;
  attribution.nfov_pointing_error_deg = 0.75f;
  attribution.nfov_geometry_gate_passed = false;
  attribution.nfov_snr_gate_passed = true;
  attribution.nfov_tracking_gate_failure_count = 1U;
  attribution.nfov_tracking_coasting = true;
  result.detection_attributions.push_back(attribution);
  return result;
}

sbirs_sensor::session::SbirsCycleResult TrackingGateLossResultForTarget(std::uint32_t cycle_index) {
  sbirs_sensor::session::SbirsCycleResult result = CoastingResultForTarget(cycle_index);
  auto& attribution = result.detection_attributions.front();
  attribution.capture_failure_reason =
      sbirs_sensor::attribution::SbirsCaptureFailureReason::kNfovTrackingGateLost;
  attribution.nfov_tracking_gate_failure_count = 2U;
  attribution.nfov_tracking_coasting = false;
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
  EXPECT_EQ(view.targets[0].tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kStrictTruthAssisted);
  EXPECT_FLOAT_EQ(view.targets[0].estimated_range_m, 1000000.0f);
  EXPECT_TRUE(view.targets[0].has_estimation_nis);
  EXPECT_FLOAT_EQ(view.targets[0].estimation_nis, 1.5f);
  EXPECT_FALSE(view.targets[0].estimation_nis_gate_exceeded);
  EXPECT_EQ(view.targets[0].status, sbirs_sensor::session::SbirsDebugTargetStatus::kDetected);
}

TEST(SbirsCycleOutputBuilderTest, DebugViewPreservesNisLossAttributionWithoutRawRecord) {
  const sbirs_sensor::session::SbirsCycleInput input = InputWithTarget(2U);
  const sbirs_sensor::session::SbirsCycleResult result = NisLossResultForTarget(2U);

  const sbirs_sensor::session::SbirsOutputDebugView view =
      sbirs_sensor::session::SbirsOutputDebugViewBuilder::Build(input, result);

  ASSERT_EQ(view.targets.size(), 1U);
  EXPECT_FALSE(view.targets[0].has_raw_output_record);
  EXPECT_EQ(view.targets[0].tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kEstimated);
  EXPECT_FLOAT_EQ(view.targets[0].estimated_range_m, 1000000.0f);
  EXPECT_TRUE(view.targets[0].has_estimation_nis);
  EXPECT_FLOAT_EQ(view.targets[0].estimation_nis, 12.5f);
  EXPECT_TRUE(view.targets[0].estimation_nis_gate_exceeded);
  EXPECT_EQ(view.targets[0].observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
  EXPECT_EQ(view.targets[0].status, sbirs_sensor::session::SbirsDebugTargetStatus::kNotInOutput);
}

TEST(SbirsCycleOutputBuilderTest, LifecycleRecorderTracksFoundUpdatedLostAndOptionalNotDetected) {
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;
  const sbirs_sensor::session::SbirsCycleInput input1 = InputWithTarget(1U);

  std::vector<sbirs_sensor::session::SbirsDetectionLifecycleEvent> events =
      recorder.Update(input1, ResultForTarget(1U, true));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind,
            sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kFirstDetected);
  EXPECT_EQ(events[0].tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kStrictTruthAssisted);

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
  sbirs_sensor::session::SbirsCycleResult executed_without_detection;
  executed_without_detection.input_cycle_index = 4U;
  executed_without_detection.executed_this_cycle = true;
  events = diagnose_recorder.Update(InputWithTarget(4U), executed_without_detection);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kNotDetected);
}

TEST(SbirsCycleOutputBuilderTest, ValidationRejectedCyclePreservesDetectedLifecycleState) {
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;
  ASSERT_EQ(recorder.Update(InputWithTarget(1U), ResultForTarget(1U, true)).size(), 1U);

  const auto rejected_events = recorder.Update(InputWithTarget(2U), RejectedResult(2U));
  EXPECT_TRUE(rejected_events.empty());

  const auto resumed_events = recorder.Update(InputWithTarget(3U), ResultForTarget(3U, true));
  ASSERT_EQ(resumed_events.size(), 1U);
  EXPECT_EQ(resumed_events.front().kind,
            sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kUpdated);
}

TEST(SbirsCycleOutputBuilderTest, ValidationRejectedEmptyInputDoesNotInventTargetMissing) {
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;
  ASSERT_EQ(recorder.Update(InputWithTarget(1U), ResultForTarget(1U, true)).size(), 1U);
  const sbirs_sensor::session::SbirsCycleInput empty_input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(2U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .Build();

  const auto rejected_events = recorder.Update(empty_input, RejectedResult(2U));
  EXPECT_TRUE(rejected_events.empty());

  const auto resumed_events = recorder.Update(InputWithTarget(3U), ResultForTarget(3U, true));
  ASSERT_EQ(resumed_events.size(), 1U);
  EXPECT_EQ(resumed_events.front().kind,
            sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kUpdated);
}

TEST(SbirsCycleOutputBuilderTest, ValidationRejectedCycleIgnoresEmitNotDetectedPolicy) {
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder(
      sbirs_sensor::session::SbirsDetectionLifecycleRecorderConfig{true});
  ASSERT_EQ(recorder.Update(InputWithTarget(1U), ResultForTarget(1U, true)).size(), 1U);

  const auto rejected_events = recorder.Update(InputWithTarget(2U), RejectedResult(2U));
  EXPECT_TRUE(rejected_events.empty());

  const auto resumed_events = recorder.Update(InputWithTarget(3U), ResultForTarget(3U, true));
  ASSERT_EQ(resumed_events.size(), 1U);
  EXPECT_EQ(resumed_events.front().kind,
            sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kUpdated);
}

TEST(SbirsCycleOutputBuilderTest, LifecycleRecorderPreservesNisLossReasonAndDiagnostics) {
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;
  std::vector<sbirs_sensor::session::SbirsDetectionLifecycleEvent> events =
      recorder.Update(InputWithTarget(1U), ResultForTarget(1U, true));
  ASSERT_EQ(events.size(), 1U);
  ASSERT_EQ(events[0].kind,
            sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kFirstDetected);

  events = recorder.Update(InputWithTarget(2U), NisLossResultForTarget(2U));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kLost);
  EXPECT_EQ(events[0].reason,
            sbirs_sensor::session::SbirsDetectionLifecycleReason::kEstimationNisGateLost);
  EXPECT_EQ(events[0].tracking_source,
            sbirs_sensor::attribution::SbirsTrackingSource::kEstimated);
  EXPECT_FLOAT_EQ(events[0].estimated_range_m, 1000000.0f);
  EXPECT_TRUE(events[0].has_estimation_nis);
  EXPECT_FLOAT_EQ(events[0].estimation_nis, 12.5f);
  EXPECT_TRUE(events[0].estimation_nis_gate_exceeded);
  EXPECT_EQ(events[0].observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
}

TEST(SbirsCycleOutputBuilderTest, DebugAndLifecyclePreservePointingTimeoutChannel) {
  const sbirs_sensor::session::SbirsCycleInput input = InputWithTarget(3U);
  const sbirs_sensor::session::SbirsCycleResult result = PointingTimeoutResultForTarget(3U);
  const sbirs_sensor::session::SbirsOutputDebugView view =
      sbirs_sensor::session::SbirsOutputDebugViewBuilder::Build(input, result);
  ASSERT_EQ(view.targets.size(), 1U);
  EXPECT_FALSE(view.targets[0].has_raw_output_record);
  EXPECT_EQ(view.targets[0].nfov_channel_id, 2);
  EXPECT_EQ(view.targets[0].observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);

  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder(
      sbirs_sensor::session::SbirsDetectionLifecycleRecorderConfig{true});
  const std::vector<sbirs_sensor::session::SbirsDetectionLifecycleEvent> events =
      recorder.Update(input, result);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kNotDetected);
  EXPECT_EQ(events[0].reason,
            sbirs_sensor::session::SbirsDetectionLifecycleReason::kNfovPointingTimeout);
  EXPECT_EQ(events[0].nfov_channel_id, 2);
  EXPECT_EQ(events[0].observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
}

TEST(SbirsCycleOutputBuilderTest, CoastingHasNoRawAndDoesNotEmitPrematureLost) {
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;
  recorder.Update(InputWithTarget(1U), ResultForTarget(1U, true));
  const auto input = InputWithTarget(2U);
  const auto result = CoastingResultForTarget(2U);

  const auto view = sbirs_sensor::session::SbirsOutputDebugViewBuilder::Build(input, result);
  ASSERT_EQ(view.targets.size(), 1U);
  EXPECT_EQ(view.targets[0].status, sbirs_sensor::session::SbirsDebugTargetStatus::kCoasting);
  EXPECT_FALSE(view.targets[0].has_raw_output_record);
  EXPECT_EQ(view.targets[0].nfov_tracking_gate_failure_count, 1U);
  EXPECT_EQ(view.targets[0].observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);

  const auto events = recorder.Update(input, result);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kCoasting);
  EXPECT_EQ(events[0].reason, sbirs_sensor::session::SbirsDetectionLifecycleReason::kNone);
  EXPECT_TRUE(events[0].nfov_tracking_coasting);
  EXPECT_EQ(events[0].observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
}

TEST(SbirsCycleOutputBuilderTest, TrackingGateLossEndsCoastingLifecycle) {
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder recorder;
  recorder.Update(InputWithTarget(1U), ResultForTarget(1U, true));
  recorder.Update(InputWithTarget(2U), CoastingResultForTarget(2U));

  const auto events = recorder.Update(InputWithTarget(3U), TrackingGateLossResultForTarget(3U));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kLost);
  EXPECT_EQ(events[0].reason,
            sbirs_sensor::session::SbirsDetectionLifecycleReason::kNfovTrackingGateLost);
  EXPECT_EQ(events[0].nfov_tracking_gate_failure_count, 2U);
  EXPECT_EQ(events[0].observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
}

}  // namespace
