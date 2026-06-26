/**
 * @file eos_cycle_output_builder_test.cpp
 * @brief 验证 EosCycleOutputAdapter 将内部 EOS 输出转换回外部 ECEF 输出。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosCycleOutputAdapter.h"
#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace {

namespace eos_config = ::electro_optical_sensor::config;
namespace eos_session = ::electro_optical_sensor::session;

eos_session::EosExternalPoseInput MakePlatformInput() {
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1200.0;
  oneq::coordinate::EcefPositionM platform_ecef;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  eos_session::EosExternalPoseInput platform;
  platform.platform_position_ecef_m = platform_ecef;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;
  return platform;
}

std::vector<eos_session::EosExternalTargetInput> MakeMovingTargets(std::size_t count) {
  std::vector<eos_session::EosExternalTargetInput> targets;
  targets.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    oneq::coordinate::LlaPositionDegM lla;
    lla.latitude_deg = 31.0;
    lla.longitude_deg = 121.001 + static_cast<double>(i) * 0.00005;
    lla.altitude_m = 1200.0;
    oneq::coordinate::EcefPositionM ecef;
    EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(lla, &ecef));

    eos_session::EosExternalTargetInput target;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = ecef;
    target.appearance.apparent_temperature_k = 600.0f;
    target.appearance.emissivity = 0.95f;
    target.appearance.reflectance = 0.4f;
    target.appearance.projected_area_m2 = 12.0f;
    targets.push_back(target);
  }
  return targets;
}

void AdvanceTargets(double dt_sec, std::vector<eos_session::EosExternalTargetInput>* targets) {
  ASSERT_NE(targets, nullptr);
  for (std::size_t i = 0; i < targets->size(); ++i) {
    eos_session::EosExternalTargetInput& target = (*targets)[i];
    target.kinematics.position_ecef_m.x_m += (4.0 + static_cast<double>(i % 3U)) * dt_sec;
    target.kinematics.position_ecef_m.y_m += (-2.0 + static_cast<double>(i % 2U)) * dt_sec;
    target.kinematics.position_ecef_m.z_m += 0.1 * static_cast<double>(i % 2U) * dt_sec;
  }
}

eos_config::EosSessionConfig MakeConfig() {
  eos_config::EosSessionConfig config = eos_config::EosSessionConfigBuilder()
                                            .Mission()
                                            .WithWorkMode(eos_config::EosWorkMode::kFused)
                                            .WithScanRateDegPerSec(1.0f)
                                            .End()
                                            .Build();
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  config.mission.scan_start_az_deg = -90.0f;
  config.mission.scan_end_az_deg = 90.0f;
  config.mission.horizontal_fov_deg = 180.0f;
  config.mission.vertical_fov_deg = 90.0f;
  return config;
}

}  // namespace

TEST(EosCycleOutputBuilderTest, MultiCycleMovingTargetsStayNearExternalTruth) {
  const eos_session::EosExternalPoseInput platform = MakePlatformInput();
  std::vector<eos_session::EosExternalTargetInput> targets = MakeMovingTargets(10U);
  eos_session::EosSession session = eos_session::EosSession::Create(MakeConfig());

  const std::size_t cycle_count = 36U;
  const float dt_sec = 0.5f;
  std::size_t compared_detection_count = 0U;
  for (std::size_t cycle = 0; cycle < cycle_count; ++cycle) {
    eos_session::EosCycleInput input;
    ASSERT_TRUE(eos_session::EosCycleInputAdapter::Build(platform, targets, dt_sec, &input))
        << "cycle=" << cycle;
    input.cycle_index = static_cast<std::uint32_t>(cycle);
    input.environment.solar_irradiance_w_m2 = 900.0f;
    input.environment.solar_altitude_deg = 45.0f;
    input.environment.cloud_coverage_ratio = 0.05f;
    input.environment.background_temperature_k = 285.0f;
    input.environment.day_night_type = eos_session::DayNightType::kDay;

    const eos_session::EosCycleResult result = session.StepWithResult(input);
    ASSERT_FALSE(result.has_validation_error) << "cycle=" << cycle;

    eos_session::EosExternalOutputFrame external_frame;
    ASSERT_TRUE(
        eos_session::EosCycleOutputAdapter::Build(platform, result.output_frame, &external_frame))
        << "cycle=" << cycle;

    for (std::size_t detection_index = 0; detection_index < external_frame.detections.size();
         ++detection_index) {
      const eos_session::EosExternalDetectionRecord& detection =
          external_frame.detections[detection_index];
      ASSERT_LE(detection.detection_id, result.detection_attributions.size()) << "cycle=" << cycle;
      const std::uint64_t target_id =
          result.detection_attributions[static_cast<std::size_t>(detection.detection_id - 1U)]
              .target_id;
      ASSERT_LT(target_id, targets.size()) << "cycle=" << cycle;
      const std::size_t target_index = static_cast<std::size_t>(target_id);
      const oneq::coordinate::EcefPositionM& truth =
          targets[target_index].kinematics.position_ecef_m;
      EXPECT_NEAR(detection.target_position_ecef_m.x_m, truth.x_m, 10.0)
          << "cycle=" << cycle << " target=" << target_index;
      EXPECT_NEAR(detection.target_position_ecef_m.y_m, truth.y_m, 10.0)
          << "cycle=" << cycle << " target=" << target_index;
      EXPECT_NEAR(detection.target_position_ecef_m.z_m, truth.z_m, 10.0)
          << "cycle=" << cycle << " target=" << target_index;
      ++compared_detection_count;
    }

    AdvanceTargets(dt_sec, &targets);
  }
  EXPECT_GT(compared_detection_count, 40U);
}

TEST(EosCycleOutputBuilderTest, ExternalOutputPreservesDetectionIdOnly) {
  const eos_session::EosExternalPoseInput platform = MakePlatformInput();
  eos_session::EosOutputFrame frame;
  frame.cycle_index = 7U;
  frame.scan_azimuth_deg = 0.0f;

  ::electro_optical_sensor::output::EosDetectionRecord detection;
  detection.detection_id = 42U;
  detection.range_m = 1000.0f;
  detection.azimuth_deg = 0.0f;
  detection.elevation_deg = 0.0f;
  detection.detected = true;
  frame.detections.push_back(detection);

  eos_session::EosExternalOutputFrame output;
  ASSERT_TRUE(eos_session::EosCycleOutputAdapter::Build(platform, frame, &output));
  ASSERT_EQ(output.detections.size(), 1U);
  EXPECT_EQ(output.detections.front().detection_id, 42U);
}

TEST(EosCycleOutputBuilderTest, DebugViewMergesRawOutputWithInputTargets) {
  eos_session::EosCycleInput input;
  input.cycle_index = 11U;
  eos_session::EosSceneTarget detected_target;
  detected_target.target_id = 1U;
  detected_target.target_name = "detected";
  eos_session::EosSceneTarget below_threshold_target;
  below_threshold_target.target_id = 2U;
  below_threshold_target.target_name = "weak";
  eos_session::EosSceneTarget outside_fov_target;
  outside_fov_target.target_id = 3U;
  outside_fov_target.target_name = "outside";
  input.scene = {detected_target, below_threshold_target, outside_fov_target};

  eos_session::EosCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.executed_this_cycle = true;
  result.output_frame.cycle_index = input.cycle_index;

  ::electro_optical_sensor::output::EosDetectionRecord detected;
  detected.detection_id = 1U;
  detected.detected = true;
  detected.fused_snr_db = 12.0f;
  detected.range_m = 1500.0f;
  ::electro_optical_sensor::output::EosDetectionRecord weak;
  weak.detection_id = 2U;
  weak.detected = false;
  weak.fused_snr_db = 1.0f;
  weak.range_m = 1600.0f;
  result.output_frame.detections = {detected, weak};
  ::electro_optical_sensor::attribution::EosDetectionAttributionRecord detected_attribution;
  detected_attribution.detection_id = 1U;
  detected_attribution.target_id = 1U;
  detected_attribution.target_name = "detected";
  ::electro_optical_sensor::attribution::EosDetectionAttributionRecord weak_attribution;
  weak_attribution.detection_id = 2U;
  weak_attribution.target_id = 2U;
  weak_attribution.target_name = "weak";
  result.detection_attributions = {detected_attribution, weak_attribution};

  const eos_session::EosOutputDebugView view =
      eos_session::EosOutputDebugViewBuilder::Build(input, result);
  ASSERT_EQ(view.targets.size(), 3U);
  EXPECT_EQ(view.targets[0].status, eos_session::EosDebugTargetStatus::kDetected);
  EXPECT_TRUE(view.targets[0].detected);
  EXPECT_EQ(view.targets[0].target_name, "detected");
  EXPECT_EQ(view.targets[1].status, eos_session::EosDebugTargetStatus::kObservedBelowThreshold);
  EXPECT_TRUE(view.targets[1].has_raw_output_record);
  EXPECT_EQ(view.targets[2].status, eos_session::EosDebugTargetStatus::kNotInOutput);
  EXPECT_FALSE(view.targets[2].has_raw_output_record);
}

TEST(EosCycleOutputBuilderTest, LifecycleRecorderTracksFoundLostAndOptionalNotDetected) {
  eos_session::EosCycleInput input;
  input.cycle_index = 20U;
  eos_session::EosSceneTarget tracked;
  tracked.target_id = 10U;
  tracked.target_name = "tracked";
  eos_session::EosSceneTarget never_seen;
  never_seen.target_id = 11U;
  never_seen.target_name = "never-seen";
  input.scene = {tracked, never_seen};

  eos_session::EosCycleResult first_result;
  first_result.input_cycle_index = input.cycle_index;
  first_result.executed_this_cycle = true;
  first_result.output_frame.cycle_index = input.cycle_index;
  ::electro_optical_sensor::output::EosDetectionRecord detected;
  detected.detection_id = 1U;
  detected.detected = true;
  detected.fused_snr_db = 15.0f;
  first_result.output_frame.detections.push_back(detected);
  ::electro_optical_sensor::attribution::EosDetectionAttributionRecord attribution;
  attribution.detection_id = 1U;
  attribution.target_id = 10U;
  attribution.target_name = "tracked";
  first_result.detection_attributions.push_back(attribution);

  eos_session::EosDetectionLifecycleRecorder recorder;
  std::vector<eos_session::EosDetectionLifecycleEvent> events =
      recorder.Update(input, first_result);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().kind, eos_session::EosDetectionLifecycleEventKind::kFirstDetected);
  EXPECT_EQ(events.front().target_name, "tracked");

  eos_session::EosCycleResult second_result;
  second_result.input_cycle_index = 21U;
  second_result.executed_this_cycle = true;
  second_result.output_frame.cycle_index = 21U;
  events = recorder.Update(input, second_result);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().kind, eos_session::EosDetectionLifecycleEventKind::kLost);
  EXPECT_EQ(events.front().reason, eos_session::EosDetectionLifecycleReason::kOutOfFov);

  eos_session::EosDetectionLifecycleRecorder diagnose_recorder(
      eos_session::EosDetectionLifecycleRecorderConfig{true});
  events = diagnose_recorder.Update(input, second_result);
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].kind, eos_session::EosDetectionLifecycleEventKind::kNotDetected);
  EXPECT_EQ(events[0].reason, eos_session::EosDetectionLifecycleReason::kOutOfFov);
  EXPECT_EQ(events[1].target_name, "never-seen");
}
