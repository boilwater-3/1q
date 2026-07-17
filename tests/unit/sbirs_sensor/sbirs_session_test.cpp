#include <gtest/gtest.h>

#include <limits>

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace {

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_sensor::session::SbirsCycleInput ValidInput(std::uint32_t cycle_index) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.target_name = "hot";
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.temperature_k = 2200.0f;
  target.projected_area_m2 = 5000.0f;
  return sbirs_sensor::session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .AddTarget(target)
      .Build();
}

sbirs_sensor::config::SbirsSessionConfig Config() {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  return config;
}

TEST(SbirsSessionTest, ValidCycleProducesDeterministicOutput) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(ValidInput(1U));
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_FALSE(result.output_frame.detections.empty());
}

TEST(SbirsSessionTest, InvalidFirstCycleReturnsEmptyOutput) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  sbirs_sensor::session::SbirsCycleInput input = ValidInput(1U);
  input.dt_sec = -1.0f;
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_TRUE(result.has_validation_error);
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(SbirsSessionTest, InvalidLaterCycleReusesLatestOutput) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  const sbirs_sensor::session::SbirsCycleResult valid = session.StepWithResult(ValidInput(1U));
  ASSERT_FALSE(valid.output_frame.detections.empty());
  sbirs_sensor::session::SbirsCycleInput invalid = ValidInput(2U);
  invalid.dt_sec = 0.0f;
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(invalid);
  EXPECT_TRUE(result.reused_previous_output);
  EXPECT_EQ(result.output_frame.detections.size(), valid.output_frame.detections.size());
}

TEST(SbirsSessionTest, ValidationRejectDoesNotAdvancePipelineState) {
  sbirs_sensor::session::SbirsSession rejected_then_valid =
      sbirs_sensor::session::SbirsSession::Create(Config());
  sbirs_sensor::session::SbirsCycleInput invalid = ValidInput(1U);
  invalid.scene.front().emissivity = std::numeric_limits<float>::quiet_NaN();
  ASSERT_TRUE(rejected_then_valid.StepWithResult(invalid).has_validation_error);

  const sbirs_sensor::session::SbirsCycleResult after_reject =
      rejected_then_valid.StepWithResult(ValidInput(2U));
  sbirs_sensor::session::SbirsSession clean =
      sbirs_sensor::session::SbirsSession::Create(Config());
  const sbirs_sensor::session::SbirsCycleResult clean_result =
      clean.StepWithResult(ValidInput(2U));

  ASSERT_TRUE(after_reject.executed_this_cycle);
  ASSERT_EQ(after_reject.output_frame.detections.size(),
            clean_result.output_frame.detections.size());
  ASSERT_FALSE(after_reject.output_frame.detections.empty());
  EXPECT_FLOAT_EQ(after_reject.output_frame.scan_azimuth_deg,
                  clean_result.output_frame.scan_azimuth_deg);
  EXPECT_EQ(after_reject.output_frame.detections.front().detection_id,
            clean_result.output_frame.detections.front().detection_id);
  EXPECT_FLOAT_EQ(after_reject.output_frame.detections.front().azimuth_deg,
                  clean_result.output_frame.detections.front().azimuth_deg);
  EXPECT_FLOAT_EQ(after_reject.output_frame.detections.front().elevation_deg,
                  clean_result.output_frame.detections.front().elevation_deg);
  EXPECT_FLOAT_EQ(after_reject.output_frame.detections.front().infrared_snr_linear,
                  clean_result.output_frame.detections.front().infrared_snr_linear);
}

TEST(SbirsRuntimeConfigResolverTest, InvalidPatchDoesNotPolluteConfig) {
  sbirs_sensor::session::SbirsSession session =
      sbirs_sensor::session::SbirsSession::Create(Config());
  const sbirs_sensor::config::SbirsRuntimeConfigPatch invalid =
      sbirs_sensor::config::SbirsRuntimeConfigBuilder().WithScanRateDegPerSec(-5.0f).Build();
  EXPECT_FALSE(session.TryApplyRuntimeConfig(invalid));
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(ValidInput(1U));
  EXPECT_TRUE(result.executed_this_cycle);
}

}  // namespace
