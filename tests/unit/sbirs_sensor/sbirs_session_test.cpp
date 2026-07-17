#include <gtest/gtest.h>

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
