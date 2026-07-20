#include <gtest/gtest.h>

#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "sbirs_sensor/pipeline/SbirsPipeline.h"
#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"

namespace {

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_sensor::config::SbirsSessionConfig Config() {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  return config;
}

sbirs_sensor::session::SbirsCycleInput InputWithTarget(std::uint32_t cycle_index) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 42U;
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

TEST(SbirsStateMachineTest, CaptureTransitionsTargetIntoEstimatedTrackingByDefault) {
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(Config()));

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(InputWithTarget(1U));
  ASSERT_FALSE(result.detections.empty());

  const sbirs_sensor::pipeline::SbirsPipelineSnapshot snapshot = pipeline.CaptureRuntimeState();
  ASSERT_EQ(snapshot.target_states.count(42U), 1U);
  // 默认 enable_estimated_tracking=true → 捕获成功进入 kEstimatedTracking
  EXPECT_EQ(snapshot.target_states.at(42U),
            sbirs_sensor::pipeline::SbirsTargetState::kEstimatedTracking);
  ASSERT_EQ(snapshot.nfov_scheduler.target_to_channel.count(42U), 1U);
  EXPECT_EQ(snapshot.nfov_scheduler.target_to_channel.at(42U), 0);
  // 滤波状态已初始化
  ASSERT_EQ(snapshot.filter_states.count(42U), 1U);
  EXPECT_TRUE(snapshot.filter_states.at(42U).mean.allFinite());
}

TEST(SbirsStateMachineTest, DisabledTrackingFallsBackToTruthAssisted) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.policy.tracking.enable_estimated_tracking = false;  // 显式关闭 → 回退真值辅助
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  pipeline.RunCycle(InputWithTarget(1U));
  const sbirs_sensor::pipeline::SbirsPipelineSnapshot snapshot = pipeline.CaptureRuntimeState();
  EXPECT_EQ(snapshot.target_states.at(42U),
            sbirs_sensor::pipeline::SbirsTargetState::kTruthAssistedTracking);
  EXPECT_EQ(snapshot.filter_states.count(42U), 0U);  // 关闭时不初始化滤波状态
}

// kEstimatedTracking 与 kTruthAssistedTracking 严格分离，满足 design 2.5 的状态拆分前置。
TEST(SbirsStateMachineTest, EstimatedTrackingStateIsReserved) {
  EXPECT_NE(sbirs_sensor::pipeline::SbirsTargetState::kEstimatedTracking,
            sbirs_sensor::pipeline::SbirsTargetState::kTruthAssistedTracking);
}

}  // namespace
