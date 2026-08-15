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
  target.radiant_intensity_w_per_sr = 1.0e8;
  return sbirs_sensor::session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
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
  // 默认 tracking_mode=kEstimated → 捕获成功进入 kEstimatedTracking
  EXPECT_EQ(snapshot.target_states.at(42U),
            sbirs_sensor::pipeline::SbirsTargetState::kEstimatedTracking);
  ASSERT_EQ(snapshot.nfov_scheduler.target_to_channel.count(42U), 1U);
  EXPECT_EQ(snapshot.nfov_scheduler.target_to_channel.at(42U), 0);
  // 滤波状态已初始化
  ASSERT_EQ(snapshot.filter_states.count(42U), 1U);
  EXPECT_TRUE(snapshot.filter_states.at(42U).mean.allFinite());
}

TEST(SbirsStateMachineTest, StrictTruthModeEntersDedicatedTrackingState) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  pipeline.RunCycle(InputWithTarget(1U));
  const sbirs_sensor::pipeline::SbirsPipelineSnapshot snapshot = pipeline.CaptureRuntimeState();
  EXPECT_EQ(snapshot.target_states.at(42U),
            sbirs_sensor::pipeline::SbirsTargetState::kStrictTruthAssistedTracking);
  EXPECT_EQ(snapshot.filter_states.count(42U), 0U);
}

TEST(SbirsStateMachineTest, SensorLikeModeEntersDedicatedTrackingState) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kSensorLikeTruthAssisted;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  pipeline.RunCycle(InputWithTarget(1U));
  const sbirs_sensor::pipeline::SbirsPipelineSnapshot snapshot = pipeline.CaptureRuntimeState();
  EXPECT_EQ(snapshot.target_states.at(42U),
            sbirs_sensor::pipeline::SbirsTargetState::kSensorLikeTruthAssistedTracking);
  EXPECT_EQ(snapshot.filter_states.count(42U), 0U);
}

TEST(SbirsStateMachineTest, ThreeTrackingStatesAreMutuallyExclusive) {
  EXPECT_NE(sbirs_sensor::pipeline::SbirsTargetState::kEstimatedTracking,
            sbirs_sensor::pipeline::SbirsTargetState::kStrictTruthAssistedTracking);
  EXPECT_NE(sbirs_sensor::pipeline::SbirsTargetState::kStrictTruthAssistedTracking,
            sbirs_sensor::pipeline::SbirsTargetState::kSensorLikeTruthAssistedTracking);
}

}  // namespace
