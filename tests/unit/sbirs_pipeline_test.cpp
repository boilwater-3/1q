#include <gtest/gtest.h>

#include "1q/sbirs_sensor/config/SbirsSessionConfigBuilder.h"
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

sbirs_sensor::session::SbirsSceneTarget HotTarget(std::uint64_t id, double y) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = id;
  target.target_name = "target";
  target.position_ecef_m = Vector(8000000.0, y, 0.0);
  target.temperature_k = 2200.0f;
  target.projected_area_m2 = 5000.0f;
  return target;
}

sbirs_sensor::config::SbirsSessionConfig PipelineConfig() {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  config.policy.error_model.angular_sigma_deg = 0.0f;
  return config;
}

TEST(SbirsPipelineTest, WideCandidateCapturesIntoNfov) {
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(HotTarget(7U, 0.0))
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_EQ(result.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
  EXPECT_TRUE(result.detections.front().attribution.used_truth_assist);
}

TEST(SbirsPipelineTest, LockedTargetProducesTruthAssistedTrack) {
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(HotTarget(7U, 0.0))
          .Build();
  pipeline.RunCycle(input);
  input.cycle_index = 2U;
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_EQ(result.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
}

TEST(SbirsSchedulerTest, HigherSnrCandidateWinsBeforeDistanceTieBreak) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsSceneTarget weak = HotTarget(1U, 0.0);
  weak.temperature_k = 1200.0f;
  sbirs_sensor::session::SbirsSceneTarget strong = HotTarget(2U, 1000.0);
  strong.temperature_k = 2400.0f;
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(weak)
          .AddTarget(strong)
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_EQ(result.detections.front().attribution.target_id, 2U);
}

}  // namespace
