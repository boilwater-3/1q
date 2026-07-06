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

sbirs_sensor::session::SbirsSceneTarget Target(std::uint64_t id, double range_m,
                                               float temperature_k) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = id;
  target.position_ecef_m = Vector(7000000.0 + range_m, 0.0, 0.0);
  target.temperature_k = temperature_k;
  target.projected_area_m2 = 5000.0f;
  return target;
}

TEST(SbirsSchedulerTest, TargetIdBreaksOtherwiseEqualCandidateTie) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  config.policy.error_model.angular_sigma_deg = 0.0f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(Target(8U, 1000000.0, 2200.0f))
          .AddTarget(Target(3U, 1000000.0, 2200.0f))
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_EQ(result.detections.front().attribution.target_id, 3U);
}

}  // namespace
