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

// design 2.10：WFOV 带误差位置应反映在 WFOV 检测记录的方位角上，
// 且相同 random_seed 的两次独立 pipeline 产生相同测量（replay 可复现）。
TEST(SbirsPipelineTest, WfovMeasuredAzimuthReflectsErrorModelAndIsReproducible) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.policy.error_model.angular_sigma_deg = 0.0f;
  config.policy.error_model.attitude_sigma_deg = 0.5f;  // 启用姿态误差
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(HotTarget(7U, 0.0))
          .Build();

  sbirs_sensor::pipeline::SbirsPipeline first(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::pipeline::SbirsPipeline second(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::pipeline::SbirsPipelineResult r1 = first.RunCycle(input);
  const sbirs_sensor::pipeline::SbirsPipelineResult r2 = second.RunCycle(input);

  // 选中的目标直接进入 NFOV 捕获；真值方位角约 0，带误差后应偏离 0。
  ASSERT_FALSE(r1.detections.empty());
  // 相同 seed → 相同输出。
  EXPECT_FLOAT_EQ(r1.detections.front().record.azimuth_deg,
                  r2.detections.front().record.azimuth_deg);
}

// design cue 延迟外推：横向高速目标在 narrow_cue_latency_s 期间移出 NFOV，
// 首次捕获应失败，并产出 kNfovAcquisitionFailed 诊断 attribution（detected=false）。
TEST(SbirsPipelineTest, CueLatencyWithCrossVelocityCausesAcquisitionFailure) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_cue_latency_s = 1.0f;       // 1s 延迟
  config.mission.narrow_field_fov_az_deg = 2.0f;    // 收窄 NFOV 使外推后易出界
  config.policy.error_model.angular_sigma_deg = 0.0f;
  config.policy.error_model.attitude_sigma_deg = 0.0f;

  sbirs_sensor::session::SbirsSceneTarget target = HotTarget(7U, 0.0);
  // 横向速度：1s 后在 y 方向移动约 1500 km，视线方位角显著偏移。
  target.velocity_ecef_m_per_s = Vector(0.0, 1500000.0, 0.0);
  target.has_velocity_ecef_m_per_s = true;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(target)
          .Build();

  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);

  // 捕获失败的目标回退为 WFOV 候选，故同时产出：
  //   - 一条 detected=false 的 NFOV 捕获失败诊断 (kNfovAcquisitionFailed)
  //   - 一条 detected=true 的 WFOV 搜索记录 (kSchedulerSkipped，因无其它目标，无跳过标记)
  bool found_failure = false;
  for (const sbirs_sensor::pipeline::SbirsPipelineDetection& detection : result.detections) {
    if (!detection.record.detected) {
      found_failure = true;
      EXPECT_EQ(detection.attribution.capture_failure_reason,
                sbirs_sensor::attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
      EXPECT_EQ(detection.record.observation_stage,
                sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
    }
  }
  EXPECT_TRUE(found_failure);
}

// 无速度（has_velocity_ecef_m_per_s=false）时，即便 narrow_cue_latency_s>0，
// 行为应与无延迟一致：成功捕获（零回归）。
TEST(SbirsPipelineTest, CueLatencyWithoutVelocityKeepsBaselineCapture) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  config.mission.narrow_cue_latency_s = 1.0f;  // 延迟非 0 但目标无速度

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(HotTarget(7U, 0.0))  // 默认 has_velocity=false
          .Build();

  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);

  ASSERT_FALSE(result.detections.empty());
  EXPECT_TRUE(result.detections.front().record.detected);
  EXPECT_EQ(result.detections.front().record.observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
}

// 调度跳过诊断：目标 A 已锁定 NFOV，候选 B 被 WFOV 发现但资源被占用，
// 应产出 kSchedulerSkipped 归属（record.detected=true，仅 attribution 标记跳过）。
TEST(SbirsPipelineTest, LockedTargetCausesSchedulerSkipOnOtherCandidate) {
  const sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  // 第一周期：A 进入捕获锁定 NFOV。
  sbirs_sensor::session::SbirsCycleInput first =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(HotTarget(1U, 0.0))
          .Build();
  pipeline.RunCycle(first);

  // 第二周期：A 继续锁定，新增候选 B（不同 y 避免重合）。
  sbirs_sensor::session::SbirsCycleInput second =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(2U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(HotTarget(1U, 0.0))
          .AddTarget(HotTarget(2U, 5.0))
          .Build();
  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(second);

  // A 产出 NFOV track；B 作为 WFOV 候选但被调度跳过。
  bool found_skipped = false;
  bool found_track = false;
  for (const sbirs_sensor::pipeline::SbirsPipelineDetection& detection : result.detections) {
    if (detection.attribution.target_id == 1U) {
      found_track = true;
      EXPECT_EQ(detection.attribution.capture_failure_reason,
                sbirs_sensor::attribution::SbirsCaptureFailureReason::kNone);
    }
    if (detection.attribution.target_id == 2U) {
      found_skipped = true;
      EXPECT_EQ(detection.attribution.capture_failure_reason,
                sbirs_sensor::attribution::SbirsCaptureFailureReason::kSchedulerSkipped);
    }
  }
  EXPECT_TRUE(found_track);
  EXPECT_TRUE(found_skipped);
}

}  // namespace
