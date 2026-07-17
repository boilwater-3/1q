#include <gtest/gtest.h>

#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "sbirs_sensor/pipeline/SbirsNfovScheduler.h"
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

// 构造一个指向给定 target_id 的候选（调度器排序只需 snr/range/target_id）。
sbirs_sensor::pipeline::SbirsCandidate MakeCandidate(std::uint64_t target_id, double snr,
                                                     double range_m) {
  static sbirs_sensor::session::SbirsSceneTarget backing[8];
  static int slot = 0;
  sbirs_sensor::session::SbirsSceneTarget& tgt = backing[slot++ % 8];
  tgt.target_id = target_id;
  sbirs_sensor::pipeline::SbirsCandidate c;
  c.target = &tgt;
  c.snr = snr;
  c.range_m = range_m;
  return c;
}

TEST(SbirsSchedulerTest, TargetIdBreaksOtherwiseEqualCandidateTie) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
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

// 多通道：max=2 时，三个候选中按优先级（SNR 降序）选出前两个进入捕获。
TEST(SbirsSchedulerTest, MultiChannelLocksUpToMaxTargets) {
  sbirs_sensor::pipeline::SbirsNfovScheduler scheduler(2);
  std::vector<sbirs_sensor::pipeline::SbirsCandidate> candidates = {
      MakeCandidate(1U, 100.0, 1.0e6),
      MakeCandidate(2U, 300.0, 1.0e6),
      MakeCandidate(3U, 200.0, 1.0e6),
  };
  const auto selected = scheduler.SelectForAcquisition(candidates);
  ASSERT_EQ(selected.size(), 2U);
  EXPECT_EQ(selected[0]->target->target_id, 2U);  // 最高 SNR
  EXPECT_EQ(selected[1]->target->target_id, 3U);  // 次高 SNR
}

// 通道满时 SelectForAcquisition 返回空，未选中的候选由 pipeline 标记 kSchedulerSkipped。
TEST(SbirsSchedulerTest, SelectForAcquisitionEmptyWhenChannelsFull) {
  sbirs_sensor::pipeline::SbirsNfovScheduler scheduler(1);
  ASSERT_EQ(scheduler.Acquire(1U), 0);  // 占用唯一通道
  std::vector<sbirs_sensor::pipeline::SbirsCandidate> candidates = {
      MakeCandidate(2U, 100.0, 1.0e6),
  };
  const auto selected = scheduler.SelectForAcquisition(candidates);
  EXPECT_TRUE(selected.empty());
}

// channel_id 采用最小可用编号分配，释放后回收。
TEST(SbirsSchedulerTest, ChannelIdReusedAfterRelease) {
  sbirs_sensor::pipeline::SbirsNfovScheduler scheduler(3);
  EXPECT_EQ(scheduler.Acquire(10U), 0);
  EXPECT_EQ(scheduler.Acquire(20U), 1);
  EXPECT_EQ(scheduler.Acquire(30U), 2);
  EXPECT_EQ(scheduler.Acquire(40U), -1);  // 已满
  scheduler.Release(20U);
  EXPECT_EQ(scheduler.Acquire(40U), 1);  // 回收最小空闲编号 1
}

// Acquire 对已锁定目标返回原通道（幂等），不重复分配。
TEST(SbirsSchedulerTest, AcquireIdempotentForLockedTarget) {
  sbirs_sensor::pipeline::SbirsNfovScheduler scheduler(2);
  EXPECT_EQ(scheduler.Acquire(1U), 0);
  EXPECT_EQ(scheduler.Acquire(1U), 0);  // 重复 Acquire 返回同一通道
  EXPECT_EQ(scheduler.LockedCount(), 1U);
}

// ChannelOf 对未锁定目标返回 -1；Capture/Restore 往返保持分配关系。
TEST(SbirsSchedulerTest, ChannelOfAndCaptureRestoreRoundtrip) {
  sbirs_sensor::pipeline::SbirsNfovScheduler scheduler(2);
  EXPECT_EQ(scheduler.ChannelOf(1U), -1);
  scheduler.Acquire(1U);
  scheduler.Acquire(2U);

  const auto snapshot = scheduler.Capture();
  sbirs_sensor::pipeline::SbirsNfovScheduler restored(2);
  restored.Restore(snapshot);
  EXPECT_EQ(restored.ChannelOf(1U), 0);
  EXPECT_EQ(restored.ChannelOf(2U), 1);
  EXPECT_EQ(restored.LockedCount(), 2U);
}

}  // namespace
