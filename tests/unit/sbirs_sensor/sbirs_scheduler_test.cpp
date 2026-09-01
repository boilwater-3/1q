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
                                               double radiant_intensity_w_per_sr) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = id;
  target.position_ecef_m = Vector(7000000.0 + range_m, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = radiant_intensity_w_per_sr;
  return target;
}

// 构造一个指向给定 target_id 的候选（调度器排序只需 snr/range/target_id；
// target 指针类型为管线内部 ECI 场景目标）。
sbirs_sensor::pipeline::SbirsCandidate MakeCandidate(std::uint64_t target_id, double snr,
                                                     double range_m) {
  static sbirs_sensor::pipeline::SbirsEciSceneTarget backing[8];
  static int slot = 0;
  sbirs_sensor::pipeline::SbirsEciSceneTarget& tgt = backing[slot++ % 8];
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
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(Target(8U, 1000000.0, 1.0e8))
          .AddTarget(Target(3U, 1000000.0, 1.0e8))
          .Build();

  const sbirs_sensor::pipeline::SbirsPipelineResult result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_EQ(result.detections.front().attribution.target_id, 3U);
}

// 单镜筒锁定集合：无并发上限，候选全部入选，顺序按优先级（SNR 降序 → 距离升序 →
// ID 升序）——容量由轮转物理涌现，不再由调度器截断。
TEST(SbirsSchedulerTest, SelectsAllCandidatesInPriorityOrderWithoutCap) {
  sbirs_sensor::pipeline::SbirsNfovScheduler scheduler;
  std::vector<sbirs_sensor::pipeline::SbirsCandidate> candidates = {
      MakeCandidate(1U, 100.0, 1.0e6),
      MakeCandidate(2U, 300.0, 1.0e6),
      MakeCandidate(3U, 200.0, 1.0e6),
      MakeCandidate(4U, 300.0, 2.0e6),  // SNR 并列 2 号，距离更远 → 排后
  };
  const auto selected = scheduler.SelectForAcquisition(candidates);
  ASSERT_EQ(selected.size(), 4U);
  EXPECT_EQ(selected[0]->target->target_id, 2U);
  EXPECT_EQ(selected[1]->target->target_id, 4U);
  EXPECT_EQ(selected[2]->target->target_id, 3U);
  EXPECT_EQ(selected[3]->target->target_id, 1U);
}

// 已锁定目标不重复入选。
TEST(SbirsSchedulerTest, LockedTargetsAreExcludedFromSelection) {
  sbirs_sensor::pipeline::SbirsNfovScheduler scheduler;
  ASSERT_TRUE(scheduler.Acquire(2U));
  std::vector<sbirs_sensor::pipeline::SbirsCandidate> candidates = {
      MakeCandidate(1U, 100.0, 1.0e6),
      MakeCandidate(2U, 300.0, 1.0e6),
  };
  const auto selected = scheduler.SelectForAcquisition(candidates);
  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected[0]->target->target_id, 1U);
}

// Acquire 幂等；cue 来源随锁定记录，Release/Clear 单点清除。
TEST(SbirsSchedulerTest, AcquireIsIdempotentAndCueSourcePersists) {
  sbirs_sensor::pipeline::SbirsNfovScheduler scheduler;
  EXPECT_TRUE(scheduler.Acquire(1U));
  EXPECT_TRUE(scheduler.Acquire(1U, 104));  // 幂等：不重复入集合
  EXPECT_EQ(scheduler.LockedCount(), 1U);
  EXPECT_EQ(scheduler.CueSourceOf(1U), 104);

  EXPECT_TRUE(scheduler.Acquire(2U));
  EXPECT_EQ(scheduler.CueSourceOf(2U), -1);

  scheduler.Release(1U);
  EXPECT_FALSE(scheduler.IsLocked(1U));
  EXPECT_EQ(scheduler.CueSourceOf(1U), -1);
  EXPECT_TRUE(scheduler.IsLocked(2U));

  scheduler.Clear();
  EXPECT_EQ(scheduler.LockedCount(), 0U);
}

// LockedTargetIds 升序（轮转服务顺序的确定性基础）；Capture/Restore 往返保持集合。
TEST(SbirsSchedulerTest, LockedTargetIdsSortedAndCaptureRestoreRoundtrip) {
  sbirs_sensor::pipeline::SbirsNfovScheduler scheduler;
  scheduler.Acquire(30U, 204);
  scheduler.Acquire(10U);
  scheduler.Acquire(20U);

  const auto ids = scheduler.LockedTargetIds();
  ASSERT_EQ(ids.size(), 3U);
  EXPECT_EQ(ids[0], 10U);
  EXPECT_EQ(ids[1], 20U);
  EXPECT_EQ(ids[2], 30U);

  const auto snapshot = scheduler.Capture();
  sbirs_sensor::pipeline::SbirsNfovScheduler restored;
  restored.Restore(snapshot);
  EXPECT_EQ(restored.LockedCount(), 3U);
  EXPECT_EQ(restored.CueSourceOf(30U), 204);
  EXPECT_TRUE(restored.IsLocked(10U));
}

}  // namespace
