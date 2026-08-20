// Copyright 2026. All Rights Reserved.
//
// @file rir_track_lifecycle_test.cpp
// @brief 验证 RIR 轻量跟踪子集航迹生命周期与 KF 接线（阶段 2-T T3/T4）。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "remote_identification_radar/tracking/RirTrackLifecycle.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using tracking::RirCycleContext;
using tracking::RirLifecycleConfig;
using tracking::RirMeasurementCovariance;
using tracking::RirTrackLifecycle;
using tracking::RirTrackMeasurement;
using tracking::RirTrackSeed;
using tracking::RirTrackSnapshotList;
using tracking::RirTrackState;
using tracking::RirTrackStatus;

RirTrackMeasurement MakeMeasurement(std::uint64_t key, float px, float vx,
                                    bool matched_existing = false, std::uint64_t external_id = 0U,
                                    const std::string& target_name = "") {
  RirTrackMeasurement measurement;
  measurement.association_key = key;
  measurement.matched_existing_track = matched_existing;
  measurement.position = Eigen::Vector3f(px, 0.0f, 0.0f);
  measurement.velocity = Eigen::Vector3f(vx, 0.0f, 0.0f);
  measurement.measurement_covariance = RirMeasurementCovariance::Identity();
  measurement.external_target_id = external_id;
  measurement.target_name = target_name;
  return measurement;
}

RirCycleContext MakeCycle(std::uint32_t cycle_index, std::uint64_t batch_id, float dt_sec = 1.0f) {
  RirCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = dt_sec;
  return cycle;
}

/// @brief tentative 达到 confirm_hits 后转 confirmed；hit/miss 计数语义正确。
TEST(RirTrackLifecycleTest, ConfirmsTrackAfterConfiguredHits) {
  RirLifecycleConfig config;
  config.confirm_hits = 2U;
  config.max_miss_before_lost = 1U;
  config.max_lost_cycles = 2U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 1001U),
                   {MakeMeasurement(7U, 0.0f, 3.0f, false, 7001U, "target-a")});

  RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kTentative);
  EXPECT_EQ(snapshots[0].hit_count, 1U);
  EXPECT_EQ(snapshots[0].miss_count, 0U);

  lifecycle.Update(MakeCycle(2U, 1002U), {MakeMeasurement(7U, 0.0f, 4.0f, true)});

  snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kConfirmed);
  EXPECT_EQ(snapshots[0].hit_count, 2U);
  // 量测仅更新位置；速度经 KF 交叉协方差从先验 3 m/s 向后验约 1.5025 m/s 修正。
  EXPECT_NEAR(snapshots[0].velocity.x(), 1.50248f, 0.01f);
  EXPECT_NEAR(snapshots[0].speed, 1.50248f, 0.01f);
  EXPECT_NEAR(snapshots[0].acceleration_mps2, 2.49752f, 0.01f);
  EXPECT_EQ(snapshots[0].external_target_id, 7001U);
  EXPECT_EQ(snapshots[0].target_name, "target-a");
  EXPECT_LT(snapshots[0].EstimationUncertaintyTrace(), 300.0f);
}

/// @brief 连续失配：confirmed → lost → 超时回收，回收后键不再出现在快照。
TEST(RirTrackLifecycleTest, RecyclesTrackAfterLostTimeout) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 0U;
  config.max_lost_cycles = 1U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 2001U), {MakeMeasurement(3U, 0.0f, 1.0f)});
  lifecycle.Update(MakeCycle(2U, 2002U), {});

  RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kLost);
  EXPECT_EQ(snapshots[0].miss_count, 1U);

  lifecycle.Update(MakeCycle(3U, 2003U), {});
  EXPECT_TRUE(lifecycle.BuildTrackSnapshots().empty());
}

/// @brief lost 航迹再次命中立即确认，并从量测速度种子重初始化（无速度尖峰）。
TEST(RirTrackLifecycleTest, LostTrackRehitResetsFilterAndConfirms) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 0U;
  config.max_lost_cycles = 3U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 2301U), {MakeMeasurement(31U, 0.0f, 0.0f, false, 8001U)});
  lifecycle.Update(MakeCycle(2U, 2302U), {});

  ASSERT_EQ(lifecycle.BuildTrackSnapshots()[0].status, RirTrackStatus::kLost);
  lifecycle.Update(MakeCycle(3U, 2303U), {MakeMeasurement(31U, 300.0f, 0.2f, true, 8001U)});

  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kConfirmed);
  EXPECT_FLOAT_EQ(snapshots[0].velocity.x(), 0.2f);
  EXPECT_LT(std::sqrt(snapshots[0].velocity.x() * snapshots[0].velocity.x()), 1.0f);
}

/// @brief 航迹回收后新键获得新 track_id；键重分配语义由单调键保证。
/// N3 池化：回收槽位被复用，generation 单调递增标识旧引用。
TEST(RirTrackLifecycleTest, ReusedKeyAfterRecycleStartsAsNewTrack) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 0U;
  config.max_lost_cycles = 1U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 2101U), {MakeMeasurement(11U, 100.0f, 5.0f)});
  ASSERT_EQ(lifecycle.BuildTrackSnapshots()[0].track_id, 1U);
  EXPECT_EQ(lifecycle.BuildTrackSnapshots()[0].generation, 0U);
  lifecycle.Update(MakeCycle(2U, 2102U), {});
  lifecycle.Update(MakeCycle(3U, 2103U), {});
  EXPECT_TRUE(lifecycle.BuildTrackSnapshots().empty());

  lifecycle.Update(MakeCycle(4U, 2104U), {MakeMeasurement(12U, 120.0f, 6.0f)});
  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].association_key, 12U);
  EXPECT_EQ(snapshots[0].track_id, 2U);
  EXPECT_EQ(snapshots[0].hit_count, 1U);
  // 复用槽位携带代次：新航迹的 generation = 槽位被回收次数（LIFO 复用刚回收的槽位）。
  EXPECT_EQ(snapshots[0].generation, 1U);
}

/// @brief 关联种子导出包含位置与高斯状态，供下一周期门控。
TEST(RirTrackLifecycleTest, BuildAssociationSeedsExportsGaussianState) {
  RirTrackLifecycle lifecycle;
  lifecycle.Update(MakeCycle(1U, 2201U), {MakeMeasurement(21U, 100.0f, 2.0f)});

  const std::vector<RirTrackSeed> seeds = lifecycle.BuildAssociationSeeds();
  ASSERT_EQ(seeds.size(), 1U);
  EXPECT_EQ(seeds[0].association_key, 21U);
  EXPECT_TRUE(seeds[0].has_position);
  EXPECT_TRUE(seeds[0].has_gaussian_state);
  EXPECT_FLOAT_EQ(seeds[0].position.x(), 100.0f);
  EXPECT_FLOAT_EQ(seeds[0].gaussian_state.mean(0), 100.0f);
}

/// @brief 非法 dt：整周期跳过，不建轨也不推进既有状态。
TEST(RirTrackLifecycleTest, InvalidDtSkipsCycleWithoutStateChange) {
  RirTrackLifecycle lifecycle;
  lifecycle.Update(MakeCycle(1U, 2401U), {MakeMeasurement(41U, 10.0f, 1.0f)});
  const RirTrackState before = lifecycle.BuildTrackSnapshots()[0];

  lifecycle.Update(MakeCycle(2U, 2402U, 0.0f), {MakeMeasurement(41U, 20.0f, 3.0f, true)});

  const RirTrackSnapshotList after = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(after.size(), 1U);
  EXPECT_FLOAT_EQ(after[0].position.x(), before.position.x());
  EXPECT_FLOAT_EQ(after[0].velocity.x(), before.velocity.x());
  EXPECT_EQ(after[0].last_update_cycle, before.last_update_cycle);
}

/// @brief 运行态捕获/恢复往返：航迹、track_id 分配器与周期号全部回滚。
TEST(RirTrackLifecycleTest, RuntimeStateRoundTrip) {
  RirTrackLifecycle source;
  source.Update(MakeCycle(1U, 2501U), {MakeMeasurement(51U, 10.0f, 1.0f)});
  source.Update(MakeCycle(2U, 2502U), {MakeMeasurement(52U, 20.0f, 2.0f)});
  const auto state = source.CaptureRuntimeState();

  RirTrackLifecycle restored;
  restored.RestoreRuntimeState(state);
  EXPECT_EQ(restored.ActiveTrackCount(), 2U);
  // 捕获发生在第 2 周期后：key=51 已按 dt=1 外推一个周期。
  EXPECT_FLOAT_EQ(restored.FindTrack(51U)->position.x(), 11.0f);
  EXPECT_FLOAT_EQ(restored.FindTrack(51U)->velocity.x(), 1.0f);
  EXPECT_EQ(restored.FindTrack(52U)->position.x(), 20.0f);

  restored.Update(MakeCycle(3U, 2503U), {MakeMeasurement(53U, 30.0f, 3.0f)});
  EXPECT_EQ(restored.FindTrack(53U)->track_id, 3U);
}

/// @brief 快照按关联键升序，保证 replay/日志输出确定性。
TEST(RirTrackLifecycleTest, SnapshotsAreOrderedByAssociationKey) {
  RirTrackLifecycle lifecycle;
  lifecycle.Update(MakeCycle(1U, 2601U),
                   {MakeMeasurement(20U, 10.0f, 1.0f), MakeMeasurement(5U, 20.0f, 2.0f)});

  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 2U);
  EXPECT_EQ(snapshots[0].association_key, 5U);
  EXPECT_EQ(snapshots[1].association_key, 20U);
  // 同周期多新航迹的 track_id 按输入顺序分配（replay 确定性）。
  EXPECT_EQ(snapshots[1].track_id, 1U);
  EXPECT_EQ(snapshots[0].track_id, 2U);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
