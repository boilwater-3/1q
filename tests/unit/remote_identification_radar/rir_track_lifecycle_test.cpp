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
using tracking::RirTrackFilterConfig;
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
  // 去真值化后速度证据只来自位置差分：两拍位置相同 → 后验速度≈0（速度无知先验
  // 9e6 m²/s² 下量测主导；旧断言 1.5025 依赖真值速度种子 3 m/s + 小速度方差）。
  EXPECT_NEAR(snapshots[0].velocity.x(), 0.0f, 0.01f);
  EXPECT_NEAR(snapshots[0].speed, 0.0f, 0.01f);
  // 加速度 = 后验速度的周期间差分：首拍种子 3（测试 helper 显式传入）→ 本拍 ≈0，
  // 差分 ≈ −3 m/s²；量测速度字段从 3 改到 4 不影响（种子不参与第二拍证据）。
  EXPECT_NEAR(snapshots[0].acceleration_mps2, 3.0f, 0.05f);
  EXPECT_EQ(snapshots[0].external_target_id, 7001U);
  EXPECT_EQ(snapshots[0].target_name, "target-a");
  // EstimationUncertaintyTrace 只累加位置分块（初始 3×100=300，速度无知先验
  // 不进该口径）；两拍后位置迹收敛至个位数量级，保持紧界。
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
  // 重捕周期滤波速度以速度种子重初始化、无差分基准：加速度显式置零（无尖峰）。
  EXPECT_FLOAT_EQ(snapshots[0].acceleration.x(), 0.0f);
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

/// @brief 退化量测（速度向量为零）：速度唯一来源是滤波后验，无种子回填路径；
///        位置量测仍按匀速真值给出时滤波速度与加速度均保持无扰动。
TEST(RirTrackLifecycleTest, DegradedZeroVectorVelocityKeepsFilterContinuity) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 2U;
  config.max_lost_cycles = 5U;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U, 2701U), {MakeMeasurement(7U, 0.0f, 10.0f, false)});
  lifecycle.Update(MakeCycle(2U, 2702U), {MakeMeasurement(7U, 10.0f, 10.0f, true)});

  // 退化量测：速度向量为全零（无独立标量速度源可回填）。
  lifecycle.Update(MakeCycle(3U, 2703U), {MakeMeasurement(7U, 20.0f, 0.0f, true)});

  const RirTrackState* track = lifecycle.FindTrack(7U);
  ASSERT_NE(track, nullptr);
  // 零新息下后验速度保持 ≈10 m/s；加速度为后验差分（上周期后验 10 → 本周期 10）→ 0。
  EXPECT_NEAR(track->velocity.x(), 10.0f, 0.01f);
  EXPECT_NEAR(track->acceleration.x(), 0.0f, 1e-4f);
  EXPECT_NEAR(track->acceleration_mps2, 0.0f, 1e-4f);
}

/// @brief 加速度 = 滤波后验速度的周期间差分（物理加速度口径）：匀加速目标
///        （v0=10 m/s、a=4 m/s²、dt=1 s）建轨周期置零，持续周期差分收敛到
///        真值加速度且方向一致（旧口径差分基准为本周期速度种子，方向相反）。
TEST(RirTrackLifecycleTest, AccelerationIsPosteriorVelocityFiniteDifference) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 3U;
  config.max_lost_cycles = 5U;
  config.enable_imm_lifecycle = false;  // CV 路径，聚焦差分口径本身
  RirTrackLifecycle lifecycle(config);

  // 真值运动学：p(k)=10k+2k²，v(k)=10+4k。
  lifecycle.Update(MakeCycle(1U, 2801U), {MakeMeasurement(9U, 12.0f, 14.0f)});
  ASSERT_NE(lifecycle.FindTrack(9U), nullptr);
  // 建轨周期：滤波速度以速度种子重初始化，无差分基准 → 加速度置零。
  EXPECT_FLOAT_EQ(lifecycle.FindTrack(9U)->acceleration.x(), 0.0f);

  float previous_velocity = lifecycle.FindTrack(9U)->velocity.x();
  for (std::uint32_t cycle = 2U; cycle <= 7U; ++cycle) {
    const float position =
        10.0f * static_cast<float>(cycle) +
        2.0f * static_cast<float>(cycle) * static_cast<float>(cycle);
    lifecycle.Update(MakeCycle(cycle, 2800U + cycle),
                     {MakeMeasurement(9U, position, 10.0f + 4.0f * static_cast<float>(cycle),
                                      true)});
    const RirTrackState* track = lifecycle.FindTrack(9U);
    ASSERT_NE(track, nullptr);
    // 逐周期口径自检：加速度等于后验速度差分（速度种子 10+4k 不参与基准）。
    EXPECT_NEAR(track->acceleration.x(), (track->velocity.x() - previous_velocity) / 1.0f,
                1e-4f);
    previous_velocity = track->velocity.x();
  }

  // 收敛：差分加速度逼近真值 4 m/s²，方向为正（旧种子差口径为负向虚高）。
  EXPECT_NEAR(lifecycle.FindTrack(9U)->acceleration.x(), 4.0f, 1.5f);
}

/// @brief IMM 模型集锚定滤波配置 q：模型 q = 配置 q 的对数等距张成
///        （q=30 → {30, 300}），配置不再被缺省 {1,10} 覆盖（审计 A5）。
TEST(RirTrackLifecycleTest, ImmModelSetAnchorsConfiguredProcessNoise) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  RirTrackFilterConfig filter_config;
  filter_config.process_noise_diff_coeff = 30.0f;
  RirTrackLifecycle lifecycle(config, filter_config);

  lifecycle.Update(MakeCycle(1U, 3001U), {MakeMeasurement(7U, 0.0f, 5.0f)});
  lifecycle.Update(MakeCycle(2U, 3002U), {MakeMeasurement(7U, 5.0f, 5.0f, true)});

  const tracking::RirImmFilter* imm = lifecycle.FindImmFilter(7U);
  ASSERT_NE(imm, nullptr);
  ASSERT_EQ(imm->ModelCount(), 2U);
  EXPECT_NEAR(imm->model_noise_diff_coeffs()[0], 30.0f, 1e-3f);
  EXPECT_NEAR(imm->model_noise_diff_coeffs()[1], 300.0f, 1e-3f);
}

/// @brief 滑行（失配）后重命中：加速度差分按距上次命中的实际经过时间
///        （(m+1)·dt）折算，不按单周期 dt 膨胀（滑行周期 CV 外推速度不变）。
TEST(RirTrackLifecycleTest, RehitAfterCoastDividesByElapsedTime) {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 2U;
  config.max_lost_cycles = 5U;
  config.enable_imm_lifecycle = false;  // CV 路径，聚焦差分口径本身
  RirTrackLifecycle lifecycle(config);

  // 周期1 命中建轨（后验速度精确=种子 10 m/s）；周期2 失配滑行；周期3 重命中
  // 加速目标（真值 p3=48、v3=18，速度差跨越 2 个周期）。
  lifecycle.Update(MakeCycle(1U, 2901U), {MakeMeasurement(9U, 0.0f, 10.0f)});
  lifecycle.Update(MakeCycle(2U, 2902U), {});
  lifecycle.Update(MakeCycle(3U, 2903U), {MakeMeasurement(9U, 48.0f, 18.0f, true)});

  const RirTrackState* track = lifecycle.FindTrack(9U);
  ASSERT_NE(track, nullptr);
  // 差分 = 后验速度增量 ÷ 实际经过时间 2s（旧单周期 dt 口径该值膨胀 2 倍 → 失败）。
  EXPECT_NEAR(track->acceleration.x(), (track->velocity.x() - 10.0f) / 2.0f, 1e-3f);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
