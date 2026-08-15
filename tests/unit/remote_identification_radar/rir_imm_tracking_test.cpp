// Copyright 2026. All Rights Reserved.
//
// @file rir_imm_tracking_test.cpp
// @brief 验证 RIR IMM 生命周期双路径（阶段 2-T N4/N5）。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "remote_identification_radar/tracking/RirImmFilter.h"
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
using tracking::RirTrackSnapshotList;
using tracking::RirTrackStatus;

RirTrackMeasurement MakeMeasurement(std::uint64_t key, float px, float vx,
                                    bool matched_existing) {
  RirTrackMeasurement measurement;
  measurement.association_key = key;
  measurement.matched_existing_track = matched_existing;
  measurement.position = Eigen::Vector3f(px, 0.0f, 0.0f);
  measurement.velocity = Eigen::Vector3f(vx, 0.0f, 0.0f);
  measurement.measurement_covariance = RirMeasurementCovariance::Identity();
  return measurement;
}

RirCycleContext MakeCycle(std::uint32_t cycle_index, float dt_sec = 1.0f) {
  RirCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = 1U;
  cycle.dt_sec = dt_sec;
  return cycle;
}

RirLifecycleConfig MakeImmConfig() {
  RirLifecycleConfig config;
  config.confirm_hits = 1U;
  config.max_miss_before_lost = 2U;
  config.max_lost_cycles = 5U;
  config.enable_imm_lifecycle = true;
  config.model_count_hint = 2U;
  return config;
}

/// @brief IMM 缺省未启用：confirmed 命中不建 IMM 运行态（CV KF 路径），
///        与 2-T 既有行为数值一致（既有生命周期用例锁定）。
TEST(RirImmTrackingTest, DisabledLifecycleKeepsCvPathWithoutImmState) {
  RirLifecycleConfig config = MakeImmConfig();
  config.enable_imm_lifecycle = false;
  RirTrackLifecycle lifecycle(config);

  lifecycle.Update(MakeCycle(1U), {MakeMeasurement(7U, 0.0f, 10.0f, false)});
  lifecycle.Update(MakeCycle(2U), {MakeMeasurement(7U, 10.0f, 10.0f, true)});
  lifecycle.Update(MakeCycle(3U), {MakeMeasurement(7U, 20.0f, 10.0f, true)});

  EXPECT_FALSE(lifecycle.IsImmEnabled());
  EXPECT_EQ(lifecycle.FindImmFilter(7U), nullptr);
  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].status, RirTrackStatus::kConfirmed);
}

/// @brief confirmed 命中激活 IMM：运行态按关联键建立，权重归一；
///        失配周期对 confirmed 航迹执行 IMM 仅预测（位置按速度推进）。
TEST(RirImmTrackingTest, ConfirmedMatchBuildsImmAndMissPredictsForward) {
  RirTrackLifecycle lifecycle(MakeImmConfig());

  lifecycle.Update(MakeCycle(1U), {MakeMeasurement(7U, 0.0f, 10.0f, false)});
  ASSERT_EQ(lifecycle.FindImmFilter(7U), nullptr);  // 新建航迹（未命中既有）不激活
  lifecycle.Update(MakeCycle(2U), {MakeMeasurement(7U, 10.0f, 10.0f, true)});
  ASSERT_NE(lifecycle.FindImmFilter(7U), nullptr);  // confirmed 命中激活 IMM

  const Eigen::VectorXf weights = lifecycle.FindImmFilter(7U)->GetModelWeights();
  ASSERT_EQ(weights.size(), 2);
  EXPECT_NEAR(weights.sum(), 1.0f, 1.0e-5f);

  lifecycle.Update(MakeCycle(3U), {});  // 失配：IMM 仅预测
  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_TRUE(snapshots[0].position.x() > 10.0f);  // 速度 10 m/s 推进一个周期
  EXPECT_LT(snapshots[0].position.x(), 25.0f);     // 组合预测不外推超过速度积分上界
  EXPECT_TRUE(std::isfinite(snapshots[0].speed));
}

/// @brief 高机动切换：持续强加速命中使高过程噪声模型权重占优，
///        且位置/速度连续（无发散/跳变）。
TEST(RirImmTrackingTest, ManeuverShiftsWeightToHighNoiseModelAndStaysContinuous) {
  RirTrackLifecycle lifecycle(MakeImmConfig());

  // 匀速段：建立 confirmed 航迹并激活 IMM。
  lifecycle.Update(MakeCycle(1U), {MakeMeasurement(7U, 0.0f, 10.0f, false)});
  lifecycle.Update(MakeCycle(2U), {MakeMeasurement(7U, 10.0f, 10.0f, true)});
  lifecycle.Update(MakeCycle(3U), {MakeMeasurement(7U, 20.0f, 10.0f, true)});

  // 机动段：每周期 +8 m/s（强加速度），量测位置按积分真值给出。
  float position = 20.0f;
  float velocity = 10.0f;
  for (std::uint32_t cycle = 4U; cycle <= 10U; ++cycle) {
    velocity += 8.0f;
    position += velocity;
    lifecycle.Update(MakeCycle(cycle),
                     {MakeMeasurement(7U, position, velocity, true)});
  }

  const tracking::RirImmFilter* imm = lifecycle.FindImmFilter(7U);
  ASSERT_NE(imm, nullptr);
  const Eigen::VectorXf weights = imm->GetModelWeights();
  // 模型序按噪声由低到高（{1.0, 10.0}）：高噪声模型应占优。
  EXPECT_GT(weights(1), weights(0));
  EXPECT_GT(weights(1), 0.5f);

  const RirTrackSnapshotList snapshots = lifecycle.BuildTrackSnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_TRUE(snapshots[0].position.allFinite());
  EXPECT_TRUE(snapshots[0].velocity.allFinite());
  // 连续性：机动段后滤波位置与量测真值的偏差在速度量级内（不发散）。
  EXPECT_LT(std::fabs(snapshots[0].position.x() - position), 50.0f);
}

/// @brief RirImmFilter 直接契约：缺省双模型 {1.0, 10.0}；转移矩阵行归一。
TEST(RirImmFilterTest, DefaultDualModelIsValid) {
  tracking::RirImmFilter filter;  // 缺省配置
  EXPECT_TRUE(filter.IsValid());
  EXPECT_EQ(filter.ModelCount(), 2U);

  filter.Process(Eigen::Vector3f(1.0f, 2.0f, 3.0f), 1.0f, RirMeasurementCovariance::Identity());
  const Eigen::VectorXf weights = filter.GetModelWeights();
  ASSERT_EQ(weights.size(), 2);
  EXPECT_NEAR(weights.sum(), 1.0f, 1.0e-5f);
  EXPECT_TRUE(filter.GetCombinedState().mean.allFinite());
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
