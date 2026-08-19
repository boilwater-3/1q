// Copyright 2026. All Rights Reserved.
//
// @file rir_track_attribution_test.cpp
// @brief 验证航迹归属视图（库内键 ↔ 场景真值目标对照，结果层产品）。
//
// 覆盖：全部航迹快照产出归属（tentative/confirmed/lost 同循环）、键↔真值映射、
// 最小航迹诊断透出、按 association_key 排序、非执行周期空列表（五模块统一规则）。
// 归属不依赖特征库（航迹链独立于识别链）。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirCycleInputTestUtil.h"
#include "remote_identification_radar/runtime/RirController.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirSceneTarget;
using session::RirSession;

config::RirMissionConfig MakeIdentifyMission() {
  config::RirMissionConfig mission;
  mission.work_mode = config::RirWorkMode::kIdentify;
  return mission;
}

config::RirPolicyConfig MakeFallbackPolicy() {
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  policy.lifecycle.confirm_hits = 1U;
  return policy;
}

RirSceneTarget MakeTarget(std::uint64_t id, const char* name, float position_x) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = name;
  target.position_x = position_x;
  target.position_z = 2000.0f;
  target.rcs = 5.0f;
  target.range_m = std::sqrt(position_x * position_x + 2000.0f * 2000.0f);
  return target;
}

RirCycleInput MakeInput(std::uint32_t cycle, const std::vector<RirSceneTarget>& targets) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  SetDefaultTestPlatformEcef(&input);
  input.scene_targets = targets;
  return input;
}

/// @brief 全部航迹快照产出归属：键↔真值一一映射、携带最小诊断、按键升序。
TEST(RirTrackAttributionTest, AttributionMapsKeysToTruthTargets) {
  runtime::RirController controller;
  controller.SetHardware(config::RirHardwareConfig{});
  controller.UpdateRuntime(MakeIdentifyMission(), MakeFallbackPolicy());

  std::vector<RirSceneTarget> targets;
  targets.push_back(MakeTarget(7U, "truth-a", 5000.0f));
  targets.push_back(MakeTarget(8U, "truth-b", 8000.0f));

  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, targets), &frame, 1U);

  const std::vector<session::RirTrackAttributionRecord>& attributions =
      controller.LatestTrackAttributions();
  ASSERT_EQ(attributions.size(), 2U);
  // 出口②同循环同覆盖（全部快照）。
  ASSERT_EQ(frame.recognition_outputs.size(), 2U);

  // 按 association_key 升序（快照列表既定顺序）。
  EXPECT_LT(attributions[0].association_key, attributions[1].association_key);
  // 键↔真值映射：两个真值各出现一次，键与出口②一致。
  std::uint64_t mapped_ids = 0U;
  for (std::size_t i = 0U; i < attributions.size(); ++i) {
    const session::RirTrackAttributionRecord& attribution = attributions[i];
    EXPECT_EQ(attribution.association_key, frame.recognition_outputs[i].association_key);
    EXPECT_TRUE(attribution.external_target_id == 7U || attribution.external_target_id == 8U);
    mapped_ids |= attribution.external_target_id;
    EXPECT_FALSE(attribution.target_name.empty());
    // 最小航迹诊断：首周期单命中、滤波位置/速度有限非负。
    EXPECT_EQ(attribution.hit_count, 1U);
    EXPECT_TRUE(std::isfinite(attribution.position_enu_x_m));
    EXPECT_TRUE(std::isfinite(attribution.position_enu_y_m));
    EXPECT_TRUE(std::isfinite(attribution.position_enu_z_m));
    EXPECT_GE(attribution.speed_m_per_s, 0.0);
  }
  EXPECT_EQ(mapped_ids, 7U | 8U);
}

/// @brief 归属不依赖特征库：无数据库的周期同样携带归属（航迹链独立）。
TEST(RirTrackAttributionTest, AttributionPresentWithoutDatabase) {
  runtime::RirController controller;
  controller.SetHardware(config::RirHardwareConfig{});
  controller.UpdateRuntime(MakeIdentifyMission(), MakeFallbackPolicy());

  std::vector<RirSceneTarget> targets;
  targets.push_back(MakeTarget(7U, "truth-a", 5000.0f));
  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, targets), &frame, 1U);

  EXPECT_EQ(controller.LatestTrackAttributions().size(), 1U);
  EXPECT_EQ(controller.LatestTrackAttributions()[0].external_target_id, 7U);
  EXPECT_EQ(controller.LatestTrackAttributions()[0].target_name, "truth-a");
}

/// @brief 会话级回填：kCompleted 周期归属入结果层，与输出帧键一致。
TEST(RirTrackAttributionTest, SessionBackfillsAttributionOnCompletedCycle) {
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  RirSession session = RirSession::Create(config);

  std::vector<RirSceneTarget> targets;
  targets.push_back(MakeTarget(7U, "truth-a", 5000.0f));
  const RirCycleResult completed = session.StepWithResult(MakeInput(1U, targets));

  ASSERT_EQ(completed.status, RirCycleStatus::kCompleted);
  ASSERT_EQ(completed.track_attributions.size(), 1U);
  ASSERT_EQ(completed.output_frame.recognition_outputs.size(), 1U);
  EXPECT_EQ(completed.track_attributions[0].association_key,
            completed.output_frame.recognition_outputs[0].association_key);
  EXPECT_EQ(completed.track_attributions[0].external_target_id, 7U);
}

/// @brief 非执行周期空列表：校验拒绝与关机周期均不复用上一周期归属。
TEST(RirTrackAttributionTest, NonExecutedCyclesReturnEmptyAttribution) {
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  RirSession session = RirSession::Create(config);

  std::vector<RirSceneTarget> targets;
  targets.push_back(MakeTarget(7U, "truth-a", 5000.0f));
  const RirCycleResult completed = session.StepWithResult(MakeInput(1U, targets));
  ASSERT_EQ(completed.status, RirCycleStatus::kCompleted);
  ASSERT_FALSE(completed.track_attributions.empty());

  // 校验拒绝：空列表，不推进状态。
  RirCycleInput rejected = MakeInput(2U, targets);
  rejected.dt_sec = 0.0;
  const RirCycleResult rejected_result = session.StepWithResult(rejected);
  EXPECT_EQ(rejected_result.status, RirCycleStatus::kRejectedInvalidInput);
  EXPECT_TRUE(rejected_result.track_attributions.empty());

  // 关机：补丁在下一个成功周期边界提交（该周期仍 kCompleted），其后周期
  // 为非执行周期 → 空列表。
  config::RirRuntimeConfigPatch power_off;
  power_off.has_sensor_enabled = true;
  power_off.sensor_enabled = false;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(power_off));
  const RirCycleResult commit_cycle = session.StepWithResult(MakeInput(3U, targets));
  EXPECT_EQ(commit_cycle.status, RirCycleStatus::kCompleted);
  const RirCycleResult powered_off = session.StepWithResult(MakeInput(4U, targets));
  EXPECT_EQ(powered_off.status, RirCycleStatus::kPoweredOff);
  EXPECT_TRUE(powered_off.track_attributions.empty());
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
