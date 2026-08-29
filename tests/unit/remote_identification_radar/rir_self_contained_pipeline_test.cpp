// Copyright 2026. All Rights Reserved.
//
// @file rir_self_contained_pipeline_test.cpp
// @brief 验证阶段 2-S 自持链路：检测 → 量测误差 → 关联/滤波/生命周期 → 内部航迹。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "RirCycleInputTestUtil.h"
#include "remote_identification_radar/runtime/RirController.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using runtime::RirController;
using session::RirCycleInput;
using session::RirOutputFrame;
using session::RirSceneTarget;

config::RirMissionConfig MakeMission(config::RirWorkMode work_mode) {
  config::RirMissionConfig mission;
  mission.work_mode = work_mode;
  return mission;
}

RirController MakeFallbackController() {
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  policy.lifecycle.confirm_hits = 1U;
  RirController controller;
  config::RirHardwareConfig hardware;
  hardware.rcs_physics.enable_physical_rcs = false;
  hardware.rcs_physics.physics_mix_ratio = 0.0f;
  // 主瓣覆盖门放宽：本文件聚焦链路级联与驻留预算摘要，不测波束覆盖门。
  hardware.antenna.nominal_az_beamwidth_deg = 160.0f;
  hardware.antenna.nominal_el_beamwidth_deg = 160.0f;
  controller.SetHardware(hardware);
  controller.UpdateRuntime(MakeMission(config::RirWorkMode::kIdentify), policy);
  return controller;
}

RirCycleInput MakeInput(std::uint32_t cycle, float range_m, float rcs_m2) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  SetDefaultTestPlatformEcef(&input);
  RirSceneTarget target;
  target.external_target_id = 7U;
  target.target_name = "target-a";
  target.position_x = range_m;
  target.position_z = 2000.0f;
  target.velocity_x = 100.0f;
  target.rcs = rcs_m2;
  target.range_m = range_m;
  input.scene_targets.push_back(target);
  return input;
}

/// @brief 6 dB 回退门控：目标进入关联/生命周期，输出内部航迹与驻留预算摘要。
TEST(RirSelfContainedPipelineTest, SnrFallbackBuildsInternalTrackAndDwellSummary) {
  RirController controller = MakeFallbackController();

  RirOutputFrame first;
  controller.RunCycle(MakeInput(1U, 5000.0f, 5.0f), &first, 1U);
  EXPECT_EQ(first.recognition_outputs.size(), 1U);
  EXPECT_EQ(first.recognition_outputs[0].association_key, 1U);
  EXPECT_TRUE(controller.HasLatestSummary());
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.scheduled_dwell_count, 1U);
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 1U);
  EXPECT_FLOAT_EQ(controller.GetLatestSummary().dwell_budget.dwell_consumed_sec, 0.05f);

  RirOutputFrame second;
  controller.RunCycle(MakeInput(2U, 5100.0f, 5.0f), &second, 2U);
  // 关联键保持稳定（自持身份），不会因外部供给重分配而新建。
  ASSERT_EQ(second.recognition_outputs.size(), 1U);
  EXPECT_EQ(second.recognition_outputs[0].association_key, 1U);
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 1U);
}

/// @brief 检测器门控：低于硬截断的远距小目标不进入内部航迹。
TEST(RirSelfContainedPipelineTest, DetectorGateRejectsUndetectableTarget) {
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kDetectorGate;
  policy.detection.random_seed = 42U;
  policy.lifecycle.confirm_hits = 1U;
  RirController controller;
  config::RirHardwareConfig hardware;
  hardware.rcs_physics.enable_physical_rcs = false;
  hardware.rcs_physics.physics_mix_ratio = 0.0f;
  // 主瓣覆盖门放宽：本文件聚焦链路级联与驻留预算摘要，不测波束覆盖门。
  hardware.antenna.nominal_az_beamwidth_deg = 160.0f;
  hardware.antenna.nominal_el_beamwidth_deg = 160.0f;
  controller.SetHardware(hardware);
  controller.UpdateRuntime(MakeMission(config::RirWorkMode::kIdentify), policy);

  RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, 300000.0f, 0.1f), &frame, 1U);
  EXPECT_TRUE(frame.recognition_outputs.empty());
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.scheduled_dwell_count, 1U);
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 0U);
  EXPECT_FLOAT_EQ(controller.GetLatestSummary().dwell_budget.dwell_consumed_sec, 0.0f);
}

/// @brief kStby 门控整链：不检测不建轨，但已有内部航迹仍回填识别结论。
TEST(RirSelfContainedPipelineTest, StandbyDoesNotAdvanceSelfContainedChain) {
  RirController controller = MakeFallbackController();
  RirOutputFrame first;
  controller.RunCycle(MakeInput(1U, 5000.0f, 5.0f), &first, 1U);
  ASSERT_EQ(first.recognition_outputs.size(), 1U);

  config::RirPolicyConfig stby_policy;
  stby_policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  stby_policy.lifecycle.confirm_hits = 1U;
  controller.UpdateRuntime(MakeMission(config::RirWorkMode::kStby), stby_policy);

  RirCycleInput standby_input = MakeInput(2U, 6000.0f, 5.0f);
  standby_input.scene_targets[0].external_target_id = 8U;  // 新目标也不应触发检测建轨。
  RirOutputFrame standby_frame;
  controller.RunCycle(standby_input, &standby_frame, 2U);
  ASSERT_EQ(standby_frame.recognition_outputs.size(), 1U);
  EXPECT_EQ(standby_frame.recognition_outputs[0].association_key, 1U);
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.scheduled_dwell_count, 0U);
}

/// @brief IMM policy 接线（N6）：enable_imm_lifecycle 经 public policy 抵达
///        生命周期双路径——confirmed 命中后链路保持稳定，键不变、逐周期产出结论。
/// @note 内部航迹不出 public 面（边界不变式）；运动学连续性由
///        rir_imm_tracking_test 在生命周期层锁定。
TEST(RirSelfContainedPipelineTest, ImmPolicyReachesLifecycleAndKeepsTrackStable) {
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  policy.lifecycle.confirm_hits = 1U;
  policy.lifecycle.enable_imm_lifecycle = true;  // N6：public policy IMM 开关
  policy.lifecycle.model_count_hint = 2U;
  RirController controller;
  config::RirHardwareConfig hardware;
  hardware.rcs_physics.enable_physical_rcs = false;
  hardware.rcs_physics.physics_mix_ratio = 0.0f;
  // 主瓣覆盖门放宽：本文件聚焦链路级联与驻留预算摘要，不测波束覆盖门。
  hardware.antenna.nominal_az_beamwidth_deg = 160.0f;
  hardware.antenna.nominal_el_beamwidth_deg = 160.0f;
  controller.SetHardware(hardware);
  controller.UpdateRuntime(MakeMission(config::RirWorkMode::kIdentify), policy);

  std::uint64_t key = 0U;
  for (std::uint32_t cycle = 1U; cycle <= 4U; ++cycle) {
    RirOutputFrame frame;
    // 位移与速度种子一致（100 m/s × dt 0.5 s = 50 m/周期），保证门内持续命中。
    controller.RunCycle(MakeInput(cycle, 5000.0f + 50.0f * static_cast<float>(cycle - 1U), 5.0f),
                        &frame, static_cast<std::uint64_t>(cycle));
    // 周期 3 起 confirmed 命中走 IMM 路径：链路不因 IMM 挂载而中断。
    ASSERT_EQ(frame.recognition_outputs.size(), 1U);
    if (key == 0U) {
      key = frame.recognition_outputs[0].association_key;
    } else {
      EXPECT_EQ(frame.recognition_outputs[0].association_key, key);
    }
    EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 1U);
  }
  EXPECT_NE(key, 0U);
}

/// @brief 环境效果开启（热带密林）：杂波按"相对热噪 dB"换算（与 AR 同口径），
///        6 dB 回退门下目标仍可检测建轨。修复前杂波被按绝对 dBW 读出 ~2 W，
///        SNR 崩塌约 136 dB，检测全灭——本用例锁死该回归。
TEST(RirSelfContainedPipelineTest, EnvironmentEffectsKeepTargetDetectable) {
  config::RirEnvironmentConfig environment;
  environment.enable_environment_effects = true;
  environment.vegetation_scatter_physics.cover_profile =
      config::RirVegetationCoverProfile::kTropicalDense;
  environment.vegetation_scatter_physics.enable_physical_model = true;
  environment.weather_attenuation_db = 0.0f;

  RirController controller = MakeFallbackController();
  controller.UpdateEnvironment(environment);

  RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, 5000.0f, 5.0f), &frame, 1U);
  ASSERT_EQ(frame.recognition_outputs.size(), 1U);
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 1U);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
