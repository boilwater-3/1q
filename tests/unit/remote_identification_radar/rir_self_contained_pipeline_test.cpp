// Copyright 2026. All Rights Reserved.
//
// @file rir_self_contained_pipeline_test.cpp
// @brief 验证阶段 2-S 自持链路：检测 → 量测误差 → 关联/滤波/生命周期 → 内部航迹。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
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
  // 波束 20°/10°：单目标驻留指向下覆盖门可过（本文件聚焦链路级联与驻留预算摘要，
  // 不测覆盖门本身）；目标仰角（≈21.8°）高于半俯仰波束宽（5°），主瓣离地零杂波，
  // 地杂波物理由 rir_surface_clutter_model_test 专项锁定。
  hardware.antenna.nominal_az_beamwidth_deg = 20.0f;
  hardware.antenna.nominal_el_beamwidth_deg = 10.0f;
  controller.SetHardware(hardware);
  controller.UpdateRuntime(MakeMission(config::RirWorkMode::kIdentify), policy);
  return controller;
}

/// @brief 驻留中心对准测试目标方向（(5000,0,2000) → el≈21.8°）：窄波束需主瓣照到；
/// 目标仰角高于半俯仰波束宽（5°）→ 主瓣离地零杂波，链路断言不被地杂波污染。
config::RirAzimuthElevationDeg TestDwellCenter() {
  return config::RirAzimuthElevationDeg{0.0f, 21.8f};
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
  input.scene_targets.push_back(target);
  return input;
}

/// @brief 6 dB 回退门控：目标进入关联/生命周期，输出内部航迹与驻留预算摘要。
TEST(RirSelfContainedPipelineTest, SnrFallbackBuildsInternalTrackAndDwellSummary) {
  RirController controller = MakeFallbackController();

  RirOutputFrame first;
  controller.RunCycle(MakeInput(1U, 5000.0f, 5.0f), &first, 1U, TestDwellCenter());
  EXPECT_EQ(first.recognition_outputs.size(), 1U);
  EXPECT_EQ(first.recognition_outputs[0].association_key, 1U);
  EXPECT_TRUE(controller.HasLatestSummary());
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.scheduled_dwell_count, 1U);
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 1U);
  EXPECT_FLOAT_EQ(controller.GetLatestSummary().dwell_budget.dwell_consumed_sec, 0.05f);

  RirOutputFrame second;
  controller.RunCycle(MakeInput(2U, 5100.0f, 5.0f), &second, 2U, TestDwellCenter());
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
  // 波束 20°/10°：单目标驻留指向下覆盖门可过（本文件聚焦链路级联与驻留预算摘要，
  // 不测覆盖门本身）；目标仰角（≈21.8°）高于半俯仰波束宽（5°），主瓣离地零杂波，
  // 地杂波物理由 rir_surface_clutter_model_test 专项锁定。
  hardware.antenna.nominal_az_beamwidth_deg = 20.0f;
  hardware.antenna.nominal_el_beamwidth_deg = 10.0f;
  controller.SetHardware(hardware);
  controller.UpdateRuntime(MakeMission(config::RirWorkMode::kIdentify), policy);

  RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, 300000.0f, 0.1f), &frame, 1U, TestDwellCenter());
  EXPECT_TRUE(frame.recognition_outputs.empty());
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.scheduled_dwell_count, 1U);
  // TAS 预算口径（2026-08-29）：executed=实际执行的驻留数——波束照常驻留，
  // 目标未过检测门只影响量测产出，不影响驻留计时。
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 1U);
  EXPECT_FLOAT_EQ(controller.GetLatestSummary().dwell_budget.dwell_consumed_sec, 0.05f);
}

/// @brief kStby 门控整链：不检测不建轨，但已有内部航迹仍回填识别结论。
TEST(RirSelfContainedPipelineTest, StandbyDoesNotAdvanceSelfContainedChain) {
  RirController controller = MakeFallbackController();
  RirOutputFrame first;
  controller.RunCycle(MakeInput(1U, 5000.0f, 5.0f), &first, 1U, TestDwellCenter());
  ASSERT_EQ(first.recognition_outputs.size(), 1U);

  config::RirPolicyConfig stby_policy;
  stby_policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  stby_policy.lifecycle.confirm_hits = 1U;
  controller.UpdateRuntime(MakeMission(config::RirWorkMode::kStby), stby_policy);

  RirCycleInput standby_input = MakeInput(2U, 6000.0f, 5.0f);
  standby_input.scene_targets[0].external_target_id = 8U;  // 新目标也不应触发检测建轨。
  RirOutputFrame standby_frame;
  controller.RunCycle(standby_input, &standby_frame, 2U, TestDwellCenter());
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
  // 波束 20°/10°：单目标驻留指向下覆盖门可过（本文件聚焦链路级联与驻留预算摘要，
  // 不测覆盖门本身）；目标仰角（≈21.8°）高于半俯仰波束宽（5°），主瓣离地零杂波，
  // 地杂波物理由 rir_surface_clutter_model_test 专项锁定。
  hardware.antenna.nominal_az_beamwidth_deg = 20.0f;
  hardware.antenna.nominal_el_beamwidth_deg = 10.0f;
  controller.SetHardware(hardware);
  controller.UpdateRuntime(MakeMission(config::RirWorkMode::kIdentify), policy);

  std::uint64_t key = 0U;
  for (std::uint32_t cycle = 1U; cycle <= 4U; ++cycle) {
    RirOutputFrame frame;
    // 位移与速度种子一致（100 m/s × dt 0.5 s = 50 m/周期），保证门内持续命中。
    controller.RunCycle(MakeInput(cycle, 5000.0f + 50.0f * static_cast<float>(cycle - 1U), 5.0f),
                        &frame, static_cast<std::uint64_t>(cycle), TestDwellCenter());
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

/// @brief 环境效果开启（热带密林）：植被恒定传播损耗生效，6 dB 回退门下目标
///        仍可检测建轨。历史回归：杂波曾被按绝对 dBW 读出 ~2 W，SNR 崩塌约
///        136 dB，检测全灭——本用例锁死"环境开启不摧毁检测链"。目标仰角
///        （≈21.8°）高于半俯仰波束宽，逐目标地杂波为主瓣离地零输出。
TEST(RirSelfContainedPipelineTest, EnvironmentEffectsKeepTargetDetectable) {
  config::RirEnvironmentConfig environment;
  environment.vegetation_cover_profile = config::RirVegetationCoverProfile::kTropicalDense;
  environment.weather_attenuation_db = 0.0f;

  RirController controller = MakeFallbackController();
  controller.UpdateEnvironment(environment);

  RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, 5000.0f, 5.0f), &frame, 1U, TestDwellCenter());
  ASSERT_EQ(frame.recognition_outputs.size(), 1U);
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 1U);
}

/// @brief 接收前端饱和周期致盲（核查 5.2）：接收线性上限压到极低（1e-15 W）→
///        自发射同平台耦合入射必越限 → 本周期全部目标不产生检测，逐目标落
///        kTargetReceiverFrontEndSaturated 排除诊断（饱和门先于 SNR 检测门，
///        旧口径仅 WARN 后照常判决）。
TEST(RirSelfContainedPipelineTest, ReceiverSaturationBlindsCycleDetections) {
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  policy.lifecycle.confirm_hits = 1U;
  RirController controller;
  config::RirHardwareConfig hardware;
  hardware.rcs_physics.enable_physical_rcs = false;
  hardware.rcs_physics.physics_mix_ratio = 0.0f;
  hardware.antenna.nominal_az_beamwidth_deg = 20.0f;
  hardware.antenna.nominal_el_beamwidth_deg = 10.0f;
  // 线性接收上限压到 1e-15 W：入射聚合功率（自发射同平台 120 dB 隔离后仍有
  // 1e-11 W 量级）必越限 → receiver_saturated=true。
  hardware.receiver.maximum_linear_input_power_w = 1.0e-15f;
  controller.SetHardware(hardware);
  controller.UpdateRuntime(MakeMission(config::RirWorkMode::kIdentify), policy);

  RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, 5000.0f, 5.0f), &frame, 1U, TestDwellCenter());

  // 饱和周期致盲：无航迹、无归属；检测链物理分项照常驻留（预算语义不变）。
  EXPECT_TRUE(frame.recognition_outputs.empty());
  EXPECT_TRUE(controller.LatestTrackAttributions().empty());
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 1U);

  // 规则 13b：每目标每周期至多一条、链上第一门优先——本周期唯一排除诊断为饱和门。
  const session::RirIssueList& issues = controller.LatestExecutionIssues();
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues[0].code, session::codes::kTargetReceiverFrontEndSaturated);
  EXPECT_EQ(issues[0].severity, session::RirIssueSeverity::kInfo);
  EXPECT_EQ(issues[0].cause, session::RirIssueCause::kNone);
  EXPECT_EQ(issues[0].location.entity_index, 0U);
}

/// @brief 默认线性上限（1e-3 W）不越限：检测链不受饱和门影响（回归——同输入
///        在默认配置下照常建轨，且不产生饱和排除诊断）。
TEST(RirSelfContainedPipelineTest, DefaultLinearLimitKeepsDetectionLink) {
  RirController controller = MakeFallbackController();

  RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, 5000.0f, 5.0f), &frame, 1U, TestDwellCenter());
  ASSERT_EQ(frame.recognition_outputs.size(), 1U);
  for (const session::RirIssue& issue : controller.LatestExecutionIssues()) {
    EXPECT_NE(issue.code, session::codes::kTargetReceiverFrontEndSaturated);
  }
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
