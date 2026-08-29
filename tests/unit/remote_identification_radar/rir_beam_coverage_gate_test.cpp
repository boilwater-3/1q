// Copyright 2026. All Rights Reserved.
//
// @file rir_beam_coverage_gate_test.cpp
// @brief 验证主瓣覆盖门（2026-08-29 架构还债：方向图恒开后"探测⟺波束照到"）。
//
// 覆盖：
//   1) 驻留对准目标 → 门放行、照常检出建航迹；
//   2) 扇区内但离轴出主瓣（半功率宽）→ 不入候选集 + kTargetOutsideBeamCoverage
//      排除诊断，被驻留照到的目标不受影响；
//   3) 方位差经 ±180° 环绕归一：波束指 179°、目标在 −179.5°（差 1.5°）在门内。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirCycleInputTestUtil.h"
#include "remote_identification_radar/runtime/RirController.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirCycleResult;
using session::RirSceneTarget;
using session::RirSession;

config::RirPolicyConfig MakeFallbackPolicy() {
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  policy.lifecycle.confirm_hits = 1U;
  return policy;
}

config::RirMissionConfig MakeIdentifyMission() {
  config::RirMissionConfig mission;
  mission.work_mode = config::RirWorkMode::kIdentify;
  return mission;
}

/** @brief 东向 10 km / 高 2 km 目标：视线角 az=0°、el≈11.31°。 */
RirSceneTarget MakeTarget(std::uint64_t id, float look_az_deg) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = "coverage-target";
  const float range_hypot_m = 10198.0f;
  const float az_rad = look_az_deg * 3.14159265358979f / 180.0f;
  target.position_x = range_hypot_m * std::cos(az_rad);
  target.position_y = range_hypot_m * std::sin(az_rad);
  target.position_z = 2000.0f;
  target.rcs = 0.5f;
  target.range_m = std::sqrt(range_hypot_m * range_hypot_m + 2000.0f * 2000.0f);
  return target;
}

config::RirAzimuthElevationDeg ExpectedTargetLookAngles(const RirSceneTarget& target) {
  config::RirAzimuthElevationDeg look;
  look.az_deg = std::atan2(target.position_y, target.position_x) * 180.0f / 3.14159265358979f;
  const float range_hypot = std::sqrt(target.position_x * target.position_x +
                                      target.position_y * target.position_y);
  look.el_deg = std::atan2(target.position_z, range_hypot) * 180.0f / 3.14159265358979f;
  return look;
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

config::RirRuntimeConfigPatch MakeDesignationPatch(std::uint64_t id,
                                                   std::uint32_t duration_cycles) {
  config::RirRuntimeConfigPatch patch;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = id;
  patch.has_designation_duration_cycles = true;
  patch.designation_duration_cycles = duration_cycles;
  return patch;
}

bool HasAttributionFor(const RirCycleResult& result, std::uint64_t target_id) {
  for (const auto& attribution : result.track_attributions) {
    if (attribution.external_target_id == target_id) {
      return true;
    }
  }
  return false;
}

const session::RirIssue* FindIssue(const RirCycleResult& result, const char* code,
                                   std::uint64_t target_id) {
  const std::string id_text = "target_id=" + std::to_string(target_id);
  for (const session::RirIssue& issue : result.issues) {
    if (issue.code == code && issue.message.find(id_text) != std::string::npos) {
      return &issue;
    }
  }
  return nullptr;
}

/// @brief 驻留对准目标（离轴 0）：主瓣门放行，照常准入建航迹。
TEST(RirBeamCoverageGateTest, DwellOnTarget_Admitted) {
  const RirSceneTarget target = MakeTarget(9301U, 0.0f);
  runtime::RirController controller;
  controller.SetHardware(config::RirHardwareConfig{});
  controller.UpdateRuntime(MakeIdentifyMission(), MakeFallbackPolicy());

  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, {target}), &frame, 1U, ExpectedTargetLookAngles(target));
  EXPECT_FALSE(frame.recognition_outputs.empty());
  EXPECT_EQ(controller.LatestTrackAttributions().size(), 1U);
}

/// @brief 扇区内但离轴出主瓣：不入候选集 + kTargetOutsideBeamCoverage，照到的目标不受影响。
TEST(RirBeamCoverageGateTest, OffBeamTarget_ExcludedWithCoverageCode) {
  const RirSceneTarget designated = MakeTarget(9302U, 0.0f);   // 驻留对准该目标。
  const RirSceneTarget off_beam = MakeTarget(9303U, 10.0f);    // 方位离轴 10° ≫ 半功率 2°。
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  RirSession session = RirSession::Create(config);
  ASSERT_TRUE(
      session.TryApplyRuntimeConfig(MakeDesignationPatch(designated.external_target_id, 3U)));

  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    const RirCycleResult result =
        session.StepWithResult(MakeInput(cycle, {designated, off_beam}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    EXPECT_TRUE(HasAttributionFor(result, designated.external_target_id))
        << "cycle=" << cycle;
    EXPECT_FALSE(HasAttributionFor(result, off_beam.external_target_id))
        << "离轴目标不应入检测候选集 cycle=" << cycle;
    const session::RirIssue* issue = FindIssue(result, session::codes::kTargetOutsideBeamCoverage,
                                               off_beam.external_target_id);
    ASSERT_NE(issue, nullptr) << "离轴目标应携带主瓣覆盖排除诊断 cycle=" << cycle;
    EXPECT_TRUE(std::string(issue->message).find("outside main lobe") != std::string::npos);
  }
}

/// @brief 方位差环绕归一：波束 179°、目标 −179.5°（差 1.5°）在 4° 波束门内。
TEST(RirBeamCoverageGateTest, AzimuthDeltaNormalizedNearWrapBoundary) {
  RirSceneTarget target = MakeTarget(9304U, 0.0f);
  // 平移方位：目标视线角 −179.5°（东向参考的等距旋转，保持仰角不变）。
  const float range_hypot_m = 10198.0f;
  const float az_rad = -179.5f * 3.14159265358979f / 180.0f;
  target.position_x = range_hypot_m * std::cos(az_rad);
  target.position_y = range_hypot_m * std::sin(az_rad);

  runtime::RirController controller;
  controller.SetHardware(config::RirHardwareConfig{});
  controller.UpdateRuntime(MakeIdentifyMission(), MakeFallbackPolicy());

  config::RirAzimuthElevationDeg dwell{179.0f, ExpectedTargetLookAngles(target).el_deg};
  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, {target}), &frame, 1U, dwell);
  EXPECT_FALSE(frame.recognition_outputs.empty())
      << "±180° 缝合处 1.5° 离轴应视作门内而非 358.5° 深度离轴";
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
