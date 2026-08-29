// Copyright 2026. All Rights Reserved.
//
// @file rir_track_dwell_scheduler_test.cpp
// @brief 验证 TAS 最小跟踪驻留调度（2026-08-29：确认航迹专用驻留 + 搜索填充）。
//
// 覆盖：
//   1) CollectTrackDwellRequests：确认航迹产出 kTrack 请求，指向 = 库内航迹预测；
//   2) 跟踪驻留豁免扫描子窗：目标走出子窗后仍被照到（无子窗排除诊断）；
//   3) 周期驻留预算封顶（floor(dt/dwell)）且搜索波位按实际搜索驻留数回填；
//   4) 同目标同周期被多个驻留照到时首驻留胜出（量测不重复）。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
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

constexpr float kDeg2RadF = 3.14159265358979f / 180.0f;

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

/** @brief 放宽主瓣覆盖门的测试硬件（本文件不测覆盖门本身）。 */
config::RirHardwareConfig MakeWideBeamHardware() {
  config::RirHardwareConfig hardware;
  hardware.antenna.nominal_az_beamwidth_deg = 160.0f;
  hardware.antenna.nominal_el_beamwidth_deg = 160.0f;
  return hardware;
}

/** @brief 东向 10 km / 指定高度目标（视线角 az=0°）。 */
RirSceneTarget MakeTarget(std::uint64_t id, float up_m) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = "tas-target";
  target.position_x = 10000.0f;
  target.position_z = up_m;
  target.rcs = 0.5f;
  target.range_m = std::sqrt(10000.0f * 10000.0f + up_m * up_m);
  return target;
}

config::RirAzimuthElevationDeg ExpectedLook(const RirSceneTarget& target) {
  config::RirAzimuthElevationDeg look;
  const float hypot = std::sqrt(target.position_x * target.position_x +
                                target.position_y * target.position_y);
  look.az_deg = std::atan2(target.position_y, target.position_x) / kDeg2RadF;
  look.el_deg = std::atan2(target.position_z, hypot) / kDeg2RadF;
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

/// @brief 确认航迹产出 kTrack 请求，指向 = 库内航迹末量测位置（静态目标无外推）。
TEST(RirTrackDwellSchedulerTest, CollectRequestsFromConfirmedTrack) {
  const RirSceneTarget target = MakeTarget(9401U, 2000.0f);
  runtime::RirController controller;
  controller.SetHardware(MakeWideBeamHardware());
  controller.UpdateRuntime(MakeIdentifyMission(), MakeFallbackPolicy());

  // 周期 1：驻留对准目标，确认航迹建立（confirm_hits=1）。
  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, {target}), &frame, 1U, ExpectedLook(target));
  ASSERT_EQ(frame.recognition_outputs.size(), 1U);

  const std::vector<runtime::RirDwellPlan> requests =
      controller.CollectTrackDwellRequests(4U, 0.5f);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().kind, runtime::RirDwellKind::kTrack);
  EXPECT_EQ(requests.front().external_target_id, target.external_target_id);
  // 静态目标：预测指向 = 末量测位置视线角。
  const config::RirAzimuthElevationDeg look = ExpectedLook(target);
  EXPECT_NEAR(requests.front().pointing_deg.az_deg, look.az_deg, 0.05f);
  EXPECT_NEAR(requests.front().pointing_deg.el_deg, look.el_deg, 0.05f);

  // 上限裁剪：max_count=0 无请求。
  EXPECT_TRUE(controller.CollectTrackDwellRequests(0U, 0.5f).empty());
}

/// @brief 跟踪驻留豁免扫描子窗：确认航迹目标走出子窗后仍被照到（无子窗排除诊断）。
TEST(RirTrackDwellSchedulerTest, TrackDwellExemptFromScanSubWindow) {
  RirSceneTarget target = MakeTarget(9402U, 523.0f);  // 周期 1：el 约 3°（子窗内）。
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  config.hardware = MakeWideBeamHardware();
  // 步长与波束宽度解耦（否则宽波束让波位表退化成角点）。
  config.mission.scan.step_scale = 0.02f;
  // 子窗压到低仰角带：周期 2 的目标（el 约 11.3°）出子窗、在硬件体积内。
  config.mission.scan_window_deg.el_min_deg = 2.0f;
  config.mission.scan_window_deg.el_max_deg = 4.0f;
  RirSession session = RirSession::Create(config);

  const RirCycleResult first = session.StepWithResult(MakeInput(1U, {target}));
  ASSERT_EQ(first.status, session::RirCycleStatus::kCompleted);
  ASSERT_FALSE(first.track_attributions.empty()) << "子窗内目标应先被检出建轨";

  target.position_z = 2000.0f;  // 周期 2：目标升到 el 约 11.3°（出子窗）。
  target.range_m = std::sqrt(10000.0f * 10000.0f + 2000.0f * 2000.0f);
  const RirCycleResult second = session.StepWithResult(MakeInput(2U, {target}));
  ASSERT_EQ(second.status, session::RirCycleStatus::kCompleted);
  // 跟踪驻留照到出窗目标：既无子窗排除诊断，也无主瓣覆盖排除诊断。
  EXPECT_EQ(FindIssue(second, session::codes::kTargetOutsideSearchVolume,
                      target.external_target_id),
            nullptr)
      << "确认航迹目标应豁免扫描子窗（TAS 跟踪连续性）";
  EXPECT_EQ(FindIssue(second, session::codes::kTargetOutsideBeamCoverage,
                      target.external_target_id),
            nullptr)
      << "跟踪驻留应照到出窗目标";
  EXPECT_FALSE(second.track_attributions.empty()) << "航迹应被跟踪驻留保持";
}

/// @brief 周期驻留预算封顶：floor(dt/dwell) 条（跟踪 + 搜索回填），搜索游标按
/// 实际搜索驻留数推进。
TEST(RirTrackDwellSchedulerTest, DwellBudgetCappedAndSearchBackfills) {
  std::vector<RirSceneTarget> targets;
  targets.push_back(MakeTarget(9403U, 523.0f));
  targets.push_back(MakeTarget(9404U, 875.0f));
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  config.hardware = MakeWideBeamHardware();
  config.mission.scan.step_scale = 0.02f;
  // TAS：dwell=0.25s 使每周期驻留数 = floor(0.5/0.25) = 2（除法精确，好断言）。
  config.mission.recognition_dwell_sec = 0.25f;
  RirSession session = RirSession::Create(config);

  // 周期 1：整周期搜索（无航迹无指定）：2 个搜索驻留。
  const RirCycleResult first = session.StepWithResult(MakeInput(1U, targets));
  ASSERT_EQ(first.status, session::RirCycleStatus::kCompleted);
  ASSERT_TRUE(first.has_recognition_summary);
  EXPECT_EQ(first.recognition_summary.dwell_budget.scheduled_dwell_count, 2U);
  EXPECT_EQ(first.recognition_summary.dwell_budget.executed_dwell_count, 2U);

  // 周期 2：两条确认航迹 → 预算封顶 + 保底 1 个搜索槽 → 1 跟踪 + 1 搜索 = 2。
  const RirCycleResult second = session.StepWithResult(MakeInput(2U, targets));
  ASSERT_EQ(second.status, session::RirCycleStatus::kCompleted);
  ASSERT_TRUE(second.has_recognition_summary);
  EXPECT_EQ(second.recognition_summary.dwell_budget.scheduled_dwell_count, 2U);
  EXPECT_EQ(second.recognition_summary.dwell_budget.executed_dwell_count, 2U);
  EXPECT_FLOAT_EQ(second.recognition_summary.dwell_budget.dwell_consumed_sec, 0.5f);
}

/// @brief 同目标同周期被多个驻留照到：量测不重复（首驻留胜出），驻留预算照计。
TEST(RirTrackDwellSchedulerTest, DuplicateDwellMeasuresTargetOnce) {
  const RirSceneTarget target = MakeTarget(9405U, 2000.0f);
  runtime::RirController controller;
  controller.SetHardware(MakeWideBeamHardware());
  controller.UpdateRuntime(MakeIdentifyMission(), MakeFallbackPolicy());

  const config::RirAzimuthElevationDeg look = ExpectedLook(target);
  std::vector<runtime::RirDwellPlan> plan(2U);
  plan.front().pointing_deg = look;
  plan.front().kind = runtime::RirDwellKind::kSearch;
  plan[1].pointing_deg = look;
  plan[1].kind = runtime::RirDwellKind::kTrack;
  plan[1].external_target_id = target.external_target_id;

  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, {target}), &frame, 1U, plan);
  EXPECT_EQ(frame.recognition_outputs.size(), 1U)
      << "同一目标两个驻留只应产出一条航迹结论";
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.scheduled_dwell_count, 2U);
  EXPECT_EQ(controller.GetLatestSummary().dwell_budget.executed_dwell_count, 2U);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
