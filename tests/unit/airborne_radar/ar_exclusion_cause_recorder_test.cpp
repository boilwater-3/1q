/**
 * @file ar_exclusion_cause_recorder_test.cpp
 * @brief 排除原因跨周期差分记录器单元测试（规则 13b 差分观测）。
 *
 * 直接构造 ArCycleResult + ArTargetInputList 调用 Update()，精准覆盖
 * A1（稳定）/A2（进入）/A3（变化）/A4（退出）四条转换与非执行周期早退。
 */

#include "1q/airborne_radar/session/ArExclusionCauseRecorder.h"

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArExternalInputAdapter.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/foundation/validation_types.h"

namespace airborne_radar {
namespace session {
namespace {

using oneq::foundation::ValidationLocation;
using oneq::foundation::ValidationLocationKind;

// 构造 kInfo 排除诊断（规则 13b），定位到 scene 实体索引。
ArIssue MakeExclusionAtEntity(std::uint64_t target_index, const std::string& code,
                              ArIssueCause cause) {
  ArIssue issue;
  issue.severity = ArIssueSeverity::kInfo;
  issue.phase = ArIssuePhase::kExecution;
  issue.code = code;
  issue.message = "target_id=" + std::to_string(target_index);
  issue.cause = cause;
  issue.location.kind = ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = static_cast<std::size_t>(target_index);
  return issue;
}

// 构造 kCompleted 周期结果，携带给定排除诊断列表。
ArCycleResult MakeCompletedResult(std::uint32_t cycle_index, ArIssueList issues) {
  ArCycleResult result;
  result.input_cycle_index = cycle_index;
  result.status = ArCycleStatus::kCompleted;
  result.issues = std::move(issues);
  return result;
}

// 构造单个输入目标。
ArTargetInput MakeTarget(std::uint64_t target_id, const std::string& name) {
  ArTargetInput target;
  target.target_id = target_id;
  target.target_name = name;
  return target;
}

// 在事件列表中按 (target_id, kind) 查找事件。
const ArExclusionCauseEvent* FindEvent(const std::vector<ArExclusionCauseEvent>& events,
                                       std::uint64_t target_id,
                                       ArExclusionCauseEventKind kind) {
  for (const ArExclusionCauseEvent& event : events) {
    if (event.external_target_id == target_id && event.kind == kind) {
      return &event;
    }
  }
  return nullptr;
}

// A2：未被排除 → 被排除产生 kEntered，previous 为空/kNone，current 携带本周 (code,cause)。
TEST(ArExclusionCauseRecorderTest, EntryIntoExclusionProducesEnteredEvent) {
  ArExclusionCauseRecorder recorder;
  ArTargetInputList targets = {MakeTarget(77U, "alpha")};
  // 周期 1：目标未被排除。
  recorder.Update(targets, MakeCompletedResult(1U, {}));
  // 周期 2：目标被 RCS 主因排除。
  const std::vector<ArExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "ar.target_snr_below_threshold",
                                                              ArIssueCause::kRcsLimited)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArExclusionCauseEventKind::kEntered);
  EXPECT_EQ(events[0].external_target_id, 77U);
  EXPECT_EQ(events[0].world_cycle_index, 2U);
  EXPECT_TRUE(events[0].previous_code.empty());
  EXPECT_EQ(events[0].previous_cause, ArIssueCause::kNone);
  EXPECT_EQ(events[0].current_code, "ar.target_snr_below_threshold");
  EXPECT_EQ(events[0].current_cause, ArIssueCause::kRcsLimited);
}

// A3：被排除中 (code,cause) 对变化产生 kChanged（本需求核心）。
TEST(ArExclusionCauseRecorderTest, CauseChangeProducesChangedEvent) {
  ArExclusionCauseRecorder recorder;
  ArTargetInputList targets = {MakeTarget(77U, "alpha")};
  // 周期 1：RCS 主因排除。
  recorder.Update(targets, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                       0U, "ar.target_snr_below_threshold",
                                                       ArIssueCause::kRcsLimited)}));
  // 周期 2：主因变为波束偏轴。
  const std::vector<ArExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "ar.target_snr_below_threshold",
                                                              ArIssueCause::kBeamLimited)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArExclusionCauseEventKind::kChanged);
  EXPECT_EQ(events[0].previous_cause, ArIssueCause::kRcsLimited);
  EXPECT_EQ(events[0].current_cause, ArIssueCause::kBeamLimited);
  // code 不变（AR 单一排除 code），但仍产事件（cause 变化）。
  EXPECT_EQ(events[0].previous_code, events[0].current_code);
}

// A1：连续两周期相同 (code,cause) → 第二周期不产事件。
TEST(ArExclusionCauseRecorderTest, StableCauseProducesNoEvent) {
  ArExclusionCauseRecorder recorder;
  ArTargetInputList targets = {MakeTarget(77U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                       0U, "ar.target_snr_below_threshold",
                                                       ArIssueCause::kRcsLimited)}));
  const std::vector<ArExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "ar.target_snr_below_threshold",
                                                              ArIssueCause::kRcsLimited)}));
  EXPECT_TRUE(events.empty());
}

// A4：被排除 → 不再被排除产生 kExited，current 为空/kNone，previous 携带上周 (code,cause)。
TEST(ArExclusionCauseRecorderTest, ExitFromExclusionProducesExitedEvent) {
  ArExclusionCauseRecorder recorder;
  ArTargetInputList targets = {MakeTarget(77U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                       0U, "ar.target_snr_below_threshold",
                                                       ArIssueCause::kRcsLimited)}));
  // 周期 2：目标不再被排除（issues 为空）。
  const std::vector<ArExclusionCauseEvent> events =
      recorder.Update(targets, MakeCompletedResult(2U, {}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArExclusionCauseEventKind::kExited);
  EXPECT_EQ(events[0].previous_code, "ar.target_snr_below_threshold");
  EXPECT_EQ(events[0].previous_cause, ArIssueCause::kRcsLimited);
  EXPECT_TRUE(events[0].current_code.empty());
  EXPECT_EQ(events[0].current_cause, ArIssueCause::kNone);
}

// 非执行周期不产生事件、不推进内部状态。
TEST(ArExclusionCauseRecorderTest, NonExecutedCycleDoesNotAdvanceState) {
  ArExclusionCauseRecorder recorder;
  ArTargetInputList targets = {MakeTarget(77U, "alpha")};
  // 周期 1：RCS 排除（建立状态）。
  recorder.Update(targets, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                       0U, "ar.target_snr_below_threshold",
                                                       ArIssueCause::kRcsLimited)}));
  const std::size_t events_after_cycle1 = recorder.GetLastEvents().size();
  // 周期 2：非执行周期（拒绝）——不推进状态、不刷新缓存。
  ArCycleResult rejected;
  rejected.input_cycle_index = 2U;
  rejected.status = ArCycleStatus::kRejectedInvalidInput;
  const std::vector<ArExclusionCauseEvent> events = recorder.Update(targets, rejected);
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(recorder.GetLastEvents().size(), events_after_cycle1);
  // 周期 3：再次 RCS 排除（与周期 1 相同）——若非执行周期未污染状态，
  // 内部仍记 RCS，本周应为 A1（稳定，不产事件）。
  const std::vector<ArExclusionCauseEvent> events3 = recorder.Update(
      targets, MakeCompletedResult(3U, {MakeExclusionAtEntity(0U, "ar.target_snr_below_threshold",
                                                              ArIssueCause::kRcsLimited)}));
  EXPECT_TRUE(events3.empty());
}

// target_id 为 0 的目标无法关联，跳过差分记录。
TEST(ArExclusionCauseRecorderTest, ZeroTargetIdIsSkipped) {
  ArExclusionCauseRecorder recorder;
  ArTargetInputList targets = {MakeTarget(0U, "unset")};
  const std::vector<ArExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(1U, {MakeExclusionAtEntity(0U, "ar.target_snr_below_threshold",
                                                              ArIssueCause::kRcsLimited)}));
  EXPECT_TRUE(events.empty());
}

// 多目标独立差分：两目标各自转换，互不干扰。
TEST(ArExclusionCauseRecorderTest, MultipleTargetsTrackIndependently) {
  ArExclusionCauseRecorder recorder;
  ArTargetInputList targets = {MakeTarget(10U, "alpha"), MakeTarget(20U, "beta")};
  // 周期 1：两目标均被 RCS 排除（各自建立状态）。
  recorder.Update(targets, MakeCompletedResult(1U,
                                               {MakeExclusionAtEntity(0U, "ar.target_snr_below_threshold",
                                                                      ArIssueCause::kRcsLimited),
                                                MakeExclusionAtEntity(1U, "ar.target_snr_below_threshold",
                                                                      ArIssueCause::kRcsLimited)}));
  // 周期 2：目标 10 变为波束主因（A3），目标 20 退出排除（A4）。
  const std::vector<ArExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "ar.target_snr_below_threshold",
                                                              ArIssueCause::kBeamLimited)}));
  EXPECT_NE(FindEvent(events, 10U, ArExclusionCauseEventKind::kChanged), nullptr);
  EXPECT_NE(FindEvent(events, 20U, ArExclusionCauseEventKind::kExited), nullptr);
}

// Reset 清空内部状态：重置后原"持续排除"目标再次排除视为 A2 进入。
TEST(ArExclusionCauseRecorderTest, ResetClearsState) {
  ArExclusionCauseRecorder recorder;
  ArTargetInputList targets = {MakeTarget(77U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                       0U, "ar.target_snr_below_threshold",
                                                       ArIssueCause::kRcsLimited)}));
  recorder.Reset();
  // 重置后再次相同排除 → 视为 A2 进入（状态已清空）。
  const std::vector<ArExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "ar.target_snr_below_threshold",
                                                              ArIssueCause::kRcsLimited)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArExclusionCauseEventKind::kEntered);
  // Reset 后再次排除视作 A2，GetLastEvents 反映本周期的 1 条事件。
  EXPECT_EQ(recorder.GetLastEvents().size(), 1U);
}

// GetLastEvents 保留最近执行周期事件，未被后续非执行周期刷新。
TEST(ArExclusionCauseRecorderTest, GetLastEventsRetainsLastExecutedCycle) {
  ArExclusionCauseRecorder recorder;
  ArTargetInputList targets = {MakeTarget(77U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                       0U, "ar.target_snr_below_threshold",
                                                       ArIssueCause::kRcsLimited)}));
  const std::size_t last_size = recorder.GetLastEvents().size();
  ArCycleResult rejected;
  rejected.status = ArCycleStatus::kRejectedInvalidInput;
  recorder.Update(targets, rejected);
  // 非执行周期后 GetLastEvents 仍保留周期 1 的缓存。
  EXPECT_EQ(recorder.GetLastEvents().size(), last_size);
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
