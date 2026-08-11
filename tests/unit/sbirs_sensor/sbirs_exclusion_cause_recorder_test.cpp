/**
 * @file sbirs_exclusion_cause_recorder_test.cpp
 * @brief SBIRS 排除原因跨周期差分记录器单元测试（规则 13b 差分观测）。
 *
 * 直接构造 SbirsCycleInput + SbirsCycleResult 调用 Update()，覆盖
 * A1/A2/A3/A4 四条转换、非执行周期早退、多目标独立差分，以及 SBIRS 特有语义——
 * 遮挡↔距离带切换（同为 kNone、code 不同）的 A3 变化（验证 code+cause 组合键的必要性）。
 */

#include "1q/sbirs_sensor/session/SbirsExclusionCauseRecorder.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "1q/foundation/validation_types.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace session {
namespace {

using oneq::foundation::ValidationLocationKind;

// 构造 kInfo 排除诊断（规则 13b），定位到 scene 实体索引。
SbirsIssue MakeExclusionAtEntity(std::uint64_t entity_index, const std::string& code,
                                 SbirsIssueCause cause) {
  SbirsIssue issue;
  issue.severity = SbirsIssueSeverity::kInfo;
  issue.phase = SbirsIssuePhase::kExecution;
  issue.code = code;
  issue.message = "target_id=" + std::to_string(entity_index);
  issue.cause = cause;
  issue.location.kind = ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = static_cast<std::size_t>(entity_index);
  return issue;
}

// 构造 kCompleted 周期结果，携带给定排除诊断列表。
SbirsCycleResult MakeCompletedResult(std::uint32_t cycle_index, SbirsIssueList issues) {
  SbirsCycleResult result;
  result.input_cycle_index = cycle_index;
  result.status = SbirsCycleStatus::kCompleted;
  result.issues = std::move(issues);
  return result;
}

// 构造单目标输入。
SbirsCycleInput MakeInputWithTarget(std::uint64_t target_id, const std::string& name) {
  SbirsCycleInput input;
  SbirsSceneTarget target;
  target.target_id = target_id;
  target.target_name = name;
  input.scene.push_back(target);
  return input;
}

// 在事件列表中按 (target_id, kind) 查找事件。
const SbirsExclusionCauseEvent* FindEvent(const std::vector<SbirsExclusionCauseEvent>& events,
                                          std::uint64_t target_id,
                                          SbirsExclusionCauseEventKind kind) {
  for (const SbirsExclusionCauseEvent& event : events) {
    if (event.target_id == target_id && event.kind == kind) {
      return &event;
    }
  }
  return nullptr;
}

// SBIRS 特有语义：遮挡↔距离带切换（同为 kNone、code 不同）产生 A3 变化事件。
// 这验证了 (code,cause) 组合键对纯 cause 键盲区的修复。
TEST(SbirsExclusionCauseRecorderTest, OccultedToRangeGateSwitchProducesChangedEvent) {
  SbirsExclusionCauseRecorder recorder;
  SbirsCycleInput input = MakeInputWithTarget(1U, "alpha");
  // 周期 1：遮挡排除（具体门，cause=kNone）。
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "sbirs.target_occulted",
                                                     SbirsIssueCause::kNone)}));
  // 周期 2：切换到距离带排除（同为 kNone、code 不同）。
  const std::vector<SbirsExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "sbirs.target_out_of_range",
                                                            SbirsIssueCause::kNone)}));
  // 纯 cause 键会把两周都判为 kNone（A1 稳定，无事件）；
  // (code,cause) 组合键正确捕获 code 变化 → A3。
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, SbirsExclusionCauseEventKind::kChanged);
  EXPECT_EQ(events[0].previous_code, "sbirs.target_occulted");
  EXPECT_EQ(events[0].previous_cause, SbirsIssueCause::kNone);
  EXPECT_EQ(events[0].current_code, "sbirs.target_out_of_range");
  EXPECT_EQ(events[0].current_cause, SbirsIssueCause::kNone);
}

// A2：未被排除 → 被排除（视场方位越界）。
TEST(SbirsExclusionCauseRecorderTest, EntryIntoExclusionProducesEnteredEvent) {
  SbirsExclusionCauseRecorder recorder;
  SbirsCycleInput input = MakeInputWithTarget(1U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {}));
  const std::vector<SbirsExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "sbirs.target_out_of_wfov",
                                                            SbirsIssueCause::kAzOutside)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, SbirsExclusionCauseEventKind::kEntered);
  EXPECT_EQ(events[0].target_id, 1U);
  EXPECT_EQ(events[0].cycle_index, 2U);
  EXPECT_TRUE(events[0].previous_code.empty());
  EXPECT_EQ(events[0].current_code, "sbirs.target_out_of_wfov");
  EXPECT_EQ(events[0].current_cause, SbirsIssueCause::kAzOutside);
}

// A3：SNR 门主因变化（距离主导 → 大气衰减主导），同一 code 下 cause 变化。
TEST(SbirsExclusionCauseRecorderTest, SnrCauseChangeProducesChangedEvent) {
  SbirsExclusionCauseRecorder recorder;
  SbirsCycleInput input = MakeInputWithTarget(1U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "sbirs.target_snr_below_threshold",
                                                     SbirsIssueCause::kDistanceLimited)}));
  const std::vector<SbirsExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "sbirs.target_snr_below_threshold",
                                                            SbirsIssueCause::kAttenuationLimited)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, SbirsExclusionCauseEventKind::kChanged);
  EXPECT_EQ(events[0].previous_cause, SbirsIssueCause::kDistanceLimited);
  EXPECT_EQ(events[0].current_cause, SbirsIssueCause::kAttenuationLimited);
}

// A1：连续两周期相同 (code,cause) → 第二周期不产事件。
TEST(SbirsExclusionCauseRecorderTest, StableCauseProducesNoEvent) {
  SbirsExclusionCauseRecorder recorder;
  SbirsCycleInput input = MakeInputWithTarget(1U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "sbirs.target_out_of_wfov",
                                                     SbirsIssueCause::kBothAxesOutside)}));
  const std::vector<SbirsExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "sbirs.target_out_of_wfov",
                                                            SbirsIssueCause::kBothAxesOutside)}));
  EXPECT_TRUE(events.empty());
}

// A4：被排除 → 不再被排除。
TEST(SbirsExclusionCauseRecorderTest, ExitFromExclusionProducesExitedEvent) {
  SbirsExclusionCauseRecorder recorder;
  SbirsCycleInput input = MakeInputWithTarget(1U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "sbirs.target_out_of_wfov",
                                                     SbirsIssueCause::kAzOutside)}));
  const std::vector<SbirsExclusionCauseEvent> events =
      recorder.Update(input, MakeCompletedResult(2U, {}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, SbirsExclusionCauseEventKind::kExited);
  EXPECT_EQ(events[0].previous_code, "sbirs.target_out_of_wfov");
  EXPECT_TRUE(events[0].current_code.empty());
}

// 非执行周期不产生事件、不推进内部状态。
TEST(SbirsExclusionCauseRecorderTest, NonExecutedCycleDoesNotAdvanceState) {
  SbirsExclusionCauseRecorder recorder;
  SbirsCycleInput input = MakeInputWithTarget(1U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "sbirs.target_out_of_wfov",
                                                     SbirsIssueCause::kAzOutside)}));
  const std::size_t last_size = recorder.GetLastEvents().size();
  SbirsCycleResult rejected;
  rejected.status = SbirsCycleStatus::kRejectedInvalidInput;
  const std::vector<SbirsExclusionCauseEvent> events = recorder.Update(input, rejected);
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(recorder.GetLastEvents().size(), last_size);
  // 周期 3：再次相同排除 → 应为 A1（状态未被非执行周期污染）。
  const std::vector<SbirsExclusionCauseEvent> events3 = recorder.Update(
      input, MakeCompletedResult(3U, {MakeExclusionAtEntity(0U, "sbirs.target_out_of_wfov",
                                                            SbirsIssueCause::kAzOutside)}));
  EXPECT_TRUE(events3.empty());
}

// 多目标独立差分。
TEST(SbirsExclusionCauseRecorderTest, MultipleTargetsTrackIndependently) {
  SbirsExclusionCauseRecorder recorder;
  SbirsCycleInput input;
  SbirsSceneTarget t1;
  t1.target_id = 10U;
  t1.target_name = "alpha";
  SbirsSceneTarget t2;
  t2.target_id = 20U;
  t2.target_name = "beta";
  input.scene.push_back(t1);
  input.scene.push_back(t2);
  // 周期 1：两目标均被方位越界排除。
  recorder.Update(input, MakeCompletedResult(1U,
                                             {MakeExclusionAtEntity(0U, "sbirs.target_out_of_wfov",
                                                                    SbirsIssueCause::kAzOutside),
                                              MakeExclusionAtEntity(1U, "sbirs.target_out_of_wfov",
                                                                    SbirsIssueCause::kAzOutside)}));
  // 周期 2：目标 10 切到俯仰越界（A3），目标 20 退出排除（A4）。
  const std::vector<SbirsExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "sbirs.target_out_of_wfov",
                                                            SbirsIssueCause::kElOutside)}));
  EXPECT_NE(FindEvent(events, 10U, SbirsExclusionCauseEventKind::kChanged), nullptr);
  EXPECT_NE(FindEvent(events, 20U, SbirsExclusionCauseEventKind::kExited), nullptr);
}

// Reset 清空内部状态。
TEST(SbirsExclusionCauseRecorderTest, ResetClearsState) {
  SbirsExclusionCauseRecorder recorder;
  SbirsCycleInput input = MakeInputWithTarget(1U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "sbirs.target_out_of_wfov",
                                                     SbirsIssueCause::kAzOutside)}));
  recorder.Reset();
  const std::vector<SbirsExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "sbirs.target_out_of_wfov",
                                                            SbirsIssueCause::kAzOutside)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, SbirsExclusionCauseEventKind::kEntered);
}

}  // namespace
}  // namespace session
}  // namespace sbirs_sensor
