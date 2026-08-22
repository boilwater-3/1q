// Copyright 2026. All Rights Reserved.
//
// @file rir_exclusion_cause_recorder_test.cpp
// @brief 排除原因跨周期差分记录器单元测试（规则 13b/13e 差分观测）。
//
// 直接构造 RirCycleResult + RirSceneTargetList 调用 Update()，精准覆盖
// A1（稳定）/A2（进入）/A3（变化）/A4（退出）四条转换、RIR 多门 code 切换、
// 非执行周期早退与零 ID 跳过。

#include "1q/remote_identification_radar/session/RirExclusionCauseRecorder.h"

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/foundation/validation_types.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using oneq::foundation::ValidationLocationKind;

using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirExclusionCauseEventKind;
using session::RirExclusionCauseRecorder;
using session::RirIssue;
using session::RirIssueCause;
using session::RirSceneTarget;
using session::RirSceneTargetList;

// 构造 kInfo 排除诊断（规则 13b），定位到 scene 实体索引。
RirIssue MakeExclusionAtEntity(std::uint64_t target_index, const std::string& code,
                               RirIssueCause cause) {
  RirIssue issue;
  issue.severity = session::RirIssueSeverity::kInfo;
  issue.phase = session::RirIssuePhase::kExecution;
  issue.code = code;
  issue.message = "target_id=" + std::to_string(target_index);
  issue.cause = cause;
  issue.location.kind = ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = static_cast<std::size_t>(target_index);
  return issue;
}

// 构造 kCompleted 周期结果，携带给定排除诊断列表。
RirCycleResult MakeCompletedResult(std::uint32_t cycle_index,
                                   std::vector<RirIssue> issues) {
  RirCycleResult result;
  result.input_cycle_index = cycle_index;
  result.status = RirCycleStatus::kCompleted;
  result.issues = std::move(issues);
  return result;
}

// 构造单个输入目标。
RirSceneTarget MakeTarget(std::uint64_t target_id, const std::string& name) {
  RirSceneTarget target;
  target.external_target_id = target_id;
  target.target_name = name;
  return target;
}

// 在事件列表中按 (target_id, kind) 查找事件。
const session::RirExclusionCauseEvent* FindEvent(
    const std::vector<session::RirExclusionCauseEvent>& events, std::uint64_t target_id,
    RirExclusionCauseEventKind kind) {
  for (const session::RirExclusionCauseEvent& event : events) {
    if (event.external_target_id == target_id && event.kind == kind) {
      return &event;
    }
  }
  return nullptr;
}

// A2：未被排除 → 被排除产生 kEntered，previous 为空/kNone，current 携带本周 (code,cause)。
TEST(RirExclusionCauseRecorderTest, EntryIntoExclusionProducesEnteredEvent) {
  RirExclusionCauseRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(1U, {}));

  const std::vector<session::RirExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(
                   2U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                              RirIssueCause::kDistanceLimited)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirExclusionCauseEventKind::kEntered);
  EXPECT_EQ(events[0].external_target_id, 1001U);
  EXPECT_EQ(events[0].world_cycle_index, 2U);
  EXPECT_TRUE(events[0].previous_code.empty());
  EXPECT_EQ(events[0].previous_cause, RirIssueCause::kNone);
  EXPECT_EQ(events[0].current_code, "rir.target_detection_gate");
  EXPECT_EQ(events[0].current_cause, RirIssueCause::kDistanceLimited);
}

// A3（RIR 多门场景）：同为 kNone 主因的具体门切换 → code 变化仍产 kChanged。
TEST(RirExclusionCauseRecorderTest, GateCodeChangeProducesChangedEvent) {
  RirExclusionCauseRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(
                               1U, {MakeExclusionAtEntity(0U, "rir.target_mode_not_identify",
                                                          RirIssueCause::kNone)}));

  const std::vector<session::RirExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(
                   2U, {MakeExclusionAtEntity(0U, "rir.target_beyond_recognition_range",
                                              RirIssueCause::kNone)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirExclusionCauseEventKind::kChanged);
  EXPECT_EQ(events[0].previous_code, "rir.target_mode_not_identify");
  EXPECT_EQ(events[0].current_code, "rir.target_beyond_recognition_range");
  EXPECT_EQ(events[0].previous_cause, RirIssueCause::kNone);
  EXPECT_EQ(events[0].current_cause, RirIssueCause::kNone);
}

// A3（聚合门主因变化）：code 不变、cause 变化仍产 kChanged。
TEST(RirExclusionCauseRecorderTest, CauseChangeProducesChangedEvent) {
  RirExclusionCauseRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(
                               1U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                                          RirIssueCause::kRcsLimited)}));

  const std::vector<session::RirExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(
                   2U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                              RirIssueCause::kBeamLimited)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirExclusionCauseEventKind::kChanged);
  EXPECT_EQ(events[0].previous_code, events[0].current_code);
  EXPECT_EQ(events[0].previous_cause, RirIssueCause::kRcsLimited);
  EXPECT_EQ(events[0].current_cause, RirIssueCause::kBeamLimited);
}

// A1：连续两周期相同 (code,cause) → 第二周期不产事件。
TEST(RirExclusionCauseRecorderTest, StableCauseProducesNoEvent) {
  RirExclusionCauseRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(
                               1U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                                          RirIssueCause::kRcsLimited)}));
  const std::vector<session::RirExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(
                   2U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                              RirIssueCause::kRcsLimited)}));
  EXPECT_TRUE(events.empty());
}

// A4：被排除 → 恢复产生 kExited，previous 携带上周 (code,cause)。
TEST(RirExclusionCauseRecorderTest, ExitFromExclusionProducesExitedEvent) {
  RirExclusionCauseRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(
                               1U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                                          RirIssueCause::kRcsLimited)}));

  const std::vector<session::RirExclusionCauseEvent> events =
      recorder.Update(targets, MakeCompletedResult(2U, {}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirExclusionCauseEventKind::kExited);
  EXPECT_EQ(events[0].previous_code, "rir.target_detection_gate");
  EXPECT_EQ(events[0].previous_cause, RirIssueCause::kRcsLimited);
  EXPECT_TRUE(events[0].current_code.empty());
  EXPECT_EQ(events[0].current_cause, RirIssueCause::kNone);
}

// 非执行周期：空事件、不推进状态（恢复执行后 A1 语义保持，不重复 A2）。
TEST(RirExclusionCauseRecorderTest, NonExecutedCycleDoesNotAdvanceState) {
  RirExclusionCauseRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(
                               1U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                                          RirIssueCause::kRcsLimited)}));

  RirCycleResult powered_off;
  powered_off.input_cycle_index = 2U;
  powered_off.status = RirCycleStatus::kPoweredOff;
  EXPECT_TRUE(recorder.Update(targets, powered_off).empty());
  ASSERT_EQ(recorder.GetLastEvents().size(), 1U);  // 缓存保留上次执行周期。

  // 持续相同 (code,cause) → A1 稳定，不产事件。
  const std::vector<session::RirExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(
                   3U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                              RirIssueCause::kRcsLimited)}));
  EXPECT_TRUE(events.empty());
}

// 零 ID 目标跳过；多目标按 entity_index 独立差分。
TEST(RirExclusionCauseRecorderTest, MultipleTargetsTrackIndependently) {
  RirExclusionCauseRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(0U, "anon"), MakeTarget(1001U, "alpha"),
                                      MakeTarget(1002U, "beta")};

  // 实体下标对位输入表：1001 在下标 1（下标 0 为零 ID 目标，其上的 issue 不消费）。
  const std::vector<session::RirExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(
                   1U, {MakeExclusionAtEntity(1U, "rir.target_detection_gate",
                                              RirIssueCause::kRcsLimited),
                        MakeExclusionAtEntity(2U, "rir.target_mode_not_identify",
                                              RirIssueCause::kNone)}));
  ASSERT_EQ(events.size(), 2U);
  EXPECT_NE(FindEvent(events, 1001U, RirExclusionCauseEventKind::kEntered), nullptr);
  EXPECT_NE(FindEvent(events, 1002U, RirExclusionCauseEventKind::kEntered), nullptr);
  EXPECT_EQ(FindEvent(events, 0U, RirExclusionCauseEventKind::kEntered), nullptr);
}

// 输入校验 phase 的 kSceneEntity 条目不作差分原料（仅执行期排除）。
TEST(RirExclusionCauseRecorderTest, InputValidationIssuesAreIgnored) {
  RirExclusionCauseRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};

  RirCycleResult result = MakeCompletedResult(
      1U, {MakeExclusionAtEntity(0U, "rir.validation.non_finite_target_field",
                                 RirIssueCause::kNone)});
  result.issues[0].phase = session::RirIssuePhase::kInputValidation;

  EXPECT_TRUE(recorder.Update(targets, result).empty());
}

// Reset 清空差分状态：其后同因排除重新 A2 进入。
TEST(RirExclusionCauseRecorderTest, ResetClearsState) {
  RirExclusionCauseRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};
  recorder.Update(targets, MakeCompletedResult(
                               1U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                                          RirIssueCause::kRcsLimited)}));
  recorder.Reset();
  EXPECT_TRUE(recorder.GetLastEvents().empty());

  const std::vector<session::RirExclusionCauseEvent> events = recorder.Update(
      targets, MakeCompletedResult(
                   2U, {MakeExclusionAtEntity(0U, "rir.target_detection_gate",
                                              RirIssueCause::kRcsLimited)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirExclusionCauseEventKind::kEntered);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
