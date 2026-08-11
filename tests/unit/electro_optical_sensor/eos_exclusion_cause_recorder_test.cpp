/**
 * @file eos_exclusion_cause_recorder_test.cpp
 * @brief EOS 排除原因跨周期差分记录器单元测试（规则 13b 差分观测）。
 *
 * 直接构造 EosCycleInput + EosCycleResult 调用 Update()，覆盖
 * A1/A2/A3/A4 四条转换、非执行周期早退、多目标独立差分。
 * EOS 仅有单一排除 code（eos.target_out_of_fov），A3 覆盖越界轴变化
 * （az → el → both）。
 */

#include "1q/electro_optical_sensor/session/EosExclusionCauseRecorder.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"
#include "1q/foundation/validation_types.h"

namespace electro_optical_sensor {
namespace session {
namespace {

using oneq::foundation::ValidationLocationKind;

// 构造 kInfo 排除诊断（规则 13b），定位到 scene 实体索引。
EosIssue MakeExclusionAtEntity(std::uint64_t entity_index, const std::string& code,
                               EosIssueCause cause) {
  EosIssue issue;
  issue.severity = EosIssueSeverity::kInfo;
  issue.phase = EosIssuePhase::kExecution;
  issue.code = code;
  issue.message = "target_id=" + std::to_string(entity_index);
  issue.cause = cause;
  issue.location.kind = ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = static_cast<std::size_t>(entity_index);
  return issue;
}

// 构造 kCompleted 周期结果，携带给定排除诊断列表。
EosCycleResult MakeCompletedResult(std::uint32_t cycle_index, EosIssueList issues) {
  EosCycleResult result;
  result.input_cycle_index = cycle_index;
  result.status = EosCycleStatus::kCompleted;
  result.issues = std::move(issues);
  return result;
}

// 构造单目标输入。
EosCycleInput MakeInputWithTarget(std::uint64_t target_id, const std::string& name) {
  EosCycleInput input;
  EosSceneTarget target;
  target.target_id = target_id;
  target.target_name = name;
  input.scene.push_back(target);
  return input;
}

// 在事件列表中按 (target_id, kind) 查找事件。
const EosExclusionCauseEvent* FindEvent(const std::vector<EosExclusionCauseEvent>& events,
                                        std::uint64_t target_id,
                                        EosExclusionCauseEventKind kind) {
  for (const EosExclusionCauseEvent& event : events) {
    if (event.target_id == target_id && event.kind == kind) {
      return &event;
    }
  }
  return nullptr;
}

// A2：未被排除 → 被排除（视场方位越界）。
TEST(EosExclusionCauseRecorderTest, EntryIntoExclusionProducesEnteredEvent) {
  EosExclusionCauseRecorder recorder;
  EosCycleInput input = MakeInputWithTarget(201U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {}));
  const std::vector<EosExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "eos.target_out_of_fov",
                                                            EosIssueCause::kAzOutside)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, EosExclusionCauseEventKind::kEntered);
  EXPECT_EQ(events[0].target_id, 201U);
  EXPECT_EQ(events[0].cycle_index, 2U);
  EXPECT_TRUE(events[0].previous_code.empty());
  EXPECT_EQ(events[0].current_code, "eos.target_out_of_fov");
  EXPECT_EQ(events[0].current_cause, EosIssueCause::kAzOutside);
}

// A3：越界轴变化（az → both）——EOS 单 code 下 cause 变化驱动 A3。
TEST(EosExclusionCauseRecorderTest, AxisChangeProducesChangedEvent) {
  EosExclusionCauseRecorder recorder;
  EosCycleInput input = MakeInputWithTarget(201U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "eos.target_out_of_fov",
                                                     EosIssueCause::kAzOutside)}));
  const std::vector<EosExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "eos.target_out_of_fov",
                                                            EosIssueCause::kBothAxesOutside)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, EosExclusionCauseEventKind::kChanged);
  EXPECT_EQ(events[0].previous_cause, EosIssueCause::kAzOutside);
  EXPECT_EQ(events[0].current_cause, EosIssueCause::kBothAxesOutside);
  // code 不变（EOS 单一排除 code），但仍产事件（cause 变化）。
  EXPECT_EQ(events[0].previous_code, events[0].current_code);
}

// A1：连续两周期相同 (code,cause) → 第二周期不产事件。
TEST(EosExclusionCauseRecorderTest, StableCauseProducesNoEvent) {
  EosExclusionCauseRecorder recorder;
  EosCycleInput input = MakeInputWithTarget(201U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "eos.target_out_of_fov",
                                                     EosIssueCause::kElOutside)}));
  const std::vector<EosExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "eos.target_out_of_fov",
                                                            EosIssueCause::kElOutside)}));
  EXPECT_TRUE(events.empty());
}

// A4：被排除 → 不再被排除。
TEST(EosExclusionCauseRecorderTest, ExitFromExclusionProducesExitedEvent) {
  EosExclusionCauseRecorder recorder;
  EosCycleInput input = MakeInputWithTarget(201U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "eos.target_out_of_fov",
                                                     EosIssueCause::kAzOutside)}));
  const std::vector<EosExclusionCauseEvent> events =
      recorder.Update(input, MakeCompletedResult(2U, {}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, EosExclusionCauseEventKind::kExited);
  EXPECT_EQ(events[0].previous_code, "eos.target_out_of_fov");
  EXPECT_TRUE(events[0].current_code.empty());
}

// 非执行周期不产生事件、不推进内部状态。
TEST(EosExclusionCauseRecorderTest, NonExecutedCycleDoesNotAdvanceState) {
  EosExclusionCauseRecorder recorder;
  EosCycleInput input = MakeInputWithTarget(201U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "eos.target_out_of_fov",
                                                     EosIssueCause::kAzOutside)}));
  const std::size_t last_size = recorder.GetLastEvents().size();
  EosCycleResult rejected;
  rejected.status = EosCycleStatus::kRejectedInvalidInput;
  const std::vector<EosExclusionCauseEvent> events = recorder.Update(input, rejected);
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(recorder.GetLastEvents().size(), last_size);
  // 周期 3：再次相同排除 → 应为 A1（状态未被非执行周期污染）。
  const std::vector<EosExclusionCauseEvent> events3 = recorder.Update(
      input, MakeCompletedResult(3U, {MakeExclusionAtEntity(0U, "eos.target_out_of_fov",
                                                            EosIssueCause::kAzOutside)}));
  EXPECT_TRUE(events3.empty());
}

// 多目标独立差分。
TEST(EosExclusionCauseRecorderTest, MultipleTargetsTrackIndependently) {
  EosExclusionCauseRecorder recorder;
  EosCycleInput input;
  EosSceneTarget t1;
  t1.target_id = 10U;
  t1.target_name = "alpha";
  EosSceneTarget t2;
  t2.target_id = 20U;
  t2.target_name = "beta";
  input.scene.push_back(t1);
  input.scene.push_back(t2);
  // 周期 1：两目标均被方位越界排除。
  recorder.Update(input, MakeCompletedResult(1U,
                                             {MakeExclusionAtEntity(0U, "eos.target_out_of_fov",
                                                                    EosIssueCause::kAzOutside),
                                              MakeExclusionAtEntity(1U, "eos.target_out_of_fov",
                                                                    EosIssueCause::kAzOutside)}));
  // 周期 2：目标 10 切到俯仰越界（A3），目标 20 退出排除（A4）。
  const std::vector<EosExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "eos.target_out_of_fov",
                                                            EosIssueCause::kElOutside)}));
  EXPECT_NE(FindEvent(events, 10U, EosExclusionCauseEventKind::kChanged), nullptr);
  EXPECT_NE(FindEvent(events, 20U, EosExclusionCauseEventKind::kExited), nullptr);
}

// Reset 清空内部状态。
TEST(EosExclusionCauseRecorderTest, ResetClearsState) {
  EosExclusionCauseRecorder recorder;
  EosCycleInput input = MakeInputWithTarget(201U, "alpha");
  recorder.Update(input, MakeCompletedResult(1U, {MakeExclusionAtEntity(
                                                     0U, "eos.target_out_of_fov",
                                                     EosIssueCause::kAzOutside)}));
  recorder.Reset();
  const std::vector<EosExclusionCauseEvent> events = recorder.Update(
      input, MakeCompletedResult(2U, {MakeExclusionAtEntity(0U, "eos.target_out_of_fov",
                                                            EosIssueCause::kAzOutside)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, EosExclusionCauseEventKind::kEntered);
}

}  // namespace
}  // namespace session
}  // namespace electro_optical_sensor
