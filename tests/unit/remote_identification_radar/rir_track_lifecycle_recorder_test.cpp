// Copyright 2026. All Rights Reserved.
//
// @file rir_track_lifecycle_recorder_test.cpp
// @brief 航迹生命周期记录器单元测试（观测投影 Lifecycle，规则 10）。
//
// 直接构造 RirCycleResult + RirSceneTargetList 调用 Update()，覆盖
// 首确认/更新/丢失/丢失后再确认、kNotTracked 显式开启、指定任务作废沿
// （回扫沿/窗口耗尽）、零 ID 跳过、非执行周期空事件不推进状态。

#include "1q/remote_identification_radar/session/RirTrackLifecycleRecorder.h"

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "1q/remote_identification_radar/session/RirCycleResult.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirSceneTarget;
using session::RirSceneTargetList;
using session::RirTrackAttributionRecord;
using session::RirTrackLifecycleEventKind;
using session::RirTrackLifecycleRecorder;
using session::RirTrackLifecycleRecorderConfig;
using session::RirTrackLifecycleStatus;

RirSceneTarget MakeTarget(std::uint64_t id, const char* name) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = name;
  target.position_x = 5000.0f;
  target.position_z = 2000.0f;
  return target;
}

RirTrackAttributionRecord MakeAttribution(std::uint64_t key, std::uint64_t id,
                                          RirTrackLifecycleStatus status) {
  RirTrackAttributionRecord attribution;
  attribution.association_key = key;
  attribution.external_target_id = id;
  attribution.target_name = "truth";
  attribution.track_status = status;
  attribution.hit_count = 3U;
  attribution.speed_m_per_s = 42.0;
  return attribution;
}

RirCycleResult MakeCompletedResult(std::uint32_t cycle_index,
                                   std::vector<RirTrackAttributionRecord> attributions) {
  RirCycleResult result;
  result.input_cycle_index = cycle_index;
  result.status = RirCycleStatus::kCompleted;
  result.track_attributions = std::move(attributions);
  return result;
}

RirCycleResult MakeNonExecutedResult(std::uint32_t cycle_index) {
  RirCycleResult result;
  result.input_cycle_index = cycle_index;
  result.status = RirCycleStatus::kPoweredOff;
  result.abort_reason = session::RirCycleAbortReason::kPoweredOff;
  return result;
}

const session::RirTrackLifecycleEvent* FindEvent(
    const std::vector<session::RirTrackLifecycleEvent>& events, std::uint64_t target_id,
    RirTrackLifecycleEventKind kind) {
  for (const session::RirTrackLifecycleEvent& event : events) {
    if (event.external_target_id == target_id && event.kind == kind) {
      return &event;
    }
  }
  return nullptr;
}

/// @brief 首确认 → 更新 → 丢失 → 再确认的状态机主干。
TEST(RirTrackLifecycleRecorderTest, TracksFirstConfirmedUpdatedLost) {
  RirTrackLifecycleRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};

  // 周期 1：tentative，无事件（默认不开 kNotTracked）。
  EXPECT_TRUE(recorder.Update(targets, MakeCompletedResult(1U, {})).empty());

  // 周期 2：首次确认。
  std::vector<session::RirTrackLifecycleEvent> events =
      recorder.Update(targets, MakeCompletedResult(2U, {MakeAttribution(
                                                            10U, 1001U,
                                                            RirTrackLifecycleStatus::kConfirmed)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirTrackLifecycleEventKind::kFirstConfirmed);
  EXPECT_EQ(events[0].association_key, 10U);
  EXPECT_EQ(events[0].track_status, RirTrackLifecycleStatus::kConfirmed);
  EXPECT_EQ(events[0].world_cycle_index, 2U);

  // 周期 3：持续确认 → kUpdated。
  events = recorder.Update(targets, MakeCompletedResult(3U, {MakeAttribution(
                                                                 10U, 1001U,
                                                                 RirTrackLifecycleStatus::kConfirmed)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirTrackLifecycleEventKind::kUpdated);

  // 周期 4：丢失 → kLost。
  events = recorder.Update(targets, MakeCompletedResult(4U, {MakeAttribution(
                                                                 10U, 1001U,
                                                                 RirTrackLifecycleStatus::kLost)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirTrackLifecycleEventKind::kLost);
  EXPECT_EQ(events[0].track_status, RirTrackLifecycleStatus::kLost);

  // 周期 5：丢失后再次确认 → 重新 kFirstConfirmed。
  events = recorder.Update(targets, MakeCompletedResult(5U, {MakeAttribution(
                                                                 10U, 1001U,
                                                                 RirTrackLifecycleStatus::kConfirmed)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirTrackLifecycleEventKind::kFirstConfirmed);
}

/// @brief kNotTracked 诊断事件需显式开启。
TEST(RirTrackLifecycleRecorderTest, NotTrackedEventsRequireExplicitEnable) {
  RirTrackLifecycleRecorderConfig config;
  config.emit_not_tracked_events = true;
  RirTrackLifecycleRecorder recorder(config);
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};

  const std::vector<session::RirTrackLifecycleEvent> events =
      recorder.Update(targets, MakeCompletedResult(1U, {}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirTrackLifecycleEventKind::kNotTracked);
  EXPECT_EQ(events[0].reason, session::RirTrackLifecycleReason::kNoTrack);
  EXPECT_EQ(events[0].association_key, 0U);

  // 默认配置：无航迹不产事件。
  RirTrackLifecycleRecorder default_recorder;
  EXPECT_TRUE(default_recorder.Update(targets, MakeCompletedResult(1U, {})).empty());
}

/// @brief 零 ID 输入目标跳过生命周期记录。
TEST(RirTrackLifecycleRecorderTest, ZeroIdTargetsAreSkipped) {
  RirTrackLifecycleRecorderConfig config;
  config.emit_not_tracked_events = true;
  RirTrackLifecycleRecorder recorder(config);
  const RirSceneTargetList targets = {MakeTarget(0U, "anon"), MakeTarget(1002U, "beta")};

  const std::vector<session::RirTrackLifecycleEvent> events =
      recorder.Update(targets, MakeCompletedResult(1U, {}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].external_target_id, 1002U);
}

/// @brief 非执行周期：空事件、不推进状态（确认态保持，恢复执行后不重发首确认）。
TEST(RirTrackLifecycleRecorderTest, NonCompletedCyclePreservesState) {
  RirTrackLifecycleRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};

  recorder.Update(targets, MakeCompletedResult(2U, {MakeAttribution(
                                                        10U, 1001U,
                                                        RirTrackLifecycleStatus::kConfirmed)}));
  EXPECT_TRUE(recorder.Update(targets, MakeNonExecutedResult(3U)).empty());
  // 缓存保留上一次执行周期的事件。
  ASSERT_EQ(recorder.GetLastEvents().size(), 1U);
  EXPECT_EQ(recorder.GetLastEvents()[0].kind, RirTrackLifecycleEventKind::kFirstConfirmed);

  const std::vector<session::RirTrackLifecycleEvent> events =
      recorder.Update(targets, MakeCompletedResult(4U, {MakeAttribution(
                                                            10U, 1001U,
                                                            RirTrackLifecycleStatus::kConfirmed)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirTrackLifecycleEventKind::kUpdated);
}

/// @brief 指定任务回扫沿：上一周期生效、本周期回扫 → kDesignationDropped 带成因。
TEST(RirTrackLifecycleRecorderTest, DesignationDropOnRevertEdge) {
  RirTrackLifecycleRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};

  // 周期 1：驻留生效。
  RirCycleResult active = MakeCompletedResult(
      1U, {MakeAttribution(10U, 1001U, RirTrackLifecycleStatus::kConfirmed)});
  active.designated_target_id = 1001U;
  active.designation_active = true;
  recorder.Update(targets, active);

  // 周期 2：任务作废回扫（未识别成因）。
  RirCycleResult reverted = MakeCompletedResult(
      2U, {MakeAttribution(10U, 1001U, RirTrackLifecycleStatus::kConfirmed)});
  reverted.designated_target_id = 1001U;
  reverted.designation_active = false;
  reverted.designation_reverted_to_scan = true;
  reverted.designation_revert_reason = session::RirDesignationRevertReason::kNotRecognized;
  const std::vector<session::RirTrackLifecycleEvent> events =
      recorder.Update(targets, reverted);

  const session::RirTrackLifecycleEvent* drop =
      FindEvent(events, 1001U, RirTrackLifecycleEventKind::kDesignationDropped);
  ASSERT_NE(drop, nullptr);
  EXPECT_EQ(drop->designation_revert_reason, session::RirDesignationRevertReason::kNotRecognized);
  EXPECT_EQ(drop->association_key, 10U);
}

/// @brief 窗口耗尽沿：指定但未生效（持续回扫）超时 → kDesignationDropped
///        （成因 kAcquisitionTimeout；目标从未驻留生效的路径）。
TEST(RirTrackLifecycleRecorderTest, DesignationTimeoutEmitsDropOnExpiryEdge) {
  RirTrackLifecycleRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};

  // 周期 1：指定但目标不在可扫描体积（未生效回扫，pending）。
  RirCycleResult pending = MakeCompletedResult(1U, {});
  pending.designated_target_id = 1001U;
  pending.designation_active = false;
  pending.designation_reverted_to_scan = true;
  pending.designation_revert_reason =
      session::RirDesignationRevertReason::kOutsideSteerableVolume;
  recorder.Update(targets, pending);

  // 周期 2：窗口耗尽（成因切换为 kAcquisitionTimeout）。
  RirCycleResult expired = MakeCompletedResult(2U, {});
  expired.designated_target_id = 1001U;
  expired.designation_active = false;
  expired.designation_reverted_to_scan = true;
  expired.designation_revert_reason = session::RirDesignationRevertReason::kAcquisitionTimeout;
  const std::vector<session::RirTrackLifecycleEvent> events = recorder.Update(targets, expired);

  const session::RirTrackLifecycleEvent* drop =
      FindEvent(events, 1001U, RirTrackLifecycleEventKind::kDesignationDropped);
  ASSERT_NE(drop, nullptr);
  EXPECT_EQ(drop->designation_revert_reason,
            session::RirDesignationRevertReason::kAcquisitionTimeout);
}

/// @brief Reset 清空状态：确认史遗忘，其后确认重新发 kFirstConfirmed。
TEST(RirTrackLifecycleRecorderTest, ResetClearsState) {
  RirTrackLifecycleRecorder recorder;
  const RirSceneTargetList targets = {MakeTarget(1001U, "alpha")};

  recorder.Update(targets, MakeCompletedResult(2U, {MakeAttribution(
                                                        10U, 1001U,
                                                        RirTrackLifecycleStatus::kConfirmed)}));
  recorder.Reset();
  EXPECT_TRUE(recorder.GetLastEvents().empty());

  const std::vector<session::RirTrackLifecycleEvent> events =
      recorder.Update(targets, MakeCompletedResult(3U, {MakeAttribution(
                                                            10U, 1001U,
                                                            RirTrackLifecycleStatus::kConfirmed)}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, RirTrackLifecycleEventKind::kFirstConfirmed);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
