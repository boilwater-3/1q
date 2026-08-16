/**
 * @file ar_track_output_debug_view_test.cpp
 * @brief 验证 AR 轨迹三层输出：debug view 构建与生命周期记录器。
 *
 * AR 的边界与 EOS 不同：track 是系统估计（非传感器原始输出），target_name
 * 作为人读标签直接附在 track 上，无需独立 attribution 层。本测试覆盖：
 * - debug view 把 track 输出 + 输入目标表合成为可读状态，name 经 ID 回填。
 * - lifecycle recorder 跨周期记录首次确认/更新/丢失，未跟踪原因需显式开启。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"

namespace airborne_radar {
namespace session {
namespace {

namespace ar_model = ::airborne_radar::session;

ArTargetInput MakeNamedTarget(std::uint64_t id, const std::string& name) {
  ArTargetInput target;
  target.target_id = id;
  target.target_name = name;
  target.rcs = 1.0f;
  return target;
}

session::TrackStateSnapshot MakeTrackSnapshot(std::uint64_t external_target_id, const std::string& name,
                                            session::TrackStatus status, float speed = 100.0f) {
  session::TrackStateSnapshot snapshot;
  snapshot.external_target_id = external_target_id;
  snapshot.target_name = name;
  snapshot.status = status;
  snapshot.association_key = external_target_id * 10U;
  snapshot.speed = speed;
  snapshot.rcs = 2.0f;
  snapshot.hit_count = 3U;
  snapshot.miss_count = 0U;
  snapshot.target_type = "HIGH_THREAT_FIGHTER";
  return snapshot;
}

ArCycleResult MakeCycleResult(std::uint64_t cycle_index, bool completed,
                              std::vector<session::TrackStateSnapshot> tracks) {
  ArCycleResult result;
  result.status = completed ? ArCycleStatus::kCompleted
                            : ArCycleStatus::kRejectedExecution;
  result.input_cycle_index = cycle_index;
  result.output_frame.cycle_index = cycle_index;
  result.output_frame.tracks = std::move(tracks);
  return result;
}

}  // namespace

// debug view：把 track 输出 + 输入目标表合成；name 经 external_target_id 回填。
TEST(RadarTrackOutputDebugViewTest, BuildMapsTracksBackToNamedTargets) {
  const ArTargetInputList targets = {MakeNamedTarget(701U, "alpha"),
                                     MakeNamedTarget(702U, "beta"),
                                     MakeNamedTarget(0U, "no-id")};

  ArCycleResult result =
      MakeCycleResult(5U, /*executed=*/true,
                      {MakeTrackSnapshot(701U, "alpha", session::TrackStatus::kConfirmed),
                       MakeTrackSnapshot(702U, "beta", session::TrackStatus::kTentative)});

  ArCycleInput input;
  input.targets = targets;
  const ArTrackOutputDebugView view = ArTrackOutputDebugViewBuilder::Build(input, result);
  EXPECT_EQ(view.world_cycle_index, 5U);
  EXPECT_TRUE(view.completed_this_cycle);
  ASSERT_EQ(view.tracks.size(), 3U);

  // alpha 已确认，name 与 status 回填。
  EXPECT_EQ(view.tracks[0].target_name, "alpha");
  EXPECT_EQ(view.tracks[0].status, ArDebugTrackStatus::kConfirmed);
  EXPECT_TRUE(view.tracks[0].has_track);
  EXPECT_FLOAT_EQ(view.tracks[0].speed, 100.0f);
  EXPECT_EQ(view.tracks[0].association_key, 7010U);

  // beta 候选。
  EXPECT_EQ(view.tracks[1].status, ArDebugTrackStatus::kTentative);

  // external_target_id=0 的输入目标无法按 ID 关联，标记为 NotInOutput。
  EXPECT_EQ(view.tracks[2].target_name, "no-id");
  EXPECT_EQ(view.tracks[2].status, ArDebugTrackStatus::kNotInOutput);
  EXPECT_FALSE(view.tracks[2].has_track);
  // 输入实体回填（规则 12）：有轨迹的目标 rcs 以 track 观测值为准（2.0），
  // 无轨迹目标回填 input 侧真值（1.0）——未跟踪也可见目标 RCS。
  EXPECT_FLOAT_EQ(view.tracks[0].rcs, 2.0f);
  EXPECT_FLOAT_EQ(view.tracks[2].rcs, 1.0f);
}

// debug view：未完成周期时所有目标标记为 kCycleNotCompleted。
TEST(RadarTrackOutputDebugViewTest, NonCompletedCycleMarksAllTargetsAsNonCompleted) {
  const ArTargetInputList targets = {MakeNamedTarget(701U, "alpha")};

  ArCycleResult result = MakeCycleResult(9U, /*completed=*/false, {});
  ArCycleInput input;
  input.targets = targets;
  const ArTrackOutputDebugView view = ArTrackOutputDebugViewBuilder::Build(input, result);
  ASSERT_EQ(view.tracks.size(), 1U);
  EXPECT_EQ(view.tracks[0].status, ArDebugTrackStatus::kCycleNotCompleted);
  EXPECT_FALSE(view.tracks[0].has_track);
  // 未完成周期同样回填 input 侧 RCS 真值。
  EXPECT_FLOAT_EQ(view.tracks[0].rcs, 1.0f);
}

// lifecycle recorder：首次确认 → 更新 → 丢失；未跟踪默认不产生事件。
TEST(RadarTrackLifecycleRecorderTest, TracksFirstConfirmedUpdatedLost) {
  const ArTargetInputList targets = {MakeNamedTarget(800U, "gamma")};

  ArTrackLifecycleRecorder recorder;

  // 周期 1：首次确认。
  ArCycleResult first = MakeCycleResult(
      1U, /*completed=*/true, {MakeTrackSnapshot(800U, "gamma", session::TrackStatus::kConfirmed)});
  std::vector<ArTrackLifecycleEvent> events = recorder.Update(targets, first);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArTrackLifecycleEventKind::kFirstConfirmed);
  EXPECT_EQ(events[0].target_name, "gamma");

  // 周期 2：已确认 → 更新。
  ArCycleResult second = MakeCycleResult(
      2U, /*completed=*/true, {MakeTrackSnapshot(800U, "gamma", session::TrackStatus::kConfirmed)});
  events = recorder.Update(targets, second);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArTrackLifecycleEventKind::kUpdated);

  // 周期 3：丢失。
  ArCycleResult lost = MakeCycleResult(
      3U, /*completed=*/true, {MakeTrackSnapshot(800U, "gamma", session::TrackStatus::kLost)});
  events = recorder.Update(targets, lost);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArTrackLifecycleEventKind::kLost);
}

TEST(RadarTrackLifecycleRecorderTest, NonCompletedCyclePreservesConfirmedState) {
  const ArTargetInputList targets = {MakeNamedTarget(801U, "recoverable")};
  ArTrackLifecycleRecorder recorder(ArTrackLifecycleRecorderConfig{true});
  ArCycleResult confirmed = MakeCycleResult(
      1U, true, {MakeTrackSnapshot(801U, "recoverable", session::TrackStatus::kConfirmed)});
  ASSERT_EQ(recorder.Update(targets, confirmed).front().kind,
            ArTrackLifecycleEventKind::kFirstConfirmed);
  ArCycleResult rejected = MakeCycleResult(2U, false, {});
  EXPECT_TRUE(recorder.Update(targets, rejected).empty());
  confirmed.input_cycle_index = 3U;
  const std::vector<ArTrackLifecycleEvent> recovered = recorder.Update(targets, confirmed);
  ASSERT_EQ(recovered.size(), 1U);
  EXPECT_EQ(recovered.front().kind, ArTrackLifecycleEventKind::kUpdated);
}

// lifecycle recorder：未跟踪目标默认不产生事件；开启后产生 kNotTracked。
TEST(RadarTrackLifecycleRecorderTest, NotTrackedEventsRequireExplicitEnable) {
  const ArTargetInputList targets = {MakeNamedTarget(900U, "delta")};

  // 默认配置：无 track 的目标不产生事件。
  ArTrackLifecycleRecorder recorder;
  ArCycleResult no_track = MakeCycleResult(1U, /*completed=*/true, {});
  std::vector<ArTrackLifecycleEvent> events = recorder.Update(targets, no_track);
  EXPECT_TRUE(events.empty());

  // 开启诊断后产生 kNotTracked。
  ArTrackLifecycleRecorder diagnose_recorder(ArTrackLifecycleRecorderConfig{true});
  events = diagnose_recorder.Update(targets, no_track);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArTrackLifecycleEventKind::kNotTracked);
  EXPECT_EQ(events[0].reason, ArTrackLifecycleReason::kNoTrack);
  EXPECT_EQ(events[0].target_name, "delta");
}

// lifecycle recorder：external_target_id=0 的目标不参与生命周期记录（无法关联）。
TEST(RadarTrackLifecycleRecorderTest, ZeroIdTargetsAreSkipped) {
  const ArTargetInputList targets = {MakeNamedTarget(0U, "no-id")};

  ArTrackLifecycleRecorder diagnose_recorder(ArTrackLifecycleRecorderConfig{true});
  ArCycleResult result = MakeCycleResult(1U, /*completed=*/true, {});
  std::vector<ArTrackLifecycleEvent> events = diagnose_recorder.Update(targets, result);
  EXPECT_TRUE(events.empty());
}

// lifecycle recorder：指定航迹回退（自动丢跟踪，回 TWS）在转换沿产生 kDesignationDropped。
TEST(RadarTrackLifecycleRecorderTest, DesignationDropEmitsEventOnRevertEdge) {
  const ArTargetInputList targets = {MakeNamedTarget(910U, "designated")};

  ArTrackLifecycleRecorder recorder;
  // 周期 1：指定跟踪生效（confirmed 航迹 + designation_active）。
  ArCycleResult active = MakeCycleResult(
      1U, /*completed=*/true, {MakeTrackSnapshot(910U, "designated", session::TrackStatus::kConfirmed)});
  active.designation_active = true;
  active.designated_target_id = 910U;
  active.effective_work_mode = config::ArWorkMode::kStt;
  std::vector<ArTrackLifecycleEvent> events = recorder.Update(targets, active);
  // 首次确认事件照常产生；回退尚未发生。
  EXPECT_EQ(events.front().kind, ArTrackLifecycleEventKind::kFirstConfirmed);

  // 周期 2：指定航迹丢失 → 回退状态（designation_active=false、reverted=true、
  // reason=kTrackLost）→ 产生 kDesignationDropped。
  ArCycleResult reverted = MakeCycleResult(
      2U, /*completed=*/true, {MakeTrackSnapshot(910U, "designated", session::TrackStatus::kLost)});
  reverted.designation_active = false;
  reverted.designated_target_id = 910U;
  reverted.designation_reverted_to_tws = true;
  reverted.designation_revert_reason = ArDesignationRevertReason::kTrackLost;
  reverted.effective_work_mode = config::ArWorkMode::kTws;
  events = recorder.Update(targets, reverted);
  const auto dropped = std::find_if(
      events.begin(), events.end(), [](const ArTrackLifecycleEvent& event) {
        return event.kind == ArTrackLifecycleEventKind::kDesignationDropped;
      });
  ASSERT_NE(dropped, events.end());
  EXPECT_EQ(dropped->external_target_id, 910U);
  EXPECT_EQ(dropped->designation_revert_reason, ArDesignationRevertReason::kTrackLost);

  // 周期 3：持续回退状态不重复产生事件（转换沿语义）。
  ArCycleResult still_reverted = reverted;
  still_reverted.input_cycle_index = 3U;
  events = recorder.Update(targets, still_reverted);
  EXPECT_TRUE(std::none_of(events.begin(), events.end(), [](const ArTrackLifecycleEvent& event) {
    return event.kind == ArTrackLifecycleEventKind::kDesignationDropped;
  }));
}

// lifecycle recorder：限时指令捕获超时（窗口耗尽指令作废）在作废沿产生
// kDesignationDropped（成因 kAcquisitionTimeout）；作废后指定清零不再触发。
TEST(RadarTrackLifecycleRecorderTest, DesignationTimeoutEmitsDropEventOnExpiryEdge) {
  const ArTargetInputList targets = {MakeNamedTarget(912U, "timeout")};

  ArTrackLifecycleRecorder recorder;
  // 周期 1-2：指定但未生效（未捕获，成因 kTrackNotConfirmed）——不产生事件。
  for (std::uint32_t cycle = 1U; cycle <= 2U; ++cycle) {
    ArCycleResult pending = MakeCycleResult(cycle, /*completed=*/true, {});
    pending.designation_active = false;
    pending.designated_target_id = 912U;
    pending.designation_reverted_to_tws = true;
    pending.designation_revert_reason = ArDesignationRevertReason::kTrackNotConfirmed;
    pending.effective_work_mode = config::ArWorkMode::kTws;
    const std::vector<ArTrackLifecycleEvent> events = recorder.Update(targets, pending);
    EXPECT_TRUE(std::none_of(events.begin(), events.end(), [](const ArTrackLifecycleEvent& event) {
      return event.kind == ArTrackLifecycleEventKind::kDesignationDropped;
    }));
  }

  // 周期 3：作废沿（成因 kAcquisitionTimeout，ID 保留）→ 产生 kDesignationDropped。
  ArCycleResult expiry = MakeCycleResult(3U, /*completed=*/true, {});
  expiry.designation_active = false;
  expiry.designated_target_id = 912U;
  expiry.designation_reverted_to_tws = true;
  expiry.designation_revert_reason = ArDesignationRevertReason::kAcquisitionTimeout;
  expiry.effective_work_mode = config::ArWorkMode::kTws;
  std::vector<ArTrackLifecycleEvent> events = recorder.Update(targets, expiry);
  const auto dropped = std::find_if(
      events.begin(), events.end(), [](const ArTrackLifecycleEvent& event) {
        return event.kind == ArTrackLifecycleEventKind::kDesignationDropped;
      });
  ASSERT_NE(dropped, events.end());
  EXPECT_EQ(dropped->external_target_id, 912U);
  EXPECT_EQ(dropped->designation_revert_reason, ArDesignationRevertReason::kAcquisitionTimeout);

  // 周期 4：作废后指定清零（无回退报告）→ 不再产生事件（转换沿语义）。
  ArCycleResult settled = expiry;
  settled.input_cycle_index = 4U;
  settled.designated_target_id = 0U;
  settled.designation_reverted_to_tws = false;
  settled.designation_revert_reason = ArDesignationRevertReason::kNone;
  events = recorder.Update(targets, settled);
  EXPECT_TRUE(std::none_of(events.begin(), events.end(), [](const ArTrackLifecycleEvent& event) {
    return event.kind == ArTrackLifecycleEventKind::kDesignationDropped;
  }));
}

// lifecycle recorder：显式 dwell 覆盖（designation_active=false 但非回退）不产生丢跟踪事件。
TEST(RadarTrackLifecycleRecorderTest, ExplicitDwellOverrideDoesNotEmitDropEvent) {
  const ArTargetInputList targets = {MakeNamedTarget(911U, "dwell-override")};

  ArTrackLifecycleRecorder recorder;
  // 周期 1：指定目标航迹 confirmed，但显式 dwell 覆盖使 designation_active=false、
  // reverted=false（指向按现状语义）。
  ArCycleResult overridden = MakeCycleResult(
      1U, /*completed=*/true,
      {MakeTrackSnapshot(911U, "dwell-override", session::TrackStatus::kConfirmed)});
  overridden.designation_active = false;
  overridden.designated_target_id = 911U;
  overridden.designation_reverted_to_tws = false;
  overridden.effective_work_mode = config::ArWorkMode::kStt;
  std::vector<ArTrackLifecycleEvent> events = recorder.Update(targets, overridden);
  EXPECT_TRUE(std::none_of(events.begin(), events.end(), [](const ArTrackLifecycleEvent& event) {
    return event.kind == ArTrackLifecycleEventKind::kDesignationDropped;
  }));
}

// debug view：STT 指定/回退状态字段从 L2 结果转写。
TEST(RadarTrackOutputDebugViewTest, BuildTranscribesDesignationState) {
  const ArTargetInputList targets = {MakeNamedTarget(920U, "designated")};
  ArCycleResult result = MakeCycleResult(
      7U, /*completed=*/true, {MakeTrackSnapshot(920U, "designated", session::TrackStatus::kLost)});
  result.designation_active = false;
  result.designated_target_id = 920U;
  result.designation_reverted_to_tws = true;
  result.designation_revert_reason = ArDesignationRevertReason::kTrackLost;
  result.effective_work_mode = config::ArWorkMode::kTws;

  const ArTrackOutputDebugView view = ArTrackOutputDebugViewBuilder::Build(ArCycleInput{}, result);
  EXPECT_EQ(view.effective_work_mode, config::ArWorkMode::kTws);
  EXPECT_FALSE(view.designation_active);
  EXPECT_EQ(view.designated_target_id, 920U);
  EXPECT_TRUE(view.designation_reverted_to_tws);
  EXPECT_EQ(view.designation_revert_reason, ArDesignationRevertReason::kTrackLost);

  // 限时指令捕获超时：作废沿成因 kAcquisitionTimeout 同样逐字段转写。
  result.designation_revert_reason = ArDesignationRevertReason::kAcquisitionTimeout;
  const ArTrackOutputDebugView timeout_view =
      ArTrackOutputDebugViewBuilder::Build(ArCycleInput{}, result);
  EXPECT_TRUE(timeout_view.designation_reverted_to_tws);
  EXPECT_EQ(timeout_view.designation_revert_reason, ArDesignationRevertReason::kAcquisitionTimeout);
}

}  // namespace session
}  // namespace airborne_radar
