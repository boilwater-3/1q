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

}  // namespace session
}  // namespace airborne_radar
