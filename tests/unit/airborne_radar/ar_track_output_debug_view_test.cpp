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

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"

namespace airborne_radar {
namespace session {
namespace {

namespace ar_model = ::airborne_radar::session;

ArSceneTarget MakeNamedTarget(std::uint64_t id, const std::string& name) {
  ArSceneTarget target;
  target.external_target_id = id;
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

ArCompleteCycleResult MakeCycleResult(std::uint64_t cycle_index, bool completed,
                                      std::vector<session::TrackStateSnapshot> tracks) {
  ArCompleteCycleResult result;
  result.status = completed ? ArCompleteCycleStatus::kCompleted : ArCompleteCycleStatus::kRejected;
  result.world_cycle_index = cycle_index;
  result.track_output_frame.cycle_index = cycle_index;
  result.track_output_frame.tracks = std::move(tracks);
  return result;
}

}  // namespace

// debug view：把 track 输出 + 输入目标表合成；name 经 external_target_id 回填。
TEST(RadarTrackOutputDebugViewTest, BuildMapsTracksBackToNamedTargets) {
  const ArSceneTargetList targets = {MakeNamedTarget(701U, "alpha"), MakeNamedTarget(702U, "beta"),
                                     MakeNamedTarget(0U, "no-id")};

  ArCompleteCycleResult result =
      MakeCycleResult(5U, /*executed=*/true,
                      {MakeTrackSnapshot(701U, "alpha", session::TrackStatus::kConfirmed),
                       MakeTrackSnapshot(702U, "beta", session::TrackStatus::kTentative)});

  const ArTrackOutputDebugView view = ArTrackOutputDebugViewBuilder::Build(targets, result);
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
}

// debug view：未完成周期时所有目标标记为 kCycleNotCompleted。
TEST(RadarTrackOutputDebugViewTest, NonCompletedCycleMarksAllTargetsAsNonCompleted) {
  const ArSceneTargetList targets = {MakeNamedTarget(701U, "alpha")};

  ArCompleteCycleResult result = MakeCycleResult(9U, /*completed=*/false, {});
  const ArTrackOutputDebugView view = ArTrackOutputDebugViewBuilder::Build(targets, result);
  ASSERT_EQ(view.tracks.size(), 1U);
  EXPECT_EQ(view.tracks[0].status, ArDebugTrackStatus::kCycleNotCompleted);
  EXPECT_FALSE(view.tracks[0].has_track);
}

// lifecycle recorder：首次确认 → 更新 → 丢失；未跟踪默认不产生事件。
TEST(RadarTrackLifecycleRecorderTest, TracksFirstConfirmedUpdatedLost) {
  const ArSceneTargetList targets = {MakeNamedTarget(800U, "gamma")};

  ArTrackLifecycleRecorder recorder;

  // 周期 1：首次确认。
  ArCompleteCycleResult first = MakeCycleResult(
      1U, /*completed=*/true, {MakeTrackSnapshot(800U, "gamma", session::TrackStatus::kConfirmed)});
  std::vector<ArTrackLifecycleEvent> events = recorder.Update(targets, first);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArTrackLifecycleEventKind::kFirstConfirmed);
  EXPECT_EQ(events[0].target_name, "gamma");

  // 周期 2：已确认 → 更新。
  ArCompleteCycleResult second = MakeCycleResult(
      2U, /*completed=*/true, {MakeTrackSnapshot(800U, "gamma", session::TrackStatus::kConfirmed)});
  events = recorder.Update(targets, second);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArTrackLifecycleEventKind::kUpdated);

  // 周期 3：丢失。
  ArCompleteCycleResult lost = MakeCycleResult(
      3U, /*completed=*/true, {MakeTrackSnapshot(800U, "gamma", session::TrackStatus::kLost)});
  events = recorder.Update(targets, lost);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, ArTrackLifecycleEventKind::kLost);
}

TEST(RadarTrackLifecycleRecorderTest, NonCompletedCyclePreservesConfirmedState) {
  const ArSceneTargetList targets = {MakeNamedTarget(801U, "recoverable")};
  ArTrackLifecycleRecorder recorder(ArTrackLifecycleRecorderConfig{true});
  ArCompleteCycleResult confirmed = MakeCycleResult(
      1U, true, {MakeTrackSnapshot(801U, "recoverable", session::TrackStatus::kConfirmed)});
  ASSERT_EQ(recorder.Update(targets, confirmed).front().kind,
            ArTrackLifecycleEventKind::kFirstConfirmed);
  ArCompleteCycleResult rejected = MakeCycleResult(2U, false, {});
  EXPECT_TRUE(recorder.Update(targets, rejected).empty());
  confirmed.world_cycle_index = 3U;
  const std::vector<ArTrackLifecycleEvent> recovered = recorder.Update(targets, confirmed);
  ASSERT_EQ(recovered.size(), 1U);
  EXPECT_EQ(recovered.front().kind, ArTrackLifecycleEventKind::kUpdated);
}

// lifecycle recorder：未跟踪目标默认不产生事件；开启后产生 kNotTracked。
TEST(RadarTrackLifecycleRecorderTest, NotTrackedEventsRequireExplicitEnable) {
  const ArSceneTargetList targets = {MakeNamedTarget(900U, "delta")};

  // 默认配置：无 track 的目标不产生事件。
  ArTrackLifecycleRecorder recorder;
  ArCompleteCycleResult no_track = MakeCycleResult(1U, /*completed=*/true, {});
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
  const ArSceneTargetList targets = {MakeNamedTarget(0U, "no-id")};

  ArTrackLifecycleRecorder diagnose_recorder(ArTrackLifecycleRecorderConfig{true});
  ArCompleteCycleResult result = MakeCycleResult(1U, /*completed=*/true, {});
  std::vector<ArTrackLifecycleEvent> events = diagnose_recorder.Update(targets, result);
  EXPECT_TRUE(events.empty());
}

}  // namespace session
}  // namespace airborne_radar
