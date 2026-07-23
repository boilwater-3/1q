/**
 * @file ar_output_boundary_contract_test.cpp
 * @brief AR 真实系统输出边界合同测试。
 *
 * AR 的边界与 EOS 不同：track 是系统估计（非传感器原始输出），target_name
 * 作为人读标签直接附在 track 上是允许的（不像 EOS 需要独立 attribution 层）。
 * 本合同锁定的边界是：
 * - external_target_id 是唯一稳定关联键；target_name 不参与关联。
 * - 同名不同 ID 是合法输入，不会互相干扰。
 * - name 可经 ArTrackOutputDebugViewBuilder 回填。
 * - 未提供 name（空字符串）不影响 track 关联与输出。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"

namespace airborne_radar {
namespace session {
namespace {

ArSceneTarget MakeTarget(std::uint64_t id, const std::string& name) {
  ArSceneTarget target;
  target.external_target_id = id;
  target.target_name = name;
  target.rcs = 1.0f;
  return target;
}

session::TrackStateSnapshot MakeSnapshot(std::uint64_t id, const std::string& name,
                                       session::TrackStatus status) {
  session::TrackStateSnapshot snapshot;
  snapshot.external_target_id = id;
  snapshot.target_name = name;
  snapshot.status = status;
  snapshot.association_key = id;
  return snapshot;
}

ArCycleResult MakeResult(std::uint64_t cycle_index,
                         std::vector<session::TrackStateSnapshot> tracks) {
  ArCycleResult result;
  result.status = ArCycleStatus::kCompleted;
  result.input_cycle_index = cycle_index;
  result.track_output_frame.cycle_index = cycle_index;
  result.track_output_frame.tracks = std::move(tracks);
  return result;
}

}  // namespace

// 合同：同名不同 ID 是合法输入，两个 track 互不干扰，name 各自回填。
// 这锁定 name 不参与关联——若 name 误入关联键，同名目标会冲突。
TEST(RarOutputBoundaryContractTest, SameNameDifferentIdDoesNotBreakAssociation) {
  // 两个同名目标，不同 ID。
  const ArSceneTargetList targets = {MakeTarget(1001U, "shared-name"),
                                     MakeTarget(1002U, "shared-name")};

  ArCycleResult result =
      MakeResult(1U, {MakeSnapshot(1001U, "shared-name", session::TrackStatus::kConfirmed),
                      MakeSnapshot(1002U, "shared-name", session::TrackStatus::kConfirmed)});

  const ArTrackOutputDebugView view = ArTrackOutputDebugViewBuilder::Build(targets, result);
  ASSERT_EQ(view.tracks.size(), 2U);
  // 两个同名目标各自关联到不同 ID 的 track，不互相吞并。
  EXPECT_EQ(view.tracks[0].external_target_id, 1001U);
  EXPECT_TRUE(view.tracks[0].has_track);
  EXPECT_EQ(view.tracks[1].external_target_id, 1002U);
  EXPECT_TRUE(view.tracks[1].has_track);
}

// 合同：空 name 不影响 track 关联与 debug view 回填（name 是可选标签）。
TEST(RarOutputBoundaryContractTest, EmptyNameDoesNotAffectAssociation) {
  const ArSceneTargetList targets = {MakeTarget(2001U, "")};

  ArCycleResult result =
      MakeResult(1U, {MakeSnapshot(2001U, "", session::TrackStatus::kConfirmed)});

  const ArTrackOutputDebugView view = ArTrackOutputDebugViewBuilder::Build(targets, result);
  ASSERT_EQ(view.tracks.size(), 1U);
  EXPECT_TRUE(view.tracks[0].has_track);
  EXPECT_TRUE(view.tracks[0].target_name.empty());
}

}  // namespace session
}  // namespace airborne_radar
