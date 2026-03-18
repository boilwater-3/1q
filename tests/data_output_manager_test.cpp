// Copyright 2026. All Rights Reserved.
//
// Description: 验证数据输出管理模块的输出帧与决策输入装配行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "airborne_radar/core/output/DataOutputManager.h"

namespace airborne_radar {
namespace tests {

namespace {

common::DecisionTrackSnapshot BuildTrackSnapshot(
    std::uint64_t association_key, common::DecisionTrackStatus status) {
  common::DecisionTrackSnapshot snapshot(120.0f, 0.0f, 0.0f, 3.0f);
  snapshot.state.association_key = association_key;
  snapshot.state.status = status;
  return snapshot;
}

} // namespace

TEST(DataOutputManagerTest, BuildsTrackOutputFrameWithCountsAndLostFlag) {
  core::output::DataOutputManager output_manager;
  common::DecisionTrackSnapshotList track_snapshots;
  track_snapshots.push_back(
      BuildTrackSnapshot(101U, common::DecisionTrackStatus::kConfirmed));
  track_snapshots.push_back(
      BuildTrackSnapshot(202U, common::DecisionTrackStatus::kLost));

  const common::TrackOutputFrame frame =
      output_manager.BuildTrackOutputFrame(7U, 88U, track_snapshots);

  EXPECT_EQ(frame.cycle_index, 7U);
  EXPECT_EQ(frame.batch_id, 88U);
  ASSERT_EQ(frame.tracks.size(), 2U);
  EXPECT_EQ(frame.published_track_count, 2U);
  EXPECT_EQ(frame.confirmed_track_count, 1U);
  EXPECT_TRUE(frame.contains_lost_tracks);
}

TEST(DataOutputManagerTest, BuildsDecisionInputFrameFromTrackOutputFrame) {
  core::output::DataOutputManager output_manager;
  common::DecisionTrackSnapshotList track_snapshots;
  track_snapshots.push_back(
      BuildTrackSnapshot(303U, common::DecisionTrackStatus::kConfirmed));
  const common::TrackOutputFrame track_output_frame =
      output_manager.BuildTrackOutputFrame(11U, 99U, track_snapshots);

  common::EccmSourceInfo eccm_source_info;
  eccm_source_info.has_jamming_signal = true;
  common::AssociationQualityInfo association_quality_info;
  association_quality_info.association_stress = 0.35f;
  common::PerceptionQualityInfo perception_quality_info;
  perception_quality_info.input_target_count = 3U;
  perception_quality_info.detection_count = 2U;

  const common::DecisionInputFrame frame =
      output_manager.BuildDecisionInputFrame(
          track_output_frame, eccm_source_info, association_quality_info,
          perception_quality_info);

  EXPECT_EQ(frame.cycle_index, 11U);
  EXPECT_EQ(frame.batch_id, 99U);
  EXPECT_TRUE(frame.environment_jamming_detected);
  EXPECT_FLOAT_EQ(frame.association_quality_info.association_stress, 0.35f);
  EXPECT_EQ(frame.perception_quality_info.input_target_count, 3U);
  ASSERT_EQ(frame.tracks.size(), 1U);
  EXPECT_EQ(frame.tracks[0].state.association_key, 303U);
  EXPECT_EQ(frame.tracks[0].state.status, common::DecisionTrackStatus::kConfirmed);
}

TEST(DataOutputManagerTest, EmptyTrackOutputFrameKeepsZeroCounts) {
  core::output::DataOutputManager output_manager;
  const common::TrackOutputFrame frame =
      output_manager.BuildTrackOutputFrame(
          5U, 12U, common::DecisionTrackSnapshotList());

  EXPECT_EQ(frame.cycle_index, 5U);
  EXPECT_EQ(frame.batch_id, 12U);
  EXPECT_TRUE(frame.tracks.empty());
  EXPECT_EQ(frame.published_track_count, 0U);
  EXPECT_EQ(frame.confirmed_track_count, 0U);
  EXPECT_FALSE(frame.contains_lost_tracks);
}

} // namespace tests
} // namespace airborne_radar
