// Copyright 2026. All Rights Reserved.
//
// @file data_output_manager_test.cpp
// @brief 验证数据输出管理模块的输出帧与决策输入装配行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/model/DecisionInputFrame.h"
#include "1q/airborne_radar/common/model/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "airborne_radar/signal/output/DataOutputManager.h"

namespace airborne_radar {
namespace tests {

namespace {

common::model::DecisionTrackSnapshot BuildTrackSnapshot(std::uint64_t association_key,
                                                 common::model::DecisionTrackStatus status) {
  common::model::DecisionTrackSnapshot snapshot(120.0f, 0.0f, 0.0f, 3.0f);
  snapshot.state.association_key = association_key;
  snapshot.state.status = status;
  return snapshot;
}

}  // namespace

TEST(DataOutputManagerTest, BuildsTrackOutputFrameWithCountsAndLostFlag) {
  signal::output::DataOutputManager output_manager;
  common::model::DecisionTrackSnapshotList track_snapshots;
  track_snapshots.push_back(BuildTrackSnapshot(101U, common::model::DecisionTrackStatus::kConfirmed));
  track_snapshots.push_back(BuildTrackSnapshot(202U, common::model::DecisionTrackStatus::kLost));

  const common::output::TrackOutputFrame frame =
      output_manager.BuildTrackOutputFrame(7U, 88U, track_snapshots);

  EXPECT_EQ(frame.cycle_index, 7U);
  EXPECT_EQ(frame.batch_id, 88U);
  ASSERT_EQ(frame.tracks.size(), 2U);
  EXPECT_EQ(frame.published_track_count, 2U);
  EXPECT_EQ(frame.confirmed_track_count, 1U);
  EXPECT_TRUE(frame.contains_lost_tracks);
}

TEST(DataOutputManagerTest, BuildsDecisionInputFrameFromTrackOutputFrame) {
  signal::output::DataOutputManager output_manager;
  common::model::DecisionTrackSnapshotList track_snapshots;
  track_snapshots.push_back(BuildTrackSnapshot(303U, common::model::DecisionTrackStatus::kConfirmed));
  const common::output::TrackOutputFrame track_output_frame =
      output_manager.BuildTrackOutputFrame(11U, 99U, track_snapshots);

  common::model::EccmSourceInfo eccm_source_info;
  eccm_source_info.has_jamming_signal = true;
  common::model::AssociationQualityInfo association_quality_info;
  association_quality_info.association_stress = 0.35f;
  common::model::PerceptionQualityInfo perception_quality_info;
  perception_quality_info.input_target_count = 3U;
  perception_quality_info.detection_count = 2U;

  const common::model::DecisionInputFrame frame = output_manager.BuildDecisionInputFrame(
      track_output_frame, eccm_source_info, association_quality_info, perception_quality_info);

  EXPECT_EQ(frame.cycle_index, 11U);
  EXPECT_EQ(frame.batch_id, 99U);
  EXPECT_TRUE(frame.environment_jamming_detected);
  EXPECT_FLOAT_EQ(frame.association_quality_info.association_stress, 0.35f);
  EXPECT_EQ(frame.perception_quality_info.input_target_count, 3U);
  ASSERT_EQ(frame.tracks.size(), 1U);
  EXPECT_EQ(frame.tracks[0].state.association_key, 303U);
  EXPECT_EQ(frame.tracks[0].state.status, common::model::DecisionTrackStatus::kConfirmed);
}

TEST(DataOutputManagerTest, EmptyTrackOutputFrameKeepsZeroCounts) {
  signal::output::DataOutputManager output_manager;
  const common::output::TrackOutputFrame frame =
      output_manager.BuildTrackOutputFrame(5U, 12U, common::model::DecisionTrackSnapshotList());

  EXPECT_EQ(frame.cycle_index, 5U);
  EXPECT_EQ(frame.batch_id, 12U);
  EXPECT_TRUE(frame.tracks.empty());
  EXPECT_EQ(frame.published_track_count, 0U);
  EXPECT_EQ(frame.confirmed_track_count, 0U);
  EXPECT_FALSE(frame.contains_lost_tracks);
}

}  // namespace tests
}  // namespace airborne_radar
