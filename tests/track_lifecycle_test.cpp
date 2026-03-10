// Copyright 2026. All Rights Reserved.
//
// Description: 验证轨迹对象池与生命周期管理的基础状态机行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/TrackTypes.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"

namespace airborne_radar { namespace tests {

TEST(TrackLifecycleManagerTest, ConfirmsTrackAfterConfiguredHits) {
  signal::tracking::BoostTrackPool pool(4, 16);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 2;
  config.max_miss_before_lost = 1;
  config.max_lost_cycles = 2;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement measurement;
  measurement.association_key = 7;
  measurement.velocity = Eigen::Vector3f(3.0f, 4.0f, 0.0f);
  measurement.acceleration = Eigen::Vector3f(0.0f, 2.0f, 0.0f);
  measurement.rcs = 1.5f;

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1;
  cycle_1.batch_id = 1001;
  manager.Update(cycle_1, {measurement});

  auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kTentative);

  signal::tracking::CycleContext cycle_2;
  cycle_2.cycle_index = 2;
  cycle_2.batch_id = 1002;
  manager.Update(cycle_2, {measurement});

  active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kConfirmed);

  const common::TargetFeatureList snapshot = manager.BuildFeatureSnapshot();
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot[0].current_track_speed, 5.0f);
  EXPECT_FLOAT_EQ(snapshot[0].current_track_acceleration, 2.0f);
  EXPECT_FLOAT_EQ(snapshot[0].current_track_rcs, 1.5f);
}

TEST(TrackLifecycleManagerTest, RecyclesTrackAfterLostTimeout) {
  signal::tracking::BoostTrackPool pool(2, 8);
  signal::tracking::LifecycleConfig config;
  config.confirm_hits = 1;
  config.max_miss_before_lost = 0;
  config.max_lost_cycles = 1;

  signal::tracking::TrackLifecycleManager manager(pool, config);

  signal::tracking::TrackMeasurement measurement;
  measurement.association_key = 3;
  measurement.velocity = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

  signal::tracking::CycleContext cycle_1;
  cycle_1.cycle_index = 1;
  cycle_1.batch_id = 2001;
  manager.Update(cycle_1, {measurement});

  signal::tracking::CycleContext cycle_2;
  cycle_2.cycle_index = 2;
  cycle_2.batch_id = 2002;
  manager.Update(cycle_2, {});

  auto active_tracks = manager.GetActiveTracks();
  ASSERT_EQ(active_tracks.size(), 1u);
  EXPECT_EQ(active_tracks[0]->status, common::TrackStatus::kLost);

  signal::tracking::CycleContext cycle_3;
  cycle_3.cycle_index = 3;
  cycle_3.batch_id = 2003;
  manager.Update(cycle_3, {});

  active_tracks = manager.GetActiveTracks();
  EXPECT_TRUE(active_tracks.empty());

  const common::TargetFeatureList snapshot = manager.BuildFeatureSnapshot();
  EXPECT_TRUE(snapshot.empty());
}

} } // namespace airborne_radar::tests
