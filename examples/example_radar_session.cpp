// Copyright 2026. All Rights Reserved.
//
// @file example_radar_session.cpp
// @brief 演示如何使用 RadarSession 驱动三周期机载雷达探测流程。

#include <iostream>

#include "1q/airborne_radar/common/TargetFeatureUtils.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/airborne_radar/core/output/TrackOutputQueries.h"
#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"

namespace {

airborne_radar::core::session::RadarSessionConfig MakeExampleSessionConfig() {
  airborne_radar::core::session::RadarSessionConfig config;
  config.signal_pipeline_config.detection.min_detection_margin_db = -100.0f;
  config.signal_pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
  config.signal_pipeline_config.lifecycle.lifecycle_config.confirm_hits = 1U;
  config.signal_pipeline_config.tracking.kalman_measurement_noise_std = 1.0f;
  return config;
}

} // namespace

int main() {
  using airborne_radar::common::MakeAirTarget;
  using airborne_radar::core::context::RadarCycleInput;
  using airborne_radar::core::output::BuildTrackMapByExternalTargetId;
  using airborne_radar::core::output::CountJammingTracks;
  using airborne_radar::core::session::RadarSession;
  using airborne_radar::environment::EnvironmentSceneBuilder;

  RadarSession session(MakeExampleSessionConfig());

  RadarCycleInput cycle_1;
  cycle_1.dt_sec = 1.0f;
  cycle_1.target_features.push_back(
      MakeAirTarget(1001U, 180.0f, -5.0f, 18.0f, 65.0f, 0.0f, 0.0f, 1.0f));
  cycle_1.target_features.push_back(
      MakeAirTarget(1002U, 260.0f, 8.0f, 22.0f, 82.0f, 1.0f, 0.0f, 1.1f));

  const airborne_radar::common::TrackOutputFrame frame_1 = session.Step(cycle_1);
  std::cout << "cycle_1 published=" << frame_1.published_track_count
            << " jamming_tracks=" << CountJammingTracks(frame_1) << std::endl;

  RadarCycleInput cycle_2 = cycle_1;
  for (std::size_t i = 0; i < cycle_2.target_features.size(); ++i) {
    cycle_2.target_features[i].position_x +=
        cycle_2.target_features[i].current_track_velocity_x * cycle_2.dt_sec;
  }

  const airborne_radar::common::TrackOutputFrame frame_2 = session.Step(cycle_2);
  std::cout << "cycle_2 published=" << frame_2.published_track_count
            << " jamming_tracks=" << CountJammingTracks(frame_2) << std::endl;

  RadarCycleInput cycle_3 = cycle_2;
  for (std::size_t i = 0; i < cycle_3.target_features.size(); ++i) {
    cycle_3.target_features[i].position_x +=
        cycle_3.target_features[i].current_track_velocity_x * cycle_3.dt_sec;
  }

  const airborne_radar::environment::EnvironmentSceneState jammed_scene =
      EnvironmentSceneBuilder()
          .AddNoiseJammer(12.0f, 8.0f, 0.25f, 0.10f, true)
          .Build();
  const airborne_radar::common::TrackOutputFrame frame_3 =
      session.Step(cycle_3, jammed_scene);
  const auto track_map = BuildTrackMapByExternalTargetId(frame_3);

  std::cout << "cycle_3 published=" << frame_3.published_track_count
            << " jamming_tracks=" << CountJammingTracks(frame_3)
            << " known_ids=" << track_map.size() << std::endl;
  return 0;
}
