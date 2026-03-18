// Copyright 2026. All Rights Reserved.
//
// @file example_radar_session.cpp
// @brief 演示如何使用 RadarSession 驱动三周期机载雷达探测流程。

#include <iostream>

#include "1q/airborne_radar/common/ConfigPresets.h"
#include "1q/airborne_radar/common/TargetFeatureUtils.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/airborne_radar/core/context/RadarInputValidation.h"
#include "1q/airborne_radar/core/output/TrackOutputQueries.h"
#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"

namespace {

} // namespace

int main() {
  using airborne_radar::common::MakeDetectionMissionRadarSessionConfig;
  using airborne_radar::common::MakeAirTarget;
  using airborne_radar::core::context::HasValidationError;
  using airborne_radar::core::context::RadarCycleInput;
  using airborne_radar::core::context::ValidateRadarCycleInput;
  using airborne_radar::core::output::CollectConfirmedTracks;
  using airborne_radar::core::output::CollectJammingTracks;
  using airborne_radar::core::session::RadarSession;
  using airborne_radar::core::session::RadarCycleResult;
  using airborne_radar::environment::EnvironmentSceneBuilder;

  // RadarSession 托管默认的 Context / Signal / Environment / Controller 链路，
  // 外部调用方只需要按周期喂输入并读取输出。
  RadarSession session(MakeDetectionMissionRadarSessionConfig());

  // 周期 1：构造两批空中目标并执行一次无干扰探测。
  RadarCycleInput cycle_1;
  cycle_1.dt_sec = 1.0f;
  cycle_1.target_features.push_back(
      MakeAirTarget(1001U, 180.0f, -5.0f, 18.0f, 65.0f, 0.0f, 0.0f, 1.0f));
  cycle_1.target_features.push_back(
      MakeAirTarget(1002U, 260.0f, 8.0f, 22.0f, 82.0f, 1.0f, 0.0f, 1.1f));

  const std::vector<airborne_radar::core::context::ValidationIssue>
      cycle_1_issues = ValidateRadarCycleInput(cycle_1);
  if (HasValidationError(cycle_1_issues)) {
    std::cerr << "cycle_1 input validation failed" << std::endl;
    return 1;
  }
  const RadarCycleResult result_1 = session.StepWithResult(cycle_1);
  std::cout << "cycle_1 published="
            << result_1.track_output_frame.published_track_count
            << " confirmed="
            << CollectConfirmedTracks(result_1.track_output_frame).size()
            << " commands=" << result_1.submitted_commands.size()
            << std::endl;

  // 周期 2：调用方按自己的平台/目标模型推进相对位置后继续执行。
  RadarCycleInput cycle_2 = cycle_1;
  for (std::size_t i = 0; i < cycle_2.target_features.size(); ++i) {
    cycle_2.target_features[i].position_x +=
        cycle_2.target_features[i].current_track_velocity_x * cycle_2.dt_sec;
  }

  const RadarCycleResult result_2 = session.StepWithResult(cycle_2);
  std::cout << "cycle_2 published="
            << result_2.track_output_frame.published_track_count
            << " confirmed="
            << CollectConfirmedTracks(result_2.track_output_frame).size()
            << " match_rate="
            << result_2.association_quality_metrics.match_rate << std::endl;

  // 周期 3：在 step 前提交场景更新，演示“当前周期切入干扰”的高层用法。
  RadarCycleInput cycle_3 = cycle_2;
  for (std::size_t i = 0; i < cycle_3.target_features.size(); ++i) {
    cycle_3.target_features[i].position_x +=
        cycle_3.target_features[i].current_track_velocity_x * cycle_3.dt_sec;
  }

  const airborne_radar::environment::EnvironmentSceneState jammed_scene =
      EnvironmentSceneBuilder()
          .AddNoiseJammer(12.0f, 8.0f, 0.25f, 0.10f, true)
          .Build();
  const RadarCycleResult result_3 = session.StepWithResult(cycle_3, jammed_scene);

  // 输出查询 helper 适合外部模块快速按状态或干扰标记获取敌情集合。
  const std::size_t confirmed_count =
      CollectConfirmedTracks(result_3.track_output_frame).size();
  const std::size_t jamming_count =
      CollectJammingTracks(result_3.track_output_frame).size();

  std::cout << "cycle_3 published="
            << result_3.track_output_frame.published_track_count
            << " confirmed=" << confirmed_count
            << " jamming_tracks=" << jamming_count
            << " commands=" << result_3.submitted_commands.size() << std::endl;
  return 0;
}
