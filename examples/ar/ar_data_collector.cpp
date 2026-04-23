// Copyright 2026. All Rights Reserved.
//
// @file ar_data_collector.cpp
// @brief AR 数据采集示例：使用 RadarTraceSession 写入 replay trace。

#include <iostream>
#include <memory>
#include <string>

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarTraceSession.h"
#include "1q/replay/ReplayTrace.h"

int main(int argc, char* argv[]) {
  namespace ar = airborne_radar;

  const std::string executable_path =
      (argc > 0 && argv != nullptr && argv[0] != nullptr) ? argv[0] : "";
  const std::size_t last_sep = executable_path.find_last_of("/\\");
  const std::string executable_dir =
      (last_sep == std::string::npos) ? "." : executable_path.substr(0, last_sep);
  const std::string trace_dir = executable_dir + "/1q-ar-data-collector-trace";

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ar-data-collector";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "example";
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  const ar::session::RadarSessionConfig config =
      ar::config::presets::MakeDetectionMissionRadarSessionConfig();
  ar::session::RadarTraceSession session(
      config, ar::session::RadarTraceSessionOptions{replay_writer, true});

  ar::session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose.attitude_deg.yaw_deg = 0.0f;
  input.platform_pose.attitude_deg.pitch_deg = 0.0f;
  input.platform_pose.attitude_deg.roll_deg = 0.0f;
  input.scene.push_back(
      ar::model::MakeTargetFromCartesian(2001U, 1500.0f, 50.0f, 100.0f, 220.0f, 0.0f, 0.0f, 1.5f));

  const ar::session::RadarCycleResult result = session.StepWithResult(input);
  replay_writer->Flush();
  std::cout << "trace_dir=" << trace_dir
            << " executed=" << (result.executed_this_cycle ? "true" : "false")
            << " published=" << result.track_output_frame.published_track_count << std::endl;
  return 0;
}
