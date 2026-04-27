// Copyright 2026. All Rights Reserved.
//
// @file ar_radar_session.cpp
// @brief 演示如何使用 RadarSession 驱动三周期机载雷达探测流程。
//
// 本文件同时展示按平台硬件参数定制会话配置：
//   - 以探测任务预设为基础，叠加平台发射机、天线、接收机参数
//   - 启用物理层雷达方程检测（enable_physics_detection）
//   - 设置平台干扰判定灵敏度
//
// 配置方式：以 preset 为基线，直接覆盖四域中的具体字段。

#include <iostream>

#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/session/RadarSceneTargetUtils.h"
#include "1q/airborne_radar/output/TrackOutputQueries.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

int main() {
  namespace aq = airborne_radar::session;
  using airborne_radar::environment::EnvironmentSceneBuilder;
  using airborne_radar::environment::JammerEmitterState;
  using airborne_radar::environment::JammingTechnique;
  using airborne_radar::output::CollectConfirmedTracks;
  using airborne_radar::output::CollectJammingTracks;
  using airborne_radar::session::HasValidationError;
  using airborne_radar::session::RadarCycleInput;
  using airborne_radar::session::RadarCycleResult;
  using airborne_radar::session::RadarSession;
  using airborne_radar::session::RadarSessionFactory;
  using airborne_radar::session::ValidateRadarCycleInput;

  // RadarSession 托管默认的 Context / Signal / Environment / Controller 链路，
  // 外部调用方只需要按周期喂输入并读取输出。
  //
  // 以探测任务预设为基础，覆盖平台雷达硬件参数：
  //   - hardware.detection：探测域参数（发射机、天线、接收机、物理检测开关等）
  //   - environment：环境默认配置（干扰判定阈值等）
  //
  // 若平台暂无硬件参数，可退回到最简形式：
  //   RadarSession session =
  //       RadarSessionFactory::Create(airborne_radar::config::presets::MakeDetectionMissionRadarSessionConfig());
  auto config = airborne_radar::config::presets::MakeDetectionMissionRadarSessionConfig();
  config.hardware.detection.enable_physics_detection = true;
  config.hardware.detection.transmitter.peak_power_w = 5e6f;
  config.hardware.detection.transmitter.frequency_hz = 9.3e9f;
  config.hardware.detection.transmitter.bandwidth_hz = 10e6f;
  config.hardware.detection.transmitter.pulse_width_s = 20e-6f;
  config.hardware.detection.transmitter.prf_hz = 500.0f;
  config.hardware.detection.antenna.main_beam_gain_db = 38.0f;
  config.hardware.detection.receiver.noise_figure_db = 3.5f;
  config.environment.jamming_sensitivity_profile = environment::ResolveJammingSensitivityProfile(5.0f);

  RadarSession session = RadarSessionFactory::Create(config);

  // 周期 1：构造两批空中目标并执行一次无干扰探测。
  RadarCycleInput cycle_1;
  cycle_1.dt_sec = 1.0f;
  cycle_1.scene.push_back(
      aq::MakeSceneTarget(1001U, 180.0f, -5.0f, 18.0f, 65.0f, 0.0f, 0.0f, 1.0f));
  cycle_1.scene.push_back(
      aq::MakeSceneTarget(1002U, 260.0f, 8.0f, 22.0f, 82.0f, 1.0f, 0.0f, 1.1f));

  const std::vector<airborne_radar::session::ValidationIssue> cycle_1_issues =
      ValidateRadarCycleInput(cycle_1);
  if (HasValidationError(cycle_1_issues)) {
    std::cerr << "cycle_1 input validation failed" << std::endl;
    return 1;
  }
  const RadarCycleResult result_1 = session.StepWithResult(cycle_1);
  std::cout << "cycle_1 published=" << result_1.track_output_frame.published_track_count
            << " confirmed=" << CollectConfirmedTracks(result_1.track_output_frame).size()
            << " commands=" << result_1.submitted_commands.size() << std::endl;

  // 周期 2：调用方按自己的平台/目标模型推进相对位置后继续执行。
  RadarCycleInput cycle_2 = cycle_1;
  for (std::size_t i = 0; i < cycle_2.scene.size(); ++i) {
    cycle_2.scene[i].position_x +=
        cycle_2.scene[i].velocity_x * cycle_2.dt_sec;
  }

  const RadarCycleResult result_2 = session.StepWithResult(cycle_2);
  std::cout << "cycle_2 published=" << result_2.track_output_frame.published_track_count
            << " confirmed=" << CollectConfirmedTracks(result_2.track_output_frame).size()
            << " match_rate=" << result_2.association_quality_metrics.match_rate << std::endl;

  // 周期 3：在 step 前提交场景更新，演示"当前周期切入干扰"的高层用法。
  RadarCycleInput cycle_3 = cycle_2;
  for (std::size_t i = 0; i < cycle_3.scene.size(); ++i) {
    cycle_3.scene[i].position_x +=
        cycle_3.scene[i].velocity_x * cycle_3.dt_sec;
  }

  JammerEmitterState noise_jammer;
  noise_jammer.technique = JammingTechnique::kNoiseSuppression;
  noise_jammer.power_db = 12.0f;
  noise_jammer.js_db = 8.0f;
  noise_jammer.has_direction_deg = true;
  noise_jammer.azimuth_deg = 20.0f;
  noise_jammer.elevation_deg = 1.0f;
  noise_jammer.angular_span_deg = 10.0f;

  const airborne_radar::environment::EnvironmentSceneState jammed_scene =
      EnvironmentSceneBuilder().AddNoiseJammer(noise_jammer).Build();
  const RadarCycleResult result_3 = session.StepWithResult(cycle_3, jammed_scene);

  // 输出查询 helper 适合外部模块快速按状态或干扰标记获取敌情集合。
  const std::size_t confirmed_count = CollectConfirmedTracks(result_3.track_output_frame).size();
  const std::size_t jamming_count = CollectJammingTracks(result_3.track_output_frame).size();

  std::cout << "cycle_3 published=" << result_3.track_output_frame.published_track_count
            << " confirmed=" << confirmed_count << " jamming_tracks=" << jamming_count
            << " commands=" << result_3.submitted_commands.size() << std::endl;
  return 0;
}
