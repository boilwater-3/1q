// Copyright 2026. All Rights Reserved.
//
// @file ar_quick_start.cpp
// @brief AR 最小接入示例：外部 ECEF 输入 + 会话一步一帧调用。

#include <iostream>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"
#include "1q/foundation/coordinate_transform.h"

int main() {
  namespace aq = airborne_radar;
  namespace geo = oneq::foundation;

  // 1) 用预设配置创建会话（推荐入口）。
  const aq::session::RadarSessionConfig session_config =
      aq::config::MakeDetectionMissionRadarSessionConfig();
  aq::session::RadarSession session = aq::session::RadarSessionFactory::Create(session_config);

  // 2) 外部已知：雷达位置与目标位置均为 ECEF（m）。
  geo::EcefCoordinateM radar_ecef;
  radar_ecef.x_m = -2180214.6;
  radar_ecef.y_m = 4380183.2;
  radar_ecef.z_m = 4071245.9;
  geo::EcefCoordinateM target_ecef;
  target_ecef.x_m = -2180030.0;
  target_ecef.y_m = 4380270.0;
  target_ecef.z_m = 4071530.0;

  // 3) 外部也可直接提供目标 ECEF 速度（m/s）。
  geo::Vector3f target_velocity_ecef_mps;
  target_velocity_ecef_mps.x = 120.0f;
  target_velocity_ecef_mps.y = -70.0f;
  target_velocity_ecef_mps.z = 30.0f;

  // 4) 统一入口：雷达 ECEF 位置 + 目标 ECEF 位置 + 指定速度参考系 -> TargetFeature。
  aq::session::TargetExternalKinematicsInput external_input;
  external_input.radar_position_ecef_m = radar_ecef;
  external_input.target_position_ecef_m = target_ecef;
  external_input.target_velocity_mps = target_velocity_ecef_mps;
  external_input.target_velocity_frame = aq::session::VelocityFrame::kEcef;
  external_input.platform_attitude_deg = session_config.beam_control.platform_attitude_deg;
  external_input.radar_mount_angles_deg =
      session_config.beam_control.radar_orientation.mount_angles_deg;
  external_input.rcs = 1.8f;
  external_input.swerling_type = 0;

  aq::model::TargetFeature target;
  if (!aq::session::TryMakeTargetFromExternalKinematics(1001U, external_input, &target)) {
    std::cerr << "failed to build target from external kinematics input" << std::endl;
    return 1;
  }

  // 5) 准备一个周期输入。
  aq::session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  input.target_features.push_back(target);

  // 6) 接入层建议先做输入校验。
  const auto issues = aq::session::ValidateRadarCycleInput(input);
  if (aq::session::HasValidationError(issues)) {
    std::cerr << "invalid radar input" << std::endl;
    return 1;
  }

  // 7) 执行一个周期并读取结果。
  const aq::session::RadarCycleResult result = session.StepWithResult(input);
  const std::size_t confirmed =
      aq::output::CollectConfirmedTracks(result.track_output_frame).size();

  std::cout << "published=" << result.track_output_frame.published_track_count
            << " confirmed=" << confirmed
            << " match_rate=" << result.association_quality_metrics.match_rate << std::endl;
  return 0;
}
