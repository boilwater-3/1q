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

  // 1) 创建会话。
  const aq::session::RadarSessionConfig session_config =
      aq::config::presets::MakeDetectionMissionRadarSessionConfig();
  aq::session::RadarSession session = aq::session::RadarSessionFactory::Create(session_config);

  // 2) 外部平台信息（位置、速度）。
  //    这里先用 LLA 构造样例，再统一转换为 ECEF，模拟外部系统直接给出地固坐标。
  geo::LlaCoordinateDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1200.0;
  geo::EcefCoordinateM platform_ecef;
  if (!geo::TryLlaToEcef(platform_lla, &platform_ecef)) {
    std::cerr << "invalid platform lla" << std::endl;
    return 1;
  }
  geo::Vector3f platform_velocity_ecef_mps;
  platform_velocity_ecef_mps.x = 80.0f;
  platform_velocity_ecef_mps.y = 0.0f;
  platform_velocity_ecef_mps.z = 0.0f;

  // 3) 目标信息（位置、速度、RCS）。
  //    AR 的目标速度按外部绝对速度输入，适配器内部会换算为相对雷达速度。
  geo::LlaCoordinateDegM target_lla = platform_lla;
  target_lla.longitude_deg += 0.003;
  target_lla.altitude_m = 1500.0;
  geo::EcefCoordinateM target_ecef;
  if (!geo::TryLlaToEcef(target_lla, &target_ecef)) {
    std::cerr << "invalid target lla" << std::endl;
    return 1;
  }
  geo::Vector3f target_velocity_ecef_mps;
  target_velocity_ecef_mps.x = 120.0f;
  target_velocity_ecef_mps.y = -70.0f;
  target_velocity_ecef_mps.z = 30.0f;

  // 4) 组装外部输入并执行单周期。
  //    先把外部平台与目标信息合成统一输入，再由适配器生成目标特征。
  aq::session::TargetExternalKinematicsInput external_input;
  external_input.radar_position_ecef_m = platform_ecef;
  external_input.has_radar_velocity_ecef_mps = true;
  external_input.radar_velocity_ecef_mps = platform_velocity_ecef_mps;
  external_input.target_position_ecef_m = target_ecef;
  external_input.target_velocity_mps = target_velocity_ecef_mps;
  external_input.target_velocity_frame = aq::session::VelocityFrame::kEcef;
  external_input.platform_attitude_deg = aq::model::PlatformAttitudeDeg{};
  external_input.radar_mount_angles_deg =
      session_config.beam_control.radar_orientation.mount_angles_deg;
  external_input.rcs = 1.8f;
  external_input.swerling_type = 0;

  aq::model::TargetFeature target;
  if (!aq::session::TryMakeTargetFromExternalKinematics(1001U, external_input, &target)) {
    std::cerr << "failed to build target from external kinematics input" << std::endl;
    return 1;
  }

  //    将单个目标特征放入当前周期输入，完成一次雷达循环。
  aq::session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose.attitude_deg.yaw_deg = 0.0f;
  input.platform_pose.attitude_deg.pitch_deg = 0.0f;
  input.platform_pose.attitude_deg.roll_deg = 0.0f;
  input.target_features.push_back(target);

  const auto issues = aq::session::ValidateRadarCycleInput(input);
  if (aq::session::HasValidationError(issues)) {
    std::cerr << "invalid radar input" << std::endl;
    return 1;
  }

  const aq::session::RadarCycleResult result = session.StepWithResult(input);
  const std::size_t confirmed =
      aq::output::CollectConfirmedTracks(result.track_output_frame).size();

  std::cout << "published=" << result.track_output_frame.published_track_count
            << " confirmed=" << confirmed
            << " match_rate=" << result.association_quality_metrics.match_rate << std::endl;
  return 0;
}
