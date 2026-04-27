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
  //    第一步：将外部平台运动学转换为雷达局部位姿和参考系。
  aq::session::RadarExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = platform_ecef;
  pose_input.has_platform_velocity_ecef_mps = true;
  pose_input.platform_velocity_mps = platform_velocity_ecef_mps;
  pose_input.platform_velocity_frame = aq::session::VelocityFrame::kEcef;
  pose_input.platform_attitude_deg = aq::model::PlatformAttitudeDeg{};
  pose_input.radar_mount_angles_deg = session_config.mission.orientation.mount_angles_deg;

  aq::session::RadarLocalFrameReference reference;
  oneq::foundation::PoseState platform_pose;
  if (!aq::session::TryMakeRadarPoseFromExternalKinematics(pose_input, &reference,
                                                            &platform_pose)) {
    std::cerr << "failed to build radar pose from external kinematics input" << std::endl;
    return 1;
  }

  //    第二步：使用预计算的参考系将目标转换为 RadarSceneTarget。
  aq::session::TargetExternalKinematics target_input;
  target_input.target_position_ecef_m = target_ecef;
  target_input.target_velocity_mps = target_velocity_ecef_mps;
  target_input.target_velocity_frame = aq::session::VelocityFrame::kEcef;
  target_input.rcs = 1.8f;
  target_input.swerling_type = 0;

  aq::session::RadarSceneTarget target;
  if (!aq::session::TryMakeTargetFromExternalKinematics(
          1001U, target_input, reference, platform_pose.velocity_mps, &target)) {
    std::cerr << "failed to build target from external kinematics input" << std::endl;
    return 1;
  }

  //    将单个场景目标放入当前周期输入，完成一次雷达循环。
  aq::session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose = platform_pose;
  input.scene.push_back(target);

  const auto issues = aq::session::ValidateRadarCycleInput(input);
  if (aq::session::HasValidationError(issues)) {
    std::cerr << "invalid radar input" << std::endl;
    return 1;
  }

  const aq::session::RadarCycleResult result = session.StepWithResult(input);
  const std::size_t confirmed =
      aq::session::CollectConfirmedTracks(result.track_output_frame).size();

  std::cout << "published=" << result.track_output_frame.published_track_count
            << " confirmed=" << confirmed
            << " match_rate=" << result.association_quality_metrics.match_rate << std::endl;
  return 0;
}
