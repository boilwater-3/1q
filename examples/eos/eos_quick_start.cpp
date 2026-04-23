// Copyright 2026. All Rights Reserved.
//
// @file eos_quick_start.cpp
// @brief EOS 最小接入示例：外部 ECEF 输入 + 会话一步一帧调用。

#include <iostream>

#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/foundation/coordinate_transform.h"

int main() {
  namespace eos = electro_optical_sensor;
  namespace geo = oneq::foundation;

  // 1) 创建会话。
  const eos::session::EosSessionConfig session_config =
      eos::config::EosSessionConfigBuilder()
          .WithDetectionProfile(eos::config::EosDetectionProfile::kAggressive)
          .Build();
  eos::session::EosSession session = eos::session::EosSessionFactory::Create(session_config);

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
  platform_velocity_ecef_mps.x = 30.0f;
  platform_velocity_ecef_mps.y = 60.0f;
  platform_velocity_ecef_mps.z = -10.0f;

  // 3) 目标信息（位置 + 外观参数）。
  //    EOS 目标位置也支持从 LLA 进入，示例里先转成 ECEF 以贴近外部接口风格。
  geo::LlaCoordinateDegM target_lla = platform_lla;
  target_lla.longitude_deg += 0.01;
  target_lla.altitude_m = 200.0;
  geo::EcefCoordinateM target_ecef;
  if (!geo::TryLlaToEcef(target_lla, &target_ecef)) {
    std::cerr << "invalid target lla" << std::endl;
    return 1;
  }
  eos::session::EosTargetAppearance appearance;
  appearance.apparent_temperature_k = 320.0f;
  appearance.emissivity = 0.9f;
  appearance.reflectance = 0.1f;
  appearance.projected_area_m2 = 2.0f;

  // 4) 组装外部输入并执行单周期。
  //    reference 定义局部参考系，pose_input 描述外部平台运动学。
  eos::session::EosCoordinateReference reference;
  reference.origin_lla = platform_lla;

  eos::session::EosExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = platform_ecef;
  pose_input.platform_velocity_frame = eos::session::EosVelocityFrame::kEcef;
  pose_input.platform_velocity_mps = platform_velocity_ecef_mps;

  geo::PoseState platform_pose;
  if (!eos::session::TryMakeEosPoseFromExternalKinematics(pose_input, reference, &platform_pose)) {
    std::cerr << "failed to build eos platform pose" << std::endl;
    return 1;
  }

  eos::session::EosExternalTargetInput target_input;
  target_input.position_frame = eos::session::EosTargetPositionFrame::kEcef;
  target_input.target_position_ecef_m = target_ecef;
  target_input.appearance = appearance;

  eos::session::EosSceneTarget target;
  if (!eos::session::TryMakeEosSceneTargetFromExternalInput(1001U, target_input, reference,
                                                            platform_pose, &target)) {
    std::cerr << "failed to build eos target" << std::endl;
    return 1;
  }

  //    将平台位姿和场景目标合成当前周期输入，然后执行一次仿真周期。
  eos::session::EosCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose = platform_pose;
  input.scene.push_back(target);

  const eos::session::ValidationIssueList issues = eos::session::ValidateEosCycleInput(input);
  if (eos::session::HasValidationError(issues)) {
    std::cerr << "invalid eos input" << std::endl;
    return 1;
  }

  const eos::session::EosCycleResult result = session.StepWithResult(input);
  std::cout << "cycle=" << result.output_frame.cycle_index
            << " detections=" << result.output_frame.detections.size()
            << " executed=" << (result.executed_this_cycle ? "true" : "false") << std::endl;
  return 0;
}
