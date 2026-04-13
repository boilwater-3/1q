// Copyright 2026. All Rights Reserved.
//
// @file esr_quick_start.cpp
// @brief ESR 最小接入示例：外部 ECEF 平台输入 + 会话一步一帧调用。

#include <iostream>

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/model/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSessionFactory.h"
#include "1q/foundation/coordinate_transform.h"

int main() {
  namespace esr = electronic_surveillance_radar;
  namespace geo = oneq::foundation;

  // 1) 创建会话。
  esr::session::EsrSessionConfig session_config =
      esr::config::EsrSessionConfigBuilder().WithDetectionMinSnrDb(0.0f).Build();
  session_config.pipeline_config.scan.scan_start_az_deg = -60.0f;
  session_config.pipeline_config.scan.scan_end_az_deg = 60.0f;
  session_config.pipeline_config.scan.scan_start_el_deg = -20.0f;
  session_config.pipeline_config.scan.scan_end_el_deg = 20.0f;
  session_config.pipeline_config.scan.az_step_deg = 120.0f;
  session_config.pipeline_config.scan.el_step_deg = 40.0f;
  esr::session::EsrSession session = esr::session::EsrSessionFactory::Create(session_config);

  // 2) 外部平台信息（位置、速度）。
  //    这里先用 LLA 构造样例，再统一转换为 ECEF，模拟外部系统直接给出地固坐标。
  geo::LlaCoordinateDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 2500.0;
  geo::EcefCoordinateM platform_ecef;
  if (!geo::TryLlaToEcef(platform_lla, &platform_ecef)) {
    std::cerr << "invalid platform lla" << std::endl;
    return 1;
  }
  esr::model::EsrVector3f platform_velocity_ecef_mps;
  platform_velocity_ecef_mps.x = 20.0f;
  platform_velocity_ecef_mps.y = 60.0f;
  platform_velocity_ecef_mps.z = -5.0f;

  // 3) 目标信息（辐射源真值）。
  //    ESR 真值输入最终需要局部坐标，这里先从 LLA 转 ECEF，再转成 ENU 作为示例。
  geo::LlaCoordinateDegM emitter_lla = platform_lla;
  emitter_lla.longitude_deg += 0.01;
  emitter_lla.altitude_m = 200.0;
  geo::EcefCoordinateM emitter_ecef;
  if (!geo::TryLlaToEcef(emitter_lla, &emitter_ecef)) {
    std::cerr << "invalid emitter lla" << std::endl;
    return 1;
  }
  geo::EnuCoordinateM emitter_enu;
  if (!geo::TryEcefToEnu(emitter_ecef, platform_lla, &emitter_enu)) {
    std::cerr << "failed to convert emitter ecef to enu" << std::endl;
    return 1;
  }
  esr::model::EmitterTruthState emitter;
  emitter.emitter_id = "emitter-1";
  emitter.pose.position_m.x = static_cast<float>(emitter_enu.x_m);
  emitter.pose.position_m.y = static_cast<float>(emitter_enu.y_m);
  emitter.pose.position_m.z = static_cast<float>(emitter_enu.z_m);
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 5.0e7;
  emitter.pulse_width_s = 1.0e-6;
  emitter.pri_s = 1.0e-4;

  // 4) 组装外部输入并执行单周期。
  //    reference 定义 ESR 局部参考系，pose_input 描述外部系统给出的平台位置和速度。
  esr::session::EsrCoordinateReference reference;
  reference.origin_lla = platform_lla;

  //    这里把外部 ECEF 位置/速度交给适配器，转换成 ESR 内部使用的平台位姿。
  esr::session::EsrExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = platform_ecef;
  pose_input.platform_velocity_frame = esr::session::EsrVelocityFrame::kEcef;
  pose_input.platform_velocity_mps = platform_velocity_ecef_mps;

  esr::model::EsrPoseState platform_pose;
  if (!esr::session::TryMakeEsrPoseFromExternalKinematics(pose_input, reference, &platform_pose)) {
    std::cerr << "failed to build esr platform pose" << std::endl;
    return 1;
  }
  //    将平台位姿与辐射源真值合成当前周期输入，然后执行一次侦察周期。
  esr::session::EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose = platform_pose;
  input.scene_emitters.push_back(emitter);

  const esr::session::EsrValidationIssueList issues = esr::session::ValidateEsrCycleInput(input);
  if (esr::session::HasEsrValidationError(issues)) {
    std::cerr << "invalid esr input" << std::endl;
    return 1;
  }

  const esr::session::EsrCycleResult result = session.StepWithResult(input);
  std::cout << "obs=" << result.output_frame.observation_output.observations.size()
            << " hypotheses=" << result.output_frame.emitter_output.hypotheses.size()
            << " associations=" << result.output_frame.truth_evaluation_output.associations.size()
            << std::endl;
  return 0;
}
