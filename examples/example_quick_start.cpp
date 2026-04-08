// Copyright 2026. All Rights Reserved.
//
// @file example_quick_start.cpp
// @brief Layer 1 快速入门示例：通过 RadarSession 覆盖典型适配场景。
//
// 本文件展示适配人员最常见的四种使用路径：
//   1. 最简路径：仅关心"是否有目标被确认"
//   2. 场景切换：动态引入/撤销噪声干扰和复合干扰
//   3. 运行时调参：更新信号流水线配置和干扰判定阈值
//   4. 批量结果读取：使用 TrackOutputQueries 全系列辅助函数

#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/model/TargetFeatureBuilder.h"
#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/output/TrackOutputQueries.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/config/SignalPipelineConfig.h"

namespace {

// ---------------------------------------------------------------------------
// 辅助：打印单帧输出摘要
// ---------------------------------------------------------------------------
void PrintFrameSummary(const char* label,
                       const airborne_radar::session::RadarCycleResult& result) {
  using namespace airborne_radar::output;
  std::cout << "[" << label << "]"
            << " published=" << result.track_output_frame.published_track_count
            << " confirmed=" << CollectConfirmedTracks(result.track_output_frame).size()
            << " lost=" << CollectLostTracks(result.track_output_frame).size()
            << " jamming=" << CountJammingTracks(result.track_output_frame)
            << " match_rate=" << result.association_quality_metrics.match_rate
            << " commands=" << result.submitted_commands.size() << "\n";
}

}  // namespace

int main() {
  namespace aq = airborne_radar::model;
  namespace ctx = airborne_radar::session;
  namespace env = airborne_radar::environment;
  namespace sig = airborne_radar::signal::pipeline;
  namespace out = airborne_radar::output;
  using session_t = airborne_radar::session::RadarSession;

  // =========================================================================
  // 1. 最简路径
  //    使用探测任务预设配置构造 Session，每周期只需填写输入、读取输出。
  //
  //    若平台有确定的雷达硬件参数（发射机功率、载频、天线增益等），
  //    推荐改用 RadarSessionConfigBuilder 在预设基础上叠加：
  //
  //      airborne_radar::config::SignalDetectionConfig detection =
  //          preset.detection;
  //      detection.enable_physics_detection = true;
  //      detection.transmitter.peak_power_w = 5e6f;
  //      detection.transmitter.frequency_hz = 9.3e9f;
  //      detection.antenna.main_beam_gain_db = 38.0f;
  //      detection.receiver.noise_figure_db = 3.5f;
  //      session_t session(
  //          airborne_radar::config::RadarSessionConfigBuilder(preset)
  //              .Detection()
  //              .WithDetection(detection)
  //              .End()
  //              .Build());
  //
  //    详细示例参见 example_radar_session.cpp。
  // =========================================================================
  session_t session(airborne_radar::config::MakeDetectionMissionRadarSessionConfig());

  // 使用 TargetFeatureBuilder 构造目标（P1-3），避免参数位置依赖错误。
  ctx::RadarCycleInput input;
  input.dt_sec = 1.0f;
  input.target_features.push_back(aq::TargetFeatureBuilder(2001U)
                                      .Position(150.0f, 10.0f, 5000.0f)
                                      .Velocity(180.0f, 0.0f, -5.0f)
                                      .Rcs(1.5f)
                                      .Build());
  input.target_features.push_back(aq::TargetFeatureBuilder(2002U)
                                      .Position(300.0f, -20.0f, 8000.0f)
                                      .Velocity(220.0f, 5.0f, 0.0f)
                                      .Rcs(2.0f)
                                      .Build());

  // 可选：用 ValidateRadarCycleInput 检查输入合法性（不强制）
  {
    const auto issues = ctx::ValidateRadarCycleInput(input);
    if (ctx::HasValidationError(issues)) {
      std::cerr << "输入校验失败，请检查 dt_sec 和 target_features\n";
      return 1;
    }
  }

  const auto r1 = session.StepWithResult(input);
  PrintFrameSummary("cycle-1 无干扰", r1);

  // =========================================================================
  // 2. 场景切换：当前周期引入噪声干扰
  //    调用 Step(input, scene) 的带场景重载，场景在本周期开始前生效。
  // =========================================================================

  // 构造噪声干扰源
  env::JammerEmitterState noise_jammer;
  noise_jammer.technique = env::JammingTechnique::kNoiseSuppression;
  noise_jammer.power_db = 14.0f;
  noise_jammer.js_db = 9.0f;
  noise_jammer.frequency_overlap_ratio = 0.3f;
  noise_jammer.prf_lock_risk = 0.15f;
  noise_jammer.in_sidelobe = false;
  noise_jammer.azimuth_deg = 10.0f;
  noise_jammer.elevation_deg = 2.0f;
  noise_jammer.confidence = 0.9f;

  // 追加大气衰减并设置杂波功率
  const env::EnvironmentSceneState jammed_scene = env::EnvironmentSceneBuilder()
                                                      .SetAtmosphericAttenuationDb(2.5f)
                                                      .SetClutterPowerDb(4.0f)
                                                      .AddNoiseJammer(noise_jammer)
                                                      .Build();

  // 推进目标位置（模拟真实仿真循环）
  for (auto& t : input.target_features) {
    t.position_x += t.current_track_velocity_x * input.dt_sec;
    t.position_y += t.current_track_velocity_y * input.dt_sec;
    t.position_z += t.current_track_velocity_z * input.dt_sec;
  }
  const auto r2 = session.StepWithResult(input, jammed_scene);
  PrintFrameSummary("cycle-2 噪声干扰", r2);

  // =========================================================================
  // 2b. 复合干扰：再追加一个欺骗式干扰源
  // =========================================================================
  env::JammerEmitterState deception_jammer;
  deception_jammer.technique = env::JammingTechnique::kDeception;
  deception_jammer.power_db = 10.0f;
  deception_jammer.js_db = 6.0f;
  deception_jammer.frequency_overlap_ratio = 0.85f;
  deception_jammer.prf_lock_risk = 0.7f;
  deception_jammer.in_sidelobe = false;
  deception_jammer.azimuth_deg = -5.0f;
  deception_jammer.confidence = 0.8f;

  const env::EnvironmentSceneState compound_scene = env::EnvironmentSceneBuilder()
                                                        .SetAtmosphericAttenuationDb(2.5f)
                                                        .SetClutterPowerDb(4.0f)
                                                        .AddNoiseJammer(noise_jammer)
                                                        .AddDeceptionJammer(deception_jammer)
                                                        .Build();

  for (auto& t : input.target_features) {
    t.position_x += t.current_track_velocity_x * input.dt_sec;
    t.position_y += t.current_track_velocity_y * input.dt_sec;
    t.position_z += t.current_track_velocity_z * input.dt_sec;
  }
  const auto r3 = session.StepWithResult(input, compound_scene);
  PrintFrameSummary("cycle-3 复合干扰", r3);

  // =========================================================================
  // 3. 运行时调参
  //    可在任意周期间更换信号流水线配置或调整干扰判定阈值，无需重建 Session。
  // =========================================================================

  // 切换为稳健模式（更保守的跟踪参数）
  session.UpdateSignalPipelineConfig(airborne_radar::config::MakeHighRobustnessSignalPipelineConfig());

  // 提高干扰判定灵敏度（默认 6.0 dB，调低意味着更容易触发干扰标记）
  session.SetJammingDetectionThresholdDb(4.5f);

  for (auto& t : input.target_features) {
    t.position_x += t.current_track_velocity_x * input.dt_sec;
    t.position_y += t.current_track_velocity_y * input.dt_sec;
    t.position_z += t.current_track_velocity_z * input.dt_sec;
  }
  const auto r4 = session.StepWithResult(input, compound_scene);
  PrintFrameSummary("cycle-4 稳健配置(首周期)", r4);

  // HighRobustness 配置下确认阈值更高，新建航迹需要积累更多命中才能确认。
  // 注意：已确认航迹的状态不会因配置变更而降级；新阈值仅影响此后新建的候选航迹。
  // cycle 5~7：持续推进目标，让跟踪器积累命中次数。
  for (int i = 5; i <= 7; ++i) {
    for (auto& t : input.target_features) {
      t.position_x += t.current_track_velocity_x * input.dt_sec;
    t.position_y += t.current_track_velocity_y * input.dt_sec;
    t.position_z += t.current_track_velocity_z * input.dt_sec;
    }
    char label[32];
    std::snprintf(label, sizeof(label), "cycle-%d 稳健配置", i);
    const auto rn = session.StepWithResult(input, compound_scene);
    PrintFrameSummary(label, rn);
  }

  // =========================================================================
  // 4. 批量结果读取
  //    使用 TrackOutputQueries 全系列辅助函数查询敌情态势。
  //    以 cycle-7（稳健配置充分收敛后）的输出帧为例。
  // =========================================================================

  // 取最后一帧做详细查询
  for (auto& t : input.target_features) {
    t.position_x += t.current_track_velocity_x * input.dt_sec;
    t.position_y += t.current_track_velocity_y * input.dt_sec;
    t.position_z += t.current_track_velocity_z * input.dt_sec;
  }
  const auto r_final = session.StepWithResult(input, compound_scene);
  PrintFrameSummary("cycle-8 最终读取", r_final);

  // 按外部目标 ID 快速查询特定目标的轨迹状态
  const auto track_map = out::BuildTrackMapByExternalTargetId(r_final.track_output_frame);
  for (const auto& kv : track_map) {
    std::cout << "  target_id=" << kv.first << " jamming=" << kv.second.state.jamming_detected
              << "\n";
  }

  // 检查某个外部目标 ID 是否还在跟踪链中
  const bool has_2001 = out::ContainsExternalTargetId(r_final.track_output_frame, 2001U);
  std::cout << "target 2001 still tracked: " << has_2001 << "\n";

  // 获取所有已确认轨迹集合做进一步处理
  const auto confirmed = out::CollectConfirmedTracks(r_final.track_output_frame);
  std::cout << "confirmed tracks in cycle-8: " << confirmed.size() << "\n";

  // 最新控制真值（仅在 HasLatestControlProfile() == true 时有效）
  if (session.HasLatestControlProfile()) {
    const auto& profile = session.GetLatestControlProfile();
    std::cout << "control_profile version=" << profile.version
              << " lpi_power=" << profile.enable_lpi_power_control
              << " eccm_rejitter=" << profile.enable_eccm_rejitter << "\n";
  }

  return 0;
}
