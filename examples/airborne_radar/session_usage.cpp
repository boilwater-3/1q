/**
 * @file airborne_radar_session_usage.cpp
 * @brief 机载雷达 Session 配置与运行示例。
 *
 * 本示例展示如何通过 Builder 模式构建雷达会话配置，
 * 并在多周期仿真中向 Session 送入平台位姿、目标运动学、环境参数等输入，
 * 获取航迹输出与控制指令。
 *
 * 典型流程：
 *   1. 使用 RadarSessionConfigBuilder 构建会话配置
 *   2. 通过 RadarSessionFactory 创建 Session
 *   3. 每个仿真周期构造 RadarCycleInput，调用 StepWithResult 执行
 *   4. 从 RadarCycleResult 中读取航迹、指令等输出
 */

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/coordinate/types.h"
#include "config_loader.h"

namespace ar = airborne_radar;
namespace ar_config = airborne_radar::config;
namespace ar_env = airborne_radar::environment;
namespace ar_model = airborne_radar::model;
namespace ar_session = airborne_radar::session;

namespace {

/// 从 JSON 配置文件加载会话配置。
ar_session::RadarSessionConfig LoadConfigFromFile() {
  ar_session::RadarSessionConfig config;
  std::string error;
  if (!examples::LoadArSessionConfigFromFile("configs/airborne_radar.json",
                                              &config, &error)) {
    std::cerr << "Failed to load AR config: " << error << "\n";
    std::exit(1);
  }
  return config;
}

/// 使用工厂从配置创建 Session。
/// Session 本身管理内部状态，支持多次 StepWithResult 调用。
ar_session::RadarSession CreateWideAreaSearchSession() {
  return ar_session::RadarSessionFactory::Create(LoadConfigFromFile());
}

/// 构造平台（载机）位姿输入。
/// 位置和速度使用 ECEF 坐标系；姿态和雷达安装角使用角度制。
ar_session::RadarExternalPoseInput MakePlatformPose(const oneq::coordinate::EcefPositionM& pos,
                                                    const oneq::coordinate::EcefVelocityMps& vel) {
  ar_session::RadarExternalPoseInput platform;
  platform.platform_position_ecef_m = pos;
  platform.platform_velocity_mps = vel;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;
  platform.radar_mount_angles_deg.yaw_deg = 0.0;
  platform.radar_mount_angles_deg.pitch_deg = 0.0;
  platform.radar_mount_angles_deg.roll_deg = 0.0;
  return platform;
}

/// 构造目标外部运动学输入。
/// external_target_id 为调用方分配的唯一目标标识（不可为 0，否则触发校验警告）。
/// rcs 为雷达截面积（m²），swerling_type 表示 Swerling 起伏模型编号。
ar_session::RadarExternalTargetInput MakeTargetKinematics(
    std::uint64_t target_id, const oneq::coordinate::EcefPositionM& pos,
    const oneq::coordinate::EcefVelocityMps& vel, float rcs) {
  ar_session::RadarExternalTargetInput target;
  target.target_id = target_id;
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  target.kinematics.position_ecef_m = pos;
  target.kinematics.velocity_mps = vel;
  target.rcs = rcs;
  target.swerling_type = 0;
  return target;
}

/// 构造环境输入，包括大气观测、空间天气和地表覆盖信息。
/// 大气物理模型启用后，气压/温度/湿度等参数将参与传播衰减计算。
ar_session::RadarEnvironmentInput MakeInitialEnvironmentInput() {
  ar_session::RadarEnvironmentInput environment;
  environment.atmospheric_observation.enable_physical_model = true;
  environment.atmospheric_observation.pressure_hpa = 1010.0f;
  environment.atmospheric_observation.temperature_k = 290.0f;
  environment.atmospheric_observation.relative_humidity = 0.45f;
  environment.atmospheric_context.has_simulation_unix_seconds = true;
  environment.atmospheric_context.simulation_unix_seconds = 1770000000;
  environment.atmospheric_context.solar_flux_f107a = 145.0f;
  environment.atmospheric_context.solar_flux_f107 = 148.0f;
  environment.atmospheric_context.geomagnetic_ap = 5.0f;
  environment.surface_observation.cover_profile = ar_env::VegetationCoverProfile::kOpenGrassland;
  environment.surface_observation.enable_physical_model = true;
  return environment;
}

/// 打印单周期结果摘要：航迹数、确认/暂定航迹统计、指令数及是否校验出错。
void PrintResult(const char* label, const ar_session::RadarCycleResult& result) {
  std::cout << label << ": cycle=" << result.input_cycle_index
            << " tracks=" << result.track_output_frame.tracks.size() << " confirmed="
            << ar_session::CountTracksByStatus(result.track_output_frame,
                                               ar_model::TrackStatus::kConfirmed)
            << " tentative="
            << ar_session::CountTracksByStatus(result.track_output_frame,
                                               ar_model::TrackStatus::kTentative)
            << " commands=" << result.submitted_commands.size()
            << " validation_errors=" << (result.has_validation_error ? "true" : "false") << "\n";
}

/// 打印外部 ECEF 轨迹输出：将雷达局部坐标转换后的世界坐标轨迹摘要。
void PrintExternalOutput(const ar_session::RadarExternalTrackOutputFrame& output) {
  std::cout << "  external_tracks=" << output.tracks.size();
  for (std::size_t j = 0; j < output.tracks.size(); ++j) {
    const auto& t = output.tracks[j];
    std::cout << "\n    [" << j << "] id=" << t.external_target_id
              << " status=" << static_cast<int>(t.status) << " ecef=("
              << t.target_position_ecef_m.x_m << "," << t.target_position_ecef_m.y_m << ","
              << t.target_position_ecef_m.z_m << ")"
              << " speed=" << t.speed << " rcs=" << t.rcs;
  }
  std::cout << "\n";
}

struct MovingAirTarget {
  std::uint64_t id;
  oneq::coordinate::EcefPositionM pos;
  oneq::coordinate::EcefVelocityMps vel;
  float rcs;
};

MovingAirTarget MakeMovingAirTarget(std::uint64_t id, double x_m, double y_m, double z_m,
                                    double vx_mps, double vy_mps, double vz_mps, float rcs) {
  MovingAirTarget target;
  target.id = id;
  target.pos.x_m = x_m;
  target.pos.y_m = y_m;
  target.pos.z_m = z_m;
  target.vel.x_mps = vx_mps;
  target.vel.y_mps = vy_mps;
  target.vel.z_mps = vz_mps;
  target.rcs = rcs;
  return target;
}

/// 多目标运动场景：3 个空中目标在 50 个仿真周期内的广域搜索与跟踪。
///
/// 每个周期：
///   1. 构造平台位姿（ECEF 位置 + 速度 + 姿态）
///   2. 构造所有目标的运动学输入
///   3. 通过 RadarCycleInputBuilder 组装完整输入
///   4. 调用 session.StepWithResult 获得输出
///   5. 根据返回的 dt 推进目标位置（简单欧拉积分）
bool RunMovingTargetsScenario() {
  ar_session::RadarSession session = CreateWideAreaSearchSession();
  ar_session::RadarEnvironmentInputState environment_state(MakeInitialEnvironmentInput());

  // 平台初始位置：ECEF 坐标（约对应中纬度某空域）
  oneq::coordinate::EcefPositionM platform_pos;
  platform_pos.x_m = -2289512.0;
  platform_pos.y_m = 4909946.0;
  platform_pos.z_m = 3640982.0;

  // 平台速度矢量（m/s）
  oneq::coordinate::EcefVelocityMps platform_vel;
  platform_vel.x_mps = 120.0;
  platform_vel.y_mps = -80.0;
  platform_vel.z_mps = 30.0;

  // 3 个空中运动目标：标识符、位置偏移、速度、RCS
  std::vector<MovingAirTarget> targets = {
      MakeMovingAirTarget(1001, -2289512.0 + 18000.0, 4909946.0 + 2500.0, 3640982.0 + 1200.0,
                          -120.0, 8.0, 0.0, 2.2f),
      MakeMovingAirTarget(1002, -2289512.0 + 24000.0, 4909946.0 - 4000.0, 3640982.0 + 2000.0, -90.0,
                          -12.0, 0.0, 1.4f),
      MakeMovingAirTarget(1003, -2289512.0 + 30000.0, 4909946.0 + 1000.0, 3640982.0 + 1500.0,
                          -150.0, 0.0, -5.0, 3.0f),
  };

  const std::uint32_t num_cycles = 50;
  std::uint32_t validation_error_count = 0;
  std::uint32_t max_tracks = 0;
  std::uint32_t min_tracks = 100;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    // 构造本周期平台位姿
    ar_session::RadarExternalPoseInput platform = MakePlatformPose(platform_pos, platform_vel);

    // 构造所有目标的运动学输入
    std::vector<ar_session::RadarExternalTargetInput> target_kinematics;
    target_kinematics.reserve(targets.size());
    for (const auto& mt : targets) {
      target_kinematics.push_back(MakeTargetKinematics(mt.id, mt.pos, mt.vel, mt.rcs));
    }

    // 组装周期输入：平台位姿 + 目标列表 + 时间步长(秒) + 环境快照
    ar_session::RadarCycleInput input;
    if (!ar_session::RadarCycleInputBuilder::Build(platform, target_kinematics, 1.0f,
                                                   environment_state.Snapshot(), &input)) {
      std::cerr << "ar-moving: cycle " << (i + 1) << " build failed\n";
      return false;
    }
    input.cycle_index = i + 1;

    // 使用返回的 dt 推进目标位置（简单欧拉积分）
    const float dt = input.dt_sec;
    for (auto& mt : targets) {
      mt.pos.x_m += mt.vel.x_mps * dt;
      mt.pos.y_m += mt.vel.y_mps * dt;
      mt.pos.z_m += mt.vel.z_mps * dt;
    }

    // 执行一个仿真周期
    ar_session::RadarCycleResult result = session.StepWithResult(input);
    if (result.has_validation_error) {
      ++validation_error_count;
    }
    std::size_t ntracks = result.track_output_frame.tracks.size();
    if (ntracks > max_tracks) max_tracks = ntracks;
    if (ntracks < min_tracks) min_tracks = ntracks;
    PrintResult("ar-moving", result);

    // 使用 RadarCycleOutputBuilder 将内部雷达局部轨迹转换为外部 ECEF 输出
    ar_session::RadarExternalTrackOutputFrame external_output;
    bool external_output_ok = ar_session::RadarCycleOutputBuilder::Build(
        platform, result.track_output_frame, &external_output);
    if (external_output_ok) {
      PrintExternalOutput(external_output);
    }
  }

  // 汇总统计
  std::cout << "\n=== AR Summary ===\n"
            << "cycles=" << num_cycles << " min_tracks=" << min_tracks
            << " max_tracks=" << max_tracks << " validation_errors=" << validation_error_count
            << "\n";
  return validation_error_count == 0;
}

}  // namespace

/// 入口：运行多目标运动场景，返回 0 表示成功（无校验错误）。
int main() { return RunMovingTargetsScenario() ? 0 : 1; }
