#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <cmath>

#include "1q/coordinate/types.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "config_loader.h"

namespace eos_session = electro_optical_sensor::session;
namespace eos_config = electro_optical_sensor::config;
namespace eos_env = electro_optical_sensor::session;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct TargetState {
  std::uint64_t id;
  double lat_deg;
  double lon_deg;
  double alt_m;
  double vel_east_mps;
  double vel_north_mps;
  float temperature_k;
  float area_m2;
};

struct SceneState {
  eos_session::EosSession session;
  oneq::coordinate::LlaPositionDegM platform_lla;
  oneq::coordinate::EcefPositionM platform_ecef;
  oneq::coordinate::EcefVelocityMps platform_vel;
  std::vector<TargetState> targets;
  eos_session::EosEnvironmentInput environment;
  std::uint32_t cycle{0};
  std::uint32_t validation_errors{0};
};

eos_session::EosExternalTargetInput ToTargetInput(const TargetState& t) {
  eos_session::EosExternalTargetInput target;
  target.target_id = t.id;
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  target.kinematics.position_lla_deg_m.latitude_deg = t.lat_deg;
  target.kinematics.position_lla_deg_m.longitude_deg = t.lon_deg;
  target.kinematics.position_lla_deg_m.altitude_m = t.alt_m;

  oneq::coordinate::LlaPositionDegM lla;
  lla.latitude_deg = t.lat_deg;
  lla.longitude_deg = t.lon_deg;
  lla.altitude_m = t.alt_m;

  oneq::coordinate::EnuVelocityMps vel_enu;
  vel_enu.east_mps = t.vel_east_mps;
  vel_enu.north_mps = t.vel_north_mps;
  vel_enu.up_mps = 0.0;

  oneq::coordinate::TryEnuToEcefVelocity(vel_enu, lla, &target.kinematics.velocity_mps);

  target.appearance.apparent_temperature_k = t.temperature_k;
  target.appearance.emissivity = 0.92f;
  target.appearance.reflectance = 0.35f;
  target.appearance.projected_area_m2 = t.area_m2;
  return target;
}

SceneState InitScene() {
  eos_config::EosSessionConfig config;
  std::string error;
  if (!examples::LoadEosSessionConfigFromFile(SCENE_CONFIG_DIR "/electro_optical.json",
                                               &config, &error)) {
    std::cerr << "Failed to load EOS config: " << error << "\n";
    std::exit(1);
  }

  SceneState s;
  s.session = eos_session::EosSessionFactory::Create(config);

  // 平台高度 7000m
  s.platform_lla.latitude_deg = 35.0;
  s.platform_lla.longitude_deg = 114.5;
  s.platform_lla.altitude_m = 7000.0;

  oneq::coordinate::TryLlaToEcef(s.platform_lla, &s.platform_ecef);

  // 平台向东飞行 150 m/s
  oneq::coordinate::EnuVelocityMps vel_enu;
  vel_enu.east_mps = 150.0;
  vel_enu.north_mps = 0.0;
  vel_enu.up_mps = 0.0;
  oneq::coordinate::TryEnuToEcefVelocity(vel_enu, s.platform_lla, &s.platform_vel);

  s.environment.solar_altitude_deg = 42.0f;
  s.environment.solar_azimuth_deg = 165.0f;
  s.environment.solar_irradiance_w_m2 = 850.0f;
  s.environment.cloud_coverage_ratio = 0.15f;
  s.environment.background_temperature_k = 288.0f;
  s.environment.day_night_type = eos_session::DayNightType::kDay;

  // 传感器足印中心经度（7000m 高度，48° 俯视角）
  double base_lon = 114.5 + 0.06925;

  // 敌方地面目标
  // 静止目标 1：热点目标
  s.targets.push_back({101, 35.0, base_lon + 0.013, 0.0,
                        0.0, 0.0, 480.0f, 15.0f});
  // 静止目标 2：高价值目标
  s.targets.push_back({102, 35.002, base_lon + 0.050, 0.0,
                        0.0, 0.0, 490.0f, 12.0f});
  // 移动目标 1：向北运动
  s.targets.push_back({201, 34.998, base_lon + 0.025, 0.0,
                        0.0, 15.0, 500.0f, 14.0f});
  // 移动目标 2：向西运动（对向）
  s.targets.push_back({202, 35.001, base_lon + 0.060, 0.0,
                        -25.0, 0.0, 520.0f, 18.0f});

  return s;
}

/// 执行一个仿真周期：推进目标位置和平台位置，执行光电探测。
eos_session::EosCycleResult Step(SceneState& s, double dt) {
  ++s.cycle;

  // 推进平台位置
  eos_session::EosExternalPoseInput platform;
  platform.platform_position_ecef_m.x_m =
      s.platform_ecef.x_m + s.platform_vel.x_mps * dt * static_cast<double>(s.cycle);
  platform.platform_position_ecef_m.y_m =
      s.platform_ecef.y_m + s.platform_vel.y_mps * dt * static_cast<double>(s.cycle);
  platform.platform_position_ecef_m.z_m =
      s.platform_ecef.z_m + s.platform_vel.z_mps * dt * static_cast<double>(s.cycle);
  platform.platform_velocity_mps = s.platform_vel;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;

  // 推进目标位置
  std::vector<eos_session::EosExternalTargetInput> target_inputs;
  target_inputs.reserve(s.targets.size());
  for (auto& t : s.targets) {
    if (t.vel_east_mps != 0.0 || t.vel_north_mps != 0.0) {
      t.lat_deg += (t.vel_north_mps * dt) / 111111.0;
      t.lon_deg += (t.vel_east_mps * dt) / (111111.0 * std::cos(t.lat_deg * kPi / 180.0));
    }
    target_inputs.push_back(ToTargetInput(t));
  }

  eos_session::EosCycleInput input;
  eos_session::EosCoordinateStatus status;
  if (!eos_session::EosCycleInputBuilder::Build(
          platform, target_inputs, static_cast<float>(dt), s.environment,
          &input, &status)) {
    std::cerr << "EOS scene: cycle " << s.cycle
              << " build failed (status=" << static_cast<int>(status) << ")\n";
    std::exit(1);
  }
  input.cycle_index = s.cycle;

  eos_session::EosCycleResult result = s.session.StepWithResult(input);
  if (result.has_validation_error) ++s.validation_errors;

  std::size_t ndetected = 0;
  for (const auto& det : result.output_frame.detections) {
    if (det.detected) ++ndetected;
  }

  std::cout << "eos-scene: cycle=" << result.input_cycle_index
            << " detections=" << result.output_frame.detections.size()
            << " detected=" << ndetected
            << " validation_errors=" << (result.has_validation_error ? "true" : "false")
            << "\n";

  return result;
}

}  // namespace

int main() {
  SceneState scene = InitScene();

  constexpr std::uint32_t kNumCycles = 50;
  constexpr double kDt = 1.0;

  for (std::uint32_t i = 0; i < kNumCycles; ++i) {
    Step(scene, kDt);
  }

  std::cout << "\n=== EOS Scene Summary ===\n"
            << "cycles=" << kNumCycles
            << " validation_errors=" << scene.validation_errors << "\n";

  return scene.validation_errors == 0 ? 0 : 1;
}
