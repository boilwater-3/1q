#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/coordinate/types.h"
#include "config_loader.h"

namespace ar = airborne_radar;
namespace ar_session = airborne_radar::session;
namespace ar_config = airborne_radar::config;
namespace ar_model = airborne_radar::session;

namespace {

struct TargetState {
  std::uint64_t id;
  oneq::coordinate::EcefPositionM pos;
  oneq::coordinate::EcefVelocityMps vel;
  float rcs;
};

struct SceneState {
  ar_session::ArSession session;
  oneq::coordinate::EcefPositionM platform_pos;
  oneq::coordinate::EcefVelocityMps platform_vel;
  std::vector<TargetState> targets;
  std::uint32_t cycle{0};
  std::uint32_t validation_errors{0};
};

TargetState MakeTarget(std::uint64_t id, double x, double y, double z,
                       double vx, double vy, double vz, float rcs) {
  TargetState t;
  t.id = id;
  t.pos.x_m = x; t.pos.y_m = y; t.pos.z_m = z;
  t.vel.x_mps = vx; t.vel.y_mps = vy; t.vel.z_mps = vz;
  t.rcs = rcs;
  return t;
}

ar_session::ArExternalTargetInput ToTargetInput(const TargetState& t) {
  ar_session::ArExternalTargetInput input;
  input.target_id = t.id;
  input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  input.kinematics.position_ecef_m = t.pos;
  input.kinematics.velocity_mps = t.vel;
  input.rcs = t.rcs;
  input.swerling_type = 0;
  return input;
}

ar_session::ArExternalPoseInput MakePlatformPose(
    const oneq::coordinate::EcefPositionM& pos,
    const oneq::coordinate::EcefVelocityMps& vel) {
  ar_session::ArExternalPoseInput platform;
  platform.platform_position_ecef_m = pos;
  platform.platform_velocity_mps = vel;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;
  return platform;
}

SceneState InitScene() {
  ar_config::ArSessionConfig config;
  std::string error;
  if (!examples::LoadArSessionConfigFromFile(SCENE_CONFIG_DIR "/airborne_radar.json",
                                              &config, &error)) {
    std::cerr << "Failed to load AR config: " << error << "\n";
    std::exit(1);
  }

  SceneState s;
  s.session = ar_session::ArSession::Create(config);

  s.platform_pos.x_m = -2289512.0;
  s.platform_pos.y_m = 4909946.0;
  s.platform_pos.z_m = 3640982.0;

  s.platform_vel.x_mps = 120.0;
  s.platform_vel.y_mps = -80.0;
  s.platform_vel.z_mps = 30.0;

  // 3 个敌方空中目标：逼近我方平台
  s.targets.push_back(MakeTarget(
      1001, -2289512.0 + 18000.0, 4909946.0 + 2500.0, 3640982.0 + 1200.0,
      -120.0, 8.0, 0.0, 2.2f));
  s.targets.push_back(MakeTarget(
      1002, -2289512.0 + 24000.0, 4909946.0 - 4000.0, 3640982.0 + 2000.0,
      -90.0, -12.0, 0.0, 1.4f));
  s.targets.push_back(MakeTarget(
      1003, -2289512.0 + 30000.0, 4909946.0 + 1000.0, 3640982.0 + 1500.0,
      -150.0, 0.0, -5.0, 3.0f));

  return s;
}

/// 执行一个仿真周期：推进敌方目标和平台位置，执行检测跟踪。
ar_session::ArCycleResult Step(SceneState& s, float dt) {
  ++s.cycle;

  // 推进敌方目标位置（欧拉积分）
  for (auto& t : s.targets) {
    t.pos.x_m += t.vel.x_mps * dt;
    t.pos.y_m += t.vel.y_mps * dt;
    t.pos.z_m += t.vel.z_mps * dt;
  }

  // 推进平台位置
  s.platform_pos.x_m += s.platform_vel.x_mps * dt;
  s.platform_pos.y_m += s.platform_vel.y_mps * dt;
  s.platform_pos.z_m += s.platform_vel.z_mps * dt;

  ar_session::ArExternalPoseInput platform = MakePlatformPose(s.platform_pos, s.platform_vel);

  std::vector<ar_session::ArExternalTargetInput> target_inputs;
  target_inputs.reserve(s.targets.size());
  for (const auto& t : s.targets) {
    target_inputs.push_back(ToTargetInput(t));
  }

  ar_session::ArCycleInput input;
  input.cycle_index = s.cycle;
  input.cycle_start_time_s =
      static_cast<double>(s.cycle - 1U) * static_cast<double>(dt);
  input.dt_sec = dt;
  platform.platform_entity_id = 1U;
  input.platform = platform;
  input.targets = target_inputs;

  ar_session::ArCycleResult result = s.session.StepWithResult(input);
  if (result.has_validation_error) ++s.validation_errors;

  std::cout << "ar-scene: cycle=" << result.input_cycle_index
            << " tracks=" << result.track_output_frame.tracks.size()
            << " confirmed="
            << ar_session::CountTracksByStatus(result.track_output_frame,
                                               ar_session::TrackStatus::kConfirmed)
            << " tentative="
            << ar_session::CountTracksByStatus(result.track_output_frame,
                                               ar_session::TrackStatus::kTentative)
            << " commands=" << result.submitted_commands.size()
            << " validation_errors=" << (result.has_validation_error ? "true" : "false")
            << "\n";

  return result;
}

}  // namespace

int main() {
  SceneState scene = InitScene();

  constexpr std::uint32_t kNumCycles = 50;
  constexpr float kDt = 1.0f;

  for (std::uint32_t i = 0; i < kNumCycles; ++i) {
    Step(scene, kDt);
  }

  std::cout << "\n=== AR Scene Summary ===\n"
            << "cycles=" << kNumCycles
            << " validation_errors=" << scene.validation_errors << "\n";

  return scene.validation_errors == 0 ? 0 : 1;
}
