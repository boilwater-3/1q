#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

#include "1q/coordinate/types.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "config_loader.h"

namespace esr_session = electronic_surveillance_radar::session;
namespace esr_config = electronic_surveillance_radar::config;
namespace esr_env = electronic_surveillance_radar::environment;

namespace {

struct EmitterState {
  std::uint64_t id;
  oneq::coordinate::EcefPositionM pos;
  double carrier_hz;
  double speed_x_mps;
};

struct SceneState {
  esr_session::EsrSession session;
  oneq::coordinate::EcefPositionM platform_pos;
  oneq::coordinate::EcefVelocityMps platform_vel;
  std::vector<EmitterState> emitters;
  esr_session::EsrEnvironmentInput environment;
  std::uint32_t cycle{0};
  std::uint32_t validation_errors{0};
};

esr_session::EsrExternalEmitterInput ToEmitterInput(const EmitterState& e) {
  esr_session::EsrExternalEmitterInput input;
  input.emitter_id = e.id;
  input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  input.kinematics.position_ecef_m = e.pos;
  input.kinematics.velocity_mps.x_mps = 0.0;
  input.kinematics.velocity_mps.y_mps = 0.0;
  input.kinematics.velocity_mps.z_mps = 0.0;
  input.carrier_hz = e.carrier_hz;
  input.bandwidth_hz = 2.0e6;
  input.tx_power_w = 5.0e7;
  input.pulse_width_s = 1.0e-6;
  input.pri_s = 1.0e-4;
  input.is_emitting = true;
  return input;
}

SceneState InitScene() {
  esr_config::EsrSessionConfig config;
  std::string error;
  if (!examples::LoadEsrSessionConfigFromFile(SCENE_CONFIG_DIR "/electronic_warfare.json",
                                               &config, &error)) {
    std::cerr << "Failed to load ESR config: " << error << "\n";
    std::exit(1);
  }

  SceneState s;
  s.session = esr_session::EsrSessionFactory::Create(config);

  // 平台位置（空中）
  s.platform_pos.x_m = -2289512.0;
  s.platform_pos.y_m = 4909946.0;
  s.platform_pos.z_m = 3640982.0 + 9000.0;

  s.platform_vel.x_mps = 120.0;
  s.platform_vel.y_mps = -80.0;
  s.platform_vel.z_mps = 30.0;

  s.environment.spectrum_occupancy_ratio = 0.25f;
  s.environment.clutter_density = esr_env::EsrClutterDensityLevel::kMedium;
  s.environment.propagation_profile = esr_env::EsrPropagationEnvironmentProfile::kOpen;

  // 3 个敌方辐射源（地面发射台）
  EmitterState e1;
  e1.id = 1001;
  e1.pos.x_m = -2289512.0; e1.pos.y_m = 4909946.0 + 12000.0; e1.pos.z_m = 3640982.0 + 5000.0;
  e1.carrier_hz = 10.0e9; e1.speed_x_mps = -20.0;

  EmitterState e2;
  e2.id = 1002;
  e2.pos.x_m = -2289512.0; e2.pos.y_m = 4909946.0 + 18000.0; e2.pos.z_m = 3640982.0 + 5000.0;
  e2.carrier_hz = 9.4e9; e2.speed_x_mps = -15.0;

  EmitterState e3;
  e3.id = 1003;
  e3.pos.x_m = -2289512.0; e3.pos.y_m = 4909946.0 + 25000.0; e3.pos.z_m = 3640982.0 + 5000.0;
  e3.carrier_hz = 9.8e9; e3.speed_x_mps = -10.0;

  s.emitters.push_back(e1);
  s.emitters.push_back(e2);
  s.emitters.push_back(e3);

  return s;
}

/// 执行一个仿真周期：推进辐射源和平台位置，执行电子侦察。
esr_session::EsrCycleResult Step(SceneState& s, float dt) {
  ++s.cycle;

  // 推进敌方辐射源位置
  for (auto& e : s.emitters) {
    e.pos.x_m += e.speed_x_mps * dt;
  }

  // 推进平台位置
  s.platform_pos.x_m += s.platform_vel.x_mps * dt;
  s.platform_pos.y_m += s.platform_vel.y_mps * dt;
  s.platform_pos.z_m += s.platform_vel.z_mps * dt;

  esr_session::EsrExternalPoseInput platform;
  platform.platform_position_ecef_m = s.platform_pos;
  platform.platform_velocity_mps = s.platform_vel;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;

  std::vector<esr_session::EsrExternalEmitterInput> emitter_inputs;
  emitter_inputs.reserve(s.emitters.size());
  for (const auto& e : s.emitters) {
    emitter_inputs.push_back(ToEmitterInput(e));
  }

  esr_session::EsrCycleInput input;
  esr_session::EsrCoordinateStatus status;
  if (!esr_session::EsrCycleInputBuilder::Build(
          platform, emitter_inputs, dt, s.environment, &input, &status)) {
    std::cerr << "ESR scene: cycle " << s.cycle
              << " build failed (status=" << static_cast<int>(status) << ")\n";
    std::exit(1);
  }
  input.cycle_index = s.cycle;

  esr_session::EsrCycleResult result = s.session.StepWithResult(input);
  if (result.has_validation_error) ++s.validation_errors;

  std::cout << "esr-scene: cycle=" << result.input_cycle_index
            << " observations=" << result.output_frame.observation_output.observations.size()
            << " hypotheses=" << result.output_frame.emitter_output.hypotheses.size()
            << " associations=" << result.output_frame.truth_evaluation_output.associations.size()
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

  std::cout << "\n=== ESR Scene Summary ===\n"
            << "cycles=" << kNumCycles
            << " validation_errors=" << scene.validation_errors << "\n";

  return scene.validation_errors == 0 ? 0 : 1;
}
