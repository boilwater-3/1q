/**
 * @file electronic_surveillance_radar_session_usage.cpp
 * @brief Demonstrates recommended ESR session configuration and runtime usage.
 */

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/coordinate/types.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "config_loader.h"

namespace esr_config = electronic_surveillance_radar::config;
namespace esr_env = electronic_surveillance_radar::session;
namespace esr_session = electronic_surveillance_radar::session;

namespace {

esr_config::EsrSessionConfig LoadConfigFromFile() {
  esr_config::EsrSessionConfig config;
  std::string error;
  if (!examples::LoadEsrSessionConfigFromFile("configs/electronic_warfare.json",
                                               &config, &error)) {
    std::cerr << "Failed to load ESR config: " << error << "\n";
    std::exit(1);
  }
  return config;
}

esr_session::EsrSession CreateEmitterSearchSession() {
  return esr_session::EsrSession::Create(LoadConfigFromFile());
}

esr_session::EsrExternalEmitterInput MakeEmitterInput(std::uint64_t id,
                                                      const oneq::coordinate::EcefPositionM& pos,
                                                      double carrier_hz) {
  esr_session::EsrExternalEmitterInput emitter;
  emitter.emitter_id = id;
  emitter.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  emitter.kinematics.position_ecef_m = pos;
  emitter.kinematics.velocity_mps.x_mps = 0.0;
  emitter.kinematics.velocity_mps.y_mps = 0.0;
  emitter.kinematics.velocity_mps.z_mps = 0.0;
  emitter.carrier_hz = carrier_hz;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 5.0e7;
  emitter.pulse_width_s = 1.0e-6;
  emitter.pri_s = 1.0e-4;
  emitter.is_emitting = true;
  return emitter;
}

void PrintResult(const char* label, const esr_session::EsrCycleResult& result) {
  std::cout << label << ": cycle=" << result.input_cycle_index
            << " observations=" << result.output_frame.observation_output.observations.size()
            << " hypotheses=" << result.output_frame.emitter_output.hypotheses.size()
            << " associations=" << result.output_frame.truth_evaluation_output.associations.size()
            << " validation_errors=" << (result.has_validation_error ? "true" : "false") << "\n";
}

struct MovingEsrEmitter {
  std::uint64_t id;
  oneq::coordinate::EcefPositionM pos;
  double carrier_hz;
  double speed_x_mps;
};

MovingEsrEmitter MakeMovingEsrEmitter(std::uint64_t id, double x_m, double y_m, double z_m,
                                      double carrier_hz, double speed_x_mps) {
  MovingEsrEmitter emitter;
  emitter.id = id;
  emitter.pos.x_m = x_m;
  emitter.pos.y_m = y_m;
  emitter.pos.z_m = z_m;
  emitter.carrier_hz = carrier_hz;
  emitter.speed_x_mps = speed_x_mps;
  return emitter;
}

bool RunMovingTargetsScenario() {
  esr_session::EsrSession session = CreateEmitterSearchSession();

  oneq::coordinate::EcefPositionM platform_pos;
  platform_pos.x_m = -2289512.0;
  platform_pos.y_m = 4909946.0;
  platform_pos.z_m = 3640982.0 + 9000.0;

  oneq::coordinate::EcefVelocityMps platform_vel;
  platform_vel.x_mps = 120.0;
  platform_vel.y_mps = -80.0;
  platform_vel.z_mps = 30.0;

  std::vector<MovingEsrEmitter> emitters = {
      MakeMovingEsrEmitter(1001U, -2289512.0, 4909946.0 + 12000.0, 3640982.0 + 5000.0,
                           10.0e9, -20.0),
      MakeMovingEsrEmitter(1002U, -2289512.0, 4909946.0 + 18000.0, 3640982.0 + 5000.0,
                           9.4e9, -15.0),
      MakeMovingEsrEmitter(1003U, -2289512.0, 4909946.0 + 25000.0, 3640982.0 + 5000.0,
                           9.8e9, -10.0),
  };

  esr_session::EsrEnvironmentInput environment;
  environment.spectrum_occupancy_ratio = 0.25f;
  environment.clutter_density = esr_env::EsrClutterDensityLevel::kMedium;
  environment.propagation_profile = esr_env::EsrPropagationEnvironmentProfile::kOpen;

  const std::uint32_t num_cycles = 50;
  std::uint32_t validation_error_count = 0;
  std::uint32_t max_observations = 0;
  std::uint32_t max_hypotheses = 0;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    esr_session::EsrExternalPoseInput platform;
    platform.platform_position_ecef_m = platform_pos;
    platform.platform_velocity_mps = platform_vel;
    platform.platform_attitude_deg.yaw_deg = 0.0;
    platform.platform_attitude_deg.pitch_deg = 0.0;
    platform.platform_attitude_deg.roll_deg = 0.0;

    std::vector<esr_session::EsrExternalEmitterInput> emitter_inputs;
    emitter_inputs.reserve(emitters.size());
    for (const auto& em : emitters) {
      emitter_inputs.push_back(MakeEmitterInput(em.id, em.pos, em.carrier_hz));
    }

    esr_session::EsrCycleInput input;
    esr_session::EsrCoordinateStatus status;
    if (!esr_session::EsrCycleInputAdapter::Build(platform, emitter_inputs, 1.0f, environment,
                                                  &input, &status)) {
      std::cerr << "esr-moving: cycle " << (i + 1)
                << " build failed (status=" << static_cast<int>(status) << ")\n";
      return false;
    }
    input.cycle_index = i + 1;

    esr_session::EsrCycleResult result = session.StepWithResult(input);
    if (result.has_validation_error) ++validation_error_count;

    oneq::coordinate::LocalFrameReference output_reference;
    output_reference.origin_lla.latitude_deg = 30.0;
    output_reference.origin_lla.longitude_deg = 120.0;
    output_reference.origin_lla.altitude_m = 0.0;
    esr_session::EsrExternalOutputFrame external_output;
    const bool external_output_ok = esr_session::EsrCycleOutputAdapter::Build(
        output_reference, input.platform_pose, result.output_frame, &external_output);

    std::size_t nobs = result.output_frame.observation_output.observations.size();
    std::size_t nhyp = result.output_frame.emitter_output.hypotheses.size();
    if (nobs > max_observations) max_observations = static_cast<std::uint32_t>(nobs);
    if (nhyp > max_hypotheses) max_hypotheses = static_cast<std::uint32_t>(nhyp);

    PrintResult("esr-moving", result);
    std::cout << "  external_observations="
              << (external_output_ok ? external_output.observations.size() : 0U)
              << " external_hypotheses="
              << (external_output_ok ? external_output.hypotheses.size() : 0U) << "\n";

    const float dt = 1.0f;
    for (auto& em : emitters) {
      em.pos.x_m += em.speed_x_mps * dt;
    }
  }

  std::cout << "\n=== ESR Summary ===\n"
            << "cycles=" << num_cycles << " max_observations=" << max_observations
            << " max_hypotheses=" << max_hypotheses
            << " validation_errors=" << validation_error_count << "\n";
  return validation_error_count == 0;
}

}  // namespace

int main() { return RunMovingTargetsScenario() ? 0 : 1; }
