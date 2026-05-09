/**
 * @file electronic_surveillance_radar_session_usage.cpp
 * @brief Demonstrates recommended ESR session configuration and runtime usage.
 */

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/coordinate/types.h"

namespace esr_config = electronic_surveillance_radar::config;
namespace esr_env = electronic_surveillance_radar::environment;
namespace esr_session = electronic_surveillance_radar::session;

namespace {

esr_session::EsrSessionConfig MakeEmitterSearchConfig() {
  esr_session::EsrSessionConfig config =
      esr_config::EsrSessionConfigBuilder()
          .Mission()
          .WithWorkMode(esr_config::EsrWorkMode::kEsm)
          .WithPowerOn(true)
          .WithScanRateHz(1.0f)
          .End()
          .Detection()
          .WithDetectionProfile(esr_config::EsrDetectionProfile::kBalanced)
          .End()
          .Environment()
          .WithEnvironmentPreset(esr_config::EsrEnvironmentPreset::kStandard)
          .End()
          .Build();
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;
  return config;
}

esr_session::EsrSession CreateEmitterSearchSession() {
  return esr_session::EsrSession(MakeEmitterSearchConfig());
}

esr_session::EsrExternalEmitterInput MakeEmitterInput(
    const std::string& id,
    const oneq::coordinate::EcefPositionM& pos,
    double carrier_hz) {
  esr_session::EsrExternalEmitterInput emitter;
  emitter.emitter_id = id;
  emitter.emitter_position_ecef_m = pos;
  emitter.emitter_velocity_mps.x_mps = 0.0;
  emitter.emitter_velocity_mps.y_mps = 0.0;
  emitter.emitter_velocity_mps.z_mps = 0.0;
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
  std::string id;
  oneq::coordinate::EcefPositionM pos;
  double carrier_hz;
  double speed_x_mps;
};

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
    {"search-radar",  {-2289512.0 + 12000.0, 4909946.0, 3640982.0 + 5000.0}, 10.0e9,  50.0},
    {"tracking-radar",{-2289512.0 + 18000.0, 4909946.0, 3640982.0 + 5000.0}, 9.4e9, -30.0},
    {"threat-emitter",{-2289512.0 + 25000.0, 4909946.0, 3640982.0 + 5000.0}, 9.8e9, -80.0},
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
      emitter_inputs.push_back(
          MakeEmitterInput(em.id, em.pos, em.carrier_hz));
    }

    esr_session::EsrCycleInput input;
    esr_session::EsrCoordinateStatus status;
    if (!esr_session::EsrCycleInputBuilder::Build(
            platform, emitter_inputs, 1.0f,
            environment, &input, &status)) {
      std::cerr << "esr-moving: cycle " << (i + 1)
                << " build failed (status=" << static_cast<int>(status) << ")\n";
      return false;
    }
    input.cycle_index = i + 1;

    esr_session::EsrCycleResult result = session.StepWithResult(input);
    if (result.has_validation_error) ++validation_error_count;

    esr_session::EsrCoordinateReference output_reference;
    output_reference.origin_lla.latitude_deg = 30.0;
    output_reference.origin_lla.longitude_deg = 120.0;
    output_reference.origin_lla.altitude_m = 0.0;
    esr_session::EsrExternalOutputFrame external_output;
    const bool external_output_ok = esr_session::EsrCycleOutputBuilder::Build(
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
