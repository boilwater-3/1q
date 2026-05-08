/**
 * @file electronic_surveillance_radar_session_usage.cpp
 * @brief Demonstrates recommended ESR session configuration and runtime usage.
 */

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"

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

esr_session::EsrSceneEmitter MakeEmitter(const std::string& id, float x_m, double carrier_hz) {
  esr_session::EsrSceneEmitter emitter;
  emitter.emitter_id = id;
  emitter.pose.position_m.x = x_m;
  emitter.pose.position_m.z = 5000.0f;
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
  float x_m;
  double carrier_hz;
  float speed_x_mps;
};

bool RunMovingTargetsScenario() {
  esr_session::EsrSession session = CreateEmitterSearchSession();

  std::vector<MovingEsrEmitter> emitters = {
    {"search-radar",  12000.0f, 10.0e9,  50.0f},
    {"tracking-radar", 18000.0f, 9.4e9, -30.0f},
    {"threat-emitter", 25000.0f, 9.8e9, -80.0f},
  };

  const std::uint32_t num_cycles = 50;
  std::uint32_t validation_error_count = 0;
  std::uint32_t max_observations = 0;
  std::uint32_t max_hypotheses = 0;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    esr_session::EsrCycleInput input;
    input.cycle_index = i + 1;
    input.dt_sec = 1.0f;
    input.platform_pose.position_m.z = 9000.0f;
    input.environment.spectrum_occupancy_ratio = 0.25f;
    input.environment.clutter_density = esr_env::EsrClutterDensityLevel::kMedium;

    for (const auto& em : emitters) {
      input.scene.push_back(MakeEmitter(em.id, em.x_m, em.carrier_hz));
    }

    const float dt = input.dt_sec;
    for (auto& em : emitters) {
      em.x_m += em.speed_x_mps * dt;
    }

    esr_session::EsrCycleResult result = session.StepWithResult(input);
    if (result.has_validation_error) ++validation_error_count;

    std::size_t nobs = result.output_frame.observation_output.observations.size();
    std::size_t nhyp = result.output_frame.emitter_output.hypotheses.size();
    if (nobs > max_observations) max_observations = nobs;
    if (nhyp > max_hypotheses) max_hypotheses = nhyp;

    PrintResult("esr-moving", result);
  }

  std::cout << "\n=== ESR Summary ===\n"
            << "cycles=" << num_cycles
            << " max_observations=" << max_observations
            << " max_hypotheses=" << max_hypotheses
            << " validation_errors=" << validation_error_count << "\n";
  return validation_error_count == 0;
}

}  // namespace

int main() { return RunMovingTargetsScenario() ? 0 : 1; }
