/**
 * @file electronic_surveillance_radar_session_usage.cpp
 * @brief ESR 单周期 RF v2 输入示例。
 */

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "config_loader.h"

namespace esr_config = electronic_surveillance_radar::config;
namespace esr_session = electronic_surveillance_radar::session;

namespace {

esr_config::EsrSessionConfig LoadConfig() {
  esr_config::EsrSessionConfig config;
  std::string error;
  if (!examples::LoadEsrSessionConfigFromFile("configs/electronic_warfare.json", &config,
                                               &error)) {
    std::cerr << "Failed to load ESR config: " << error << "\n";
  }
  return config;
}

esr_session::EsrCycleInput MakeInput(std::uint32_t cycle_index,
                                     const oneq::coordinate::EcefPositionM& platform) {
  esr_session::EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U);
  input.dt_sec = 1.0f;
  input.platform_entity_id = 9001U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m = platform;
  input.environment.propagation_profile = esr_session::EsrPropagationEnvironmentProfile::kOpen;
  input.rf_emissions.world_cycle_index = cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;

  oneq::electromagnetics::RfSceneEmission emitter;
  emitter.identity.platform_id = 1001U;
  emitter.identity.equipment_id = 1U;
  emitter.identity.emission_id = 1U;
  emitter.position_ecef_m = platform;
  emitter.position_ecef_m.y_m += 12000.0;
  emitter.antenna.peak_gain_dbi = 30.0;
  emitter.polarization = oneq::electromagnetics::RfScenePolarization::kHorizontal;
  if (oneq::electromagnetics::TryCreateRfNoiseWaveform(
          input.cycle_start_time_s, input.dt_sec, 10.0e9, 2.0e6, 5.0e7,
          &emitter.waveform)) {
    input.rf_emissions.emissions.push_back(emitter);
  }
  return input;
}

}  // namespace

int main() {
  esr_session::EsrSession session = esr_session::EsrSession::Create(LoadConfig());
  oneq::coordinate::EcefPositionM platform{-2289512.0, 4909946.0, 3650982.0};
  for (std::uint32_t cycle = 1U; cycle <= 10U; ++cycle) {
    const esr_session::EsrCycleResult result = session.StepWithResult(MakeInput(cycle, platform));
    std::cout << "cycle=" << result.input_cycle_index
              << " status=" << static_cast<int>(result.status)
              << " observations=" << result.output_frame.observation_output.observations.size()
              << " hypotheses=" << result.output_frame.emitter_output.hypotheses.size() << "\n";
  }
  return 0;
}
