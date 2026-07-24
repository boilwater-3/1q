/** @file integration_demo.cpp @brief ECM/RF v2 无关的 ESR 外部引擎集成示例。 */

#include <cstdint>
#include <iostream>
#include <string>

#include "EsrModule.h"
#include "1q/electromagnetics/RfScene.h"

namespace esr_session = electronic_surveillance_radar::session;

namespace {

esr_session::EsrCycleInput MakeInput(std::uint32_t cycle) {
  esr_session::EsrCycleInput input;
  input.cycle_index = cycle;
  input.cycle_start_time_s = static_cast<double>(cycle - 1U);
  input.dt_sec = 1.0f;
  input.platform_entity_id = 9001U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = -2289512.0;
  input.platform_position_ecef_m.y_m = 4909946.0;
  input.platform_position_ecef_m.z_m = 3650982.0;
  input.environment.propagation_profile = esr_session::EsrPropagationEnvironmentProfile::kOpen;
  input.rf_emissions.world_cycle_index = cycle;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = 1001U;
  emission.identity.equipment_id = 1U;
  emission.identity.emission_id = cycle;
  emission.position_ecef_m = input.platform_position_ecef_m;
  emission.position_ecef_m.y_m += 15000.0;
  emission.antenna.peak_gain_dbi = 30.0;
  emission.polarization = oneq::electromagnetics::RfScenePolarization::kHorizontal;
  if (oneq::electromagnetics::TryCreateRfNoiseWaveform(
          input.cycle_start_time_s, input.dt_sec, 10.0e9, 2.0e6, 5.0e7,
          &emission.waveform)) {
    input.rf_emissions.emissions.push_back(emission);
  }
  return input;
}

}  // namespace

int main(int argc, char** argv) {
  EsrModule esr;
  const std::string config_path = argc > 1 ? argv[1] : "configs/electronic_warfare.json";
  if (!esr.preStart(config_path)) {
    std::cerr << "Could not load ESR configuration\n";
    return 1;
  }
  for (std::uint32_t cycle = 1U; cycle <= 20U; ++cycle) {
    esr.stepImp(MakeInput(cycle));
    const esr_session::EsrCycleResult& result = esr.lastResult();
    std::cout << "cycle=" << result.input_cycle_index
              << " status=" << static_cast<int>(result.status)
              << " observations=" << result.output_frame.observation_output.observations.size()
              << " hypotheses=" << result.output_frame.emitter_output.hypotheses.size() << "\n";
  }
  return 0;
}
