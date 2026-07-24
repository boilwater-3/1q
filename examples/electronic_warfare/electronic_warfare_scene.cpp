/** @file electronic_warfare_scene.cpp @brief ESR RF v2 场景输入示例。 */

#include <cstdint>
#include <iostream>
#include <string>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "config_loader.h"

namespace esr_config = electronic_surveillance_radar::config;
namespace esr_session = electronic_surveillance_radar::session;

namespace {

esr_session::EsrCycleInput MakeSceneInput(std::uint32_t cycle) {
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
  for (std::uint64_t index = 0U; index < 3U; ++index) {
    oneq::electromagnetics::RfSceneEmission emission;
    emission.identity.platform_id = 1001U + index;
    emission.identity.equipment_id = 1U;
    emission.identity.emission_id = index + 1U;
    emission.position_ecef_m = input.platform_position_ecef_m;
    emission.position_ecef_m.y_m += 12000.0 + 6000.0 * static_cast<double>(index);
    emission.position_ecef_m.x_m -= 20.0 * static_cast<double>(cycle);
    emission.antenna.peak_gain_dbi = 30.0;
    emission.polarization = oneq::electromagnetics::RfScenePolarization::kHorizontal;
    if (oneq::electromagnetics::TryCreateRfNoiseWaveform(
            input.cycle_start_time_s, input.dt_sec, 9.4e9 + index * 0.3e9, 2.0e6, 5.0e7,
            &emission.waveform)) {
      input.rf_emissions.emissions.push_back(emission);
    }
  }
  return input;
}

}  // namespace

int main() {
  esr_config::EsrSessionConfig config;
  std::string error;
  (void)examples::LoadEsrSessionConfigFromFile(SCENE_CONFIG_DIR "/electronic_warfare.json",
                                                &config, &error);
  esr_session::EsrSession session = esr_session::EsrSession::Create(config);
  for (std::uint32_t cycle = 1U; cycle <= 20U; ++cycle) {
    const esr_session::EsrCycleResult result = session.StepWithResult(MakeSceneInput(cycle));
    std::cout << "cycle=" << cycle << " observations="
              << result.output_frame.observation_output.observations.size() << " hypotheses="
              << result.output_frame.emitter_output.hypotheses.size() << "\n";
  }
  return 0;
}
