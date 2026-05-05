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

esr_session::EsrSessionConfig MakeHighGainSearchConfig() {
  return esr_config::EsrSessionConfigBuilder()
      .Mission()
      .WithWorkMode(esr_config::EsrWorkMode::kHgesm)
      .WithScanRateHz(2.0f)
      .End()
      .Detection()
      .WithDetectionProfile(esr_config::EsrDetectionProfile::kSensitive)
      .End()
      .Environment()
      .WithEnvironmentPreset(esr_config::EsrEnvironmentPreset::kLowClutter)
      .End()
      .Build();
}

esr_session::EsrSessionConfig MakeDenseJammingEsrConfig() {
  return esr_config::EsrSessionConfigBuilder()
      .Mission()
      .WithWorkMode(esr_config::EsrWorkMode::kRwr)
      .WithScanRateHz(4.0f)
      .End()
      .Detection()
      .WithDetectionProfile(esr_config::EsrDetectionProfile::kSensitive)
      .End()
      .Environment()
      .WithEnvironmentPreset(esr_config::EsrEnvironmentPreset::kJammed)
      .End()
      .Build();
}

esr_session::EsrSession CreateEmitterSearchSession() {
  return esr_session::EsrSession(MakeEmitterSearchConfig());
}

esr_session::EsrSession CreateHighGainSearchSession() {
  return esr_session::EsrSession(MakeHighGainSearchConfig());
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

esr_env::EsrJammerSource MakeJammer(double center_hz) {
  esr_env::EsrJammerSource jammer;
  jammer.technique = esr_env::EsrJammingTechnique::kNoiseSuppression;
  jammer.active = true;
  jammer.center_hz = center_hz;
  jammer.bandwidth_hz = 5.0e6;
  jammer.power_w = 2.0e4f;
  jammer.confidence = 0.85f;
  return jammer;
}

esr_session::EsrCycleInput MakeCycleInput(std::uint32_t cycle_index, bool jammed) {
  esr_session::EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0f;
  input.platform_pose.position_m.z = 9000.0f;
  input.environment.spectrum_occupancy_ratio = jammed ? 0.75f : 0.25f;
  input.environment.clutter_density =
      jammed ? esr_env::EsrClutterDensityLevel::kHigh : esr_env::EsrClutterDensityLevel::kMedium;
  if (jammed) {
    input.environment.jammer_sources.push_back(MakeJammer(10.0e9));
  }
  input.scene.push_back(MakeEmitter("search-radar", 12000.0f, 10.0e9));
  input.scene.push_back(MakeEmitter("tracking-radar", 18000.0f, 9.4e9));
  return input;
}

void PrintResult(const char* label, const esr_session::EsrCycleResult& result) {
  std::cout << label << ": input_cycle=" << result.input_cycle_index
            << " output_cycle=" << result.output_frame.cycle_index
            << " observations=" << result.output_frame.observation_output.observations.size()
            << " hypotheses=" << result.output_frame.emitter_output.hypotheses.size()
            << " associations=" << result.output_frame.truth_evaluation_output.associations.size()
            << " validation_errors=" << (result.has_validation_error ? "true" : "false") << "\n";
}

bool RunRecommendedScenario() {
  esr_session::EsrSession session = CreateEmitterSearchSession();
  esr_session::EsrCycleInput input = MakeCycleInput(1U, false);
  const esr_session::ValidationIssueList issues = esr_session::ValidateEsrCycleInput(input);
  if (esr_session::HasValidationError(issues)) {
    std::cerr << "ESR input is invalid: " << issues.size() << " issues\n";
    return false;
  }
  PrintResult("emitter-search", session.StepWithResult(input));

  const esr_session::EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                                       .WithWorkMode(esr_config::EsrWorkMode::kRwr)
                                                       .WithScanRateHz(3.0f)
                                                       .WithSensorEnabled(true)
                                                       .Build();
  session.ApplyRuntimeConfig(patch);
  PrintResult("runtime-rwr", session.StepWithResult(MakeCycleInput(2U, true)));

  const esr_session::EsrOutputFrame output_only = session.Step(MakeCycleInput(3U, true));
  std::cout << "output-only observations=" << output_only.observation_output.observations.size()
            << "\n";

  esr_session::EsrSession high_gain_session = CreateHighGainSearchSession();
  PrintResult("high-gain", high_gain_session.StepWithResult(MakeCycleInput(10U, false)));

  esr_session::EsrSession jammed_session = esr_session::EsrSession(MakeDenseJammingEsrConfig());
  PrintResult("dense-jamming", jammed_session.StepWithResult(MakeCycleInput(20U, true)));
  return true;
}

}  // namespace

int main() { return RunRecommendedScenario() ? 0 : 1; }
