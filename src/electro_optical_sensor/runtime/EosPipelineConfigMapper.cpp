/**
 * @file EosPipelineConfigMapper.cpp
 * @brief 实现 EOS 会话配置到内部执行配置的映射。
 */

#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
void ApplyEnvironmentModelToInternal(
    const environment::EosEnvironmentModelConfig& model_config,
    config::execution::EosInternalExecutionConfig* exec) {
  exec->environment.radiative_transfer_model = model_config.radiative_transfer_model;
  exec->environment.aerosol_density_factor = model_config.aerosol_density_factor;
  exec->environment.turbulence_factor = model_config.turbulence_factor;
}

}  // namespace session
}  // namespace runtime

namespace environment {

environment::EosEnvironmentModelConfig BuildModelConfigFromScenario(
    const environment::EosEnvironmentScenarioConfig& scenario_config) {
  environment::EosEnvironmentModelConfig model_config;
  model_config.model_type = scenario_config.model_type;

  using Model = foundation::radiative_transfer::RadiativeTransferModel;
  if (scenario_config.preset == environment::EosEnvironmentPreset::kHumid) {
    model_config.radiative_transfer_model = Model::kHumidityWeighted;
    model_config.aerosol_density_factor = 1.1f;
    model_config.turbulence_factor = 1.1f;
  } else if (scenario_config.preset == environment::EosEnvironmentPreset::kDusty) {
    model_config.radiative_transfer_model = Model::kAdaptivePathRadiance;
    model_config.aerosol_density_factor = 2.0f;
    model_config.turbulence_factor = 1.2f;
  } else if (scenario_config.preset == environment::EosEnvironmentPreset::kTurbulent) {
    model_config.radiative_transfer_model = Model::kAdaptivePathRadiance;
    model_config.aerosol_density_factor = 1.3f;
    model_config.turbulence_factor = 1.8f;
  } else if (scenario_config.preset == environment::EosEnvironmentPreset::kMaritime) {
    model_config.radiative_transfer_model = Model::kHumidityWeighted;
    model_config.aerosol_density_factor = 1.5f;
    model_config.turbulence_factor = 1.4f;
  } else {
    model_config.radiative_transfer_model = Model::kDerivedBeerLambert;
    model_config.aerosol_density_factor = 1.0f;
    model_config.turbulence_factor = 1.0f;
  }

  if (scenario_config.has_custom_overrides) {
    model_config.radiative_transfer_model =
        scenario_config.custom_overrides.radiative_transfer_model;
    model_config.aerosol_density_factor =
        scenario_config.custom_overrides.aerosol_density_factor;
    model_config.turbulence_factor = scenario_config.custom_overrides.turbulence_factor;
    model_config.enable_optical_countermeasure_extension =
        scenario_config.custom_overrides.enable_optical_countermeasure_extension;
  }

  return model_config;
}

}  // namespace environment

namespace runtime {
namespace session {

config::execution::EosInternalExecutionConfig MapSessionToInternal(
    const ::electro_optical_sensor::config::EosSessionConfig& config) {
  config::execution::EosInternalExecutionConfig exec;
  const environment::EosEnvironmentModelConfig environment_model_config =
      environment::BuildModelConfigFromScenario(config.environment.scenario_config);

  exec.optics = config.hardware;
  exec.scan = config.mission;

  exec.detection = config.policy.detection;
  exec.stray_light = config.policy.stray_light;
  ApplyEnvironmentModelToInternal(environment_model_config, &exec);

  exec.environment.environment_model_type = environment_model_config.model_type;
  return exec;
}

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
