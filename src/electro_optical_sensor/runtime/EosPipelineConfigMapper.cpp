/**
 * @file EosPipelineConfigMapper.cpp
 * @brief 实现 EOS 会话配置到内部执行配置的映射。
 */

#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
void ApplyEnvironmentModelToInternal(
    const config::EosEnvironmentModelConfig& model_config,
    config::execution::EosInternalExecutionConfig* exec) {
  exec->environment.model_type = model_config.model_type;
  exec->environment.radiative_transfer_model = model_config.radiative_transfer_model;
  exec->environment.aerosol_density_factor = model_config.aerosol_density_factor;
  exec->environment.turbulence_factor = model_config.turbulence_factor;
  exec->environment.has_atmospheric_observation = model_config.has_atmospheric_observation;
  exec->environment.atmospheric_observation = model_config.atmospheric_observation;
}

}  // namespace session
}  // namespace runtime

namespace config {

EosEnvironmentModelConfig BuildModelConfigFromScenario(
    const config::EosEnvironmentScenarioConfig& scenario_config) {
  config::EosEnvironmentModelConfig model_config;
  model_config.model_type = scenario_config.model_type;

  using Model = foundation::radiative_transfer::RadiativeTransferModel;
  if (scenario_config.preset == config::EosEnvironmentPreset::kHumid) {
    model_config.radiative_transfer_model = Model::kHumidityWeighted;
    model_config.aerosol_density_factor = 1.1f;
    model_config.turbulence_factor = 1.1f;
  } else if (scenario_config.preset == config::EosEnvironmentPreset::kDusty) {
    model_config.radiative_transfer_model = Model::kAdaptivePathRadiance;
    model_config.aerosol_density_factor = 2.0f;
    model_config.turbulence_factor = 1.2f;
  } else if (scenario_config.preset == config::EosEnvironmentPreset::kTurbulent) {
    model_config.radiative_transfer_model = Model::kAdaptivePathRadiance;
    model_config.aerosol_density_factor = 1.3f;
    model_config.turbulence_factor = 1.8f;
  } else if (scenario_config.preset == config::EosEnvironmentPreset::kMaritime) {
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
  }

  // 传递可选大气物理观测
  model_config.has_atmospheric_observation = scenario_config.has_atmospheric_observation;
  model_config.atmospheric_observation = scenario_config.atmospheric_observation;

  return model_config;
}

}  // namespace config

namespace runtime {
namespace session {

config::execution::EosInternalExecutionConfig MapSessionToInternal(
    const ::electro_optical_sensor::config::EosSessionConfig& config) {
  config::execution::EosInternalExecutionConfig exec;
  const config::EosEnvironmentModelConfig environment_model_config =
      BuildModelConfigFromScenario(config.environment.scenario_config);

  exec.sensor_enabled = config.mission.power_on;
  exec.optics = config.hardware;
  exec.detector.detector_detectivity_cm_sqrt_hz_per_w = config.hardware.detector_detectivity_cm_sqrt_hz_per_w;
  exec.detector.detector_area_cm2 = config.hardware.detector_area_cm2;
  exec.detector.min_detection_depression_deg = config.hardware.min_detection_depression_deg;
  exec.detector.max_detection_depression_deg = config.hardware.max_detection_depression_deg;
  exec.scan = config.mission;

  exec.detection = config.policy.detection;
  exec.stray_light = config.policy.stray_light;
  ApplyEnvironmentModelToInternal(environment_model_config, &exec);

  return exec;
}

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
