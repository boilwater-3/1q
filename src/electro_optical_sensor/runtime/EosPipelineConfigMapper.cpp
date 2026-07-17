/**
 * @file EosPipelineConfigMapper.cpp
 * @brief 实现 EOS 会话配置到内部执行配置的映射。
 */

#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
void ApplyEnvironmentModelToInternal(
    const config::execution::EnvironmentConfig& model_config,
    config::execution::EosInternalExecutionConfig* exec) {
  exec->environment.radiative_transfer_model = model_config.radiative_transfer_model;
  exec->environment.aerosol_density_factor = model_config.aerosol_density_factor;
  exec->environment.turbulence_factor = model_config.turbulence_factor;
  exec->environment.atmospheric_physics = model_config.atmospheric_physics;
}

}  // namespace session
}  // namespace runtime

namespace config {

execution::EnvironmentConfig BuildModelConfigFromScenario(
    const config::EosEnvironmentScenarioConfig& scenario_config) {
  execution::EnvironmentConfig model_config;

  using Model = execution::RadiativeTransferModel;
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

  model_config.atmospheric_physics = scenario_config.atmospheric_physics;

  return model_config;
}

}  // namespace config

namespace runtime {
namespace session {

config::execution::EosInternalExecutionConfig MapSessionToInternal(
    const ::electro_optical_sensor::config::EosSessionConfig& config) {
  config::execution::EosInternalExecutionConfig exec;
  const config::execution::EnvironmentConfig environment_model_config =
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
