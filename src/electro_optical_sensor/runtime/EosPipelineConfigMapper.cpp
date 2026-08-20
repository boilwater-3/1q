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
  exec->environment.solar_altitude_deg = model_config.solar_altitude_deg;
  exec->environment.solar_azimuth_deg = model_config.solar_azimuth_deg;
  exec->environment.solar_irradiance_w_m2 = model_config.solar_irradiance_w_m2;
  exec->environment.cloud_coverage_ratio = model_config.cloud_coverage_ratio;
  exec->environment.ambient_wind_speed_mps = model_config.ambient_wind_speed_mps;
  exec->environment.day_night_type = model_config.day_night_type;
  exec->environment.background_temperature_k = model_config.background_temperature_k;
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
  model_config.solar_altitude_deg = scenario_config.solar_altitude_deg;
  model_config.solar_azimuth_deg = scenario_config.solar_azimuth_deg;
  model_config.solar_irradiance_w_m2 = scenario_config.solar_irradiance_w_m2;
  model_config.cloud_coverage_ratio = scenario_config.cloud_coverage_ratio;
  model_config.ambient_wind_speed_mps = scenario_config.ambient_wind_speed_mps;
  model_config.day_night_type = scenario_config.day_night_type;
  model_config.background_temperature_k = scenario_config.background_temperature_k;

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

  exec.sensor_enabled = config.sensor_enabled;
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
