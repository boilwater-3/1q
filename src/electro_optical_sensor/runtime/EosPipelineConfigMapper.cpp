/**
 * @file EosPipelineConfigMapper.cpp
 * @brief 实现 EOS 会话配置到内部执行配置的映射。
 */

#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace {

void ApplyDetectionProfile(config::EosDetectionProfile profile,
                           config::execution::EosInternalExecutionConfig* exec) {
  if (profile == config::EosDetectionProfile::kConservative) {
    exec->detection.minimum_snr_db = 60.0f;
    exec->detection.detection_sensitivity_w = 2.0e-12f;
    exec->detection.visible_reference_irradiance_w_m2 = 1000.0f;
    return;
  }
  if (profile == config::EosDetectionProfile::kAggressive) {
    exec->detection.minimum_snr_db = 4.5f;
    exec->detection.detection_sensitivity_w = 0.8e-12f;
    exec->detection.visible_reference_irradiance_w_m2 = 700.0f;
    return;
  }
  exec->detection.minimum_snr_db = 6.0f;
  exec->detection.detection_sensitivity_w = 1.0e-12f;
  exec->detection.visible_reference_irradiance_w_m2 = 800.0f;
}

void ApplyStrayLightProfile(config::EosStrayLightProfile profile,
                            config::execution::EosInternalExecutionConfig* exec) {
  if (profile == config::EosStrayLightProfile::kEnhancedHood) {
    exec->stray_light.enable_straylight_filter = true;
    exec->stray_light.hood_inner_half_angle_deg = 8.0f;
    exec->stray_light.hood_outer_half_angle_deg = 55.0f;
    exec->stray_light.hood_min_suppression_ratio = 0.35f;
    exec->stray_light.hood_max_suppression_ratio = 0.95f;
    return;
  }
  if (profile == config::EosStrayLightProfile::kStandardHood) {
    exec->stray_light.enable_straylight_filter = true;
    exec->stray_light.hood_inner_half_angle_deg = 12.0f;
    exec->stray_light.hood_outer_half_angle_deg = 75.0f;
    exec->stray_light.hood_min_suppression_ratio = 0.20f;
    exec->stray_light.hood_max_suppression_ratio = 0.85f;
    return;
  }
  exec->stray_light.enable_straylight_filter = false;
  exec->stray_light.hood_inner_half_angle_deg = 12.0f;
  exec->stray_light.hood_outer_half_angle_deg = 75.0f;
  exec->stray_light.hood_min_suppression_ratio = 0.20f;
  exec->stray_light.hood_max_suppression_ratio = 0.85f;
}

}  // namespace

void ApplyDetectionPolicyToInternal(
    const config::EosDetectionPolicyConfig& policy,
    config::execution::EosInternalExecutionConfig* exec) {
  if (policy.use_profile_defaults) {
    ApplyDetectionProfile(policy.profile, exec);
    return;
  }
  exec->detection.minimum_snr_db = policy.minimum_snr_db;
  exec->detection.detection_sensitivity_w = policy.detection_sensitivity_w;
  exec->detection.visible_reference_irradiance_w_m2 =
      policy.visible_reference_irradiance_w_m2;
}

void ApplyStrayLightPolicyToInternal(
    const config::EosStrayLightPolicyConfig& policy,
    config::execution::EosInternalExecutionConfig* exec) {
  if (policy.use_profile_defaults) {
    ApplyStrayLightProfile(policy.profile, exec);
    return;
  }
  exec->stray_light.enable_straylight_filter = policy.enable_straylight_filter;
  exec->stray_light.hood_inner_half_angle_deg = policy.hood_inner_half_angle_deg;
  exec->stray_light.hood_outer_half_angle_deg = policy.hood_outer_half_angle_deg;
  exec->stray_light.hood_min_suppression_ratio = policy.hood_min_suppression_ratio;
  exec->stray_light.hood_max_suppression_ratio = policy.hood_max_suppression_ratio;
}

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

  ApplyDetectionPolicyToInternal(config.policy.detection, &exec);
  ApplyStrayLightPolicyToInternal(config.policy.stray_light, &exec);
  ApplyEnvironmentModelToInternal(environment_model_config, &exec);

  exec.environment.environment_model_type = environment_model_config.model_type;
  return exec;
}

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
