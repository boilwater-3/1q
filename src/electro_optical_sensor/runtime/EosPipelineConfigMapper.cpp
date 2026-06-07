/**
 * @file EosPipelineConfigMapper.cpp
 * @brief 实现 EOS 会话配置到 pipeline 配置的内部映射。
 */

#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace {

void ApplyDetectionProfile(config::EosDetectionProfile profile,
                           ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  if (profile == config::EosDetectionProfile::kConservative) {
    config_out->detection_policy.minimum_snr_db = 60.0f;
    config_out->detection_policy.detection_sensitivity_w = 2.0e-12f;
    config_out->detection_policy.visible_reference_irradiance_w_m2 = 1000.0f;
    return;
  }
  if (profile == config::EosDetectionProfile::kAggressive) {
    config_out->detection_policy.minimum_snr_db = 4.5f;
    config_out->detection_policy.detection_sensitivity_w = 0.8e-12f;
    config_out->detection_policy.visible_reference_irradiance_w_m2 = 700.0f;
    return;
  }
  config_out->detection_policy.minimum_snr_db = 6.0f;
  config_out->detection_policy.detection_sensitivity_w = 1.0e-12f;
  config_out->detection_policy.visible_reference_irradiance_w_m2 = 800.0f;
}

void ApplyDetectionPolicy(
    const config::EosDetectionPolicyConfig& detection,
    ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  if (detection.use_profile_defaults) {
    ApplyDetectionProfile(detection.profile, config_out);
    return;
  }
  config_out->detection_policy.minimum_snr_db = detection.minimum_snr_db;
  config_out->detection_policy.detection_sensitivity_w = detection.detection_sensitivity_w;
  config_out->detection_policy.visible_reference_irradiance_w_m2 =
      detection.visible_reference_irradiance_w_m2;
}

void ApplyStrayLightProfile(
    config::EosStrayLightProfile profile,
    ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  if (profile == config::EosStrayLightProfile::kEnhancedHood) {
    config_out->stray_light_policy.enable_straylight_filter = true;
    config_out->stray_light_policy.hood_inner_half_angle_deg = 8.0f;
    config_out->stray_light_policy.hood_outer_half_angle_deg = 55.0f;
    config_out->stray_light_policy.hood_min_suppression_ratio = 0.35f;
    config_out->stray_light_policy.hood_max_suppression_ratio = 0.95f;
    return;
  }
  if (profile == config::EosStrayLightProfile::kStandardHood) {
    config_out->stray_light_policy.enable_straylight_filter = true;
    config_out->stray_light_policy.hood_inner_half_angle_deg = 12.0f;
    config_out->stray_light_policy.hood_outer_half_angle_deg = 75.0f;
    config_out->stray_light_policy.hood_min_suppression_ratio = 0.20f;
    config_out->stray_light_policy.hood_max_suppression_ratio = 0.85f;
    return;
  }
  config_out->stray_light_policy.enable_straylight_filter = false;
  config_out->stray_light_policy.hood_inner_half_angle_deg = 12.0f;
  config_out->stray_light_policy.hood_outer_half_angle_deg = 75.0f;
  config_out->stray_light_policy.hood_min_suppression_ratio = 0.20f;
  config_out->stray_light_policy.hood_max_suppression_ratio = 0.85f;
}

void ApplyStrayLightPolicy(
    const config::EosStrayLightPolicyConfig& stray_light,
    ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  if (stray_light.use_profile_defaults) {
    ApplyStrayLightProfile(stray_light.profile, config_out);
    return;
  }
  config_out->stray_light_policy.enable_straylight_filter = stray_light.enable_straylight_filter;
  config_out->stray_light_policy.hood_inner_half_angle_deg =
      stray_light.hood_inner_half_angle_deg;
  config_out->stray_light_policy.hood_outer_half_angle_deg =
      stray_light.hood_outer_half_angle_deg;
  config_out->stray_light_policy.hood_min_suppression_ratio =
      stray_light.hood_min_suppression_ratio;
  config_out->stray_light_policy.hood_max_suppression_ratio =
      stray_light.hood_max_suppression_ratio;
}

void ApplyEnvironmentModelConfig(
    const environment::EosEnvironmentModelConfig& environment_model_config,
    ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  config_out->radiative_transfer_model = environment_model_config.radiative_transfer_model;
  config_out->aerosol_density_factor = environment_model_config.aerosol_density_factor;
  config_out->turbulence_factor = environment_model_config.turbulence_factor;
}

}  // namespace

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

::electro_optical_sensor::extension::EosPipelineConfig BuildEosPipelineConfig(
    const ::electro_optical_sensor::config::EosSessionConfig& config) {
  ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config;
  const environment::EosEnvironmentModelConfig environment_model_config =
      environment::BuildModelConfigFromScenario(config.environment.scenario_config);

  pipeline_config.hardware = config.hardware;
  pipeline_config.mission = config.mission;

  ApplyDetectionPolicy(config.policy.detection, &pipeline_config);
  ApplyStrayLightPolicy(config.policy.stray_light, &pipeline_config);
  ApplyEnvironmentModelConfig(environment_model_config, &pipeline_config);

  pipeline_config.environment_model_type = environment_model_config.model_type;
  return pipeline_config;
}

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
