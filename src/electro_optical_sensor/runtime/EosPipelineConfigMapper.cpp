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

::electro_optical_sensor::extension::EosPipelineConfig InternalToPipelineConfig(
    const config::execution::EosInternalExecutionConfig& internal) {
  ::electro_optical_sensor::extension::EosPipelineConfig config;
  config.wavelength_lower_um = internal.optics.wavelength_lower_um;
  config.wavelength_upper_um = internal.optics.wavelength_upper_um;
  config.optical_aperture_m = internal.optics.optical_aperture_m;
  config.focal_length_m = internal.optics.focal_length_m;
  config.work_mode = internal.scan.work_mode;
  config.horizontal_fov_deg = internal.scan.horizontal_fov_deg;
  config.vertical_fov_deg = internal.scan.vertical_fov_deg;
  config.scan_rate_deg_per_sec = internal.scan.scan_rate_deg_per_sec;
  config.frame_rate_hz = internal.scan.frame_rate_hz;
  config.scan_start_az_deg = internal.scan.scan_start_az_deg;
  config.scan_end_az_deg = internal.scan.scan_end_az_deg;
  config.scan_center_el_deg = internal.scan.scan_center_el_deg;
  config.boresight_depression_deg = internal.scan.boresight_depression_deg;
  config.minimum_snr_db = internal.detection.minimum_snr_db;
  config.detection_sensitivity_w = internal.detection.detection_sensitivity_w;
  config.visible_reference_irradiance_w_m2 =
      internal.detection.visible_reference_irradiance_w_m2;
  config.enable_straylight_filter = internal.stray_light.enable_straylight_filter;
  config.hood_inner_half_angle_deg = internal.stray_light.hood_inner_half_angle_deg;
  config.hood_outer_half_angle_deg = internal.stray_light.hood_outer_half_angle_deg;
  config.hood_min_suppression_ratio = internal.stray_light.hood_min_suppression_ratio;
  config.hood_max_suppression_ratio = internal.stray_light.hood_max_suppression_ratio;
  config.environment_model_type = internal.environment.environment_model_type;
  config.radiative_transfer_model = internal.environment.radiative_transfer_model;
  config.aerosol_density_factor = internal.environment.aerosol_density_factor;
  config.turbulence_factor = internal.environment.turbulence_factor;
  config.detector_detectivity_cm_sqrt_hz_per_w =
      internal.detector.detector_detectivity_cm_sqrt_hz_per_w;
  config.detector_area_cm2 = internal.detector.detector_area_cm2;
  config.min_detection_depression_deg = internal.detector.min_detection_depression_deg;
  config.max_detection_depression_deg = internal.detector.max_detection_depression_deg;
  return config;
}

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
