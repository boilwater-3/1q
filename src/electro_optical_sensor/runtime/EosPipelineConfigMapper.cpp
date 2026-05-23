/**
 * @file EosPipelineConfigMapper.cpp
 * @brief 实现 EOS 会话配置到 pipeline 配置的内部映射。
 */

#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace {

::electro_optical_sensor::extension::EosPipelineWorkMode ToPipelineWorkMode(
    ::electro_optical_sensor::config::EosWorkMode mode) {
  if (mode == ::electro_optical_sensor::config::EosWorkMode::kInfraredOnly) {
    return ::electro_optical_sensor::extension::EosPipelineWorkMode::kInfraredOnly;
  }
  if (mode == ::electro_optical_sensor::config::EosWorkMode::kVisibleOnly) {
    return ::electro_optical_sensor::extension::EosPipelineWorkMode::kVisibleOnly;
  }
  return ::electro_optical_sensor::extension::EosPipelineWorkMode::kFused;
}

::electro_optical_sensor::extension::EosPipelineEnvironmentModelType ToPipelineEnvironmentModelType(
    environment::EosEnvironmentModelType model_type) {
  if (model_type == environment::EosEnvironmentModelType::kAdvanced) {
    return ::electro_optical_sensor::extension::EosPipelineEnvironmentModelType::kAdvanced;
  }
  return ::electro_optical_sensor::extension::EosPipelineEnvironmentModelType::kSimplified;
}

void ApplyDetectionProfile(config::EosDetectionProfile profile,
                           ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  if (profile == config::EosDetectionProfile::kConservative) {
    config_out->minimum_snr_db = 60.0f;
    config_out->detection_sensitivity_w = 2.0e-12f;
    config_out->visible_reference_irradiance_w_m2 = 1000.0f;
    return;
  }
  if (profile == config::EosDetectionProfile::kAggressive) {
    config_out->minimum_snr_db = 4.5f;
    config_out->detection_sensitivity_w = 0.8e-12f;
    config_out->visible_reference_irradiance_w_m2 = 700.0f;
    return;
  }
  config_out->minimum_snr_db = 6.0f;
  config_out->detection_sensitivity_w = 1.0e-12f;
  config_out->visible_reference_irradiance_w_m2 = 800.0f;
}

void ApplyDetectionPolicy(
    const config::EosDetectionPolicyConfig& detection,
    ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  if (detection.use_profile_defaults) {
    ApplyDetectionProfile(detection.profile, config_out);
    return;
  }
  config_out->minimum_snr_db = detection.minimum_snr_db;
  config_out->detection_sensitivity_w = detection.detection_sensitivity_w;
  config_out->visible_reference_irradiance_w_m2 = detection.visible_reference_irradiance_w_m2;
}

void ApplyStrayLightProfile(
    config::EosStrayLightProfile profile,
    ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  if (profile == config::EosStrayLightProfile::kEnhancedHood) {
    config_out->enable_straylight_filter = true;
    config_out->hood_inner_half_angle_deg = 8.0f;
    config_out->hood_outer_half_angle_deg = 55.0f;
    config_out->hood_min_suppression_ratio = 0.35f;
    config_out->hood_max_suppression_ratio = 0.95f;
    return;
  }
  if (profile == config::EosStrayLightProfile::kStandardHood) {
    config_out->enable_straylight_filter = true;
    config_out->hood_inner_half_angle_deg = 12.0f;
    config_out->hood_outer_half_angle_deg = 75.0f;
    config_out->hood_min_suppression_ratio = 0.20f;
    config_out->hood_max_suppression_ratio = 0.85f;
    return;
  }
  config_out->enable_straylight_filter = false;
  config_out->hood_inner_half_angle_deg = 12.0f;
  config_out->hood_outer_half_angle_deg = 75.0f;
  config_out->hood_min_suppression_ratio = 0.20f;
  config_out->hood_max_suppression_ratio = 0.85f;
}

void ApplyStrayLightPolicy(
    const config::EosStrayLightPolicyConfig& stray_light,
    ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  if (stray_light.use_profile_defaults) {
    ApplyStrayLightProfile(stray_light.profile, config_out);
    return;
  }
  config_out->enable_straylight_filter = stray_light.enable_straylight_filter;
  config_out->hood_inner_half_angle_deg = stray_light.hood_inner_half_angle_deg;
  config_out->hood_outer_half_angle_deg = stray_light.hood_outer_half_angle_deg;
  config_out->hood_min_suppression_ratio = stray_light.hood_min_suppression_ratio;
  config_out->hood_max_suppression_ratio = stray_light.hood_max_suppression_ratio;
}

void ApplyEnvironmentModelConfig(
    const environment::EosEnvironmentModelConfig& environment_model_config,
    ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  config_out->radiative_transfer_model = environment_model_config.radiative_transfer_model;
  config_out->aerosol_density_factor = environment_model_config.aerosol_density_factor;
  config_out->turbulence_factor = environment_model_config.turbulence_factor;
}

}  // namespace

::electro_optical_sensor::extension::EosPipelineConfig BuildEosPipelineConfig(
    const ::electro_optical_sensor::config::EosSessionConfig& config) {
  ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config;
  const environment::EosEnvironmentModelConfig environment_model_config =
      environment::BuildModelConfigFromScenario(config.environment.scenario_config);
  pipeline_config.wavelength_lower_um = config.hardware.wavelength_lower_um;
  pipeline_config.wavelength_upper_um = config.hardware.wavelength_upper_um;
  pipeline_config.optical_aperture_m = config.hardware.optical_aperture_m;
  pipeline_config.focal_length_m = config.hardware.focal_length_m;
  pipeline_config.work_mode = ToPipelineWorkMode(config.mission.work_mode);
  pipeline_config.horizontal_fov_deg = config.mission.horizontal_fov_deg;
  pipeline_config.vertical_fov_deg = config.mission.vertical_fov_deg;
  pipeline_config.scan_rate_deg_per_sec = config.mission.scan_rate_deg_per_sec;
  pipeline_config.frame_rate_hz = config.mission.frame_rate_hz;
  pipeline_config.scan_start_az_deg = config.mission.scan_start_az_deg;
  pipeline_config.scan_end_az_deg = config.mission.scan_end_az_deg;
  pipeline_config.scan_center_el_deg = config.mission.scan_center_el_deg;
  pipeline_config.boresight_depression_deg = config.mission.boresight_depression_deg;
  pipeline_config.min_detection_depression_deg = 1.0f;
  pipeline_config.max_detection_depression_deg = 89.0f;

  ApplyDetectionPolicy(config.policy.detection, &pipeline_config);
  ApplyStrayLightPolicy(config.policy.stray_light, &pipeline_config);
  ApplyEnvironmentModelConfig(environment_model_config, &pipeline_config);

  pipeline_config.environment_model_type =
      ToPipelineEnvironmentModelType(environment_model_config.model_type);
  return pipeline_config;
}

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
