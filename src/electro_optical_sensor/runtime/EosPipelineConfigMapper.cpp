/**
 * @file EosPipelineConfigMapper.cpp
 * @brief 实现 EOS 会话配置到 pipeline 配置的内部映射。
 */

#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace internal {
namespace {

::electro_optical_sensor::extension::EosPipelineWorkMode ToPipelineWorkMode(
    ::electro_optical_sensor::session::EosWorkMode mode) {
  if (mode == ::electro_optical_sensor::session::EosWorkMode::kInfraredOnly) {
    return ::electro_optical_sensor::extension::EosPipelineWorkMode::kInfraredOnly;
  }
  if (mode == ::electro_optical_sensor::session::EosWorkMode::kVisibleOnly) {
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
    config_out->minimum_snr_db = 8.0f;
    config_out->detection_sensitivity_w = 1.4e-12f;
    config_out->visible_reference_irradiance_w_m2 = 900.0f;
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

void ApplyStrayLightProfile(config::EosStrayLightProfile profile,
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

void ApplyEnvironmentPreset(config::EosEnvironmentPreset preset,
                            ::electro_optical_sensor::extension::EosPipelineConfig* config_out) {
  using Model = foundation::radiative_transfer::RadiativeTransferModel;
  if (preset == config::EosEnvironmentPreset::kHumid) {
    config_out->radiative_transfer_model = Model::kHumidityWeighted;
    config_out->aerosol_density_factor = 1.1f;
    config_out->turbulence_factor = 1.1f;
    return;
  }
  if (preset == config::EosEnvironmentPreset::kDusty) {
    config_out->radiative_transfer_model = Model::kAdaptivePathRadiance;
    config_out->aerosol_density_factor = 2.0f;
    config_out->turbulence_factor = 1.2f;
    return;
  }
  if (preset == config::EosEnvironmentPreset::kTurbulent) {
    config_out->radiative_transfer_model = Model::kAdaptivePathRadiance;
    config_out->aerosol_density_factor = 1.3f;
    config_out->turbulence_factor = 1.8f;
    return;
  }
  if (preset == config::EosEnvironmentPreset::kMaritime) {
    config_out->radiative_transfer_model = Model::kHumidityWeighted;
    config_out->aerosol_density_factor = 1.5f;
    config_out->turbulence_factor = 1.4f;
    return;
  }
  config_out->radiative_transfer_model = Model::kDerivedBeerLambert;
  config_out->aerosol_density_factor = 1.0f;
  config_out->turbulence_factor = 1.0f;
}

}  // namespace

::electro_optical_sensor::extension::EosPipelineConfig BuildEosPipelineConfig(
    const ::electro_optical_sensor::session::EosSessionConfig& config) {
  ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config;
  pipeline_config.wavelength_lower_um = config.optical.wavelength_lower_um;
  pipeline_config.wavelength_upper_um = config.optical.wavelength_upper_um;
  pipeline_config.optical_aperture_m = config.optical.optical_aperture_m;
  pipeline_config.focal_length_m = config.optical.focal_length_m;
  pipeline_config.work_mode = ToPipelineWorkMode(config.scan.work_mode);
  pipeline_config.horizontal_fov_deg = config.scan.horizontal_fov_deg;
  pipeline_config.vertical_fov_deg = config.scan.vertical_fov_deg;
  pipeline_config.scan_rate_deg_per_sec = config.scan.scan_rate_deg_per_sec;
  pipeline_config.frame_rate_hz = config.scan.frame_rate_hz;
  pipeline_config.scan_start_az_deg = config.pointing.scan_start_az_deg;
  pipeline_config.scan_end_az_deg = config.pointing.scan_end_az_deg;
  pipeline_config.scan_center_el_deg = config.pointing.scan_center_el_deg;
  pipeline_config.boresight_depression_deg = config.pointing.boresight_depression_deg;
  pipeline_config.min_detection_depression_deg = 1.0f;
  pipeline_config.max_detection_depression_deg = 89.0f;

  ApplyDetectionProfile(config.detection.profile, &pipeline_config);
  ApplyStrayLightProfile(config.stray_light.profile, &pipeline_config);
  ApplyEnvironmentPreset(config.environment.preset, &pipeline_config);

  pipeline_config.environment_model_type =
      ToPipelineEnvironmentModelType(config.environment.model_type);
  return pipeline_config;
}

}  // namespace internal
}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
