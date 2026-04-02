#include "electro_optical_sensor/core/pipeline/EosPipeline.h"

#include <algorithm>
#include <cmath>

#include "1q/electro_optical_sensor/foundation/EosNoiseModel.h"
#include "1q/electro_optical_sensor/foundation/EosOpticalCharacteristics.h"
#include "1q/electro_optical_sensor/foundation/EosPropagation.h"
#include "1q/electro_optical_sensor/foundation/EosRadiometry.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"
#include "1q/electro_optical_sensor/foundation/EosSpatialSpectrum.h"
#include "1q/electro_optical_sensor/foundation/EosStrayLight.h"
#include "electro_optical_sensor/environment/EosEnvironmentModel.h"

namespace electro_optical_sensor {
namespace core {
namespace pipeline {

namespace {

float Clamp(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

float Clamp01(float value) { return Clamp(value, 0.0f, 1.0f); }

float SafePositive(float value, float fallback) {
  if (std::isfinite(value) == 0 || value <= 0.0f) {
    return fallback;
  }
  return value;
}

float NormalizeAngle180(float angle_deg) {
  float normalized = angle_deg;
  while (normalized > 180.0f) {
    normalized -= 360.0f;
  }
  while (normalized < -180.0f) {
    normalized += 360.0f;
  }
  return normalized;
}

bool WorkModeIncludesInfrared(EosPipelineWorkMode mode) {
  return mode == EosPipelineWorkMode::kInfraredOnly || mode == EosPipelineWorkMode::kFused;
}

bool WorkModeIncludesVisible(EosPipelineWorkMode mode) {
  return mode == EosPipelineWorkMode::kVisibleOnly || mode == EosPipelineWorkMode::kFused;
}

foundation::radiometry::IlluminationCondition ToIlluminationCondition(
    context::DayNightType day_night_type) {
  if (day_night_type == context::DayNightType::kNight) {
    return foundation::radiometry::IlluminationCondition::kNight;
  }
  if (day_night_type == context::DayNightType::kTwilight) {
    return foundation::radiometry::IlluminationCondition::kTwilight;
  }
  return foundation::radiometry::IlluminationCondition::kDay;
}

float ComputeApertureAreaM2(float optical_aperture_m) {
  const float safe_diameter_m = SafePositive(optical_aperture_m, 0.2f);
  const float radius_m = 0.5f * safe_diameter_m;
  return 3.1415926f * radius_m * radius_m;
}

foundation::radiative_transfer::RadiativeTransferResult ComputePathRadiativeTransfer(
    const EosPipelineConfig& config, const context::EosCycleInput& input, float range_m,
    float aerosol_density_factor, float turbulence_factor) {
  foundation::radiative_transfer::RadiativeTransferInputs transfer_inputs;
  transfer_inputs.model = config.radiative_transfer_model;
  transfer_inputs.base_transmittance = Clamp01(input.atmospheric_transmittance);
  transfer_inputs.cloud_coverage_ratio = Clamp01(input.cloud_coverage_ratio);
  transfer_inputs.path_length_m = std::max(0.0f, range_m);
  transfer_inputs.aerosol_density_factor = std::max(1.0f, aerosol_density_factor);
  transfer_inputs.turbulence_factor = std::max(1.0f, turbulence_factor);
  return foundation::radiative_transfer::EvaluateRadiativeTransfer(transfer_inputs);
}

environment::EosEnvironmentModelType ToEnvironmentModelType(
    EosPipelineEnvironmentModelType model_type) {
  if (model_type == EosPipelineEnvironmentModelType::kAdvanced) {
    return environment::EosEnvironmentModelType::kAdvanced;
  }
  return environment::EosEnvironmentModelType::kSimplified;
}

float ComputeFovSolidAngleSr(float horizontal_fov_deg, float vertical_fov_deg) {
  const float horizontal_fov_rad = std::max(0.0f, horizontal_fov_deg) * 3.1415926f / 180.0f;
  const float vertical_fov_rad = std::max(0.0f, vertical_fov_deg) * 3.1415926f / 180.0f;
  return std::max(0.0f, horizontal_fov_rad * vertical_fov_rad);
}

}  // namespace

EosPipeline::EosPipeline(const EosPipelineConfig& config)
    : config_(config), current_scan_azimuth_deg_(config.scan_start_az_deg) {}

void EosPipeline::UpdateConfig(const EosPipelineConfig& config) {
  config_ = config;
  current_scan_azimuth_deg_ = config_.scan_start_az_deg;
}

common::EosOutputFrame EosPipeline::Execute(const context::EosCycleInput& input) {
  common::EosOutputFrame output;
  output.cycle_index = input.cycle_index;
  AdvanceScan(input.dt_sec);
  output.scan_azimuth_deg = current_scan_azimuth_deg_;

  for (std::size_t i = 0; i < input.scene_targets.size(); ++i) {
    const context::EosTargetState& target = input.scene_targets[i];
    if (!IsTargetInCurrentFov(target)) {
      continue;
    }
    output.detections.push_back(BuildDetectionRecord(target, input));
  }

  return output;
}

void EosPipeline::AdvanceScan(float dt_sec) {
  const float scan_width_deg = config_.scan_end_az_deg - config_.scan_start_az_deg;
  if (scan_width_deg <= 0.0f) {
    current_scan_azimuth_deg_ = config_.scan_start_az_deg;
    return;
  }
  const float total_offset_deg = (current_scan_azimuth_deg_ - config_.scan_start_az_deg) +
                                 config_.scan_rate_deg_per_sec * dt_sec;
  float wrapped_offset_deg = total_offset_deg;
  while (wrapped_offset_deg >= scan_width_deg) {
    wrapped_offset_deg -= scan_width_deg;
  }
  while (wrapped_offset_deg < 0.0f) {
    wrapped_offset_deg += scan_width_deg;
  }
  current_scan_azimuth_deg_ = config_.scan_start_az_deg + wrapped_offset_deg;
}

bool EosPipeline::IsTargetInCurrentFov(const context::EosTargetState& target) const {
  const float azimuth_delta_deg =
      std::fabs(NormalizeAngle180(target.azimuth_deg - current_scan_azimuth_deg_));
  const float elevation_delta_deg = std::fabs(target.elevation_deg - config_.scan_center_el_deg);
  return azimuth_delta_deg <= 0.5f * config_.horizontal_fov_deg &&
         elevation_delta_deg <= 0.5f * config_.vertical_fov_deg;
}

common::EosDetectionRecord EosPipeline::BuildDetectionRecord(
    const context::EosTargetState& target, const context::EosCycleInput& input) const {
  common::EosDetectionRecord record;
  record.target_id = target.target_id;
  record.range_m = target.range_m;
  record.azimuth_deg = target.azimuth_deg;
  record.elevation_deg = target.elevation_deg;

  const bool infrared_enabled = WorkModeIncludesInfrared(config_.work_mode);
  const bool visible_enabled = WorkModeIncludesVisible(config_.work_mode);
  const float aperture_area_m2 = ComputeApertureAreaM2(config_.optical_aperture_m);
  environment::EosEnvironmentModelInputs environment_inputs;
  environment_inputs.model_type = ToEnvironmentModelType(config_.environment_model_type);
  environment_inputs.platform_altitude_m = std::fabs(input.platform_pose.position_m.z);
  environment_inputs.cloud_coverage_ratio = Clamp01(input.cloud_coverage_ratio);
  environment_inputs.wind_speed_mps = std::max(0.0f, input.ambient_wind_speed_mps);
  environment_inputs.base_aerosol_density_factor = config_.aerosol_density_factor;
  environment_inputs.base_turbulence_factor = config_.turbulence_factor;
  const environment::EosEnvironmentModelResult environment_result =
      environment::ResolveEnvironmentFactors(environment_inputs);
  const foundation::radiative_transfer::RadiativeTransferResult transfer_result =
      ComputePathRadiativeTransfer(config_, input, SafePositive(target.range_m, 1000.0f),
                                   environment_result.aerosol_density_factor,
                                   environment_result.turbulence_factor);
  const float path_transmittance = transfer_result.transmittance;
  const float path_radiance_penalty_scale =
      transfer_result.path_radiance_penalty_scale * environment_result.path_radiance_scale_bias;
  const float optical_transmittance = 0.85f;
  const float fov_solid_angle_sr =
      ComputeFovSolidAngleSr(config_.horizontal_fov_deg, config_.vertical_fov_deg);
  const float wavelength_center_um =
      0.5f * (SafePositive(config_.wavelength_lower_um, 3.0f) +
              SafePositive(config_.wavelength_upper_um, 5.0f));
  foundation::optics::DetectionRangeInputs range_inputs;
  range_inputs.platform_altitude_m = std::max(std::fabs(input.platform_pose.position_m.z), 1.0f);
  range_inputs.boresight_depression_deg = config_.boresight_depression_deg;
  range_inputs.vertical_fov_deg = config_.vertical_fov_deg;
  range_inputs.min_depression_deg = config_.min_detection_depression_deg;
  range_inputs.max_depression_deg = config_.max_detection_depression_deg;
  const float dmin_m = foundation::optics::ComputeMinimumDetectionRangeM(range_inputs);
  const float dmax_m = foundation::optics::ComputeMaximumDetectionRangeM(range_inputs);
  const bool is_within_detection_range = target.range_m >= dmin_m && target.range_m <= dmax_m;

  const float diffraction_resolution_rad = foundation::optics::ComputeDiffractionLimitedAngularResolutionRad(
      wavelength_center_um, config_.optical_aperture_m);
  const float gsd_m =
      foundation::optics::ComputeGroundSampleDistanceM(target.range_m, diffraction_resolution_rad);
  const float target_linear_size_m = std::sqrt(SafePositive(target.projected_area_m2, 1.0f));
  const float geometric_quality_gain = target_linear_size_m / (target_linear_size_m + gsd_m);
  foundation::spatial_spectrum::SpatialSpectrumInputs spectrum_inputs;
  spectrum_inputs.target_characteristic_size_m = target_linear_size_m;
  spectrum_inputs.ground_sample_distance_m = gsd_m;
  spectrum_inputs.optical_mtf_reference = 0.65f;
  spectrum_inputs.sampling_efficiency = 0.9f;
  spectrum_inputs.scene_contrast_ratio =
      Clamp01(0.5f * (std::max(0.0f, target.reflectance) + std::max(0.0f, target.emissivity)));
  const foundation::spatial_spectrum::SpatialSpectrumResult spectrum_result =
      foundation::spatial_spectrum::EvaluateSpatialResolvability(spectrum_inputs);
  const float imaging_quality_gain =
      Clamp(0.5f * geometric_quality_gain + 0.5f * spectrum_result.spectrum_quality_gain,
            0.05f, 1.0f);

  foundation::propagation::NepNoiseModelInputs nep_inputs;
  nep_inputs.optical_transmittance = optical_transmittance;
  nep_inputs.integration_time_sec =
      1.0f / std::max(SafePositive(config_.scan_rate_deg_per_sec, 20.0f), 1.0f);
  nep_inputs.electrical_bandwidth_hz = std::max(100.0f, SafePositive(config_.scan_rate_deg_per_sec, 20.0f) * 100.0f);
  nep_inputs.system_noise_factor = std::max(1.0f, 1.0e-12f / SafePositive(config_.detection_sensitivity_w, 1.0e-12f));
  foundation::noise::BackgroundNoiseModelInputs noise_inputs;
  noise_inputs.electrical_bandwidth_hz = nep_inputs.electrical_bandwidth_hz;
  noise_inputs.integration_time_sec = nep_inputs.integration_time_sec;
  noise_inputs.cloud_coverage_ratio = input.cloud_coverage_ratio;
  noise_inputs.detector_area_cm2 = nep_inputs.detector_area_cm2;
  noise_inputs.scene_complexity_factor =
      std::max(1.0f, 1.0f + 0.3f * (1.0f - imaging_quality_gain));

  float infrared_snr_linear = 0.0f;
  float visible_snr_linear = 0.0f;
  foundation::stray_light::StrayLightFilterInputs stray_light_inputs;
  stray_light_inputs.enabled = config_.enable_straylight_filter;
  stray_light_inputs.target_azimuth_deg = target.azimuth_deg;
  stray_light_inputs.target_elevation_deg = target.elevation_deg;
  stray_light_inputs.sun_azimuth_deg = input.solar_azimuth_deg;
  stray_light_inputs.sun_altitude_deg = input.solar_altitude_deg;
  stray_light_inputs.cloud_coverage_ratio = input.cloud_coverage_ratio;
  stray_light_inputs.hood_inner_half_angle_deg = config_.hood_inner_half_angle_deg;
  stray_light_inputs.hood_outer_half_angle_deg = config_.hood_outer_half_angle_deg;
  stray_light_inputs.min_suppression_ratio = config_.hood_min_suppression_ratio;
  stray_light_inputs.max_suppression_ratio = config_.hood_max_suppression_ratio;
  const foundation::stray_light::StrayLightFilterResult stray_light_result =
      foundation::stray_light::EvaluateStrayLightFilter(stray_light_inputs);

  if (infrared_enabled) {
    foundation::radiometry::InfraredRadianceInputs infrared_inputs;
    infrared_inputs.wavelength_um = wavelength_center_um;
    infrared_inputs.target_temperature_k = target.apparent_temperature_k;
    infrared_inputs.emissivity = target.emissivity;
    infrared_inputs.background_temperature_k = input.background_temperature_k;
    const float infrared_delta_radiance =
        foundation::radiometry::ComputeInfraredRadianceDelta(infrared_inputs);
    const float background_radiance = foundation::radiometry::ComputePlanckRadiance(
        wavelength_center_um, input.background_temperature_k);
    const float infrared_contrast = foundation::radiometry::ComputeRelativeContrast(
        background_radiance + infrared_delta_radiance, background_radiance);
    const float infrared_source_radiance =
        std::max(0.0f, infrared_delta_radiance) * (1.0f + std::max(0.0f, infrared_contrast));
    const float infrared_received_power_w = foundation::propagation::ComputeReceivedPowerW(
        infrared_source_radiance, target.projected_area_m2, target.range_m, aperture_area_m2,
        path_transmittance, optical_transmittance) * stray_light_result.signal_transmission_scale;
    const float infrared_background_flux_w = foundation::propagation::ComputeBackgroundFluxW(
        std::max(0.0f, background_radiance), aperture_area_m2, fov_solid_angle_sr,
        optical_transmittance) * stray_light_result.background_penalty_scale *
        path_radiance_penalty_scale;
    noise_inputs.background_flux_w = infrared_background_flux_w;
    const foundation::noise::BackgroundNoiseStatistics infrared_noise_stats =
        foundation::noise::ComputeBackgroundNoiseStatistics(noise_inputs);
    const float infrared_effective_signal_w = foundation::noise::ComputeEffectiveSignalPowerW(
        infrared_received_power_w, infrared_background_flux_w, infrared_noise_stats);
    const foundation::propagation::SnrEvaluationResult snr_result =
        foundation::propagation::EvaluateSnrWithNep(infrared_effective_signal_w, nep_inputs);
    infrared_snr_linear = snr_result.snr_linear * imaging_quality_gain;
  }

  if (visible_enabled) {
    foundation::radiometry::VisibleChannelInputs visible_inputs;
    visible_inputs.target.solar_irradiance_w_m2 = input.solar_irradiance_w_m2;
    visible_inputs.target.solar_altitude_deg = input.solar_altitude_deg;
    visible_inputs.target.atmospheric_transmittance = path_transmittance;
    visible_inputs.target.cloud_coverage_ratio = input.cloud_coverage_ratio;
    visible_inputs.target.reflectance = target.reflectance;
    visible_inputs.target.projected_area_m2 = target.projected_area_m2;
    visible_inputs.target.range_m = target.range_m;
    visible_inputs.target.illumination = ToIlluminationCondition(input.day_night_type);
    visible_inputs.background_reflectance = 0.12f;
    visible_inputs.background_patch_area_m2 = 20.0f;
    const foundation::radiometry::VisibleChannelResult visible_result =
        foundation::radiometry::ComputeVisibleChannelResult(visible_inputs);
    const float visible_received_power_w = foundation::propagation::ComputeReceivedPowerW(
        std::max(0.0f, visible_result.target_radiance) *
            (1.0f + std::max(0.0f, visible_result.normalized_contrast)),
        target.projected_area_m2, target.range_m, aperture_area_m2, path_transmittance,
        optical_transmittance) * stray_light_result.signal_transmission_scale;
    const float visible_background_flux_w = foundation::propagation::ComputeBackgroundFluxW(
        visible_result.background_radiance, aperture_area_m2, fov_solid_angle_sr,
        optical_transmittance) * stray_light_result.background_penalty_scale *
        path_radiance_penalty_scale;
    noise_inputs.background_flux_w = visible_background_flux_w;
    noise_inputs.photon_noise_enhancement_factor = 1.15f;
    const foundation::noise::BackgroundNoiseStatistics visible_noise_stats =
        foundation::noise::ComputeBackgroundNoiseStatistics(noise_inputs);
    const float visible_effective_signal_w = foundation::noise::ComputeEffectiveSignalPowerW(
        visible_received_power_w, visible_background_flux_w, visible_noise_stats);
    const foundation::propagation::SnrEvaluationResult snr_result =
        foundation::propagation::EvaluateSnrWithNep(visible_effective_signal_w, nep_inputs);
    visible_snr_linear = snr_result.snr_linear * imaging_quality_gain;
  }

  record.infrared_snr_linear = infrared_snr_linear;
  record.visible_snr_linear = visible_snr_linear;
  if (config_.work_mode == EosPipelineWorkMode::kInfraredOnly) {
    record.fused_snr_linear = infrared_snr_linear;
  } else if (config_.work_mode == EosPipelineWorkMode::kVisibleOnly) {
    record.fused_snr_linear = visible_snr_linear;
  } else {
    record.fused_snr_linear = 0.5f * (infrared_snr_linear + visible_snr_linear);
  }
  record.fused_snr_db = foundation::propagation::ComputeSnrDb(record.fused_snr_linear);
  record.detected = is_within_detection_range && record.fused_snr_db >= config_.minimum_snr_db;
  return record;
}

}  // namespace pipeline
}  // namespace core
}  // namespace electro_optical_sensor
