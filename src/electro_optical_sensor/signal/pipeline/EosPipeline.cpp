#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"
#include "common/numerics/ClampUtils.h"
#include "electro_optical_sensor/environment/EosEnvironmentModel.h"
#include "electro_optical_sensor/foundation/EosNoiseModel.h"
#include "electro_optical_sensor/foundation/EosOpticalCharacteristics.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"
#include "electro_optical_sensor/foundation/EosPropagation.h"
#include "electro_optical_sensor/foundation/EosRadiometry.h"
#include "electro_optical_sensor/foundation/EosSpatialSpectrum.h"
#include "electro_optical_sensor/foundation/EosStrayLight.h"
#include "common/logging/ProjectLog.h"

namespace electro_optical_sensor {
namespace signal {
namespace pipeline {

namespace {

class DefaultEosEnvironmentService final : public environment::IEosEnvironmentService {
 public:
	environment::EosEnvironmentModelResult ResolveFactors(
			const environment::EosEnvironmentModelInputs& inputs) const override {
		return environment::ResolveEnvironmentFactors(inputs);
	}
};

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
		::electro_optical_sensor::session::DayNightType day_night_type) {
	if (day_night_type == ::electro_optical_sensor::session::DayNightType::kNight) {
		return foundation::radiometry::IlluminationCondition::kNight;
	}
	if (day_night_type == ::electro_optical_sensor::session::DayNightType::kTwilight) {
		return foundation::radiometry::IlluminationCondition::kTwilight;
	}
	return foundation::radiometry::IlluminationCondition::kDay;
}

float ComputeApertureAreaM2(float optical_aperture_m) {
	const float safe_diameter_m = SafePositive(optical_aperture_m, 0.2f);
	const float radius_m = 0.5f * safe_diameter_m;
	return foundation::constants::kPi * radius_m * radius_m;
}

foundation::radiative_transfer::RadiativeTransferResult ComputePathRadiativeTransfer(
		const EosPipelineConfig& config, const ::electro_optical_sensor::session::EosCycleInput& input,
		float range_m, const environment::EosEnvironmentModelResult& environment_result) {
	const float cloud_ratio = oneq::internal::numerics::Clamp01(input.environment.cloud_coverage_ratio);
	const float aerosol_excess = std::max(0.0f, environment_result.aerosol_density_factor - 1.0f);
	const float turbulence_excess =
			std::max(0.0f, environment_result.turbulence_factor - 1.0f);
	const float path_km = std::max(0.0f, range_m) * 1.0e-3f;
	const float altitude_km =
			std::max(0.0f, std::fabs(input.platform_pose.position_m.z)) * 1.0e-3f;
	const float attenuation_per_km =
			0.03f + 0.02f * cloud_ratio + 0.035f * aerosol_excess + 0.015f * turbulence_excess;
	const float altitude_relief_scale =
			oneq::internal::numerics::Clamp(1.0f + 0.04f * altitude_km, 1.0f, 1.12f);
	const float derived_base_transmittance = oneq::internal::numerics::Clamp(
			std::exp(-attenuation_per_km * path_km) * altitude_relief_scale, 0.05f, 0.98f);

	foundation::radiative_transfer::RadiativeTransferInputs transfer_inputs;
	transfer_inputs.model = config.radiative_transfer_model;
	transfer_inputs.base_transmittance = derived_base_transmittance;
	transfer_inputs.cloud_coverage_ratio =
			oneq::internal::numerics::Clamp01(input.environment.cloud_coverage_ratio);
	transfer_inputs.path_length_m = std::max(0.0f, range_m);
	transfer_inputs.aerosol_density_factor =
			std::max(1.0f, environment_result.aerosol_density_factor);
	transfer_inputs.turbulence_factor =
			std::max(1.0f, environment_result.turbulence_factor);
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
	const float horizontal_fov_rad =
			std::max(0.0f, horizontal_fov_deg) * foundation::constants::kPi / 180.0f;
	const float vertical_fov_rad =
			std::max(0.0f, vertical_fov_deg) * foundation::constants::kPi / 180.0f;
	return std::max(0.0f, horizontal_fov_rad * vertical_fov_rad);
}

float ComputeSensorIntegrationTimeSec(const EosPipelineConfig& config,
																			const ::electro_optical_sensor::session::EosCycleInput& input) {
	const float frame_period_sec = 1.0f / std::max(SafePositive(config.frame_rate_hz, 30.0f), 1.0f);
	const float cycle_dt_sec = SafePositive(input.dt_sec, frame_period_sec);
	const float base_integration_sec = std::min(cycle_dt_sec, frame_period_sec);
	const float scan_blur_penalty = 1.0f + SafePositive(config.scan_rate_deg_per_sec, 20.0f) / 120.0f;
	return std::max(1.0e-4f, base_integration_sec / scan_blur_penalty);
}

float ComputeVisiblePhotonNoiseEnhancement(const EosPipelineConfig& config,
																					 const ::electro_optical_sensor::session::EosCycleInput& input) {
	const float reference_irradiance = SafePositive(config.visible_reference_irradiance_w_m2, 800.0f);
	const float observed_irradiance = std::max(0.0f, input.environment.solar_irradiance_w_m2);
	const float irradiance_ratio = std::max(observed_irradiance, 1.0e-3f) / reference_irradiance;
	const float irradiance_mismatch = std::fabs(std::log2(std::max(irradiance_ratio, 1.0e-6f)));
	const float cloud_factor =
			1.0f + 0.5f * oneq::internal::numerics::Clamp01(input.environment.cloud_coverage_ratio);
	return oneq::internal::numerics::Clamp(
			1.0f + 0.12f * irradiance_mismatch * cloud_factor, 1.0f, 2.5f);
}

float ComputeSystemNoiseFactorFromSensitivity(float detection_sensitivity_w) {
	const float kReferenceSensitivityW = 1.0e-12f;
	const float safe_sensitivity_w = SafePositive(detection_sensitivity_w, kReferenceSensitivityW);
	return std::max(1.0f, safe_sensitivity_w / kReferenceSensitivityW);
}

struct DetectionComputationContext {
	bool infrared_enabled{false};
	bool visible_enabled{false};
	bool is_within_detection_range{false};
	float aperture_area_m2{0.0f};
	float path_transmittance{0.0f};
	float path_radiance_penalty_scale{1.0f};
	float optical_transmittance{0.85f};
	float fov_solid_angle_sr{0.0f};
	float wavelength_center_um{4.0f};
	float wavelength_bandwidth_um{1.0f};
	float imaging_quality_gain{1.0f};
	foundation::propagation::NepNoiseModelInputs nep_inputs{};
	foundation::noise::BackgroundNoiseModelInputs noise_inputs{};
	foundation::stray_light::StrayLightFilterResult stray_light_result{};
};

DetectionComputationContext BuildDetectionComputationContext(
		const EosPipelineConfig& config, const ::electro_optical_sensor::session::EosTargetState& target,
		const ::electro_optical_sensor::session::EosCycleInput& input,
		const std::shared_ptr<environment::IEosEnvironmentService>& environment_service) {
	DetectionComputationContext context_values;
	context_values.infrared_enabled = WorkModeIncludesInfrared(config.work_mode);
	context_values.visible_enabled = WorkModeIncludesVisible(config.work_mode);
	context_values.aperture_area_m2 = ComputeApertureAreaM2(config.optical_aperture_m);

	environment::EosEnvironmentModelInputs environment_inputs;
	environment_inputs.model_type = ToEnvironmentModelType(config.environment_model_type);
	environment_inputs.platform_altitude_m = std::fabs(input.platform_pose.position_m.z);
	environment_inputs.cloud_coverage_ratio =
			oneq::internal::numerics::Clamp01(input.environment.cloud_coverage_ratio);
	environment_inputs.wind_speed_mps = std::max(0.0f, input.environment.ambient_wind_speed_mps);
	const environment::EosEnvironmentModelResult environment_result =
			environment_service->ResolveFactors(environment_inputs);
	const foundation::radiative_transfer::RadiativeTransferResult transfer_result =
			ComputePathRadiativeTransfer(
					config, input, SafePositive(target.range_m, 1000.0f), environment_result);
	context_values.path_transmittance = transfer_result.transmittance;
	context_values.path_radiance_penalty_scale =
			transfer_result.path_radiance_penalty_scale * environment_result.path_radiance_scale_bias;
	context_values.fov_solid_angle_sr =
			ComputeFovSolidAngleSr(config.horizontal_fov_deg, config.vertical_fov_deg);

	const float wavelength_lower_um = SafePositive(config.wavelength_lower_um, 3.0f);
	const float wavelength_upper_um = SafePositive(config.wavelength_upper_um, 5.0f);
	context_values.wavelength_center_um = 0.5f * (wavelength_lower_um + wavelength_upper_um);
	context_values.wavelength_bandwidth_um =
			std::max(std::fabs(wavelength_upper_um - wavelength_lower_um), 0.1f);

	foundation::optics::DetectionRangeInputs range_inputs;
	range_inputs.platform_altitude_m = std::max(std::fabs(input.platform_pose.position_m.z), 1.0f);
	range_inputs.boresight_depression_deg = config.boresight_depression_deg;
	range_inputs.vertical_fov_deg = config.vertical_fov_deg;
	range_inputs.min_depression_deg = config.min_detection_depression_deg;
	range_inputs.max_depression_deg = config.max_detection_depression_deg;
	const float dmin_m = foundation::optics::ComputeMinimumDetectionRangeM(range_inputs);
	const float dmax_m = foundation::optics::ComputeMaximumDetectionRangeM(range_inputs);
	context_values.is_within_detection_range = target.range_m >= dmin_m && target.range_m <= dmax_m;

	const float diffraction_resolution_rad =
			foundation::optics::ComputeDiffractionLimitedAngularResolutionRad(
					context_values.wavelength_center_um, config.optical_aperture_m);
	const float gsd_m =
			foundation::optics::ComputeGroundSampleDistanceM(target.range_m, diffraction_resolution_rad);
	const float target_linear_size_m = std::sqrt(SafePositive(target.projected_area_m2, 1.0f));
	const float geometric_quality_gain = target_linear_size_m / (target_linear_size_m + gsd_m);
	foundation::spatial_spectrum::SpatialSpectrumInputs spectrum_inputs;
	spectrum_inputs.target_characteristic_size_m = target_linear_size_m;
	spectrum_inputs.ground_sample_distance_m = gsd_m;
	spectrum_inputs.optical_mtf_reference = 0.65f;
	spectrum_inputs.sampling_efficiency = 0.9f;
	spectrum_inputs.scene_contrast_ratio = oneq::internal::numerics::Clamp01(
			0.5f * (std::max(0.0f, target.reflectance) + std::max(0.0f, target.emissivity)));
	const foundation::spatial_spectrum::SpatialSpectrumResult spectrum_result =
			foundation::spatial_spectrum::EvaluateSpatialResolvability(spectrum_inputs);
	context_values.imaging_quality_gain = oneq::internal::numerics::Clamp(
			0.5f * geometric_quality_gain + 0.5f * spectrum_result.spectrum_quality_gain, 0.05f, 1.0f);

	context_values.nep_inputs.optical_transmittance = context_values.optical_transmittance;
	context_values.nep_inputs.integration_time_sec = ComputeSensorIntegrationTimeSec(config, input);
	const float scan_coupled_bandwidth_hz =
			SafePositive(config.scan_rate_deg_per_sec, 20.0f) * 100.0f;
	const float frame_coupled_bandwidth_hz = 0.5f / context_values.nep_inputs.integration_time_sec;
	context_values.nep_inputs.electrical_bandwidth_hz =
			std::max(100.0f, std::max(scan_coupled_bandwidth_hz, frame_coupled_bandwidth_hz));
	context_values.nep_inputs.system_noise_factor =
			ComputeSystemNoiseFactorFromSensitivity(config.detection_sensitivity_w);

	context_values.noise_inputs.electrical_bandwidth_hz =
			context_values.nep_inputs.electrical_bandwidth_hz;
	context_values.noise_inputs.integration_time_sec = context_values.nep_inputs.integration_time_sec;
	context_values.noise_inputs.cloud_coverage_ratio = input.environment.cloud_coverage_ratio;
	context_values.noise_inputs.detector_area_cm2 = context_values.nep_inputs.detector_area_cm2;
	context_values.noise_inputs.scene_complexity_factor =
			std::max(1.0f, 1.0f + 0.3f * (1.0f - context_values.imaging_quality_gain));

	foundation::stray_light::StrayLightFilterInputs stray_light_inputs;
	stray_light_inputs.enabled = config.enable_straylight_filter;
	stray_light_inputs.target_azimuth_deg = target.azimuth_deg;
	stray_light_inputs.target_elevation_deg = target.elevation_deg;
	stray_light_inputs.sun_azimuth_deg = input.environment.solar_azimuth_deg;
	stray_light_inputs.sun_altitude_deg = input.environment.solar_altitude_deg;
	stray_light_inputs.cloud_coverage_ratio = input.environment.cloud_coverage_ratio;
	stray_light_inputs.hood_inner_half_angle_deg = config.hood_inner_half_angle_deg;
	stray_light_inputs.hood_outer_half_angle_deg = config.hood_outer_half_angle_deg;
	stray_light_inputs.min_suppression_ratio = config.hood_min_suppression_ratio;
	stray_light_inputs.max_suppression_ratio = config.hood_max_suppression_ratio;
	context_values.stray_light_result =
			foundation::stray_light::EvaluateStrayLightFilter(stray_light_inputs);
	return context_values;
}

float ComputeInfraredSnrLinear(const ::electro_optical_sensor::session::EosTargetState& target,
															 const ::electro_optical_sensor::session::EosCycleInput& input,
															 const DetectionComputationContext& context_values) {
	foundation::radiometry::InfraredRadianceInputs infrared_inputs;
	infrared_inputs.wavelength_um = context_values.wavelength_center_um;
	infrared_inputs.target_temperature_k = target.apparent_temperature_k;
	infrared_inputs.emissivity = target.emissivity;
	infrared_inputs.background_temperature_k = input.environment.background_temperature_k;
	const float infrared_delta_spectral_radiance =
			foundation::radiometry::ComputeInfraredRadianceDelta(infrared_inputs);
	const float background_spectral_radiance = foundation::radiometry::ComputePlanckRadiance(
			context_values.wavelength_center_um, input.environment.background_temperature_k);
	const float infrared_delta_radiance = foundation::radiometry::IntegrateSpectralRadianceOverBand(
			infrared_delta_spectral_radiance, context_values.wavelength_bandwidth_um);
	const float background_radiance = foundation::radiometry::IntegrateSpectralRadianceOverBand(
			background_spectral_radiance, context_values.wavelength_bandwidth_um);
	const float infrared_contrast = foundation::radiometry::ComputeRelativeContrast(
			background_radiance + infrared_delta_radiance, background_radiance);
	const float infrared_source_radiance =
			std::max(0.0f, infrared_delta_radiance) * (1.0f + std::max(0.0f, infrared_contrast));
	const float infrared_received_power_w =
			foundation::propagation::ComputeReceivedPowerW(
					infrared_source_radiance, target.projected_area_m2, target.range_m,
					context_values.aperture_area_m2, context_values.path_transmittance,
					context_values.optical_transmittance) *
			context_values.stray_light_result.signal_transmission_scale;
	const float infrared_background_flux_w =
			foundation::propagation::ComputeBackgroundFluxW(
					std::max(0.0f, background_radiance), context_values.aperture_area_m2,
					context_values.fov_solid_angle_sr, context_values.optical_transmittance) *
			context_values.stray_light_result.background_penalty_scale *
			context_values.path_radiance_penalty_scale;
	foundation::noise::BackgroundNoiseModelInputs noise_inputs = context_values.noise_inputs;
	noise_inputs.background_flux_w = infrared_background_flux_w;
	const foundation::noise::BackgroundNoiseStatistics infrared_noise_stats =
			foundation::noise::ComputeBackgroundNoiseStatistics(noise_inputs);
	const float infrared_effective_signal_w = foundation::noise::ComputeEffectiveSignalPowerW(
			infrared_received_power_w, infrared_background_flux_w, infrared_noise_stats);
	const foundation::propagation::SnrEvaluationResult snr_result =
			foundation::propagation::EvaluateSnrWithNep(infrared_effective_signal_w,
																									context_values.nep_inputs);
	return snr_result.snr_linear * context_values.imaging_quality_gain;
}

float ComputeVisibleSnrLinear(const EosPipelineConfig& config,
															const ::electro_optical_sensor::session::EosTargetState& target,
															const ::electro_optical_sensor::session::EosCycleInput& input,
															const DetectionComputationContext& context_values) {
	foundation::radiometry::VisibleChannelInputs visible_inputs;
	visible_inputs.target.solar_irradiance_w_m2 = input.environment.solar_irradiance_w_m2;
	visible_inputs.target.solar_altitude_deg = input.environment.solar_altitude_deg;
	visible_inputs.target.cloud_coverage_ratio = input.environment.cloud_coverage_ratio;
	visible_inputs.target.reflectance = target.reflectance;
	visible_inputs.target.illumination = ToIlluminationCondition(input.environment.day_night_type);
	visible_inputs.background_reflectance = 0.12f;
	visible_inputs.background_patch_area_m2 = 20.0f;
	const foundation::radiometry::VisibleChannelResult visible_result =
			foundation::radiometry::ComputeVisibleChannelResult(visible_inputs);
	const float visible_received_power_w =
			foundation::propagation::ComputeReceivedPowerW(
					std::max(0.0f, visible_result.target_radiance) *
							(1.0f + std::max(0.0f, visible_result.normalized_contrast)),
					target.projected_area_m2, target.range_m, context_values.aperture_area_m2,
					context_values.path_transmittance, context_values.optical_transmittance) *
			context_values.stray_light_result.signal_transmission_scale;
	const float visible_background_flux_w =
			foundation::propagation::ComputeBackgroundFluxW(
					visible_result.background_radiance, context_values.aperture_area_m2,
					context_values.fov_solid_angle_sr, context_values.optical_transmittance) *
			context_values.stray_light_result.background_penalty_scale *
			context_values.path_radiance_penalty_scale;
	const float visible_noise_enhancement = ComputeVisiblePhotonNoiseEnhancement(config, input);
	foundation::noise::BackgroundNoiseModelInputs noise_inputs = context_values.noise_inputs;
	noise_inputs.background_flux_w = visible_background_flux_w;
	noise_inputs.photon_noise_enhancement_factor = visible_noise_enhancement;
	const foundation::noise::BackgroundNoiseStatistics visible_noise_stats =
			foundation::noise::ComputeBackgroundNoiseStatistics(noise_inputs);
	const float visible_effective_signal_w = foundation::noise::ComputeEffectiveSignalPowerW(
			visible_received_power_w, visible_background_flux_w, visible_noise_stats);
	foundation::propagation::NepNoiseModelInputs visible_nep_inputs = context_values.nep_inputs;
	visible_nep_inputs.system_noise_factor *= visible_noise_enhancement;
	const foundation::propagation::SnrEvaluationResult snr_result =
			foundation::propagation::EvaluateSnrWithNep(visible_effective_signal_w, visible_nep_inputs);
	return snr_result.snr_linear * context_values.imaging_quality_gain;
}

float ComputeFusedSnrLinear(EosPipelineWorkMode work_mode,
														::electro_optical_sensor::session::DayNightType day_night_type,
														float infrared_snr_linear, float visible_snr_linear) {
	if (work_mode == EosPipelineWorkMode::kInfraredOnly) {
		return infrared_snr_linear;
	}
	if (work_mode == EosPipelineWorkMode::kVisibleOnly) {
		return visible_snr_linear;
	}

	const float safe_infrared_snr_linear = std::max(0.0f, infrared_snr_linear);
	const float safe_visible_snr_linear = std::max(0.0f, visible_snr_linear);
	if (safe_infrared_snr_linear <= 1.0e-9f) {
		return safe_visible_snr_linear;
	}
	if (safe_visible_snr_linear <= 1.0e-9f) {
		return safe_infrared_snr_linear;
	}

	float infrared_weight = 0.5f;
	float visible_weight = 0.5f;
	if (day_night_type == ::electro_optical_sensor::session::DayNightType::kDay) {
		infrared_weight = 0.35f;
		visible_weight = 0.65f;
	} else if (day_night_type == ::electro_optical_sensor::session::DayNightType::kNight) {
		infrared_weight = 0.80f;
		visible_weight = 0.20f;
	}
	return infrared_weight * safe_infrared_snr_linear + visible_weight * safe_visible_snr_linear;
}

bool IsCompatiblePipelineRuntimeState(const extension::EosPipelineRuntimeState& state,
																const EosPipeline* pipeline,
																const EosPipelineConfig& config) {
	return state.owner_identity == pipeline && state.schema_version == 1U &&
				 state.scan_start_az_deg == config.scan_start_az_deg &&
				 state.scan_end_az_deg == config.scan_end_az_deg &&
				 state.scan_rate_deg_per_sec == config.scan_rate_deg_per_sec;
}

}  // namespace

EosPipeline::EosPipeline(const EosPipelineConfig& config,
												 std::shared_ptr<environment::IEosEnvironmentService> environment_service)
		: config_(config),
			current_scan_azimuth_deg_(config.scan_start_az_deg),
			environment_service_(std::move(environment_service)) {
	if (environment_service_ == nullptr) {
		environment_service_.reset(new DefaultEosEnvironmentService());
	}
}

void EosPipeline::UpdateConfig(const EosPipelineConfig& config, bool reset_scan_phase) {
	config_ = config;
	if (reset_scan_phase) {
		current_scan_azimuth_deg_ = config_.scan_start_az_deg;
	}
}

extension::EosPipelineRuntimeState EosPipeline::CaptureRuntimeState() const {
	extension::EosPipelineRuntimeState state;
	state.owner_identity = this;
	state.schema_version = 1U;
	state.current_scan_azimuth_deg = current_scan_azimuth_deg_;
	state.scan_start_az_deg = config_.scan_start_az_deg;
	state.scan_end_az_deg = config_.scan_end_az_deg;
	state.scan_rate_deg_per_sec = config_.scan_rate_deg_per_sec;
	return state;
}

bool EosPipeline::RestoreRuntimeState(const extension::EosPipelineRuntimeState& state) {
	if (!IsCompatiblePipelineRuntimeState(state, this, config_)) {
		PROJECT_LOG_ERROR(
				"[EosPipeline] runtime state restore rejected: owner/schema/config mismatch.");
		return false;
	}
	current_scan_azimuth_deg_ = state.current_scan_azimuth_deg;
	return true;
}

extension::EosPipelineExecuteResult EosPipeline::Execute(
		const ::electro_optical_sensor::session::EosCycleInput& input) {
	extension::EosPipelineExecuteResult result;
	output::EosOutputFrame& output = result.output_frame;
	output.cycle_index = input.cycle_index;
	AdvanceScan(input.dt_sec);
	output.scan_azimuth_deg = current_scan_azimuth_deg_;

	for (std::size_t i = 0; i < input.scene.targets.size(); ++i) {
		const ::electro_optical_sensor::session::EosTargetState& target = input.scene.targets[i];
		if (!IsTargetInCurrentFov(target)) {
			continue;
		}
		output.detections.push_back(BuildDetectionRecord(target, input));
	}

	result.executed_this_cycle = true;
	result.abort_reason = extension::EosPipelineAbortReason::kNone;
	return result;
}

void EosPipeline::AdvanceScan(float dt_sec) {
	const float scan_width_deg = config_.scan_end_az_deg - config_.scan_start_az_deg;
	if (scan_width_deg < 0.001f) {
		current_scan_azimuth_deg_ = config_.scan_start_az_deg;
		return;
	}
	const float total_offset_deg = (current_scan_azimuth_deg_ - config_.scan_start_az_deg) +
																 config_.scan_rate_deg_per_sec * dt_sec;
	float wrapped_offset_deg = std::fmod(total_offset_deg, scan_width_deg);
	if (wrapped_offset_deg < 0.0f) {
		wrapped_offset_deg += scan_width_deg;
	}
	current_scan_azimuth_deg_ = config_.scan_start_az_deg + wrapped_offset_deg;
}

bool EosPipeline::IsTargetInCurrentFov(
		const ::electro_optical_sensor::session::EosTargetState& target) const {
	const float azimuth_delta_deg =
			std::fabs(NormalizeAngle180(target.azimuth_deg - current_scan_azimuth_deg_));
	const float elevation_delta_deg = std::fabs(target.elevation_deg - config_.scan_center_el_deg);
	return azimuth_delta_deg <= 0.5f * config_.horizontal_fov_deg &&
				 elevation_delta_deg <= 0.5f * config_.vertical_fov_deg;
}

output::EosDetectionRecord EosPipeline::BuildDetectionRecord(
		const ::electro_optical_sensor::session::EosTargetState& target,
		const ::electro_optical_sensor::session::EosCycleInput& input) const {
	output::EosDetectionRecord record;
	record.target_id = target.target_id;
	record.range_m = target.range_m;
	record.azimuth_deg = target.azimuth_deg;
	record.elevation_deg = target.elevation_deg;
	const DetectionComputationContext context_values =
			BuildDetectionComputationContext(config_, target, input, environment_service_);
	const float infrared_snr_linear = context_values.infrared_enabled
																				? ComputeInfraredSnrLinear(target, input, context_values)
																				: 0.0f;
	const float visible_snr_linear =
			context_values.visible_enabled
					? ComputeVisibleSnrLinear(config_, target, input, context_values)
					: 0.0f;

	record.infrared_snr_linear = infrared_snr_linear;
	record.visible_snr_linear = visible_snr_linear;
	record.fused_snr_linear = ComputeFusedSnrLinear(config_.work_mode, input.environment.day_night_type,
																									infrared_snr_linear, visible_snr_linear);
	record.fused_snr_db = foundation::propagation::ComputeSnrDb(record.fused_snr_linear);
	record.detected =
			context_values.is_within_detection_range && record.fused_snr_db >= config_.minimum_snr_db;
	return record;
}

}  // namespace pipeline
}  // namespace signal
}  // namespace electro_optical_sensor
