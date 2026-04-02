#include "1q/electro_optical_sensor/core/session/EosSession.h"

#include <algorithm>
#include <cmath>

#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"
#include "1q/electro_optical_sensor/foundation/EosOpticalCharacteristics.h"
#include "1q/electro_optical_sensor/foundation/EosPropagation.h"
#include "1q/electro_optical_sensor/foundation/EosRadiometry.h"

namespace electro_optical_sensor {
namespace core {
namespace session {

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

bool WorkModeIncludesInfrared(EosWorkMode mode) {
  return mode == EosWorkMode::kInfraredOnly || mode == EosWorkMode::kFused;
}

bool WorkModeIncludesVisible(EosWorkMode mode) {
  return mode == EosWorkMode::kVisibleOnly || mode == EosWorkMode::kFused;
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

float ComputeDerivedAtmosphericTransmittance(const context::EosCycleInput& input, float range_m) {
  const float base_transmittance = Clamp01(input.atmospheric_transmittance);
  const float safe_base_transmittance = std::max(base_transmittance, 1.0e-4f);
  const float base_extinction_coeff_per_m = -std::log(safe_base_transmittance) / 5000.0f;
  const float humidity_absorption_coeff_per_m = 2.5e-5f * Clamp01(input.cloud_coverage_ratio);
  return foundation::propagation::ComputeAtmosphericTransmittance(
      base_extinction_coeff_per_m, humidity_absorption_coeff_per_m, range_m);
}

float ComputeFovSolidAngleSr(float horizontal_fov_deg, float vertical_fov_deg) {
  const float horizontal_fov_rad = std::max(0.0f, horizontal_fov_deg) * 3.1415926f / 180.0f;
  const float vertical_fov_rad = std::max(0.0f, vertical_fov_deg) * 3.1415926f / 180.0f;
  return std::max(0.0f, horizontal_fov_rad * vertical_fov_rad);
}

}  // namespace

struct EosSession::Impl {
  explicit Impl(EosSessionConfig session_config) : config(session_config), current_scan_azimuth_deg(0.0f) {
    current_scan_azimuth_deg = config.scan_start_az_deg;
  }

  void AdvanceScan(float dt_sec) {
    const float scan_width_deg = config.scan_end_az_deg - config.scan_start_az_deg;
    if (scan_width_deg <= 0.0f) {
      current_scan_azimuth_deg = config.scan_start_az_deg;
      return;
    }
    const float total_offset_deg =
        (current_scan_azimuth_deg - config.scan_start_az_deg) + config.scan_rate_deg_per_sec * dt_sec;
    float wrapped_offset_deg = total_offset_deg;
    while (wrapped_offset_deg >= scan_width_deg) {
      wrapped_offset_deg -= scan_width_deg;
    }
    while (wrapped_offset_deg < 0.0f) {
      wrapped_offset_deg += scan_width_deg;
    }
    current_scan_azimuth_deg = config.scan_start_az_deg + wrapped_offset_deg;
  }

  bool IsTargetInCurrentFov(const context::EosTargetState& target) const {
    const float azimuth_delta_deg =
        std::fabs(NormalizeAngle180(target.azimuth_deg - current_scan_azimuth_deg));
    const float elevation_delta_deg = std::fabs(target.elevation_deg - config.scan_center_el_deg);
    return azimuth_delta_deg <= 0.5f * config.horizontal_fov_deg &&
           elevation_delta_deg <= 0.5f * config.vertical_fov_deg;
  }

  common::EosDetectionRecord BuildDetectionRecord(const context::EosTargetState& target,
                                                  const context::EosCycleInput& input) const {
    common::EosDetectionRecord record;
    record.target_id = target.target_id;
    record.range_m = target.range_m;
    record.azimuth_deg = target.azimuth_deg;
    record.elevation_deg = target.elevation_deg;

    const bool infrared_enabled = WorkModeIncludesInfrared(config.work_mode);
    const bool visible_enabled = WorkModeIncludesVisible(config.work_mode);
    const float aperture_area_m2 = ComputeApertureAreaM2(config.optical_aperture_m);
    const float path_transmittance =
        ComputeDerivedAtmosphericTransmittance(input, SafePositive(target.range_m, 1000.0f));
    const float optical_transmittance = 0.85f;
    const float fov_solid_angle_sr =
        ComputeFovSolidAngleSr(config.horizontal_fov_deg, config.vertical_fov_deg);
    const float nep_w = SafePositive(config.detection_sensitivity_w, 1.0e-12f);
    const float wavelength_center_um =
        0.5f * (SafePositive(config.wavelength_lower_um, 3.0f) +
                SafePositive(config.wavelength_upper_um, 5.0f));
    const float diffraction_resolution_rad =
        foundation::optics::ComputeDiffractionLimitedAngularResolutionRad(
            wavelength_center_um, config.optical_aperture_m);
    const float gsd_m =
        foundation::optics::ComputeGroundSampleDistanceM(target.range_m, diffraction_resolution_rad);
    const float target_linear_size_m = std::sqrt(SafePositive(target.projected_area_m2, 1.0f));
    const float imaging_quality_gain = target_linear_size_m / (target_linear_size_m + gsd_m);

    float infrared_snr_linear = 0.0f;
    float visible_snr_linear = 0.0f;

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
      const float infrared_source_radiance = std::max(0.0f, infrared_delta_radiance) *
                                             (1.0f + std::max(0.0f, infrared_contrast));
      const float infrared_received_power_w = foundation::propagation::ComputeReceivedPowerW(
          infrared_source_radiance, target.projected_area_m2, target.range_m, aperture_area_m2,
          path_transmittance, optical_transmittance);
      const float infrared_background_flux_w = foundation::propagation::ComputeBackgroundFluxW(
          std::max(0.0f, background_radiance), aperture_area_m2, fov_solid_angle_sr,
          optical_transmittance);
      infrared_snr_linear = foundation::propagation::ComputeSnrLinear(
          std::max(0.0f, infrared_received_power_w - 0.1f * infrared_background_flux_w), nep_w) *
                            imaging_quality_gain;
    }
    if (visible_enabled) {
      foundation::radiometry::VisibleRadianceInputs visible_inputs;
      visible_inputs.solar_irradiance_w_m2 = input.solar_irradiance_w_m2;
      visible_inputs.solar_altitude_deg = input.solar_altitude_deg;
      visible_inputs.atmospheric_transmittance = path_transmittance;
      visible_inputs.cloud_coverage_ratio = input.cloud_coverage_ratio;
      visible_inputs.reflectance = target.reflectance;
      visible_inputs.projected_area_m2 = target.projected_area_m2;
      visible_inputs.range_m = target.range_m;
      visible_inputs.illumination = ToIlluminationCondition(input.day_night_type);
      const float visible_radiance =
          foundation::radiometry::ComputeVisibleLambertianRadiance(visible_inputs);
      const float visible_background_radiance =
          0.12f * std::max(0.0f, input.solar_irradiance_w_m2) * path_transmittance;
      const float visible_contrast = foundation::radiometry::ComputeRelativeContrast(
          visible_radiance + visible_background_radiance, visible_background_radiance);
      const float visible_received_power_w = foundation::propagation::ComputeReceivedPowerW(
          std::max(0.0f, visible_radiance) * (1.0f + std::max(0.0f, visible_contrast)),
          target.projected_area_m2, target.range_m, aperture_area_m2, path_transmittance,
          optical_transmittance);
      const float visible_background_flux_w = foundation::propagation::ComputeBackgroundFluxW(
          visible_background_radiance, aperture_area_m2, fov_solid_angle_sr, optical_transmittance);
      visible_snr_linear = foundation::propagation::ComputeSnrLinear(
          std::max(0.0f, visible_received_power_w - 0.15f * visible_background_flux_w), nep_w) *
                           imaging_quality_gain;
    }

    record.infrared_snr_linear = infrared_snr_linear;
    record.visible_snr_linear = visible_snr_linear;
    if (config.work_mode == EosWorkMode::kInfraredOnly) {
      record.fused_snr_linear = infrared_snr_linear;
    } else if (config.work_mode == EosWorkMode::kVisibleOnly) {
      record.fused_snr_linear = visible_snr_linear;
    } else {
      record.fused_snr_linear = 0.5f * (infrared_snr_linear + visible_snr_linear);
    }
    record.fused_snr_db = foundation::propagation::ComputeSnrDb(record.fused_snr_linear);
    record.detected = record.fused_snr_db >= config.minimum_snr_db;
    return record;
  }

  EosSessionConfig config{};
  float current_scan_azimuth_deg{0.0f};
};

EosSession::EosSession(EosSessionConfig config) : impl_(new Impl(config)) {}

EosSession::~EosSession() = default;

common::EosOutputFrame EosSession::Step(const context::EosCycleInput& input) {
  return StepWithResult(input).output_frame;
}

EosCycleResult EosSession::StepWithResult(const context::EosCycleInput& input) {
  EosCycleResult result;
  result.validation_issues = context::ValidateEosCycleInput(input);
  result.has_validation_error = context::HasEosValidationError(result.validation_issues);
  result.output_frame.cycle_index = input.cycle_index;
  impl_->AdvanceScan(input.dt_sec);
  result.output_frame.scan_azimuth_deg = impl_->current_scan_azimuth_deg;

  if (result.has_validation_error) {
    return result;
  }

  for (std::size_t i = 0; i < input.scene_targets.size(); ++i) {
    const context::EosTargetState& target = input.scene_targets[i];
    if (!impl_->IsTargetInCurrentFov(target)) {
      continue;
    }
    result.output_frame.detections.push_back(impl_->BuildDetectionRecord(target, input));
  }

  return result;
}

}  // namespace session
}  // namespace core
}  // namespace electro_optical_sensor
