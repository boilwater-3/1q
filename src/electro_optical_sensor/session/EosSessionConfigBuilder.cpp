// @file EosSessionConfigBuilder.cpp
// @brief Implementation of EosSessionConfigBuilder with profile translation.

#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"

#include <cmath>

#include "1q/electro_optical_sensor/config/EosSessionConfigValidation.h"

namespace electro_optical_sensor {
namespace config {

namespace {

bool IsValidEnvironmentPreset(EosEnvironmentPreset preset) {
  switch (preset) {
    case EosEnvironmentPreset::kStandard:
    case EosEnvironmentPreset::kHumid:
    case EosEnvironmentPreset::kDusty:
    case EosEnvironmentPreset::kTurbulent:
    case EosEnvironmentPreset::kMaritime:
      return true;
  }
  return false;
}

void ApplyEosMissionSemanticConfig(EosMissionProfile profile, EosMissionConfig* mission,
                                   EosDetectionPolicyConfig* detection) {
  if (mission == nullptr || detection == nullptr) {
    return;
  }
  auto& m = *mission;
  auto& d = *detection;

  switch (profile) {
    case EosMissionProfile::kWideAreaSearch:
      m.work_mode = EosWorkMode::kFused;
      m.horizontal_fov_deg = 12.0f;
      m.vertical_fov_deg = 8.0f;
      m.scan_rate_deg_per_sec = 30.0f;
      m.frame_rate_hz = 15.0f;
      d.minimum_snr_db = 6.0f;
      break;
    case EosMissionProfile::kLongRangeSurveillance:
      m.work_mode = EosWorkMode::kInfraredOnly;
      m.horizontal_fov_deg = 3.0f;
      m.vertical_fov_deg = 2.0f;
      m.scan_rate_deg_per_sec = 10.0f;
      m.frame_rate_hz = 10.0f;
      d.minimum_snr_db = 3.0f;
      break;
    case EosMissionProfile::kHighResolutionTrack:
      m.work_mode = EosWorkMode::kFused;
      m.horizontal_fov_deg = 1.5f;
      m.vertical_fov_deg = 1.0f;
      m.scan_rate_deg_per_sec = 5.0f;
      m.frame_rate_hz = 60.0f;
      d.minimum_snr_db = 2.0f;
      break;
  }
}

void ApplyEosHardwareSemanticConfig(EosHardwareProfile profile, EosHardwareConfig* hardware) {
  if (hardware == nullptr) {
    return;
  }
  auto& h = *hardware;

  switch (profile) {
    case EosHardwareProfile::kStandardMidWaveIR:
      h.wavelength_lower_um = 3.0f;
      h.wavelength_upper_um = 5.0f;
      h.optical_aperture_m = 0.2f;
      h.detector_detectivity_cm_sqrt_hz_per_w = 1.0e10f;
      break;
    case EosHardwareProfile::kLongRangeLargeAperture:
      h.wavelength_lower_um = 3.0f;
      h.wavelength_upper_um = 5.0f;
      h.optical_aperture_m = 0.4f;
      h.detector_detectivity_cm_sqrt_hz_per_w = 2.0e10f;
      break;
    case EosHardwareProfile::kWideAreaCompact:
      h.wavelength_lower_um = 8.0f;
      h.wavelength_upper_um = 12.0f;
      h.optical_aperture_m = 0.1f;
      h.detector_detectivity_cm_sqrt_hz_per_w = 5.0e9f;
      break;
  }
}

}  // namespace

config::EosSessionConfig EosSessionConfigBuilder::Build() const noexcept {
  config::EosSessionConfig result = config_;

  if (mission_profile_dirty_) {
    ApplyEosMissionSemanticConfig(mission_profile_, &result.mission, &result.policy.detection);
  }
  if (hardware_profile_dirty_) {
    ApplyEosHardwareSemanticConfig(hardware_profile_, &result.hardware);
  }

  return result;
}

ValidationIssueList ValidateEosSessionConfig(const config::EosSessionConfig& config) noexcept {
  ValidationIssueList issues;
  const auto push = [&issues](ConfigValidationCode code, const char* field, const char* msg) {
    ConfigValidationIssue issue;
    issue.code = code;
    issue.field = field;
    issue.message = msg;
    issues.push_back(issue);
  };

  if (config.mission.horizontal_fov_deg <= 0.0f) {
    push(ConfigValidationCode::kHorizontalFovNotPositive, "mission.horizontal_fov_deg",
         "Horizontal FOV must be positive.");
  }
  if (config.mission.vertical_fov_deg <= 0.0f) {
    push(ConfigValidationCode::kVerticalFovNotPositive, "mission.vertical_fov_deg",
         "Vertical FOV must be positive.");
  }
  if (config.mission.scan_rate_deg_per_sec <= 0.0f) {
    push(ConfigValidationCode::kScanRateNotPositive, "mission.scan_rate_deg_per_sec",
         "Scan rate must be positive.");
  }
  if (config.mission.frame_rate_hz <= 0.0f) {
    push(ConfigValidationCode::kFrameRateNotPositive, "mission.frame_rate_hz",
         "Frame rate must be positive.");
  }
  if (config.mission.scan_start_az_deg >= config.mission.scan_end_az_deg) {
    push(ConfigValidationCode::kScanRangeAzSwapped, "mission.scan_start_az_deg / scan_end_az_deg",
         "Scan start azimuth must be less than end azimuth.");
  }

  const EosEnvironmentScenarioConfig& environment = config.environment.scenario_config;
  if (!IsValidEnvironmentPreset(environment.preset)) {
    push(ConfigValidationCode::kEnvironmentPresetInvalid,
         "environment.scenario_config.preset", "Environment preset is invalid.");
  }
  const EosAtmosphericPhysicsConfig& atmosphere = environment.atmospheric_physics;
  if (atmosphere.enable_physical_model &&
      (!std::isfinite(atmosphere.pressure_hpa) || atmosphere.pressure_hpa <= 0.0f ||
       !std::isfinite(atmosphere.temperature_k) || atmosphere.temperature_k <= 0.0f ||
       !std::isfinite(atmosphere.relative_humidity) || atmosphere.relative_humidity < 0.0f ||
       atmosphere.relative_humidity > 1.0f)) {
    push(ConfigValidationCode::kAtmosphericPhysicsInvalid,
         "environment.scenario_config.atmospheric_physics",
         "Enabled atmospheric physics values are invalid.");
  }

  return issues;
}

}  // namespace config
}  // namespace electro_optical_sensor
