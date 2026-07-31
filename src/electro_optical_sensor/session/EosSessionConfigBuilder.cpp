// @file EosSessionConfigBuilder.cpp
// @brief Implementation of EosSessionConfigBuilder (thin wrapper).

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

}  // namespace

config::EosSessionConfig EosSessionConfigBuilder::Build() const noexcept { return config_; }

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
