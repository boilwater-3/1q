// @file EosSessionConfigBuilder.cpp
// @brief Implementation of EosSessionConfigBuilder (thin wrapper).

#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"

#include <cmath>

#include "1q/electro_optical_sensor/config/EosSessionConfigValidation.h"
#include "1q/electro_optical_sensor/session/EosIssueCodes.h"

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

session::EosIssueList ValidateEosSessionConfig(const config::EosSessionConfig& config) noexcept {
  session::EosIssueList issues;
  const auto push = [&issues](const char* code, const char* field, const char* msg) {
    session::EosIssue issue;
    issue.severity = session::EosIssueSeverity::kError;
    issue.phase = session::EosIssuePhase::kInputValidation;
    issue.code = code;
    issue.field = field;
    issue.message = msg;
    issues.push_back(issue);
  };

  if (config.mission.horizontal_fov_deg <= 0.0f) {
    push(session::codes::kHorizontalFovNotPositive, "mission.horizontal_fov_deg",
         "Horizontal FOV must be positive.");
  }
  if (config.mission.vertical_fov_deg <= 0.0f) {
    push(session::codes::kVerticalFovNotPositive, "mission.vertical_fov_deg",
         "Vertical FOV must be positive.");
  }
  if (config.mission.scan_rate_deg_per_sec <= 0.0f) {
    push(session::codes::kScanRateNotPositive, "mission.scan_rate_deg_per_sec",
         "Scan rate must be positive.");
  }
  if (config.mission.frame_rate_hz <= 0.0f) {
    push(session::codes::kFrameRateNotPositive, "mission.frame_rate_hz",
         "Frame rate must be positive.");
  }
  if (config.mission.scan_start_az_deg >= config.mission.scan_end_az_deg) {
    push(session::codes::kScanRangeAzSwapped, "mission.scan_start_az_deg / scan_end_az_deg",
         "Scan start azimuth must be less than end azimuth.");
  }

  const EosEnvironmentScenarioConfig& environment = config.environment.scenario_config;
  if (!IsValidEnvironmentPreset(environment.preset)) {
    push(session::codes::kEnvironmentPresetInvalid,
         "environment.scenario_config.preset", "Environment preset is invalid.");
  }
  const EosAtmosphericPhysicsConfig& atmosphere = environment.atmospheric_physics;
  if (atmosphere.enable_physical_model &&
      (!std::isfinite(atmosphere.pressure_hpa) || atmosphere.pressure_hpa <= 0.0f ||
       !std::isfinite(atmosphere.temperature_k) || atmosphere.temperature_k <= 0.0f ||
       !std::isfinite(atmosphere.relative_humidity) || atmosphere.relative_humidity < 0.0f ||
       atmosphere.relative_humidity > 1.0f)) {
    push(session::codes::kAtmosphericPhysicsInvalid,
         "environment.scenario_config.atmospheric_physics",
         "Enabled atmospheric physics values are invalid.");
  }

  return issues;
}

}  // namespace config
}  // namespace electro_optical_sensor
