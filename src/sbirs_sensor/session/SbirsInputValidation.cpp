#include "1q/sbirs_sensor/session/SbirsInputValidation.h"

#include <cmath>
#include <set>

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"

namespace sbirs_sensor {
namespace session {
namespace {

bool Finite(double value) { return std::isfinite(value); }

bool FiniteVector(const SbirsVector3M& value) {
  return Finite(value.x) && Finite(value.y) && Finite(value.z);
}

bool NonZeroVector(const SbirsVector3M& value) {
  return value.x != 0.0 || value.y != 0.0 || value.z != 0.0;
}

bool ZeroVector(const SbirsVector3M& value) {
  return value.x == 0.0 && value.y == 0.0 && value.z == 0.0;
}

bool ValidWeather(config::SbirsWeatherType value) {
  switch (value) {
    case config::SbirsWeatherType::kClear:
    case config::SbirsWeatherType::kCloudy:
    case config::SbirsWeatherType::kRain:
    case config::SbirsWeatherType::kFog:
      return true;
  }
  return false;
}

bool ValidSeaState(config::SbirsSeaState value) {
  switch (value) {
    case config::SbirsSeaState::kLow:
    case config::SbirsSeaState::kMedium:
    case config::SbirsSeaState::kHigh:
      return true;
  }
  return false;
}

bool ValidEnvironment(const config::SbirsEnvironmentConfig& environment) {
  return ValidWeather(environment.weather_type) && ValidSeaState(environment.sea_state) &&
         std::isfinite(environment.temperature_c) && environment.temperature_c > -273.15f &&
         std::isfinite(environment.relative_humidity_percent) &&
         environment.relative_humidity_percent >= 0.0f &&
         environment.relative_humidity_percent <= 100.0f &&
         std::isfinite(environment.visibility_km) && environment.visibility_km > 0.0f &&
         std::isfinite(environment.base_atmospheric_transmittance) &&
         environment.base_atmospheric_transmittance >= 0.0f &&
         environment.base_atmospheric_transmittance <= 1.0f &&
         std::isfinite(environment.humidity_visibility_interaction_weight) &&
         environment.humidity_visibility_interaction_weight >= 0.0f &&
         std::isfinite(environment.rain_humidity_interaction_weight) &&
         environment.rain_humidity_interaction_weight >= 0.0f;
}

void AddError(const char* message, ValidationLocation location, ValidationIssueList* issues) {
  ValidationIssue issue;
  issue.severity = ValidationSeverity::kError;
  issue.location = location;
  issue.message = message;
  issues->push_back(issue);
}

}  // namespace

ValidationIssueList ValidateSbirsCycleInput(const SbirsCycleInput& input) {
  ValidationIssueList issues;
  if (input.dt_sec <= 0.0f || !std::isfinite(input.dt_sec)) {
    ValidationLocation location;
    location.kind = ValidationLocationKind::kGlobal;
    AddError("dt_sec must be positive and finite", location, &issues);
  }
  if (!input.has_satellite_position || !FiniteVector(input.satellite_position_ecef_m) ||
      !NonZeroVector(input.satellite_position_ecef_m)) {
    ValidationLocation location;
    location.kind = ValidationLocationKind::kPlatform;
    AddError("satellite position must be provided, finite, and non-zero", location, &issues);
  }
  std::set<std::uint64_t> target_ids;
  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    const SbirsSceneTarget& target = input.scene[i];
    const bool valid_velocity =
        FiniteVector(target.velocity_ecef_m_per_s) &&
        (target.has_velocity_ecef_m_per_s || ZeroVector(target.velocity_ecef_m_per_s));
    if (target.target_id == 0U || !target_ids.insert(target.target_id).second ||
        !FiniteVector(target.position_ecef_m) || !NonZeroVector(target.position_ecef_m) ||
        !std::isfinite(target.temperature_k) || target.temperature_k <= 0.0f ||
        !std::isfinite(target.emissivity) || target.emissivity < 0.0f ||
        target.emissivity > 1.0f || !std::isfinite(target.projected_area_m2) ||
        target.projected_area_m2 < 0.0f || !valid_velocity) {
      ValidationLocation location;
      location.kind = ValidationLocationKind::kSceneEntity;
      location.entity_index = i;
      AddError("target physical inputs must be finite and positive where required", location,
               &issues);
    }
  }
  if (input.environment.has_environment_override &&
      !ValidEnvironment(input.environment.environment)) {
    ValidationLocation location;
    location.kind = ValidationLocationKind::kGlobal;
    AddError("environment override contains invalid enum or physical value", location, &issues);
  }
  return issues;
}

bool HasValidationError(const ValidationIssueList& issues) {
  for (const ValidationIssue& issue : issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return true;
    }
  }
  return false;
}

}  // namespace session
}  // namespace sbirs_sensor
