#include "1q/electro_optical_sensor/session/EosInputValidation.h"

#include "common/validation/ValidationUtils.h"

namespace electro_optical_sensor {
namespace session {

using ::electro_optical_sensor::session::DayNightType;
using ::electro_optical_sensor::session::EosCycleInput;
using ::electro_optical_sensor::session::EosSceneTarget;

namespace {

using oneq::common::validation::IsFinite;

ValidationIssue MakeIssue(ValidationSeverity severity, ValidationCode code,
                          ValidationLocationKind location_kind, std::size_t entity_index,
                          const std::string& field, const std::string& message) {
  return oneq::common::validation::MakeLocatedIssue<ValidationIssue, ValidationLocation>(
      severity, code, location_kind, entity_index, field, message);
}

void ValidateDayNightConsistency(const EosCycleInput& input, ValidationIssueList* issues) {
  if (issues == nullptr || !IsFinite(input.environment.solar_altitude_deg)) {
    return;
  }
  if (input.environment.day_night_type == DayNightType::kDay &&
      input.environment.solar_altitude_deg < 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kWarning,
                                ValidationCode::kInconsistentDayNightType,
                                ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                                "environment.day_night_type",
                                "day/night type is day while solar altitude is below horizon"));
    return;
  }
  if (input.environment.day_night_type == DayNightType::kNight &&
      input.environment.solar_altitude_deg > -6.0f) {
    issues->push_back(
        MakeIssue(ValidationSeverity::kWarning, ValidationCode::kInconsistentDayNightType,
                  ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                  "environment.day_night_type",
                  "day/night type is night while solar altitude indicates twilight/day"));
  }
}

void ValidatePlatformPose(const oneq::foundation::PoseState& platform_pose,
                          ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (!IsFinite(platform_pose.position_m.x) || !IsFinite(platform_pose.position_m.y) ||
      !IsFinite(platform_pose.position_m.z) || !IsFinite(platform_pose.velocity_mps.x) ||
      !IsFinite(platform_pose.velocity_mps.y) || !IsFinite(platform_pose.velocity_mps.z) ||
      !IsFinite(platform_pose.attitude_deg.yaw_deg) ||
      !IsFinite(platform_pose.attitude_deg.pitch_deg) ||
      !IsFinite(platform_pose.attitude_deg.roll_deg)) {
    issues->push_back(
        MakeIssue(ValidationSeverity::kError, ValidationCode::kNonFinitePlatformNumericField,
                  ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1), "platform_pose",
                  "platform pose contains non-finite numeric field"));
  }
}

void ValidatePlatformAltitude(float platform_altitude_m, ValidationIssueList* issues) {
  if (issues == nullptr || IsFinite(platform_altitude_m)) {
    return;
  }
  issues->push_back(MakeIssue(ValidationSeverity::kError,
                              ValidationCode::kNonFinitePlatformNumericField,
                              ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1),
                              "platform_altitude_m", "platform altitude must be finite"));
}

void ValidateTarget(const EosSceneTarget& target, std::size_t target_index,
                    ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (!IsFinite(target.range_m) || !IsFinite(target.azimuth_deg) ||
      !IsFinite(target.elevation_deg) || !IsFinite(target.appearance.apparent_temperature_k) ||
      !IsFinite(target.appearance.emissivity) || !IsFinite(target.appearance.reflectance) ||
      !IsFinite(target.appearance.projected_area_m2)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kNonFiniteTargetNumericField,
                                ValidationLocationKind::kSceneEntity, target_index, "scene",
                                "target contains non-finite numeric field"));
  }

  if (target.range_m <= 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidTargetRange,
                                ValidationLocationKind::kSceneEntity, target_index, "range_m",
                                "target range must be positive"));
  }
  if (target.appearance.apparent_temperature_k <= 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidTargetTemperature,
                                ValidationLocationKind::kSceneEntity, target_index,
                                "apparent_temperature_k", "target temperature must be positive"));
  }
  if (!oneq::common::validation::IsRatio01(target.appearance.emissivity)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidTargetEmissivity,
                                ValidationLocationKind::kSceneEntity, target_index, "emissivity",
                                "target emissivity must be in [0, 1]"));
  }
  if (!oneq::common::validation::IsRatio01(target.appearance.reflectance)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidTargetReflectance,
                                ValidationLocationKind::kSceneEntity, target_index, "reflectance",
                                "target reflectance must be in [0, 1]"));
  }
  if (IsFinite(target.appearance.emissivity) && IsFinite(target.appearance.reflectance) &&
      (target.appearance.emissivity + target.appearance.reflectance > 1.0f + 1.0e-4f)) {
    issues->push_back(
        MakeIssue(ValidationSeverity::kWarning, ValidationCode::kInconsistentTargetEnergyBalance,
                  ValidationLocationKind::kSceneEntity, target_index, "emissivity+reflectance",
                  "target emissivity + reflectance should not exceed 1"));
  }
  if (target.appearance.projected_area_m2 <= 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidTargetProjectedArea,
                                ValidationLocationKind::kSceneEntity, target_index,
                                "projected_area_m2", "target projected area must be positive"));
  }
}

}  // namespace

ValidationIssueList ValidateEosCycleInput(
    const ::electro_optical_sensor::session::EosCycleInput& input) {
  ValidationIssueList issues;

  if (!IsFinite(input.dt_sec)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kNonFiniteCycleDeltaTime,
                               ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1),
                               "dt_sec", "cycle delta time must be finite"));
  } else if (input.dt_sec <= 0.0f) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidCycleDeltaTime,
                               ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1),
                               "dt_sec", "cycle delta time must be positive"));
  }

  ValidatePlatformPose(input.platform_pose, &issues);
  ValidatePlatformAltitude(input.platform_altitude_m, &issues);

  if (!IsFinite(input.environment.solar_altitude_deg) ||
      !IsFinite(input.environment.solar_azimuth_deg)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kNonFiniteSolarAngles,
                               ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                               "environment.solar_altitude_deg/solar_azimuth_deg",
                               "solar angles must be finite"));
  } else if (input.environment.solar_altitude_deg < -90.0f ||
             input.environment.solar_altitude_deg > 90.0f) {
    issues.push_back(
        MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidSolarAltitudeRange,
                  ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                  "environment.solar_altitude_deg", "solar altitude must be in [-90, 90] degrees"));
  }
  ValidateDayNightConsistency(input, &issues);

  if (!IsFinite(input.environment.solar_irradiance_w_m2) ||
      input.environment.solar_irradiance_w_m2 < 0.0f) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidSolarIrradiance,
                               ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                               "environment.solar_irradiance_w_m2",
                               "solar irradiance must be finite and non-negative"));
  }
  if (!oneq::common::validation::IsRatio01(input.environment.cloud_coverage_ratio)) {
    issues.push_back(
        MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidCloudCoverageRatio,
                  ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                  "environment.cloud_coverage_ratio", "cloud coverage ratio must be in [0, 1]"));
  }
  if (!IsFinite(input.environment.ambient_wind_speed_mps) ||
      input.environment.ambient_wind_speed_mps < 0.0f) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidAmbientWindSpeed,
                               ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                               "environment.ambient_wind_speed_mps",
                               "ambient wind speed must be finite and non-negative"));
  }
  if (!IsFinite(input.environment.background_temperature_k) ||
      input.environment.background_temperature_k <= 0.0f) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kInvalidBackgroundTemperature,
                               ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                               "environment.background_temperature_k",
                               "background temperature must be finite and positive"));
  }

  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    ValidateTarget(input.scene[i], i, &issues);
  }

  return issues;
}

bool HasValidationError(const ValidationIssueList& issues) {
  return oneq::common::validation::HasSeverity<ValidationIssueList, ValidationSeverity,
                                                 &ValidationIssue::severity>(
      issues, ValidationSeverity::kError);
}

}  // namespace session
}  // namespace electro_optical_sensor
