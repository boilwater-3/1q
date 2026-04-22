#include "1q/electro_optical_sensor/session/EosInputValidation.h"

#include "common/validation/ValidationUtils.h"

namespace electro_optical_sensor {
namespace session {

using ::electro_optical_sensor::session::DayNightType;
using ::electro_optical_sensor::session::EosCycleInput;
using ::electro_optical_sensor::session::EosTargetState;

namespace {

ValidationIssue MakeIssue(ValidationSeverity severity, ValidationCode code,
                             std::size_t target_index, const std::string& message) {
  return oneq::internal::validation::MakeIndexedIssue<ValidationIssue, ValidationSeverity,
                                                      ValidationCode,
                                                      &ValidationIssue::target_index>(
      severity, code, target_index, message);
}

template <typename T>
bool IsFinite(T value) {
  return oneq::internal::validation::IsFinite(value);
}

bool IsRatioValid(float value) { return IsFinite(value) && value >= 0.0f && value <= 1.0f; }

void ValidateDayNightConsistency(const EosCycleInput& input, ValidationIssueList* issues) {
  if (issues == nullptr || !IsFinite(input.solar_altitude_deg)) {
    return;
  }
  if (input.day_night_type == DayNightType::kDay && input.solar_altitude_deg < 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kWarning,
                                ValidationCode::kInconsistentDayNightType,
                                static_cast<std::size_t>(-1),
                                "day/night type is day while solar altitude is below horizon"));
    return;
  }
  if (input.day_night_type == DayNightType::kNight && input.solar_altitude_deg > -6.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kWarning,
                                ValidationCode::kInconsistentDayNightType,
                                static_cast<std::size_t>(-1),
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
                  static_cast<std::size_t>(-1), "platform pose contains non-finite numeric field"));
  }
}

void ValidateTarget(const EosTargetState& target, std::size_t target_index,
                    ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (target.target_id == 0U) {
    issues->push_back(MakeIssue(ValidationSeverity::kWarning,
                                ValidationCode::kInvalidTargetId, target_index,
                                "target id is zero"));
  }

  if (!IsFinite(target.range_m) || !IsFinite(target.azimuth_deg) ||
      !IsFinite(target.elevation_deg) || !IsFinite(target.apparent_temperature_k) ||
      !IsFinite(target.emissivity) || !IsFinite(target.reflectance) ||
      !IsFinite(target.projected_area_m2)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kNonFiniteTargetNumericField, target_index,
                                "target contains non-finite numeric field"));
  }

  if (target.range_m <= 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidTargetRange, target_index,
                                "target range must be positive"));
  }
  if (target.apparent_temperature_k <= 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidTargetTemperature, target_index,
                                "target temperature must be positive"));
  }
  if (!IsRatioValid(target.emissivity)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidTargetEmissivity, target_index,
                                "target emissivity must be in [0, 1]"));
  }
  if (!IsRatioValid(target.reflectance)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidTargetReflectance, target_index,
                                "target reflectance must be in [0, 1]"));
  }
  if (IsFinite(target.emissivity) && IsFinite(target.reflectance) &&
      (target.emissivity + target.reflectance > 1.0f + 1.0e-4f)) {
    issues->push_back(MakeIssue(ValidationSeverity::kWarning,
                                ValidationCode::kInconsistentTargetEnergyBalance, target_index,
                                "target emissivity + reflectance should not exceed 1"));
  }
  if (target.projected_area_m2 <= 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidTargetProjectedArea, target_index,
                                "target projected area must be positive"));
  }
}

}  // namespace

ValidationIssueList ValidateEosCycleInput(
    const ::electro_optical_sensor::session::EosCycleInput& input) {
  ValidationIssueList issues;

  if (!IsFinite(input.dt_sec)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kNonFiniteCycleDeltaTime,
                               static_cast<std::size_t>(-1), "cycle delta time must be finite"));
  } else if (input.dt_sec <= 0.0f) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kInvalidCycleDeltaTime,
                               static_cast<std::size_t>(-1), "cycle delta time must be positive"));
  }

  ValidatePlatformPose(input.platform_pose, &issues);

  if (!IsFinite(input.solar_altitude_deg) || !IsFinite(input.solar_azimuth_deg)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kNonFiniteSolarAngles,
                               static_cast<std::size_t>(-1), "solar angles must be finite"));
  } else if (input.solar_altitude_deg < -90.0f || input.solar_altitude_deg > 90.0f) {
    issues.push_back(
        MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidSolarAltitudeRange,
                  static_cast<std::size_t>(-1), "solar altitude must be in [-90, 90] degrees"));
  }
  ValidateDayNightConsistency(input, &issues);

  if (!IsFinite(input.solar_irradiance_w_m2) || input.solar_irradiance_w_m2 < 0.0f) {
    issues.push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidSolarIrradiance,
        static_cast<std::size_t>(-1), "solar irradiance must be finite and non-negative"));
  }
  if (!IsRatioValid(input.cloud_coverage_ratio)) {
    issues.push_back(
        MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidCloudCoverageRatio,
                  static_cast<std::size_t>(-1), "cloud coverage ratio must be in [0, 1]"));
  }
  if (!IsFinite(input.ambient_wind_speed_mps) || input.ambient_wind_speed_mps < 0.0f) {
    issues.push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidAmbientWindSpeed,
        static_cast<std::size_t>(-1), "ambient wind speed must be finite and non-negative"));
  }
  if (!IsFinite(input.background_temperature_k) || input.background_temperature_k <= 0.0f) {
    issues.push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidBackgroundTemperature,
        static_cast<std::size_t>(-1), "background temperature must be finite and positive"));
  }

  for (std::size_t i = 0; i < input.scene_targets.size(); ++i) {
    ValidateTarget(input.scene_targets[i], i, &issues);
  }

  return issues;
}

bool HasValidationError(const ValidationIssueList& issues) {
  return oneq::internal::validation::HasSeverity<ValidationIssueList, ValidationSeverity,
                                                 &ValidationIssue::severity>(
      issues, ValidationSeverity::kError);
}

}  // namespace session
}  // namespace electro_optical_sensor
