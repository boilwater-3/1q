#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"

#include "common/validation/ValidationUtils.h"

namespace electro_optical_sensor {
namespace core {
namespace context {

namespace {

EosValidationIssue MakeIssue(EosValidationSeverity severity, EosValidationCode code,
                             std::size_t target_index, const std::string& message) {
  return oneq::internal::validation::MakeIndexedIssue<EosValidationIssue, EosValidationSeverity,
                                                      EosValidationCode,
                                                      &EosValidationIssue::target_index>(
      severity, code, target_index, message);
}

template <typename T>
bool IsFinite(T value) {
  return oneq::internal::validation::IsFinite(value);
}

bool IsRatioValid(float value) { return IsFinite(value) && value >= 0.0f && value <= 1.0f; }

void ValidatePlatformPose(const oneq::common::PoseState& platform_pose,
                          EosValidationIssueList* issues) {
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
        MakeIssue(EosValidationSeverity::kError, EosValidationCode::kNonFinitePlatformNumericField,
                  static_cast<std::size_t>(-1), "platform pose contains non-finite numeric field"));
  }
}

void ValidateTarget(const EosTargetState& target, std::size_t target_index,
                    EosValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (target.target_id == 0U) {
    issues->push_back(MakeIssue(EosValidationSeverity::kWarning,
                                EosValidationCode::kInvalidTargetId, target_index,
                                "target id is zero"));
  }

  if (!IsFinite(target.range_m) || !IsFinite(target.azimuth_deg) ||
      !IsFinite(target.elevation_deg) || !IsFinite(target.apparent_temperature_k) ||
      !IsFinite(target.emissivity) || !IsFinite(target.reflectance) ||
      !IsFinite(target.projected_area_m2)) {
    issues->push_back(MakeIssue(EosValidationSeverity::kError,
                                EosValidationCode::kNonFiniteTargetNumericField, target_index,
                                "target contains non-finite numeric field"));
  }

  if (target.range_m <= 0.0f) {
    issues->push_back(MakeIssue(EosValidationSeverity::kError,
                                EosValidationCode::kInvalidTargetRange, target_index,
                                "target range must be positive"));
  }
  if (target.apparent_temperature_k <= 0.0f) {
    issues->push_back(MakeIssue(EosValidationSeverity::kError,
                                EosValidationCode::kInvalidTargetTemperature, target_index,
                                "target temperature must be positive"));
  }
  if (!IsRatioValid(target.emissivity)) {
    issues->push_back(MakeIssue(EosValidationSeverity::kError,
                                EosValidationCode::kInvalidTargetEmissivity, target_index,
                                "target emissivity must be in [0, 1]"));
  }
  if (!IsRatioValid(target.reflectance)) {
    issues->push_back(MakeIssue(EosValidationSeverity::kError,
                                EosValidationCode::kInvalidTargetReflectance, target_index,
                                "target reflectance must be in [0, 1]"));
  }
  if (target.projected_area_m2 <= 0.0f) {
    issues->push_back(MakeIssue(EosValidationSeverity::kError,
                                EosValidationCode::kInvalidTargetProjectedArea, target_index,
                                "target projected area must be positive"));
  }
}

}  // namespace

EosValidationIssueList ValidateEosCycleInput(const EosCycleInput& input) {
  EosValidationIssueList issues;

  if (!IsFinite(input.dt_sec)) {
    issues.push_back(MakeIssue(EosValidationSeverity::kError,
                               EosValidationCode::kNonFiniteCycleDeltaTime,
                               static_cast<std::size_t>(-1), "cycle delta time must be finite"));
  } else if (input.dt_sec <= 0.0f) {
    issues.push_back(MakeIssue(EosValidationSeverity::kError,
                               EosValidationCode::kInvalidCycleDeltaTime,
                               static_cast<std::size_t>(-1), "cycle delta time must be positive"));
  }

  ValidatePlatformPose(input.platform_pose, &issues);

  if (!IsFinite(input.solar_altitude_deg) || !IsFinite(input.solar_azimuth_deg)) {
    issues.push_back(MakeIssue(EosValidationSeverity::kError,
                               EosValidationCode::kNonFiniteSolarAngles,
                               static_cast<std::size_t>(-1), "solar angles must be finite"));
  } else if (input.solar_altitude_deg < -90.0f || input.solar_altitude_deg > 90.0f) {
    issues.push_back(
        MakeIssue(EosValidationSeverity::kError, EosValidationCode::kInvalidSolarAltitudeRange,
                  static_cast<std::size_t>(-1), "solar altitude must be in [-90, 90] degrees"));
  }

  if (!IsFinite(input.solar_irradiance_w_m2) || input.solar_irradiance_w_m2 < 0.0f) {
    issues.push_back(MakeIssue(
        EosValidationSeverity::kError, EosValidationCode::kInvalidSolarIrradiance,
        static_cast<std::size_t>(-1), "solar irradiance must be finite and non-negative"));
  }
  if (!IsRatioValid(input.atmospheric_transmittance)) {
    issues.push_back(MakeIssue(
        EosValidationSeverity::kError, EosValidationCode::kInvalidAtmosphericTransmittance,
        static_cast<std::size_t>(-1), "atmospheric transmittance must be in [0, 1]"));
  }
  if (!IsRatioValid(input.cloud_coverage_ratio)) {
    issues.push_back(
        MakeIssue(EosValidationSeverity::kError, EosValidationCode::kInvalidCloudCoverageRatio,
                  static_cast<std::size_t>(-1), "cloud coverage ratio must be in [0, 1]"));
  }
  if (!IsFinite(input.ambient_wind_speed_mps) || input.ambient_wind_speed_mps < 0.0f) {
    issues.push_back(MakeIssue(
        EosValidationSeverity::kError, EosValidationCode::kInvalidAmbientWindSpeed,
        static_cast<std::size_t>(-1), "ambient wind speed must be finite and non-negative"));
  }
  if (!IsFinite(input.background_temperature_k) || input.background_temperature_k <= 0.0f) {
    issues.push_back(MakeIssue(
        EosValidationSeverity::kError, EosValidationCode::kInvalidBackgroundTemperature,
        static_cast<std::size_t>(-1), "background temperature must be finite and positive"));
  }

  for (std::size_t i = 0; i < input.scene_targets.size(); ++i) {
    ValidateTarget(input.scene_targets[i], i, &issues);
  }

  return issues;
}

bool HasEosValidationError(const EosValidationIssueList& issues) {
  return oneq::internal::validation::HasSeverity<EosValidationIssueList, EosValidationSeverity,
                                                 &EosValidationIssue::severity>(
      issues, EosValidationSeverity::kError);
}

}  // namespace context
}  // namespace core
}  // namespace electro_optical_sensor
