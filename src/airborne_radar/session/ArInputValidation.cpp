#include "1q/airborne_radar/session/ArInputValidation.h"

#include <sstream>
#include <unordered_map>

#include "common/validation/ValidationUtils.h"

namespace airborne_radar {
namespace session {

namespace {

using oneq::common::validation::IsFinite;

ValidationIssue MakeIssue(ValidationSeverity severity, ValidationCode code,
                          ValidationLocationKind location_kind, std::size_t entity_index,
                          const std::string& field, const std::string& message) {
  return oneq::common::validation::MakeLocatedIssue<ValidationIssue, ValidationLocation>(
      severity, code, location_kind, entity_index, field, message);
}

bool IsDefaultEnvironmentInput(const ArEnvironmentInput& environment) {
  const ArEnvironmentInput defaults;
  return environment.atmospheric_observation.enable_physical_model ==
             defaults.atmospheric_observation.enable_physical_model &&
         environment.atmospheric_observation.pressure_hpa ==
             defaults.atmospheric_observation.pressure_hpa &&
         environment.atmospheric_observation.temperature_k ==
             defaults.atmospheric_observation.temperature_k &&
         environment.atmospheric_observation.relative_humidity ==
             defaults.atmospheric_observation.relative_humidity &&
         environment.atmospheric_context.has_k_factor ==
             defaults.atmospheric_context.has_k_factor &&
         environment.atmospheric_context.k_factor == defaults.atmospheric_context.k_factor &&
         environment.atmospheric_context.has_day_of_year ==
             defaults.atmospheric_context.has_day_of_year &&
         environment.atmospheric_context.day_of_year == defaults.atmospheric_context.day_of_year &&
         environment.atmospheric_context.solar_flux_f107a ==
             defaults.atmospheric_context.solar_flux_f107a &&
         environment.atmospheric_context.solar_flux_f107 ==
             defaults.atmospheric_context.solar_flux_f107 &&
         environment.atmospheric_context.geomagnetic_ap ==
             defaults.atmospheric_context.geomagnetic_ap &&
         environment.atmospheric_context.has_simulation_unix_seconds ==
             defaults.atmospheric_context.has_simulation_unix_seconds &&
         environment.atmospheric_context.simulation_unix_seconds ==
             defaults.atmospheric_context.simulation_unix_seconds &&
         environment.surface_observation.cover_profile ==
             defaults.surface_observation.cover_profile &&
         environment.surface_observation.enable_physical_model ==
             defaults.surface_observation.enable_physical_model &&
         environment.jammer_sources.empty() &&
         environment.interference.mode == defaults.interference.mode &&
         environment.interference.legacy_jammer_sources.empty() &&
         environment.interference.engineering_emissions.empty();
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

bool HasEngineeringInterference(const ArEnvironmentInput& environment) {
  return environment.interference.mode == oneq::electromagnetics::RfInterferenceMode::kEngineering;
}

bool HasNonDefaultPlatformEcefKinematics(const ArCycleInput& input) {
  return input.platform_position_ecef_m.x_m != 0.0 || input.platform_position_ecef_m.y_m != 0.0 ||
         input.platform_position_ecef_m.z_m != 0.0 ||
         input.platform_velocity_ecef_mps.x_mps != 0.0 ||
         input.platform_velocity_ecef_mps.y_mps != 0.0 ||
         input.platform_velocity_ecef_mps.z_mps != 0.0;
}

void ValidatePlatformEcefKinematics(const ArCycleInput& input, ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }
  if (!input.has_platform_ecef_kinematics) {
    if (HasNonDefaultPlatformEcefKinematics(input)) {
      issues->push_back(
          MakeIssue(ValidationSeverity::kError, ValidationCode::kPlatformEcefFlagMismatch,
                    ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1),
                    "has_platform_ecef_kinematics",
                    "platform ECEF data is present but has_platform_ecef_kinematics is false"));
    }
    if (input.has_environment && HasEngineeringInterference(input.environment)) {
      issues->push_back(
          MakeIssue(ValidationSeverity::kError, ValidationCode::kMissingEngineeringRfReceiverSite,
                    ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1),
                    "has_platform_ecef_kinematics",
                    "engineering RF interference requires explicit platform ECEF kinematics"));
    }
    return;
  }
  const oneq::coordinate::EcefPositionM& position = input.platform_position_ecef_m;
  const oneq::coordinate::EcefVelocityMps& velocity = input.platform_velocity_ecef_mps;
  if (!IsFinite(position.x_m) || !IsFinite(position.y_m) || !IsFinite(position.z_m) ||
      !IsFinite(velocity.x_mps) || !IsFinite(velocity.y_mps) || !IsFinite(velocity.z_mps)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidPlatformEcefKinematics,
        ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1), "platform_ecef_kinematics",
        "platform ECEF position and velocity must be finite"));
  }
}

bool IsKnownInterferenceMode(oneq::electromagnetics::RfInterferenceMode mode) {
  switch (mode) {
    case oneq::electromagnetics::RfInterferenceMode::kNone:
    case oneq::electromagnetics::RfInterferenceMode::kLegacy:
    case oneq::electromagnetics::RfInterferenceMode::kEngineering:
      return true;
  }
  return false;
}

bool IsValidLegacyJammer(const config::JammerEmitterState& jammer) {
  return IsFinite(jammer.power_db) && IsFinite(jammer.js_db) && IsFinite(jammer.position_x) &&
         IsFinite(jammer.position_y) && IsFinite(jammer.position_z) &&
         IsFinite(jammer.angular_span_deg) && IsFinite(jammer.confidence) &&
         jammer.power_db >= 0.0f && jammer.js_db >= 0.0f && jammer.angular_span_deg >= 0.0f &&
         oneq::common::validation::IsRatio01(jammer.confidence);
}

void ValidateInterferenceInput(const ArEnvironmentInput& environment, float cycle_duration_s,
                               ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }
  const config::ArInterferenceInput& input = environment.interference;
  const bool has_compat_payload = !environment.jammer_sources.empty();
  const bool has_legacy_payload = !input.legacy_jammer_sources.empty();
  const bool has_engineering_payload = !input.engineering_emissions.empty();
  bool valid = IsKnownInterferenceMode(input.mode);
  if (has_compat_payload) {
    valid = valid && input.mode == oneq::electromagnetics::RfInterferenceMode::kNone &&
            !has_legacy_payload && !has_engineering_payload;
  } else {
    switch (input.mode) {
      case oneq::electromagnetics::RfInterferenceMode::kNone:
        valid = valid && !has_legacy_payload && !has_engineering_payload;
        break;
      case oneq::electromagnetics::RfInterferenceMode::kLegacy:
        valid = valid && !has_engineering_payload;
        break;
      case oneq::electromagnetics::RfInterferenceMode::kEngineering:
        valid = valid && !has_legacy_payload &&
                oneq::electromagnetics::TryValidateRfEmissionFrame(
                    input.engineering_emissions, static_cast<double>(cycle_duration_s));
        break;
    }
  }
  const config::JammerEmitterStateList& legacy_sources =
      has_compat_payload ? environment.jammer_sources : input.legacy_jammer_sources;
  for (const config::JammerEmitterState& jammer : legacy_sources) {
    valid = valid && IsValidLegacyJammer(jammer);
  }
  if (!valid) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidInterferenceInput,
        ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
        "environment.interference",
        "interference mode and payload must be mutually consistent and RF facts must be valid"));
  }
}

void ValidateEnvironmentInput(const ArEnvironmentInput& environment, float cycle_duration_s,
                              ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }
  const config::AtmosphericPhysicsConfig& atmosphere = environment.atmospheric_observation;
  if (!IsFinite(atmosphere.pressure_hpa) || !IsFinite(atmosphere.temperature_k) ||
      !IsFinite(atmosphere.relative_humidity) || atmosphere.pressure_hpa <= 0.0f ||
      atmosphere.temperature_k <= 0.0f ||
      !oneq::common::validation::IsRatio01(atmosphere.relative_humidity)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidEnvironmentObservation,
                                ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                                "environment.atmospheric_observation",
                                "atmospheric observation must contain positive "
                                "pressure/temperature and humidity in [0, 1]"));
  }
  const config::AtmosphericDerivedContext& context = environment.atmospheric_context;
  if (!IsFinite(context.solar_flux_f107a) || !IsFinite(context.solar_flux_f107) ||
      !IsFinite(context.geomagnetic_ap)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidEnvironmentObservation,
        ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
        "environment.atmospheric_context", "atmospheric context numeric fields must be finite"));
  }
  for (std::size_t i = 0; i < environment.jammer_sources.size(); ++i) {
    const config::JammerEmitterState& jammer = environment.jammer_sources[i];
    if (!IsValidLegacyJammer(jammer)) {
      issues->push_back(MakeIssue(
          ValidationSeverity::kError, ValidationCode::kInvalidEnvironmentObservation,
          ValidationLocationKind::kEnvironment, i, "environment.jammer_sources",
          "jammer source must contain finite non-negative powers/span and confidence in [0, 1]"));
    }
  }
  ValidateInterferenceInput(environment, cycle_duration_s, issues);
}

/**
 * @brief 判断目标是否携带笛卡尔位置。
 * @param target 目标特征。
 * @return 存在非零笛卡尔位置分量时返回 `true`。
 */
bool HasCartesianPosition(const ArSceneTarget& target) {
  return target.position_x != 0.0f || target.position_y != 0.0f || target.position_z != 0.0f;
}

/**
 * @brief 收集单个目标的字段级校验问题。
 * @param target 目标特征。
 * @param target_index 目标索引。
 * @param[out] issues 输出问题列表。
 */
void ValidateSingleTarget(const ArSceneTarget& target, std::size_t target_index,
                          ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (!IsFinite(target.position_x) || !IsFinite(target.position_y) ||
      !IsFinite(target.position_z) || !IsFinite(target.velocity_x) ||
      !IsFinite(target.velocity_y) || !IsFinite(target.velocity_z) || !IsFinite(target.rcs) ||
      !IsFinite(target.range_m)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kNonFiniteTargetField,
                                ValidationLocationKind::kSceneEntity, target_index, "scene",
                                "target contains non-finite numeric field"));
  }

  if (target.range_m <= 0.0f && !HasCartesianPosition(target)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kMissingRangeAndCartesianPosition,
                                ValidationLocationKind::kSceneEntity, target_index, "range_m",
                                "target requires positive range or cartesian position"));
  }

  if (target.external_target_id == 0U) {
    issues->push_back(MakeIssue(ValidationSeverity::kInfo, ValidationCode::kUnknownExternalTargetId,
                                ValidationLocationKind::kSceneEntity, target_index,
                                "external_target_id", "target external id is unknown"));
  }

  if (target.rcs < 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kWarning, ValidationCode::kNegativeRcs,
                                ValidationLocationKind::kSceneEntity, target_index, "rcs",
                                "target rcs is negative"));
  }
}

}  // namespace

ValidationIssueList ValidateArCycleDeltaTime(float dt_sec) {
  ValidationIssueList issues;
  if (!IsFinite(dt_sec)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kNonFiniteCycleDeltaTime,
                               ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1),
                               "dt_sec", "cycle delta time must be finite"));
  } else if (dt_sec <= 0.0f) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidCycleDeltaTime,
                               ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1),
                               "dt_sec", "cycle delta time is non-positive"));
  }
  return issues;
}

ValidationIssueList ValidateArCycleInput(const ArCycleInput& input) {
  ValidationIssueList issues = ValidateArCycleDeltaTime(input.dt_sec);
  ValidatePlatformPose(input.platform_pose, &issues);
  ValidatePlatformAltitude(input.platform_altitude_m, &issues);
  ValidatePlatformEcefKinematics(input, &issues);
  if (input.has_environment) {
    ValidateEnvironmentInput(input.environment, input.dt_sec, &issues);
  } else if (!IsDefaultEnvironmentInput(input.environment)) {
    issues.push_back(
        MakeIssue(ValidationSeverity::kError, ValidationCode::kEnvironmentSnapshotFlagMismatch,
                  ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                  "has_environment", "environment data is present but has_environment is false"));
  }

  const ValidationIssueList target_issues = ValidateArSceneTargets(input.scene);
  issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  return issues;
}

ValidationIssueList ValidateArSceneTargets(const ArSceneTargetList& targets) {
  ValidationIssueList issues;
  std::unordered_map<std::uint64_t, std::size_t> first_seen_target_index;

  for (std::size_t i = 0; i < targets.size(); ++i) {
    ValidateSingleTarget(targets[i], i, &issues);
    const std::uint64_t external_target_id = targets[i].external_target_id;
    if (external_target_id == 0U) {
      continue;
    }

    const std::unordered_map<std::uint64_t, std::size_t>::const_iterator it =
        first_seen_target_index.find(external_target_id);
    if (it == first_seen_target_index.end()) {
      first_seen_target_index[external_target_id] = i;
      continue;
    }

    std::ostringstream stream;
    stream << "duplicate external target id " << external_target_id << " first seen at index "
           << it->second;
    issues.push_back(
        MakeIssue(ValidationSeverity::kError, ValidationCode::kDuplicateExternalTargetId,
                  ValidationLocationKind::kSceneEntity, i, "external_target_id", stream.str()));
  }

  return issues;
}

bool HasValidationError(const ValidationIssueList& issues) {
  return oneq::common::validation::HasSeverity<ValidationIssueList, ValidationSeverity,
                                               &ValidationIssue::severity>(
      issues, ValidationSeverity::kError);
}

}  // namespace session
}  // namespace airborne_radar
