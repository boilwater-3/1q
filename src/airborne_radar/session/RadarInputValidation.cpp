#include "1q/airborne_radar/session/RadarInputValidation.h"

#include <sstream>
#include <unordered_map>

#include "common/validation/ValidationUtils.h"

namespace airborne_radar {
namespace session {

namespace {

ValidationLocation MakeLocation(ValidationLocationKind kind, std::size_t entity_index) {
  ValidationLocation location;
  location.kind = kind;
  location.entity_index = entity_index;
  return location;
}

ValidationIssue MakeIssue(ValidationSeverity severity, ValidationCode code,
                          ValidationLocationKind location_kind, std::size_t entity_index,
                          const std::string& field, const std::string& message) {
  ValidationIssue issue;
  issue.severity = severity;
  issue.code = code;
  issue.location = MakeLocation(location_kind, entity_index);
  issue.field = field;
  issue.message = message;
  return issue;
}

/**
 * @brief 判断输入浮点值是否为有限数。
 * @param value 输入浮点值。
 * @return 有限数时返回 `true`。
 */
bool IsFinite(float value) { return oneq::internal::validation::IsFinite(value); }

bool IsRatioValid(float value) { return IsFinite(value) && value >= 0.0f && value <= 1.0f; }

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

void ValidateEnvironmentInput(const RadarEnvironmentInput& environment,
                              ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }
  const environment::AtmosphericPhysicsConfig& atmosphere = environment.atmospheric_observation;
  if (!IsFinite(atmosphere.pressure_hpa) || !IsFinite(atmosphere.temperature_k) ||
      !IsFinite(atmosphere.relative_humidity) || atmosphere.pressure_hpa <= 0.0f ||
      atmosphere.temperature_k <= 0.0f || !IsRatioValid(atmosphere.relative_humidity)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidEnvironmentObservation,
                                ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
                                "environment.atmospheric_observation",
                                "atmospheric observation must contain positive "
                                "pressure/temperature and humidity in [0, 1]"));
  }
  const environment::AtmosphericDerivedContext& context = environment.atmospheric_context;
  if (!IsFinite(context.solar_flux_f107a) || !IsFinite(context.solar_flux_f107) ||
      !IsFinite(context.geomagnetic_ap)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidEnvironmentObservation,
        ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
        "environment.atmospheric_context", "atmospheric context numeric fields must be finite"));
  }
  for (std::size_t i = 0; i < environment.jammer_sources.size(); ++i) {
    const environment::JammerEmitterState& jammer = environment.jammer_sources[i];
    if (!IsFinite(jammer.power_db) || !IsFinite(jammer.js_db) || !IsFinite(jammer.azimuth_deg) ||
        !IsFinite(jammer.elevation_deg) || !IsFinite(jammer.angular_span_deg) ||
        !IsFinite(jammer.confidence) || jammer.power_db < 0.0f || jammer.js_db < 0.0f ||
        jammer.angular_span_deg < 0.0f || !IsRatioValid(jammer.confidence)) {
      issues->push_back(MakeIssue(
          ValidationSeverity::kError, ValidationCode::kInvalidEnvironmentObservation,
          ValidationLocationKind::kEnvironment, i, "environment.jammer_sources",
          "jammer source must contain finite non-negative powers/span and confidence in [0, 1]"));
    }
  }
}

/**
 * @brief 判断目标是否携带笛卡尔位置。
 * @param target 目标特征。
 * @return 存在非零笛卡尔位置分量时返回 `true`。
 */
bool HasCartesianPosition(const RadarSceneTarget& target) {
  return target.position_x != 0.0f || target.position_y != 0.0f || target.position_z != 0.0f;
}

/**
 * @brief 收集单个目标的字段级校验问题。
 * @param target 目标特征。
 * @param target_index 目标索引。
 * @param[out] issues 输出问题列表。
 */
void ValidateSingleTarget(const RadarSceneTarget& target, std::size_t target_index,
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

ValidationIssueList ValidateRadarCycleDeltaTime(float dt_sec) {
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

ValidationIssueList ValidateRadarCycleInput(const RadarCycleInput& input) {
  ValidationIssueList issues = ValidateRadarCycleDeltaTime(input.dt_sec);
  ValidatePlatformPose(input.platform_pose, &issues);
  ValidatePlatformAltitude(input.platform_altitude_m, &issues);
  ValidateEnvironmentInput(input.environment, &issues);

  const ValidationIssueList target_issues = ValidateRadarSceneTargets(input.scene);
  issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  return issues;
}

ValidationIssueList ValidateRadarSceneTargets(const RadarSceneTargetList& targets) {
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
  return oneq::internal::validation::HasSeverity<ValidationIssueList, ValidationSeverity,
                                                 &ValidationIssue::severity>(
      issues, ValidationSeverity::kError);
}

}  // namespace session
}  // namespace airborne_radar
