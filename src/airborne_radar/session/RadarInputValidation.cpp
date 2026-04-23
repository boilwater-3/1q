#include "1q/airborne_radar/session/RadarInputValidation.h"

#include <sstream>
#include <unordered_map>

#include "common/validation/ValidationUtils.h"

namespace airborne_radar {
namespace session {

namespace {

/**
 * @brief 构造一条结构化校验结果。
 * @param severity 严重级别。
 * @param code 问题编码。
 * @param target_index 目标索引。
 * @param message 面向调用方的简短说明。
 * @return 组装后的校验结果。
 */
ValidationIssue MakeIssue(ValidationSeverity severity, ValidationCode code,
                          std::size_t target_index, const std::string& message) {
  return oneq::internal::validation::MakeIndexedIssue<
      ValidationIssue, ValidationSeverity, ValidationCode, &ValidationIssue::entity_index>(
      severity, code, target_index, message);
}

/**
 * @brief 判断输入浮点值是否为有限数。
 * @param value 输入浮点值。
 * @return 有限数时返回 `true`。
 */
bool IsFinite(float value) { return oneq::internal::validation::IsFinite(value); }

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

/**
 * @brief 判断目标是否携带笛卡尔位置。
 * @param target 目标特征。
 * @return `has_cartesian_position` 为 `true` 时返回 `true`。
 */
bool HasCartesianPosition(const model::TargetFeature& target) {
  return target.has_cartesian_position;
}

/**
 * @brief 收集单个目标的字段级校验问题。
 * @param target 目标特征。
 * @param target_index 目标索引。
 * @param[out] issues 输出问题列表。
 */
void ValidateSingleTarget(const model::TargetFeature& target, std::size_t target_index,
                          ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (!IsFinite(target.position_x) || !IsFinite(target.position_y) ||
      !IsFinite(target.position_z) || !IsFinite(target.current_track_velocity_x) ||
      !IsFinite(target.current_track_velocity_y) || !IsFinite(target.current_track_velocity_z) ||
      !IsFinite(target.current_track_speed) || !IsFinite(target.current_track_rcs) ||
      !IsFinite(target.range_m)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kNonFiniteTargetField,
                                target_index, "target contains non-finite numeric field"));
  }

  if (target.range_m <= 0.0f && !HasCartesianPosition(target)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kMissingRangeAndCartesianPosition, target_index,
                                "target requires positive range or cartesian position"));
  }

  if (target.external_target_id == 0U) {
    issues->push_back(MakeIssue(ValidationSeverity::kInfo, ValidationCode::kUnknownExternalTargetId,
                                target_index, "target external id is unknown"));
  }

  if (target.current_track_rcs < 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kWarning, ValidationCode::kNegativeRcs,
                                target_index, "target rcs is negative"));
  }
}

}  // namespace

ValidationIssueList ValidateRadarCycleDeltaTime(float dt_sec) {
  ValidationIssueList issues;
  if (!IsFinite(dt_sec)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kNonFiniteCycleDeltaTime,
                               static_cast<std::size_t>(-1), "cycle delta time must be finite"));
  } else if (dt_sec <= 0.0f) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidCycleDeltaTime,
                               static_cast<std::size_t>(-1), "cycle delta time is non-positive"));
  }
  return issues;
}

ValidationIssueList ValidateRadarCycleInput(const RadarCycleInput& input) {
  ValidationIssueList issues = ValidateRadarCycleDeltaTime(input.dt_sec);
  ValidatePlatformPose(input.platform_pose, &issues);

  const ValidationIssueList target_issues = ValidateTargetFeatures(input.scene.targets);
  issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  return issues;
}

ValidationIssueList ValidateTargetFeatures(const model::TargetFeatureList& targets) {
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
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kDuplicateExternalTargetId, i, stream.str()));
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
