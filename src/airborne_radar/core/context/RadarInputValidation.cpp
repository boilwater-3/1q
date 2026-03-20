#include "1q/airborne_radar/core/context/RadarInputValidation.h"

#include <cmath>
#include <sstream>
#include <unordered_map>

namespace airborne_radar {
namespace core {
namespace context {

namespace {

/**
 * @brief 构造一条结构化校验结果。
 * @param severity 严重级别。
 * @param code 问题编码。
 * @param target_index 目标索引。
 * @param message 面向调用方的简短说明。
 * @return 组装后的校验结果。
 */
ValidationIssue MakeIssue(ValidationSeverity severity,
                          ValidationCode code,
                          std::size_t target_index,
                          const std::string& message) {
  ValidationIssue issue;
  issue.severity = severity;
  issue.code = code;
  issue.target_index = target_index;
  issue.message = message;
  return issue;
}

/**
 * @brief 判断输入浮点值是否为有限数。
 * @param value 输入浮点值。
 * @return 有限数时返回 `true`。
 */
bool IsFinite(float value) {
  return std::isfinite(value) != 0;
}

/**
 * @brief 判断目标是否携带笛卡尔位置。
 * @param target 目标特征。
 * @return 至少存在一个非零位置分量时返回 `true`。
 */
bool HasCartesianPosition(const common::TargetFeature& target) {
  return target.position_x != 0.0f || target.position_y != 0.0f ||
         target.position_z != 0.0f;
}

/**
 * @brief 收集单个目标的字段级校验问题。
 * @param target 目标特征。
 * @param target_index 目标索引。
 * @param[out] issues 输出问题列表。
 */
void ValidateSingleTarget(const common::TargetFeature& target,
                          std::size_t target_index,
                          std::vector<ValidationIssue>* issues) {
  if (issues == nullptr) {
    return;
  }

  if (!IsFinite(target.position_x) || !IsFinite(target.position_y) ||
      !IsFinite(target.position_z) ||
      !IsFinite(target.current_track_velocity_x) ||
      !IsFinite(target.current_track_velocity_y) ||
      !IsFinite(target.current_track_velocity_z) ||
      !IsFinite(target.current_track_speed) ||
      !IsFinite(target.current_track_rcs) || !IsFinite(target.range_m)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kNonFiniteTargetField,
        target_index, "target contains non-finite numeric field"));
  }

  if (target.range_m <= 0.0f && !HasCartesianPosition(target)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError,
        ValidationCode::kMissingRangeAndCartesianPosition, target_index,
        "target requires positive range or cartesian position"));
  }

  if (target.external_target_id == 0U) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kInfo, ValidationCode::kUnknownExternalTargetId,
        target_index, "target external id is unknown"));
  }

  if (target.current_track_rcs < 0.0f) {
    issues->push_back(MakeIssue(ValidationSeverity::kWarning,
                                ValidationCode::kNegativeRcs, target_index,
                                "target rcs is negative"));
  }
}

} // namespace

std::vector<ValidationIssue>
ValidateRadarCycleInput(const RadarCycleInput& input) {
  std::vector<ValidationIssue> issues;

  if (!IsFinite(input.dt_sec)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kNonFiniteCycleDeltaTime,
                               static_cast<std::size_t>(-1),
                               "cycle delta time must be finite"));
  } else if (input.dt_sec <= 0.0f) {
    issues.push_back(MakeIssue(ValidationSeverity::kWarning,
                               ValidationCode::kInvalidCycleDeltaTime,
                               static_cast<std::size_t>(-1),
                               "cycle delta time is non-positive"));
  }

  const std::vector<ValidationIssue> target_issues =
      ValidateTargetFeatures(input.target_features);
  issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  return issues;
}

std::vector<ValidationIssue>
ValidateTargetFeatures(const common::TargetFeatureList& targets) {
  std::vector<ValidationIssue> issues;
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
    stream << "duplicate external target id " << external_target_id
           << " first seen at index " << it->second;
    issues.push_back(MakeIssue(ValidationSeverity::kWarning,
                               ValidationCode::kDuplicateExternalTargetId, i,
                               stream.str()));
  }

  return issues;
}

bool HasValidationError(const std::vector<ValidationIssue>& issues) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].severity == ValidationSeverity::kError) {
      return true;
    }
  }
  return false;
}

} // namespace context
} // namespace core
} // namespace airborne_radar
