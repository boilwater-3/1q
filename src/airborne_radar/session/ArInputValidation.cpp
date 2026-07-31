#include "1q/airborne_radar/session/ArInputValidation.h"

#include <cmath>
#include <sstream>
#include <unordered_map>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
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

bool FrameMatchesCycle(const ArCycleInput& input) {
  if (input.interference.emissions.empty()) {
    return true;
  }
  return input.interference.world_cycle_index == input.cycle_index &&
         input.interference.window_start_time_s == input.cycle_start_time_s &&
         input.interference.window_duration_s == input.dt_sec;
}

bool HasCartesianPosition(const ArSceneTarget& target) {
  return target.position_x != 0.0f || target.position_y != 0.0f || target.position_z != 0.0f;
}

void ValidateSingleSceneTarget(const ArSceneTarget& target, std::size_t target_index,
                               ValidationIssueList* issues) {
  if (!IsFinite(target.position_x) || !IsFinite(target.position_y) ||
      !IsFinite(target.position_z) || !IsFinite(target.velocity_x) ||
      !IsFinite(target.velocity_y) || !IsFinite(target.velocity_z) || !IsFinite(target.rcs) ||
      !IsFinite(target.range_m)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kNonFiniteTargetField,
                                ValidationLocationKind::kSceneEntity, target_index, "target",
                                "target contains non-finite numeric field"));
  }
  if (target.range_m <= 0.0f && !HasCartesianPosition(target)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kMissingRangeAndCartesianPosition,
                                ValidationLocationKind::kSceneEntity, target_index, "range_m",
                                "target requires positive range or cartesian position"));
  }
  if (target.external_target_id == 0U) {
    issues->push_back(MakeIssue(ValidationSeverity::kInfo,
                                ValidationCode::kUnknownExternalTargetId,
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

ValidationIssueList ValidateArCycleDeltaTime(double dt_sec) {
  ValidationIssueList issues;
  if (!std::isfinite(dt_sec)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kNonFiniteCycleDeltaTime,
                               ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "dt_sec",
                               "cycle duration must be finite"));
  } else if (dt_sec <= 0.0) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kInvalidCycleDeltaTime,
                               ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "dt_sec",
                               "cycle duration must be positive"));
  }
  return issues;
}

ValidationIssueList ValidateArCycleInput(const ArCycleInput& input) {
  ValidationIssueList issues = ValidateArCycleDeltaTime(input.dt_sec);
  if (input.cycle_index == 0U) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kInvalidCycleIndex,
                               ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "cycle_index",
                               "cycle index must be non-zero"));
  }
  if (!std::isfinite(input.cycle_start_time_s) || input.cycle_start_time_s < 0.0) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kInvalidCycleStartTime,
                               ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "cycle_start_time_s",
                               "cycle start time must be finite and non-negative"));
  }

  oneq::coordinate::LocalFrameReference reference;
  oneq::foundation::Vector3f radar_local_velocity;
  const oneq::coordinate::EulerAnglesDeg zero_mount{};
  if (input.platform.platform_entity_id == 0U ||
      !oneq::coordinate::IsFinite(input.platform.platform_position_ecef_m) ||
      !oneq::coordinate::IsFinite(input.platform.platform_velocity_mps) ||
      !oneq::coordinate::IsFinite(input.platform.platform_attitude_deg) ||
      !TryMakeArPoseFromExternalKinematics(input.platform, zero_mount, &reference,
                                            &radar_local_velocity)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kInvalidPlatformInput,
                               ValidationLocationKind::kPlatform,
                               static_cast<std::size_t>(-1), "platform",
                               "platform requires non-zero identity and finite world kinematics"));
  } else {
    std::unordered_map<std::uint64_t, std::size_t> first_seen_target_index;
    for (std::size_t index = 0U; index < input.targets.size(); ++index) {
      const ArTargetInput& target = input.targets[index];
      ArSceneTarget local_target;
      if (!TryMakeArTargetFromExternalKinematics(target, reference,
                                                 radar_local_velocity,
                                                 &local_target)) {
        issues.push_back(MakeIssue(ValidationSeverity::kError,
                                   ValidationCode::kInvalidTargetInput,
                                   ValidationLocationKind::kSceneEntity, index, "targets",
                                   "target world kinematics cannot be converted"));
        continue;
      }
      ValidateSingleSceneTarget(local_target, index, &issues);
      if (target.target_id == 0U) {
        continue;
      }
      const std::unordered_map<std::uint64_t, std::size_t>::const_iterator found =
          first_seen_target_index.find(target.target_id);
      if (found == first_seen_target_index.end()) {
        first_seen_target_index[target.target_id] = index;
      } else {
        std::ostringstream stream;
        stream << "duplicate target id " << target.target_id << " first seen at index "
               << found->second;
        issues.push_back(MakeIssue(ValidationSeverity::kError,
                                   ValidationCode::kDuplicateExternalTargetId,
                                   ValidationLocationKind::kSceneEntity, index, "target_id",
                                   stream.str()));
      }
    }
  }

  if (!FrameMatchesCycle(input)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kInterferenceFrameMismatch,
                               ValidationLocationKind::kEnvironment,
                               static_cast<std::size_t>(-1), "interference",
                               "non-empty interference frame must match the AR cycle window"));
  } else if (!input.interference.emissions.empty() &&
             !oneq::electromagnetics::TryValidateRfSceneFrame(input.interference)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError,
                               ValidationCode::kInvalidInterferenceInput,
                               ValidationLocationKind::kEnvironment,
                               static_cast<std::size_t>(-1), "interference",
                               "interference frame contains invalid RF facts"));
  }
  return issues;
}

ValidationIssueList ValidateArSceneTargets(const ArSceneTargetList& targets) {
  ValidationIssueList issues;
  std::unordered_map<std::uint64_t, std::size_t> first_seen_target_index;
  for (std::size_t index = 0U; index < targets.size(); ++index) {
    ValidateSingleSceneTarget(targets[index], index, &issues);
    const std::uint64_t id = targets[index].external_target_id;
    if (id == 0U) {
      continue;
    }
    const std::unordered_map<std::uint64_t, std::size_t>::const_iterator found =
        first_seen_target_index.find(id);
    if (found == first_seen_target_index.end()) {
      first_seen_target_index[id] = index;
    } else {
      issues.push_back(MakeIssue(ValidationSeverity::kError,
                                 ValidationCode::kDuplicateExternalTargetId,
                                 ValidationLocationKind::kSceneEntity, index,
                                 "external_target_id", "duplicate external target id"));
    }
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
