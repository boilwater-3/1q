#include "1q/sbirs_sensor/session/SbirsInputValidation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "common/validation/ValidationUtils.h"

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

// 统一问题列表模型（规则 14）：校验问题 code 为 "sbirs.validation.<snake_case>"，
// phase 固定为 kInputValidation；location/field 为可选定位。
void AddError(const char* code, const char* field, const char* message,
              oneq::foundation::ValidationLocation location, SbirsIssueList* issues) {
  SbirsIssue issue;
  issue.severity = SbirsIssueSeverity::kError;
  issue.phase = SbirsIssuePhase::kInputValidation;
  issue.code = std::string("sbirs.validation.") + code;
  issue.message = message;
  issue.location = location;
  issue.field = field;
  issues->push_back(issue);
}

}  // namespace

SbirsIssueList ValidateSbirsCycleInput(const SbirsCycleInput& input, float frame_rate_hz) {
  SbirsIssueList issues;
  if (input.dt_sec <= 0.0f || !std::isfinite(input.dt_sec)) {
    oneq::foundation::ValidationLocation location;
    location.kind = oneq::foundation::ValidationLocationKind::kGlobal;
    AddError("invalid_cycle_delta_time", "dt_sec", "dt_sec must be positive and finite", location,
             &issues);
  } else {
    constexpr float kMaxDtFactor = 10.0f;
    const float max_dt_sec =
        kMaxDtFactor / std::max(frame_rate_hz, std::numeric_limits<float>::min());
    if (input.dt_sec > max_dt_sec) {
      oneq::foundation::ValidationLocation location;
      location.kind = oneq::foundation::ValidationLocationKind::kGlobal;
      AddError("cycle_delta_time_exceeds_frame_period", "dt_sec",
               "dt_sec exceeds reasonable range based on frame_rate_hz", location, &issues);
    }
  }
  if (!input.has_satellite_position || !FiniteVector(input.satellite_position_ecef_m) ||
      !NonZeroVector(input.satellite_position_ecef_m)) {
    oneq::foundation::ValidationLocation location;
    location.kind = oneq::foundation::ValidationLocationKind::kPlatform;
    AddError("invalid_satellite_position", "satellite_position_ecef_m",
             "satellite position must be provided, finite, and non-zero", location, &issues);
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
      oneq::foundation::ValidationLocation location;
      location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
      location.entity_index = i;
      AddError("invalid_target_physical", "scene",
               "target physical inputs must be finite and positive where required", location,
               &issues);
    }
  }
  return issues;
}

bool HasValidationError(const SbirsIssueList& issues) {
  // RED-1 收敛：统一判定逻辑在 common::validation::HasValidationPhaseError
  // （phase == kInputValidation && severity == kError）。
  return oneq::common::validation::HasValidationPhaseError(
      issues, &SbirsIssue::phase, &SbirsIssue::severity);
}

}  // namespace session
}  // namespace sbirs_sensor
