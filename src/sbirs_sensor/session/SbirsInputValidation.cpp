#include "1q/sbirs_sensor/session/SbirsInputValidation.h"

#include <cmath>

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"

namespace sbirs_sensor {
namespace session {
namespace {

bool Finite(double value) { return std::isfinite(value); }

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
  if (!input.has_satellite_position || !Finite(input.satellite_position_ecef_m.x) ||
      !Finite(input.satellite_position_ecef_m.y) || !Finite(input.satellite_position_ecef_m.z)) {
    ValidationLocation location;
    location.kind = ValidationLocationKind::kPlatform;
    AddError("satellite position must be provided and finite", location, &issues);
  }
  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    const SbirsSceneTarget& target = input.scene[i];
    if (!Finite(target.position_ecef_m.x) || !Finite(target.position_ecef_m.y) ||
        !Finite(target.position_ecef_m.z) || target.temperature_k <= 0.0f ||
        target.projected_area_m2 < 0.0f) {
      ValidationLocation location;
      location.kind = ValidationLocationKind::kSceneEntity;
      location.entity_index = i;
      AddError("target physical inputs must be finite and positive where required", location,
               &issues);
    }
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
