#include "1q/electro_optical_sensor/session/EosInputValidation.h"

#include <algorithm>
#include <limits>

#include "common/validation/ValidationUtils.h"

namespace electro_optical_sensor {
namespace session {

using ::electro_optical_sensor::session::EosCycleInput;
using ::electro_optical_sensor::session::EosSceneTarget;

namespace {

using oneq::common::validation::IsFinite;

// 统一问题列表模型（规则 14）：校验问题 code 为 "eos.validation.<snake_case>"，
// phase 固定为 kInputValidation；location/field 为可选定位。
EosIssue MakeIssue(EosIssueSeverity severity, const char* code,
                   oneq::foundation::ValidationLocationKind location_kind,
                   std::size_t entity_index, const std::string& field,
                   const std::string& message) {
  EosIssue issue = oneq::common::validation::MakeLocatedIssue<EosIssue,
                                                              oneq::foundation::ValidationLocation>(
      severity, std::string("eos.validation.") + code, location_kind, entity_index, field,
      message);
  issue.phase = EosIssuePhase::kInputValidation;
  return issue;
}

void ValidatePlatformPose(const oneq::foundation::PoseState& platform_pose,
                          EosIssueList* issues) {
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
        MakeIssue(EosIssueSeverity::kError, "non_finite_platform_numeric_field",
                  oneq::foundation::ValidationLocationKind::kPlatform,
                  static_cast<std::size_t>(-1), "platform_pose",
                  "platform pose contains non-finite numeric field"));
  }
}

void ValidatePlatformAltitude(float platform_altitude_m, EosIssueList* issues) {
  if (issues == nullptr || IsFinite(platform_altitude_m)) {
    return;
  }
  issues->push_back(MakeIssue(EosIssueSeverity::kError, "non_finite_platform_numeric_field",
                              oneq::foundation::ValidationLocationKind::kPlatform,
                              static_cast<std::size_t>(-1), "platform_altitude_m",
                              "platform altitude must be finite"));
}

void ValidateTarget(const EosSceneTarget& target, std::size_t target_index,
                    EosIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (!IsFinite(target.range_m) || !IsFinite(target.azimuth_deg) ||
      !IsFinite(target.elevation_deg) || !IsFinite(target.appearance.apparent_temperature_k) ||
      !IsFinite(target.appearance.emissivity) || !IsFinite(target.appearance.reflectance) ||
      !IsFinite(target.appearance.projected_area_m2)) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, "non_finite_target_numeric_field",
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "scene",
                                "target contains non-finite numeric field"));
  }

  if (target.range_m <= 0.0f) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, "invalid_target_range",
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "range_m", "target range must be positive"));
  }
  if (target.appearance.apparent_temperature_k <= 0.0f) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, "invalid_target_temperature",
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "apparent_temperature_k",
                                "target temperature must be positive"));
  }
  if (!oneq::common::validation::IsRatio01(target.appearance.emissivity)) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, "invalid_target_emissivity",
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "emissivity", "target emissivity must be in [0, 1]"));
  }
  if (!oneq::common::validation::IsRatio01(target.appearance.reflectance)) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, "invalid_target_reflectance",
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "reflectance",
                                "target reflectance must be in [0, 1]"));
  }
  if (IsFinite(target.appearance.emissivity) && IsFinite(target.appearance.reflectance) &&
      (target.appearance.emissivity + target.appearance.reflectance > 1.0f + 1.0e-4f)) {
    issues->push_back(
        MakeIssue(EosIssueSeverity::kWarning, "inconsistent_target_energy_balance",
                  oneq::foundation::ValidationLocationKind::kSceneEntity, target_index,
                  "emissivity+reflectance",
                  "target emissivity + reflectance should not exceed 1"));
  }
  if (target.appearance.projected_area_m2 <= 0.0f) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, "invalid_target_projected_area",
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "projected_area_m2",
                                "target projected area must be positive"));
  }
}

}  // namespace

EosIssueList ValidateEosCycleInput(
    const ::electro_optical_sensor::session::EosCycleInput& input, float frame_rate_hz) {
  EosIssueList issues;

  if (!IsFinite(input.dt_sec)) {
    issues.push_back(MakeIssue(EosIssueSeverity::kError, "non_finite_cycle_delta_time",
                               oneq::foundation::ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "dt_sec",
                               "cycle delta time must be finite"));
  } else if (input.dt_sec <= 0.0f) {
    issues.push_back(MakeIssue(EosIssueSeverity::kError, "invalid_cycle_delta_time",
                               oneq::foundation::ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "dt_sec",
                               "cycle delta time must be positive"));
  } else {
    constexpr float kMaxDtFactor = 10.0f;
    const float max_dt_sec =
        kMaxDtFactor / std::max(frame_rate_hz, std::numeric_limits<float>::min());
    if (input.dt_sec > max_dt_sec) {
      issues.push_back(MakeIssue(EosIssueSeverity::kError,
                                 "cycle_delta_time_exceeds_frame_period",
                                 oneq::foundation::ValidationLocationKind::kGlobal,
                                 static_cast<std::size_t>(-1), "dt_sec",
                                 "cycle delta time exceeds reasonable range based on frame_rate_hz"));
    }
  }

  ValidatePlatformPose(input.platform_pose, &issues);
  ValidatePlatformAltitude(input.platform_altitude_m, &issues);

  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    ValidateTarget(input.scene[i], i, &issues);
  }

  return issues;
}

bool HasValidationError(const EosIssueList& issues) {
  for (const EosIssue& issue : issues) {
    if (issue.phase == EosIssuePhase::kInputValidation &&
        issue.severity == EosIssueSeverity::kError) {
      return true;
    }
  }
  return false;
}

}  // namespace session
}  // namespace electro_optical_sensor
