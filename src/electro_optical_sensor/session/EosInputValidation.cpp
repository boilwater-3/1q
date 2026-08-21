#include "1q/electro_optical_sensor/session/EosInputValidation.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "1q/electro_optical_sensor/session/EosIssueCodes.h"
#include "common/validation/ValidationUtils.h"
#include "electro_optical_sensor/foundation/EosLookAngles.h"

namespace electro_optical_sensor {
namespace session {

using ::electro_optical_sensor::session::EosCycleInput;
using ::electro_optical_sensor::session::EosSceneTarget;

namespace {

using oneq::common::validation::IsFinite;

// 统一问题列表模型（规则 14）：校验问题 code 引用 EosIssueCodes.h 常量
// （"eos.validation.<snake_case>" 全集单一事实来源）；phase 固定为
// kInputValidation；location/field 为可选定位。
EosIssue MakeIssue(EosIssueSeverity severity, const char* code,
                   oneq::foundation::ValidationLocationKind location_kind,
                   std::size_t entity_index, const std::string& field,
                   const std::string& message) {
  EosIssue issue = oneq::common::validation::MakeLocatedIssue<EosIssue,
                                                              oneq::foundation::ValidationLocation>(
      severity, code, location_kind, entity_index, field, message);
  issue.phase = EosIssuePhase::kInputValidation;
  return issue;
}

void ValidatePlatformAttitude(const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
                             EosIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (!IsFinite(platform_attitude_deg.yaw_deg) ||
      !IsFinite(platform_attitude_deg.pitch_deg) ||
      !IsFinite(platform_attitude_deg.roll_deg)) {
    issues->push_back(
        MakeIssue(EosIssueSeverity::kError, codes::kNonFinitePlatformNumericField,
                  oneq::foundation::ValidationLocationKind::kPlatform,
                  static_cast<std::size_t>(-1), "platform_attitude_deg",
                  "platform attitude contains non-finite numeric field"));
  }
}

void ValidatePlatformAltitude(float platform_altitude_m, EosIssueList* issues) {
  if (issues == nullptr || IsFinite(platform_altitude_m)) {
    return;
  }
  issues->push_back(MakeIssue(EosIssueSeverity::kError, codes::kNonFinitePlatformNumericField,
                              oneq::foundation::ValidationLocationKind::kPlatform,
                              static_cast<std::size_t>(-1), "platform_altitude_m",
                              "platform altitude must be finite"));
}

void ValidateTarget(const EosSceneTarget& target, std::size_t target_index,
                    EosIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (!IsFinite(target.position_x) || !IsFinite(target.position_y) ||
      !IsFinite(target.position_z) || !IsFinite(target.velocity_x) ||
      !IsFinite(target.velocity_y) || !IsFinite(target.velocity_z) ||
      !IsFinite(target.appearance.apparent_temperature_k) ||
      !IsFinite(target.appearance.emissivity) || !IsFinite(target.appearance.reflectance) ||
      !IsFinite(target.appearance.projected_area_m2)) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, codes::kNonFiniteTargetNumericField,
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "scene",
                                "target contains non-finite numeric field"));
  }

  // 平台锚点 ENU 位置模长即斜距：退化几何（目标与平台重合）拒绝。
  const double position_norm =
      std::sqrt(static_cast<double>(target.position_x) * target.position_x +
                static_cast<double>(target.position_y) * target.position_y +
                static_cast<double>(target.position_z) * target.position_z);
  if (IsFinite(target.position_x) && IsFinite(target.position_y) &&
      IsFinite(target.position_z) &&
      position_norm <= static_cast<double>(foundation::EosLookAngleNormFloorM())) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, codes::kInvalidTargetRange,
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "position",
                                "target ENU position must be non-degenerate (range positive)"));
  }
  if (target.appearance.apparent_temperature_k <= 0.0f) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, codes::kInvalidTargetTemperature,
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "apparent_temperature_k",
                                "target temperature must be positive"));
  }
  if (!oneq::common::validation::IsRatio01(target.appearance.emissivity)) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, codes::kInvalidTargetEmissivity,
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "emissivity", "target emissivity must be in [0, 1]"));
  }
  if (!oneq::common::validation::IsRatio01(target.appearance.reflectance)) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, codes::kInvalidTargetReflectance,
                                oneq::foundation::ValidationLocationKind::kSceneEntity,
                                target_index, "reflectance",
                                "target reflectance must be in [0, 1]"));
  }
  if (IsFinite(target.appearance.emissivity) && IsFinite(target.appearance.reflectance) &&
      (target.appearance.emissivity + target.appearance.reflectance > 1.0f + 1.0e-4f)) {
    issues->push_back(
        MakeIssue(EosIssueSeverity::kWarning, codes::kInconsistentTargetEnergyBalance,
                  oneq::foundation::ValidationLocationKind::kSceneEntity, target_index,
                  "emissivity+reflectance",
                  "target emissivity + reflectance should not exceed 1"));
  }
  if (target.appearance.projected_area_m2 <= 0.0f) {
    issues->push_back(MakeIssue(EosIssueSeverity::kError, codes::kInvalidTargetProjectedArea,
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
    issues.push_back(MakeIssue(EosIssueSeverity::kError, codes::kNonFiniteCycleDeltaTime,
                               oneq::foundation::ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "dt_sec",
                               "cycle delta time must be finite"));
  } else if (input.dt_sec <= 0.0f) {
    issues.push_back(MakeIssue(EosIssueSeverity::kError, codes::kInvalidCycleDeltaTime,
                               oneq::foundation::ValidationLocationKind::kGlobal,
                               static_cast<std::size_t>(-1), "dt_sec",
                               "cycle delta time must be positive"));
  } else {
    constexpr float kMaxDtFactor = 10.0f;
    const float max_dt_sec =
        kMaxDtFactor / std::max(frame_rate_hz, std::numeric_limits<float>::min());
    if (input.dt_sec > max_dt_sec) {
      issues.push_back(MakeIssue(EosIssueSeverity::kError,
                                 codes::kCycleDeltaTimeExceedsFramePeriod,
                                 oneq::foundation::ValidationLocationKind::kGlobal,
                                 static_cast<std::size_t>(-1), "dt_sec",
                                 "cycle delta time exceeds reasonable range based on frame_rate_hz"));
    }
  }

  ValidatePlatformAttitude(input.platform_attitude_deg, &issues);
  ValidatePlatformAltitude(input.platform_altitude_m, &issues);

  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    ValidateTarget(input.scene[i], i, &issues);
  }

  return issues;
}

bool HasValidationError(const EosIssueList& issues) {
  // RED-1 收敛：统一判定逻辑在 common::validation::HasValidationPhaseError
  // （phase == kInputValidation && severity == kError）。
  return oneq::common::validation::HasValidationPhaseError(
      issues, &EosIssue::phase, &EosIssue::severity);
}

}  // namespace session
}  // namespace electro_optical_sensor
