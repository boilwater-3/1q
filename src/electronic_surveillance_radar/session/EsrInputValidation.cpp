#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"

#include <cstddef>
#include <string>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "common/validation/ValidationUtils.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

using oneq::common::validation::IsFinite;

ValidationIssue MakeIssue(ValidationSeverity severity, ValidationCode code,
                          ValidationLocationKind location_kind, std::size_t entity_index,
                          const std::string& field, const std::string& message) {
  return oneq::common::validation::MakeLocatedIssue<ValidationIssue, ValidationLocation>(
      severity, code, location_kind, entity_index, field, message);
}

void ValidatePlatform(const EsrCycleInput& input, ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }
  if (!IsFinite(input.platform_attitude_deg.yaw_deg) ||
      !IsFinite(input.platform_attitude_deg.pitch_deg) ||
      !IsFinite(input.platform_attitude_deg.roll_deg)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kNonFinitePlatformNumericField,
        ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1), "platform",
        "platform attitude must contain only finite values"));
  }
  if (!input.has_platform_ecef_kinematics || input.platform_entity_id == 0U ||
      !IsFinite(input.platform_position_ecef_m.x_m) ||
      !IsFinite(input.platform_position_ecef_m.y_m) ||
      !IsFinite(input.platform_position_ecef_m.z_m) ||
      !IsFinite(input.platform_velocity_ecef_mps.x_mps) ||
      !IsFinite(input.platform_velocity_ecef_mps.y_mps) ||
      !IsFinite(input.platform_velocity_ecef_mps.z_mps)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidRfEmissionFrame,
        ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1),
        "platform_entity_id/platform_ecef_kinematics",
        "RF reception requires a non-zero platform identity and finite ECEF kinematics"));
    return;
  }
  // 可定位性前置校验：接收链后续要把平台 ECEF 转 WGS84 LLA / ENU 求 AoA。一个 finite 但
  // 不可定位的点（如地心）会通过上面的 finite 校验，却在 pipeline 运行期失败并被伪装成
  // RF-link 拒绝。这里用与运行期相同的 TryEcefToLla 提前判定，让非法 ECEF 在输入校验即拒。
  oneq::coordinate::LlaPositionDegM platform_lla;
  if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &platform_lla)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kUnlocatablePlatformEcef,
        ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1),
        "platform_position_ecef_m",
        "platform ECEF must be geolocatable (convertible to a valid WGS84 LLA)"));
  }
}

}  // namespace

ValidationIssueList ValidateEsrCycleInput(const EsrCycleInput& input) {
  ValidationIssueList issues;
  if (!IsFinite(input.dt_sec)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kNonFiniteCycleDeltaTime,
                               ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1),
                               "dt_sec", "cycle delta time must be finite"));
  } else if (input.dt_sec <= 0.0f) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidCycleDeltaTime,
                               ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1),
                               "dt_sec", "cycle delta time must be positive"));
  }
  if (!IsFinite(input.cycle_start_time_s)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidCycleStartTime,
                               ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1),
                               "cycle_start_time_s", "cycle start time must be finite"));
  }
  const oneq::electromagnetics::RfEmissionFrame& frame = input.rf_emissions;
  if (!oneq::electromagnetics::RfFrameMatchesCycleWindow(
          frame, input.cycle_index, input.cycle_start_time_s,
          static_cast<double>(input.dt_sec)) ||
      !oneq::electromagnetics::TryValidateRfSceneFrame(frame)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidRfEmissionFrame,
                               ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1),
                               "rf_emissions",
                               "RF emission frame must be valid and match the cycle window"));
  }
  ValidatePlatform(input, &issues);
  return issues;
}

bool HasValidationError(const ValidationIssueList& issues) {
  return oneq::common::validation::HasSeverity<ValidationIssueList, ValidationSeverity,
                                               &ValidationIssue::severity>(
      issues, ValidationSeverity::kError);
}

}  // namespace session
}  // namespace electronic_surveillance_radar
