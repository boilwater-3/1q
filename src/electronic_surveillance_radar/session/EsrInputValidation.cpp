#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"

#include <cstddef>
#include <string>

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
  if (!IsFinite(input.platform_altitude_m) ||
      !IsFinite(input.platform_pose.position_m.x) || !IsFinite(input.platform_pose.position_m.y) ||
      !IsFinite(input.platform_pose.position_m.z) ||
      !IsFinite(input.platform_pose.velocity_mps.x) || !IsFinite(input.platform_pose.velocity_mps.y) ||
      !IsFinite(input.platform_pose.velocity_mps.z) ||
      !IsFinite(input.platform_pose.attitude_deg.yaw_deg) ||
      !IsFinite(input.platform_pose.attitude_deg.pitch_deg) ||
      !IsFinite(input.platform_pose.attitude_deg.roll_deg)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kNonFinitePlatformNumericField,
        ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1), "platform",
        "platform altitude and pose must contain only finite values"));
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
  }
}

void ValidateEnvironment(const EsrEnvironmentInput& environment, ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }
  const EsrAtmosphericObservation& atmosphere = environment.atmospheric_observation;
  if (!oneq::common::validation::IsRatio01(environment.spectrum_occupancy_ratio) ||
      !oneq::common::validation::IsRatio01(atmosphere.relative_humidity_ratio) ||
      !IsFinite(atmosphere.precipitation_rate_mmph) || !IsFinite(atmosphere.visibility_km) ||
      atmosphere.precipitation_rate_mmph < 0.0f || atmosphere.visibility_km <= 0.0f) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidEnvironmentObservation,
        ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1), "environment",
        "environment must contain finite ratios in [0, 1] and positive visibility"));
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
  const oneq::electromagnetics::RfEmissionFrame& frame = input.interference;
  if (frame.world_cycle_index != input.cycle_index ||
      frame.window_start_time_s != input.cycle_start_time_s ||
      frame.window_duration_s != static_cast<double>(input.dt_sec) ||
      !oneq::electromagnetics::TryValidateRfSceneFrame(frame)) {
    issues.push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidRfEmissionFrame,
                               ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1),
                               "interference",
                               "RF emission frame must be valid and match the cycle window"));
  }
  ValidatePlatform(input, &issues);
  ValidateEnvironment(input.environment, &issues);
  return issues;
}

bool HasValidationError(const ValidationIssueList& issues) {
  return oneq::common::validation::HasSeverity<ValidationIssueList, ValidationSeverity,
                                               &ValidationIssue::severity>(
      issues, ValidationSeverity::kError);
}

}  // namespace session
}  // namespace electronic_surveillance_radar
