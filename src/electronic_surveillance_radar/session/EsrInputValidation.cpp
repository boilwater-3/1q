#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"

#include <cstddef>
#include <cstdint>
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

/**
 * @brief 校验平台位姿输入字段。
 * @param[in] platform_pose 平台位姿输入。
 * @param[out] issues 校验问题列表。
 */
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

void ValidatePlatformRfKinematics(const EsrCycleInput& input, ValidationIssueList* issues) {
  if (issues == nullptr || !input.has_platform_ecef_kinematics) {
    return;
  }
  if (!IsFinite(input.platform_position_ecef_m.x_m) ||
      !IsFinite(input.platform_position_ecef_m.y_m) ||
      !IsFinite(input.platform_position_ecef_m.z_m) ||
      !IsFinite(input.platform_velocity_ecef_mps.x_mps) ||
      !IsFinite(input.platform_velocity_ecef_mps.y_mps) ||
      !IsFinite(input.platform_velocity_ecef_mps.z_mps)) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kNonFinitePlatformNumericField,
        ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1),
        "platform_position_ecef_m/platform_velocity_ecef_mps",
        "enabled platform ECEF kinematics must be finite"));
  }
}

void ValidateEnvironmentObservation(const EsrEnvironmentInput& observation, double cycle_duration_s,
                                    ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }
  const EsrAtmosphericObservation& atmosphere = observation.atmospheric_observation;
  if (!oneq::common::validation::IsRatio01(observation.spectrum_occupancy_ratio) ||
      !oneq::common::validation::IsRatio01(atmosphere.relative_humidity_ratio) ||
      !IsFinite(atmosphere.precipitation_rate_mmph) || !IsFinite(atmosphere.visibility_km) ||
      atmosphere.precipitation_rate_mmph < 0.0f || atmosphere.visibility_km <= 0.0f) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidEnvironmentObservation,
        ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1), "environment",
        "environment observation must contain finite ratios in [0, 1] and positive visibility"));
  }
  const bool has_legacy = !observation.jammer_sources.empty();
  const bool has_engineering = !observation.engineering_emissions.empty();
  bool mode_matches_payload = false;
  switch (observation.interference_mode) {
    case oneq::electromagnetics::RfInterferenceMode::kNone:
      mode_matches_payload = !has_legacy && !has_engineering;
      break;
    case oneq::electromagnetics::RfInterferenceMode::kLegacy:
      mode_matches_payload = has_legacy && !has_engineering;
      break;
    case oneq::electromagnetics::RfInterferenceMode::kEngineering:
      mode_matches_payload = !has_legacy && has_engineering;
      break;
    default:
      break;
  }
  if (!mode_matches_payload ||
      (has_engineering && !oneq::electromagnetics::TryValidateRfEmissionFrame(
                              observation.engineering_emissions, cycle_duration_s))) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidInterferenceInput,
        ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1),
        "environment.interference_mode", "interference mode and payload must match exactly"));
  }
  for (std::size_t i = 0; i < observation.jammer_sources.size(); ++i) {
    const session::EsrJammerSource& jammer = observation.jammer_sources[i];
    if (!IsFinite(jammer.center_hz) || !IsFinite(jammer.bandwidth_hz) ||
        !IsFinite(jammer.power_w) || !IsFinite(jammer.deception_risk) ||
        !IsFinite(jammer.confidence) || jammer.center_hz < 0.0 || jammer.bandwidth_hz < 0.0 ||
        jammer.power_w < 0.0f || !oneq::common::validation::IsRatio01(jammer.deception_risk) ||
        !oneq::common::validation::IsRatio01(jammer.confidence)) {
      issues->push_back(MakeIssue(
          ValidationSeverity::kError, ValidationCode::kInvalidEnvironmentObservation,
          ValidationLocationKind::kEnvironment, i, "environment.jammer_sources",
          "jammer source must contain finite non-negative RF/power fields and ratios in [0, 1]"));
    }
  }
}

/**
 * @brief 校验单个辐射源输入字段。
 * @param[in] emitter 单个辐射源输入。
 * @param[in] emitter_index 实体索引。
 * @param[out] issues 校验问题列表。
 */
void ValidateEmitter(const session::EsrSceneEmitter& emitter, std::size_t emitter_index,
                     ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (!IsFinite(emitter.carrier_hz) || !IsFinite(emitter.bandwidth_hz) ||
      !IsFinite(emitter.tx_power_w) || !IsFinite(emitter.pulse_width_s) ||
      !IsFinite(emitter.pri_s) || !IsFinite(emitter.pose.position_m.x) ||
      !IsFinite(emitter.pose.position_m.y) || !IsFinite(emitter.pose.position_m.z) ||
      !IsFinite(emitter.pose.velocity_mps.x) || !IsFinite(emitter.pose.velocity_mps.y) ||
      !IsFinite(emitter.pose.velocity_mps.z) || !IsFinite(emitter.pose.attitude_deg.yaw_deg) ||
      !IsFinite(emitter.pose.attitude_deg.pitch_deg) ||
      !IsFinite(emitter.pose.attitude_deg.roll_deg) ||
      !IsFinite(emitter.beam_state.center_az_deg) || !IsFinite(emitter.beam_state.center_el_deg) ||
      !IsFinite(emitter.beam_state.az_beamwidth_deg) ||
      !IsFinite(emitter.beam_state.el_beamwidth_deg)) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kNonFiniteEmitterNumericField,
                                ValidationLocationKind::kSceneEntity, emitter_index, "scene",
                                "emitter contains non-finite numeric field"));
  }

  if (emitter.carrier_hz <= 0.0) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidEmitterFrequency,
                                ValidationLocationKind::kSceneEntity, emitter_index, "carrier_hz",
                                "emitter carrier frequency must be positive"));
  }
  if (emitter.bandwidth_hz <= 0.0) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidEmitterBandwidth,
                                ValidationLocationKind::kSceneEntity, emitter_index, "bandwidth_hz",
                                "emitter bandwidth must be positive"));
  }
  if (emitter.tx_power_w <= 0.0) {
    issues->push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidEmitterPower,
                                ValidationLocationKind::kSceneEntity, emitter_index, "tx_power_w",
                                "emitter transmit power must be positive"));
  }
  if (emitter.pulse_width_s <= 0.0) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kInvalidEmitterPulseWidth,
                                ValidationLocationKind::kSceneEntity, emitter_index,
                                "pulse_width_s", "emitter pulse width must be positive"));
  }
  if (emitter.pri_s <= 0.0) {
    issues->push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kInvalidEmitterPri,
                                ValidationLocationKind::kSceneEntity, emitter_index, "pri_s",
                                "emitter pri must be positive"));
  }
  const bool pri_valid = emitter.pri_s > 0.0;
  const bool pulse_width_valid = emitter.pulse_width_s > 0.0;
  if (pri_valid && pulse_width_valid) {
    if (emitter.pri_s < emitter.pulse_width_s) {
      issues->push_back(
          MakeIssue(ValidationSeverity::kError, ValidationCode::kEmitterPriLessThanPulseWidth,
                    ValidationLocationKind::kSceneEntity, emitter_index, "pri_s/pulse_width_s",
                    "emitter pri must be greater than or equal to pulse width"));
    }
  }
  if (emitter.beam_state.az_beamwidth_deg <= 0.0 || emitter.beam_state.el_beamwidth_deg <= 0.0) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidEmitterBeamwidth,
        ValidationLocationKind::kSceneEntity, emitter_index,
        "beam_state.az_beamwidth_deg/el_beamwidth_deg", "emitter beam width must be positive"));
  }
  if (emitter.has_ecef_kinematics &&
      (!IsFinite(emitter.position_ecef_m.x_m) || !IsFinite(emitter.position_ecef_m.y_m) ||
       !IsFinite(emitter.position_ecef_m.z_m) || !IsFinite(emitter.velocity_ecef_mps.x_mps) ||
       !IsFinite(emitter.velocity_ecef_mps.y_mps) ||
       !IsFinite(emitter.velocity_ecef_mps.z_mps))) {
    issues->push_back(MakeIssue(ValidationSeverity::kError,
                                ValidationCode::kNonFiniteEmitterNumericField,
                                ValidationLocationKind::kSceneEntity, emitter_index,
                                "position_ecef_m/velocity_ecef_mps",
                                "enabled emitter ECEF kinematics must be finite"));
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
  if (input.has_rf_emission_frame) {
    const oneq::electromagnetics::RfEmissionFrame& frame = input.rf_emission_frame;
    const bool window_matches = frame.world_cycle_index == input.cycle_index &&
                                frame.window_start_time_s == input.cycle_start_time_s &&
                                frame.window_duration_s == static_cast<double>(input.dt_sec);
    if (!window_matches || !oneq::electromagnetics::TryValidateRfSceneFrame(frame)) {
      issues.push_back(MakeIssue(ValidationSeverity::kError,
                                 ValidationCode::kInvalidRfEmissionFrame,
                                 ValidationLocationKind::kGlobal,
                                 static_cast<std::size_t>(-1), "rf_emission_frame",
                                 "RF emission frame must be valid and match the cycle window"));
    }
  }
  ValidatePlatformPose(input.platform_pose, &issues);
  ValidatePlatformAltitude(input.platform_altitude_m, &issues);
  ValidatePlatformRfKinematics(input, &issues);
  ValidateEnvironmentObservation(input.environment, static_cast<double>(input.dt_sec), &issues);
  if (input.environment.interference_mode ==
          oneq::electromagnetics::RfInterferenceMode::kEngineering &&
      !input.has_platform_ecef_kinematics) {
    issues.push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidInterferenceInput,
        ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1),
        "has_platform_ecef_kinematics",
        "engineering interference requires explicit receiver ECEF kinematics"));
  }

  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    ValidateEmitter(input.scene[i], i, &issues);
  }

  return issues;
}

bool HasValidationError(const ValidationIssueList& issues) {
  return oneq::common::validation::HasSeverity<ValidationIssueList, ValidationSeverity,
                                               &ValidationIssue::severity>(
      issues, ValidationSeverity::kError);
}

}  // namespace session

}  // namespace electronic_surveillance_radar
