#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"

#include <cstddef>
#include <cstdint>
#include <string>

#include "common/validation/ValidationUtils.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

ValidationLocation MakeLocation(ValidationLocationKind kind, std::size_t entity_index) {
  ValidationLocation location;
  location.kind = kind;
  location.entity_index = entity_index;
  return location;
}

ValidationIssue MakeIssue(ValidationSeverity severity, ValidationCode code,
                          ValidationLocationKind location_kind, std::size_t entity_index,
                          const std::string& field, const std::string& message) {
  ValidationIssue issue;
  issue.severity = severity;
  issue.code = code;
  issue.location = MakeLocation(location_kind, entity_index);
  issue.field = field;
  issue.message = message;
  return issue;
}

/**
 * @brief 判断浮点输入是否有限。
 * @param[in] value 输入标量。
 * @return 当输入为有限数时返回 `true`。
 */
template <typename T>
bool IsFinite(T value) {
  return oneq::internal::validation::IsFinite(value);
}

bool IsRatioValid(float value) { return IsFinite(value) && value >= 0.0f && value <= 1.0f; }

/**
 * @brief 校验平台位姿输入字段。
 * @param[in] platform_pose 平台位姿输入。
 * @param[out] issues 校验问题列表。
 */
void ValidatePlatformPose(const session::EsrPoseState& platform_pose, ValidationIssueList* issues) {
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

void ValidateEnvironmentObservation(const environment::EsrEnvironmentObservation& observation,
                                    ValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }
  const environment::EsrAtmosphericObservation& atmosphere = observation.atmospheric_observation;
  if (!IsRatioValid(observation.spectrum_occupancy_ratio) ||
      !IsRatioValid(atmosphere.relative_humidity_ratio) ||
      !IsFinite(atmosphere.precipitation_rate_mmph) || !IsFinite(atmosphere.visibility_km) ||
      atmosphere.precipitation_rate_mmph < 0.0f || atmosphere.visibility_km <= 0.0f) {
    issues->push_back(MakeIssue(
        ValidationSeverity::kError, ValidationCode::kInvalidEnvironmentObservation,
        ValidationLocationKind::kEnvironment, static_cast<std::size_t>(-1), "environment",
        "environment observation must contain finite ratios in [0, 1] and positive visibility"));
  }
  for (std::size_t i = 0; i < observation.jammer_sources.size(); ++i) {
    const environment::EsrJammerSource& jammer = observation.jammer_sources[i];
    if (!IsFinite(jammer.center_hz) || !IsFinite(jammer.bandwidth_hz) ||
        !IsFinite(jammer.power_w) || !IsFinite(jammer.deception_risk) ||
        !IsFinite(jammer.confidence) || jammer.center_hz < 0.0 || jammer.bandwidth_hz < 0.0 ||
        jammer.power_w < 0.0f || !IsRatioValid(jammer.deception_risk) ||
        !IsRatioValid(jammer.confidence)) {
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

  if (emitter.emitter_id.empty()) {
    issues->push_back(MakeIssue(ValidationSeverity::kError, ValidationCode::kEmptyEmitterId,
                                ValidationLocationKind::kSceneEntity, emitter_index, "emitter_id",
                                "emitter id must not be empty"));
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
  ValidatePlatformPose(input.platform_pose, &issues);
  ValidateEnvironmentObservation(input.environment, &issues);

  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    ValidateEmitter(input.scene[i], i, &issues);
  }

  return issues;
}

bool HasValidationError(const ValidationIssueList& issues) {
  return oneq::internal::validation::HasSeverity<ValidationIssueList, ValidationSeverity,
                                                 &ValidationIssue::severity>(
      issues, ValidationSeverity::kError);
}

}  // namespace session

}  // namespace electronic_surveillance_radar
