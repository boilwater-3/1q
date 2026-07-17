// @file EsrSessionConfigBuilder.cpp
// @brief Implementation of EsrSessionConfigBuilder with profile translation.

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigValidation.h"
#include "common/validation/ValidationUtils.h"

namespace electronic_surveillance_radar {
namespace config {

namespace {

void ApplyEsrMissionSemanticConfig(EsrMissionProfile profile, EsrMissionConfig* mission) {
  if (mission == nullptr) {
    return;
  }
  auto& m = *mission;

  switch (profile) {
    case EsrMissionProfile::kElectronicOrderOfBattle:
      m.work_mode = EsrWorkMode::kEsm;
      m.scan.scan_rate_hz = 2.0f;
      m.scan.use_explicit_scan_bounds = true;
      m.scan.scan_start_az_deg = -60.0f;
      m.scan.scan_end_az_deg = 60.0f;
      m.scan.scan_start_el_deg = -10.0f;
      m.scan.scan_end_el_deg = 10.0f;
      break;
    case EsrMissionProfile::kPrecisionEmitterAnalysis:
      m.work_mode = EsrWorkMode::kHgesm;
      m.scan.scan_rate_hz = 0.5f;
      m.scan.use_explicit_scan_bounds = true;
      m.scan.scan_start_az_deg = -30.0f;
      m.scan.scan_end_az_deg = 30.0f;
      m.scan.scan_start_el_deg = -5.0f;
      m.scan.scan_end_el_deg = 5.0f;
      break;
    case EsrMissionProfile::kThreatWarning:
      m.work_mode = EsrWorkMode::kRwr;
      m.scan.scan_rate_hz = 5.0f;
      m.scan.use_explicit_scan_bounds = true;
      m.scan.scan_start_az_deg = -60.0f;
      m.scan.scan_end_az_deg = 60.0f;
      m.scan.scan_start_el_deg = -10.0f;
      m.scan.scan_end_el_deg = 10.0f;
      break;
  }
}

void ApplyEsrSensitivitySemanticConfig(EsrSensitivityProfile profile,
                                       EsrDetectionPolicyConfig* detection) {
  if (detection == nullptr) {
    return;
  }
  auto& d = *detection;

  switch (profile) {
    case EsrSensitivityProfile::kStandard:
      d.minimum_snr_db = 6.0f;
      d.pulse_count = 8U;
      d.pfa = 1.0e-6f;
      d.threshold_scale = 1.0f;
      break;
    case EsrSensitivityProfile::kHighSensitivity:
      d.minimum_snr_db = 3.0f;
      d.pulse_count = 16U;
      d.pfa = 5.0e-6f;
      d.threshold_scale = 1.0f;
      break;
    case EsrSensitivityProfile::kRobust:
      d.minimum_snr_db = 10.0f;
      d.pulse_count = 4U;
      d.pfa = 1.0e-7f;
      d.threshold_scale = 1.0f;
      break;
  }
}

}  // namespace

config::EsrSessionConfig EsrSessionConfigBuilder::Build() const {
  config::EsrSessionConfig result = config_;

  if (mission_profile_dirty_) {
    ApplyEsrMissionSemanticConfig(mission_profile_, &result.mission);
  }
  if (sensitivity_profile_dirty_) {
    ApplyEsrSensitivitySemanticConfig(sensitivity_profile_, &result.policy.detection);
  }

  return result;
}

ValidationIssueList ValidateEsrSessionConfig(const config::EsrSessionConfig& config) noexcept {
  ValidationIssueList issues;
  const auto push = [&issues](ConfigValidationCode code, const char* field, const char* msg) {
    ConfigValidationIssue issue;
    issue.code = code;
    issue.field = field;
    issue.message = msg;
    issues.push_back(issue);
  };

  if (config.mission.scan.scan_rate_hz <= 0.0f) {
    push(ConfigValidationCode::kScanRateNotPositive, "mission.scan.scan_rate_hz",
         "Scan rate must be positive.");
  }

  if (config.hardware.receiver_band_lower_hz >= config.hardware.receiver_band_upper_hz) {
    push(ConfigValidationCode::kReceiverBandLowerAboveUpper,
         "hardware.receiver_band_lower_hz / receiver_band_upper_hz",
         "Receiver band lower bound must be below upper bound.");
  }

  if (config.hardware.beam_az_width_deg <= 0.0f) {
    push(ConfigValidationCode::kBeamAzWidthNotPositive, "hardware.beam_az_width_deg",
         "Azimuth beamwidth must be positive.");
  }
  if (config.hardware.beam_el_width_deg <= 0.0f) {
    push(ConfigValidationCode::kBeamElWidthNotPositive, "hardware.beam_el_width_deg",
         "Elevation beamwidth must be positive.");
  }

  if (config.mission.scan.use_explicit_scan_bounds) {
    const bool bounds_finite =
        oneq::common::validation::IsFinite(config.mission.scan.scan_start_az_deg) &&
        oneq::common::validation::IsFinite(config.mission.scan.scan_end_az_deg) &&
        oneq::common::validation::IsFinite(config.mission.scan.scan_start_el_deg) &&
        oneq::common::validation::IsFinite(config.mission.scan.scan_end_el_deg);
    if (!bounds_finite) {
      push(ConfigValidationCode::kExplicitScanBoundsNotFinite,
           "mission.scan explicit bounds",
           "Explicit scan bounds must all be finite.");
    } else if (config.mission.scan.scan_start_az_deg >=
               config.mission.scan.scan_end_az_deg) {
      push(ConfigValidationCode::kExplicitScanBoundsAzSwapped,
           "mission.scan.scan_start_az_deg / scan_end_az_deg",
           "Scan start azimuth must be less than end azimuth.");
    }
    if (bounds_finite && config.mission.scan.scan_start_el_deg >=
                             config.mission.scan.scan_end_el_deg) {
      push(ConfigValidationCode::kExplicitScanBoundsElSwapped,
           "mission.scan.scan_start_el_deg / scan_end_el_deg",
           "Scan start elevation must be less than end elevation.");
    }
  }

  return issues;
}

}  // namespace config
}  // namespace electronic_surveillance_radar
