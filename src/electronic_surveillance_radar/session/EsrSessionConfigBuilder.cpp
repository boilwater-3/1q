// @file EsrSessionConfigBuilder.cpp
// @brief Implementation of EsrSessionConfigBuilder with profile translation.

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"

#include "common/logging/ProjectLog.h"

namespace electronic_surveillance_radar {
namespace config {

namespace {

void ApplyEsrMissionSemanticConfig(EsrMissionProfile profile,
                                   EsrMissionConfig* mission) {
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
    ApplyEsrSensitivitySemanticConfig(sensitivity_profile_,
                                      &result.policy.detection);
  }

  return result;
}

ValidationIssueList EsrSessionConfigBuilder::Validate() const {
  ValidationIssueList issues;

  if (config_.mission.scan.scan_rate_hz <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kScanRateNotPositive;
    issue.field = "mission.scan.scan_rate_hz";
    issue.message = "Scan rate must be positive.";
    issues.push_back(issue);
  }

  if (config_.hardware.receiver_band_lower_hz >= config_.hardware.receiver_band_upper_hz) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kReceiverBandLowerAboveUpper;
    issue.field = "hardware.receiver_band_lower_hz / receiver_band_upper_hz";
    issue.message = "Receiver band lower bound must be below upper bound.";
    issues.push_back(issue);
  }

  if (config_.hardware.beam_az_width_deg <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kBeamAzWidthNotPositive;
    issue.field = "hardware.beam_az_width_deg";
    issue.message = "Azimuth beamwidth must be positive.";
    issues.push_back(issue);
  }
  if (config_.hardware.beam_el_width_deg <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kBeamElWidthNotPositive;
    issue.field = "hardware.beam_el_width_deg";
    issue.message = "Elevation beamwidth must be positive.";
    issues.push_back(issue);
  }

  if (config_.mission.scan.use_explicit_scan_bounds) {
    if (config_.mission.scan.scan_start_az_deg >= config_.mission.scan.scan_end_az_deg) {
      ConfigValidationIssue issue;
      issue.code = ConfigValidationCode::kExplicitScanBoundsAzSwapped;
      issue.field = "mission.scan.scan_start_az_deg / scan_end_az_deg";
      issue.message = "Scan start azimuth must be less than end azimuth.";
      issues.push_back(issue);
    }
    if (config_.mission.scan.scan_start_el_deg >= config_.mission.scan.scan_end_el_deg) {
      ConfigValidationIssue issue;
      issue.code = ConfigValidationCode::kExplicitScanBoundsElSwapped;
      issue.field = "mission.scan.scan_start_el_deg / scan_end_el_deg";
      issue.message = "Scan start elevation must be less than end elevation.";
      issues.push_back(issue);
    }
  }

  return issues;
}

}  // namespace config
}  // namespace electronic_surveillance_radar
