// @file EsrSessionConfigBuilder.cpp
// @brief Implementation of EsrSessionConfigBuilder with profile translation.

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigValidation.h"
#include "common/validation/ValidationUtils.h"
#include "electronic_surveillance_radar/session/EsrConfigDomainValidation.h"

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

  if (!oneq::common::validation::IsFinite(config.mission.scan.scan_rate_hz) ||
      config.mission.scan.scan_rate_hz <= 0.0f) {
    push(ConfigValidationCode::kScanRateNotPositive, "mission.scan.scan_rate_hz",
         "Scan pattern cycle rate must be finite and positive.");
  }

  if (!session::config_validation::IsValidMissionEnums(config.mission)) {
    push(ConfigValidationCode::kMissionEnumInvalid, "mission enum fields",
         "Work mode, scan start position, and scan sequence must be known values.");
  }

  if (!oneq::common::validation::IsFinite(config.hardware.receiver_band_lower_hz) ||
      !oneq::common::validation::IsFinite(config.hardware.receiver_band_upper_hz) ||
      config.hardware.receiver_band_lower_hz <= 0.0 ||
      config.hardware.receiver_band_lower_hz >= config.hardware.receiver_band_upper_hz) {
    push(ConfigValidationCode::kReceiverBandLowerAboveUpper,
         "hardware.receiver_band_lower_hz / receiver_band_upper_hz",
         "Receiver band lower bound must be below upper bound.");
  }

  if (!oneq::common::validation::IsFinite(config.hardware.beam_az_width_deg) ||
      config.hardware.beam_az_width_deg <= 0.0f) {
    push(ConfigValidationCode::kBeamAzWidthNotPositive, "hardware.beam_az_width_deg",
         "Azimuth beamwidth must be positive.");
  }
  if (!oneq::common::validation::IsFinite(config.hardware.beam_el_width_deg) ||
      config.hardware.beam_el_width_deg <= 0.0f) {
    push(ConfigValidationCode::kBeamElWidthNotPositive, "hardware.beam_el_width_deg",
         "Elevation beamwidth must be positive.");
  }

  const config::EsrHardwareConfig& hardware = config.hardware;
  if (hardware.receiver_equipment_id == 0U ||
      !oneq::common::validation::IsFinite(hardware.receiver_noise_figure_db) ||
      hardware.receiver_noise_figure_db < 0.0f ||
      !oneq::common::validation::IsFinite(hardware.receiver_reference_temperature_k) ||
      hardware.receiver_reference_temperature_k <= 0.0f ||
      !oneq::common::validation::IsFinite(hardware.antenna_peak_gain_dbi) ||
      !oneq::common::validation::IsFinite(hardware.antenna_sidelobe_level_db) ||
      !oneq::common::validation::IsFinite(hardware.antenna_backlobe_level_db) ||
      !oneq::common::validation::IsFinite(hardware.cross_polarization_isolation_db) ||
      hardware.cross_polarization_isolation_db < 0.0f ||
      !oneq::common::validation::IsFinite(hardware.minimum_far_field_range_m) ||
      hardware.minimum_far_field_range_m <= 0.0f ||
      !oneq::common::validation::IsFinite(hardware.maximum_linear_input_power_w) ||
      hardware.maximum_linear_input_power_w <= 0.0f ||
      static_cast<int>(hardware.polarization) <
          static_cast<int>(oneq::electromagnetics::RfScenePolarization::kHorizontal) ||
      static_cast<int>(hardware.polarization) >
          static_cast<int>(oneq::electromagnetics::RfScenePolarization::kUnpolarized)) {
    push(ConfigValidationCode::kReceiverRfHardwareInvalid, "hardware RF receiver fields",
         "Receiver RF hardware parameters must be finite and physically valid.");
  }
  for (const config::EsrCoSiteIsolationPath& path : hardware.co_site_paths) {
    if (path.transmitter_equipment_id == 0U ||
        path.transmitter_equipment_id == hardware.receiver_equipment_id ||
        !oneq::common::validation::IsFinite(path.isolation_db) || path.isolation_db < 0.0) {
      push(ConfigValidationCode::kReceiverRfHardwareInvalid, "hardware.co_site_paths",
           "Co-site paths require a distinct non-zero transmitter equipment ID and non-negative isolation.");
      break;
    }
  }
  for (std::size_t i = 0; i < hardware.tuning_plan.size(); ++i) {
    const EsrTuningWindow& window = hardware.tuning_plan[i];
    const double lower_hz = window.center_frequency_hz - 0.5 * window.bandwidth_hz;
    const double upper_hz = window.center_frequency_hz + 0.5 * window.bandwidth_hz;
    if (!oneq::common::validation::IsFinite(window.center_frequency_hz) ||
        !oneq::common::validation::IsFinite(window.bandwidth_hz) ||
        window.center_frequency_hz <= 0.0 || window.bandwidth_hz <= 0.0 ||
        window.dwell_cycles == 0U || lower_hz < hardware.receiver_band_lower_hz ||
        upper_hz > hardware.receiver_band_upper_hz) {
      push(ConfigValidationCode::kTuningPlanInvalid, "hardware.tuning_plan",
           "Tuning windows must be finite, non-empty, and inside the hardware band.");
      break;
    }
  }

  if (config.mission.scan.use_explicit_scan_bounds) {
    const bool bounds_finite =
        oneq::common::validation::IsFinite(config.mission.scan.scan_start_az_deg) &&
        oneq::common::validation::IsFinite(config.mission.scan.scan_end_az_deg) &&
        oneq::common::validation::IsFinite(config.mission.scan.scan_start_el_deg) &&
        oneq::common::validation::IsFinite(config.mission.scan.scan_end_el_deg);
    if (!bounds_finite) {
      push(ConfigValidationCode::kExplicitScanBoundsNotFinite, "mission.scan explicit bounds",
           "Explicit scan bounds must all be finite.");
    } else if (config.mission.scan.scan_start_az_deg >= config.mission.scan.scan_end_az_deg) {
      push(ConfigValidationCode::kExplicitScanBoundsAzSwapped,
           "mission.scan.scan_start_az_deg / scan_end_az_deg",
           "Scan start azimuth must be less than end azimuth.");
    }
    if (bounds_finite &&
        config.mission.scan.scan_start_el_deg >= config.mission.scan.scan_end_el_deg) {
      push(ConfigValidationCode::kExplicitScanBoundsElSwapped,
           "mission.scan.scan_start_el_deg / scan_end_el_deg",
           "Scan start elevation must be less than end elevation.");
    }
  } else if (!oneq::common::validation::IsFinite(
                 config.mission.scan.scan_center_az_deg) ||
             !oneq::common::validation::IsFinite(
                 config.mission.scan.scan_center_el_deg)) {
    push(ConfigValidationCode::kScanCenterNotFinite,
         "mission.scan center fields",
         "Center-driven scan angles must be finite.");
  }

  if (!session::config_validation::IsValidDetectionPolicy(
          config.policy.detection)) {
    push(ConfigValidationCode::kDetectionPolicyInvalid, "policy.detection",
         "Detection policy requires finite SNR, Pfa in (0,1), positive pulse count, and positive threshold scale.");
  }

  if (!session::config_validation::IsValidEnvironment(
          config.environment.scenario_config)) {
    push(ConfigValidationCode::kEnvironmentInvalid, "environment.scenario_config",
         "Environment preset and enabled atmospheric physics values must be valid.");
  }

  return issues;
}

}  // namespace config
}  // namespace electronic_surveillance_radar
