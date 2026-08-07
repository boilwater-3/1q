// @file EsrSessionConfigBuilder.cpp
// @brief Implementation of EsrSessionConfigBuilder (thin wrapper).

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigValidation.h"
#include "common/validation/ValidationUtils.h"
#include "electronic_surveillance_radar/session/EsrConfigDomainValidation.h"

namespace electronic_surveillance_radar {
namespace config {

config::EsrSessionConfig EsrSessionConfigBuilder::Build() const { return config_; }

session::EsrIssueList ValidateEsrSessionConfig(const config::EsrSessionConfig& config) noexcept {
  session::EsrIssueList issues;
  // 统一问题列表模型（规则 14）：config 域问题 severity 固定 kError、phase 固定
  // kInputValidation，code 为 "esr.validation.<snake_case>"，location 保持默认（kGlobal）。
  const auto push = [&issues](const char* code_snake, const char* field, const char* msg) {
    session::EsrIssue issue;
    issue.severity = session::EsrIssueSeverity::kError;
    issue.phase = session::EsrIssuePhase::kInputValidation;
    issue.code = std::string("esr.validation.") + code_snake;
    issue.field = field;
    issue.message = msg;
    issues.push_back(issue);
  };

  if (!oneq::common::validation::IsFinite(config.mission.scan.scan_rate_hz) ||
      config.mission.scan.scan_rate_hz <= 0.0f) {
    push("scan_rate_not_positive", "mission.scan.scan_rate_hz",
         "Scan pattern cycle rate must be finite and positive.");
  }

  if (!session::config_validation::IsValidMissionEnums(config.mission)) {
    push("mission_enum_invalid", "mission enum fields",
         "Work mode, scan start position, and scan sequence must be known values.");
  }

  if (!oneq::common::validation::IsFinite(config.hardware.receiver_band_lower_hz) ||
      !oneq::common::validation::IsFinite(config.hardware.receiver_band_upper_hz) ||
      config.hardware.receiver_band_lower_hz <= 0.0 ||
      config.hardware.receiver_band_lower_hz >= config.hardware.receiver_band_upper_hz) {
    push("receiver_band_lower_above_upper",
         "hardware.receiver_band_lower_hz / receiver_band_upper_hz",
         "Receiver band lower bound must be below upper bound.");
  }

  if (!oneq::common::validation::IsFinite(config.hardware.beam_az_width_deg) ||
      config.hardware.beam_az_width_deg <= 0.0f) {
    push("beam_az_width_not_positive", "hardware.beam_az_width_deg",
         "Azimuth beamwidth must be positive.");
  }
  if (!oneq::common::validation::IsFinite(config.hardware.beam_el_width_deg) ||
      config.hardware.beam_el_width_deg <= 0.0f) {
    push("beam_el_width_not_positive", "hardware.beam_el_width_deg",
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
    push("receiver_rf_hardware_invalid", "hardware RF receiver fields",
         "Receiver RF hardware parameters must be finite and physically valid.");
  }
  for (const config::EsrCoSiteIsolationPath& path : hardware.co_site_paths) {
    if (path.transmitter_equipment_id == 0U ||
        path.transmitter_equipment_id == hardware.receiver_equipment_id ||
        !oneq::common::validation::IsFinite(path.isolation_db) || path.isolation_db < 0.0) {
      push("receiver_rf_hardware_invalid", "hardware.co_site_paths",
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
      push("tuning_plan_invalid", "hardware.tuning_plan",
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
      push("explicit_scan_bounds_not_finite", "mission.scan explicit bounds",
           "Explicit scan bounds must all be finite.");
    } else if (config.mission.scan.scan_start_az_deg >= config.mission.scan.scan_end_az_deg) {
      push("explicit_scan_bounds_az_swapped",
           "mission.scan.scan_start_az_deg / scan_end_az_deg",
           "Scan start azimuth must be less than end azimuth.");
    }
    if (bounds_finite &&
        config.mission.scan.scan_start_el_deg >= config.mission.scan.scan_end_el_deg) {
      push("explicit_scan_bounds_el_swapped",
           "mission.scan.scan_start_el_deg / scan_end_el_deg",
           "Scan start elevation must be less than end elevation.");
    }
  } else if (!oneq::common::validation::IsFinite(
                 config.mission.scan.scan_center_az_deg) ||
             !oneq::common::validation::IsFinite(
                 config.mission.scan.scan_center_el_deg)) {
    push("scan_center_not_finite",
         "mission.scan center fields",
         "Center-driven scan angles must be finite.");
  }

  if (!session::config_validation::IsValidDetectionPolicy(
          config.policy.detection)) {
    push("detection_policy_invalid", "policy.detection",
         "Detection policy requires finite SNR, Pfa in (0,1), positive pulse count, and positive threshold scale.");
  }

  if (!session::config_validation::IsValidEnvironment(
          config.environment.scenario_config)) {
    push("environment_invalid", "environment.scenario_config",
         "Environment preset and enabled atmospheric physics values must be valid.");
  }

  return issues;
}

}  // namespace config
}  // namespace electronic_surveillance_radar
