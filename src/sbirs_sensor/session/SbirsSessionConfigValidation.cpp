#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"

#include <cmath>

namespace sbirs_sensor {
namespace config {
namespace {

void AddError(const char* message, ValidationIssueList* issues) {
  ValidationIssue issue;
  issue.severity = oneq::foundation::ValidationSeverity::kError;
  issue.message = message;
  issues->push_back(issue);
}

}  // namespace

ValidationIssueList ValidateSbirsSessionConfig(const SbirsSessionConfig& config) {
  ValidationIssueList issues;
  if (config.hardware.wavelength_lower_um <= 0.0f ||
      config.hardware.wavelength_upper_um <= config.hardware.wavelength_lower_um) {
    AddError("hardware wavelength band must be positive and ordered", &issues);
  }
  if (config.hardware.optical_aperture_m <= 0.0f) {
    AddError("hardware optical aperture must be positive", &issues);
  }
  if (config.mission.wide_field_fov_az_deg <= 0.0f ||
      config.mission.wide_field_fov_el_deg <= 0.0f ||
      config.mission.narrow_field_fov_az_deg <= 0.0f ||
      config.mission.narrow_field_fov_el_deg <= 0.0f) {
    AddError("mission FOV values must be positive", &issues);
  }
  if (config.mission.max_range_m <= config.mission.min_range_m ||
      config.mission.min_range_m < 0.0f) {
    AddError("mission range gate must be ordered and non-negative", &issues);
  }
  if (config.mission.frame_rate_hz <= 0.0f) {
    AddError("mission frame rate must be positive", &issues);
  }
  if (config.mission.scan_rate_deg_per_sec < 0.0f) {
    AddError("mission scan rate must be non-negative", &issues);
  }
  if (!std::isfinite(config.mission.narrow_pointing_max_slew_rate_deg_per_sec) ||
      config.mission.narrow_pointing_max_slew_rate_deg_per_sec <= 0.0f) {
    AddError("mission narrow pointing max slew rate must be positive and finite", &issues);
  }
  if (!std::isfinite(config.mission.narrow_pointing_settle_tolerance_deg) ||
      config.mission.narrow_pointing_settle_tolerance_deg < 0.0f) {
    AddError("mission narrow pointing settle tolerance must be non-negative and finite", &issues);
  }
  if (config.policy.detection.wide_min_snr_linear < 0.0f ||
      config.policy.detection.narrow_min_snr_linear < 0.0f) {
    AddError("detection thresholds must be non-negative", &issues);
  }
  const SbirsErrorModelConfig& error_model = config.policy.error_model;
  if (!std::isfinite(error_model.angular_sigma_deg) || error_model.angular_sigma_deg < 0.0f ||
      !std::isfinite(error_model.orbit_sigma_deg) || error_model.orbit_sigma_deg < 0.0f ||
      !std::isfinite(error_model.attitude_sigma_deg) || error_model.attitude_sigma_deg < 0.0f ||
      !std::isfinite(error_model.fov_sigma_deg) || error_model.fov_sigma_deg < 0.0f ||
      !std::isfinite(error_model.range_fraction_sigma) ||
      error_model.range_fraction_sigma < 0.0f) {
    AddError("error model sigma values must be non-negative and finite", &issues);
  }
  if (!std::isfinite(error_model.detector_bandwidth_hz) ||
      error_model.detector_bandwidth_hz <= 0.0f) {
    AddError("error model detector bandwidth must be positive and finite", &issues);
  }
  const SbirsPointingDisturbanceConfig& disturbance = config.policy.pointing_disturbance;
  if (!std::isfinite(disturbance.common_attitude_sigma_deg) ||
      disturbance.common_attitude_sigma_deg < 0.0f ||
      !std::isfinite(disturbance.channel_pointing_sigma_deg) ||
      disturbance.channel_pointing_sigma_deg < 0.0f ||
      !std::isfinite(disturbance.channel_vibration_amplitude_deg) ||
      disturbance.channel_vibration_amplitude_deg < 0.0f ||
      !std::isfinite(disturbance.channel_vibration_frequency_hz) ||
      disturbance.channel_vibration_frequency_hz < 0.0f) {
    AddError("pointing disturbance amplitudes and frequency must be non-negative and finite",
             &issues);
  }
  if (!std::isfinite(disturbance.common_attitude_correlation_time_s) ||
      disturbance.common_attitude_correlation_time_s <= 0.0f ||
      !std::isfinite(disturbance.channel_pointing_correlation_time_s) ||
      disturbance.channel_pointing_correlation_time_s <= 0.0f) {
    AddError("pointing disturbance correlation times must be positive and finite", &issues);
  }
  if (disturbance.channel_vibration_amplitude_deg > 0.0f &&
      disturbance.channel_vibration_frequency_hz <= 0.0f) {
    AddError("pointing disturbance vibration frequency must be positive when amplitude is non-zero",
             &issues);
  }
  if (config.policy.scheduler.max_concurrent_nfov_locks < 1) {
    AddError("scheduler max_concurrent_nfov_locks must be at least 1", &issues);
  }
  if (config.policy.tracking.nfov_tracking_gate_loss_cycles < 1U) {
    AddError("tracking nfov_tracking_gate_loss_cycles must be at least 1", &issues);
  }
  return issues;
}

}  // namespace config
}  // namespace sbirs_sensor
