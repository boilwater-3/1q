#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"

#include <cmath>
#include <string>

namespace sbirs_sensor {
namespace config {
namespace {

// 统一问题列表模型（规则 14）：配置校验问题 code 为 "sbirs.validation.<snake_case>"，
// severity 固定为 kError，phase 固定为 kInputValidation；location/field 保持默认（无定位）。
void AddError(const char* code, const char* message, session::SbirsIssueList* issues) {
  session::SbirsIssue issue;
  issue.severity = session::SbirsIssueSeverity::kError;
  issue.phase = session::SbirsIssuePhase::kInputValidation;
  issue.code = std::string("sbirs.validation.") + code;
  issue.message = message;
  issues->push_back(issue);
}

}  // namespace

session::SbirsIssueList ValidateSbirsSessionConfig(const SbirsSessionConfig& config) {
  session::SbirsIssueList issues;
  if (config.hardware.wavelength_lower_um <= 0.0f ||
      config.hardware.wavelength_upper_um <= config.hardware.wavelength_lower_um) {
    AddError("wavelength_band_invalid", "hardware wavelength band must be positive and ordered",
             &issues);
  }
  if (config.hardware.optical_aperture_m <= 0.0f) {
    AddError("optical_aperture_not_positive", "hardware optical aperture must be positive", &issues);
  }
  if (config.mission.wide_field_fov_az_deg <= 0.0f ||
      config.mission.wide_field_fov_el_deg <= 0.0f ||
      config.mission.narrow_field_fov_az_deg <= 0.0f ||
      config.mission.narrow_field_fov_el_deg <= 0.0f) {
    AddError("mission_fov_not_positive", "mission FOV values must be positive", &issues);
  }
  if (!std::isfinite(config.mission.scan_start_az_deg) ||
      config.mission.scan_start_az_deg < -180.0f || config.mission.scan_start_az_deg >= 180.0f) {
    AddError("invalid_scan_start_azimuth",
             "mission scan start azimuth must be finite and in [-180, 180)", &issues);
  }
  if (!std::isfinite(config.mission.scan_span_deg) || config.mission.scan_span_deg <= 0.0f ||
      config.mission.scan_span_deg > 360.0f) {
    AddError("invalid_scan_span", "mission scan span must be finite and in (0, 360]", &issues);
  }
  if (config.mission.scan_direction != SbirsScanDirection::kIncreasingAzimuth &&
      config.mission.scan_direction != SbirsScanDirection::kDecreasingAzimuth) {
    AddError("invalid_scan_direction", "mission scan direction is invalid", &issues);
  }
  if (config.mission.max_range_m <= config.mission.min_range_m ||
      config.mission.min_range_m < 0.0f) {
    AddError("invalid_range_gate", "mission range gate must be ordered and non-negative", &issues);
  }
  if (config.mission.frame_rate_hz <= 0.0f) {
    AddError("frame_rate_not_positive", "mission frame rate must be positive", &issues);
  }
  if (!std::isfinite(config.mission.scan_rate_deg_per_sec) ||
      config.mission.scan_rate_deg_per_sec < 0.0f) {
    AddError("invalid_scan_rate", "mission scan rate must be non-negative and finite", &issues);
  }
  if (!std::isfinite(config.mission.narrow_pointing_max_slew_rate_deg_per_sec) ||
      config.mission.narrow_pointing_max_slew_rate_deg_per_sec <= 0.0f) {
    AddError("invalid_narrow_pointing_slew_rate",
             "mission narrow pointing max slew rate must be positive and finite", &issues);
  }
  if (!std::isfinite(config.mission.narrow_pointing_settle_tolerance_deg) ||
      config.mission.narrow_pointing_settle_tolerance_deg < 0.0f) {
    AddError("invalid_narrow_pointing_settle_tolerance",
             "mission narrow pointing settle tolerance must be non-negative and finite", &issues);
  }
  if (config.policy.detection.wide_min_snr_linear < 0.0f ||
      config.policy.detection.narrow_min_snr_linear < 0.0f) {
    AddError("invalid_detection_thresholds", "detection thresholds must be non-negative", &issues);
  }
  const SbirsErrorModelConfig& error_model = config.policy.error_model;
  if (!std::isfinite(error_model.orbit_sigma_deg) || error_model.orbit_sigma_deg < 0.0f ||
      !std::isfinite(error_model.attitude_sigma_deg) || error_model.attitude_sigma_deg < 0.0f ||
      !std::isfinite(error_model.fov_sigma_deg) || error_model.fov_sigma_deg < 0.0f ||
      !std::isfinite(error_model.range_fraction_sigma) ||
      error_model.range_fraction_sigma < 0.0f) {
    AddError("invalid_error_model_sigmas",
             "error model sigma values must be non-negative and finite", &issues);
  }
  if (!std::isfinite(error_model.detector_bandwidth_hz) ||
      error_model.detector_bandwidth_hz <= 0.0f) {
    AddError("invalid_detector_bandwidth",
             "error model detector bandwidth must be positive and finite", &issues);
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
    AddError("invalid_pointing_disturbance_values",
             "pointing disturbance amplitudes and frequency must be non-negative and finite",
             &issues);
  }
  if (!std::isfinite(disturbance.common_attitude_correlation_time_s) ||
      disturbance.common_attitude_correlation_time_s <= 0.0f ||
      !std::isfinite(disturbance.channel_pointing_correlation_time_s) ||
      disturbance.channel_pointing_correlation_time_s <= 0.0f) {
    AddError("invalid_pointing_disturbance_correlation",
             "pointing disturbance correlation times must be positive and finite", &issues);
  }
  if (disturbance.channel_vibration_amplitude_deg > 0.0f &&
      disturbance.channel_vibration_frequency_hz <= 0.0f) {
    AddError("invalid_pointing_disturbance_vibration_frequency",
             "pointing disturbance vibration frequency must be positive when amplitude is non-zero",
             &issues);
  }
  if (config.policy.scheduler.max_concurrent_nfov_locks < 1) {
    AddError("invalid_scheduler_nfov_locks",
             "scheduler max_concurrent_nfov_locks must be at least 1", &issues);
  }
  const SbirsTrackingConfig& tracking = config.policy.tracking;
  if (tracking.tracking_mode != SbirsTrackingMode::kEstimated &&
      tracking.tracking_mode != SbirsTrackingMode::kStrictTruthAssisted &&
      tracking.tracking_mode != SbirsTrackingMode::kSensorLikeTruthAssisted) {
    AddError("invalid_tracking_mode", "tracking mode is invalid", &issues);
  }
  if (tracking.estimated_backend != SbirsEstimatedTrackingBackend::kEkf &&
      tracking.estimated_backend != SbirsEstimatedTrackingBackend::kImm) {
    AddError("invalid_estimated_tracking_backend", "estimated tracking backend is invalid", &issues);
  }
  if (config.policy.tracking.nfov_tracking_gate_loss_cycles < 1U) {
    AddError("invalid_tracking_gate_loss_cycles",
             "tracking nfov_tracking_gate_loss_cycles must be at least 1", &issues);
  }
  return issues;
}

}  // namespace config
}  // namespace sbirs_sensor
