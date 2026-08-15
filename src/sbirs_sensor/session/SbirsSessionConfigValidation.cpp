#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"

#include <cmath>
#include <string>

#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"

namespace sbirs_sensor {
namespace config {
namespace {

// 统一问题列表模型（规则 14）：配置校验问题 code 引用 SbirsIssueCodes.h 常量
// （"sbirs.validation.<snake_case>" 全集单一事实来源），severity 固定为 kError，
// phase 固定为 kInputValidation；location/field 保持默认（无定位）。
void AddError(const char* code, const char* message, session::SbirsIssueList* issues) {
  session::SbirsIssue issue;
  issue.severity = session::SbirsIssueSeverity::kError;
  issue.phase = session::SbirsIssuePhase::kInputValidation;
  issue.code = code;
  issue.message = message;
  issues->push_back(issue);
}

}  // namespace

session::SbirsIssueList ValidateSbirsSessionConfig(const SbirsSessionConfig& config) {
  session::SbirsIssueList issues;
  if (config.hardware.wavelength_lower_um <= 0.0f ||
      config.hardware.wavelength_upper_um <= config.hardware.wavelength_lower_um) {
    AddError(session::codes::kWavelengthBandInvalid,
             "hardware wavelength band must be positive and ordered", &issues);
  }
  if (config.hardware.optical_aperture_m <= 0.0f) {
    AddError(session::codes::kOpticalApertureNotPositive,
             "hardware optical aperture must be positive", &issues);
  }
  if (config.mission.wide_field_fov_az_deg <= 0.0f ||
      config.mission.wide_field_fov_el_deg <= 0.0f ||
      config.mission.narrow_field_fov_az_deg <= 0.0f ||
      config.mission.narrow_field_fov_el_deg <= 0.0f) {
    AddError(session::codes::kMissionFovNotPositive, "mission FOV values must be positive",
             &issues);
  }
  // ECI 方位角约定（2026-08 正式变更）：扫描起始方位为 ECI 极坐标 az，
  // 取值范围 [0, 360)。
  if (!std::isfinite(config.mission.scan_start_az_deg) ||
      config.mission.scan_start_az_deg < 0.0f || config.mission.scan_start_az_deg >= 360.0f) {
    AddError(session::codes::kInvalidScanStartAzimuth,
             "mission scan start azimuth must be finite and in [0, 360)", &issues);
  }
  if (!std::isfinite(config.mission.scan_span_deg) || config.mission.scan_span_deg <= 0.0f ||
      config.mission.scan_span_deg > 360.0f) {
    AddError(session::codes::kInvalidScanSpan,
             "mission scan span must be finite and in (0, 360]", &issues);
  }
  if (config.mission.scan_direction != SbirsScanDirection::kIncreasingAzimuth &&
      config.mission.scan_direction != SbirsScanDirection::kDecreasingAzimuth) {
    AddError(session::codes::kInvalidScanDirection, "mission scan direction is invalid", &issues);
  }
  if (config.mission.max_range_m <= config.mission.min_range_m ||
      config.mission.min_range_m < 0.0f) {
    AddError(session::codes::kInvalidRangeGate,
             "mission range gate must be ordered and non-negative", &issues);
  }
  if (config.mission.frame_rate_hz <= 0.0f) {
    AddError(session::codes::kFrameRateNotPositive, "mission frame rate must be positive", &issues);
  }
  if (!std::isfinite(config.mission.scan_rate_deg_per_sec) ||
      config.mission.scan_rate_deg_per_sec < 0.0f) {
    AddError(session::codes::kInvalidScanRate,
             "mission scan rate must be non-negative and finite", &issues);
  }
  if (!std::isfinite(config.mission.narrow_pointing_max_slew_rate_deg_per_sec) ||
      config.mission.narrow_pointing_max_slew_rate_deg_per_sec <= 0.0f) {
    AddError(session::codes::kInvalidNarrowPointingSlewRate,
             "mission narrow pointing max slew rate must be positive and finite", &issues);
  }
  if (!std::isfinite(config.mission.narrow_pointing_settle_tolerance_deg) ||
      config.mission.narrow_pointing_settle_tolerance_deg < 0.0f) {
    AddError(session::codes::kInvalidNarrowPointingSettleTolerance,
             "mission narrow pointing settle tolerance must be non-negative and finite", &issues);
  }
  if (config.policy.detection.wide_min_snr_linear < 0.0f ||
      config.policy.detection.narrow_min_snr_linear < 0.0f) {
    AddError(session::codes::kInvalidDetectionThresholds,
             "detection thresholds must be non-negative", &issues);
  }
  const SbirsErrorModelConfig& error_model = config.policy.error_model;
  if (!std::isfinite(error_model.orbit_sigma_deg) || error_model.orbit_sigma_deg < 0.0f ||
      !std::isfinite(error_model.attitude_sigma_deg) || error_model.attitude_sigma_deg < 0.0f ||
      !std::isfinite(error_model.fov_sigma_deg) || error_model.fov_sigma_deg < 0.0f ||
      !std::isfinite(error_model.range_fraction_sigma) ||
      error_model.range_fraction_sigma < 0.0f) {
    AddError(session::codes::kInvalidErrorModelSigmas,
             "error model sigma values must be non-negative and finite", &issues);
  }
  if (!std::isfinite(error_model.detector_bandwidth_hz) ||
      error_model.detector_bandwidth_hz <= 0.0f) {
    AddError(session::codes::kInvalidDetectorBandwidth,
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
    AddError(session::codes::kInvalidPointingDisturbanceValues,
             "pointing disturbance amplitudes and frequency must be non-negative and finite",
             &issues);
  }
  if (!std::isfinite(disturbance.common_attitude_correlation_time_s) ||
      disturbance.common_attitude_correlation_time_s <= 0.0f ||
      !std::isfinite(disturbance.channel_pointing_correlation_time_s) ||
      disturbance.channel_pointing_correlation_time_s <= 0.0f) {
    AddError(session::codes::kInvalidPointingDisturbanceCorrelation,
             "pointing disturbance correlation times must be positive and finite", &issues);
  }
  if (disturbance.channel_vibration_amplitude_deg > 0.0f &&
      disturbance.channel_vibration_frequency_hz <= 0.0f) {
    AddError(session::codes::kInvalidPointingDisturbanceVibrationFrequency,
             "pointing disturbance vibration frequency must be positive when amplitude is non-zero",
             &issues);
  }
  if (config.policy.scheduler.max_concurrent_nfov_locks < 1) {
    AddError(session::codes::kInvalidSchedulerNfovLocks,
             "scheduler max_concurrent_nfov_locks must be at least 1", &issues);
  }
  const SbirsTrackingConfig& tracking = config.policy.tracking;
  if (tracking.tracking_mode != SbirsTrackingMode::kEstimated &&
      tracking.tracking_mode != SbirsTrackingMode::kStrictTruthAssisted &&
      tracking.tracking_mode != SbirsTrackingMode::kSensorLikeTruthAssisted) {
    AddError(session::codes::kInvalidTrackingMode, "tracking mode is invalid", &issues);
  }
  if (tracking.estimated_backend != SbirsEstimatedTrackingBackend::kEkf &&
      tracking.estimated_backend != SbirsEstimatedTrackingBackend::kImm) {
    AddError(session::codes::kInvalidEstimatedTrackingBackend,
             "estimated tracking backend is invalid", &issues);
  }
  if (config.policy.tracking.nfov_tracking_gate_loss_cycles < 1U) {
    AddError(session::codes::kInvalidTrackingGateLossCycles,
             "tracking nfov_tracking_gate_loss_cycles must be at least 1", &issues);
  }
  return issues;
}

}  // namespace config
}  // namespace sbirs_sensor
