#include "electronic_surveillance_radar/session/EsrSessionConfigResolver.h"
#include "common/validation/ValidationUtils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace electronic_surveillance_radar {
namespace session {
namespace {

constexpr std::uint32_t kActiveScanPulseMultiplier = 4U;
constexpr std::uint32_t kMaxPulseCount = 4096U;
constexpr float kMinimumThresholdScale = 0.1f;
constexpr float kHgesmThresholdScale = 0.85f;
constexpr float kRwrThresholdScale = 1.25f;
void NormalizeScanBounds(float* start, float* end) {
  if (start == nullptr || end == nullptr) {
    return;
  }
  if (*start > *end) {
    std::swap(*start, *end);
  }
}

void ApplyScanPolicy(const config::EsrHardwareConfig& hardware,
                     const config::EsrScanPolicyConfig& scan_policy,
                     const config::EsrMissionConfig& mission,
                     extension::InterceptScanConfig* scan_config) {
  if (scan_config == nullptr) {
    return;
  }

  const float mount_az = oneq::internal::validation::IsFinite(hardware.antenna_mount_az_deg)
                             ? hardware.antenna_mount_az_deg
                             : 0.0f;
  const float mount_el = oneq::internal::validation::IsFinite(hardware.antenna_mount_el_deg)
                             ? hardware.antenna_mount_el_deg
                             : 0.0f;
  scan_config->scan_start_pos = static_cast<int>(scan_policy.scan_start_position);
  scan_config->scan_sequence = static_cast<int>(scan_policy.scan_sequence);

  if (oneq::internal::validation::IsFinite(hardware.beam_az_width_deg) && hardware.beam_az_width_deg > 0.0f) {
    scan_config->az_step_deg = hardware.beam_az_width_deg;
  }
  if (oneq::internal::validation::IsFinite(hardware.beam_el_width_deg) && hardware.beam_el_width_deg > 0.0f) {
    scan_config->el_step_deg = hardware.beam_el_width_deg;
  }

  const bool explicit_bounds_valid =
      scan_policy.use_explicit_scan_bounds && oneq::internal::validation::IsFinite(scan_policy.scan_start_az_deg) &&
      oneq::internal::validation::IsFinite(scan_policy.scan_end_az_deg) && oneq::internal::validation::IsFinite(scan_policy.scan_start_el_deg) &&
      oneq::internal::validation::IsFinite(scan_policy.scan_end_el_deg);
  if (explicit_bounds_valid) {
    float start_az = scan_policy.scan_start_az_deg - mount_az;
    float end_az = scan_policy.scan_end_az_deg - mount_az;
    float start_el = scan_policy.scan_start_el_deg - mount_el;
    float end_el = scan_policy.scan_end_el_deg - mount_el;
    NormalizeScanBounds(&start_az, &end_az);
    NormalizeScanBounds(&start_el, &end_el);
    scan_config->scan_start_az_deg = start_az;
    scan_config->scan_end_az_deg = end_az;
    scan_config->scan_start_el_deg = start_el;
    scan_config->scan_end_el_deg = end_el;
    return;
  }

  const bool has_center_az = oneq::internal::validation::IsFinite(scan_policy.scan_center_az_deg);
  const bool has_center_el = oneq::internal::validation::IsFinite(scan_policy.scan_center_el_deg);
  if (has_center_az) {
    float half_az_span =
        0.5f * std::fabs(scan_config->scan_end_az_deg - scan_config->scan_start_az_deg);
    if (oneq::internal::validation::IsFinite(hardware.az_scan_range_deg) && hardware.az_scan_range_deg > 0.0f) {
      half_az_span = 0.5f * hardware.az_scan_range_deg;
    }
    const float center_az = scan_policy.scan_center_az_deg - mount_az;
    scan_config->scan_start_az_deg = center_az - half_az_span;
    scan_config->scan_end_az_deg = center_az + half_az_span;
  }
  if (has_center_el) {
    float half_el_span =
        0.5f * std::fabs(scan_config->scan_end_el_deg - scan_config->scan_start_el_deg);
    if (oneq::internal::validation::IsFinite(hardware.el_scan_range_deg) && hardware.el_scan_range_deg > 0.0f) {
      half_el_span = 0.5f * hardware.el_scan_range_deg;
    }
    const float center_el = scan_policy.scan_center_el_deg - mount_el;
    scan_config->scan_start_el_deg = center_el - half_el_span;
    scan_config->scan_end_el_deg = center_el + half_el_span;
  }
  NormalizeScanBounds(&scan_config->scan_start_az_deg, &scan_config->scan_end_az_deg);
  NormalizeScanBounds(&scan_config->scan_start_el_deg, &scan_config->scan_end_el_deg);
}

void ApplyWorkModeAdjustment(config::EsrWorkMode mode,
                             DetectionConfig* detection_config) {
  if (detection_config == nullptr) {
    return;
  }
  detection_config->pulse_count = std::max<std::uint32_t>(1U, detection_config->pulse_count);
  detection_config->threshold_scale = oneq::internal::validation::IsFinite(detection_config->threshold_scale) && detection_config->threshold_scale > 0.0f
                                          ? detection_config->threshold_scale
                                          : 1.0f;
  switch (mode) {
    case config::EsrWorkMode::kHgesm:
      detection_config->pulse_count =
          std::min<std::uint32_t>(detection_config->pulse_count * kActiveScanPulseMultiplier, kMaxPulseCount);
      detection_config->threshold_scale =
          std::max(kMinimumThresholdScale, detection_config->threshold_scale * kHgesmThresholdScale);
      break;
    case config::EsrWorkMode::kRwr:
      detection_config->pulse_count = std::max<std::uint32_t>(1U, detection_config->pulse_count / 2U);
      detection_config->threshold_scale =
          std::max(kMinimumThresholdScale, detection_config->threshold_scale * kRwrThresholdScale);
      break;
    case config::EsrWorkMode::kEsm:
    default:
      break;
  }
}

}  // namespace

EsrInternalExecutionConfig MapSessionToInternal(const config::EsrSessionConfig& session_config) {
  EsrInternalExecutionConfig exec;
  const config::EsrHardwareConfig& hardware = session_config.hardware;
  const config::EsrMissionConfig& mission = session_config.mission;
  const config::EsrScanPolicyConfig& scan_policy = session_config.mission.scan;

  // Hardware/Mission: using aliases, direct assignment
  exec.hardware = hardware;
  exec.mission = mission;

  // Detection: direct policy assignment (no profile abstraction)
  exec.detection = session_config.policy.detection;

  // Work mode adjustment
  ApplyWorkModeAdjustment(mission.work_mode, &exec.detection);

  // Scan resolution
  ApplyScanPolicy(hardware, scan_policy, mission, &exec.resolved_scan);

  // Intercept sub-configs (direct defaults from internal types)
  exec.intercept.algorithm.random_seed = 20260323U;
  exec.intercept.algorithm.angle_error_coefficient = 0.51f;
  exec.intercept.preprocess.normalize_quality = true;
  exec.intercept.detection.max_detect_range_m = 450000.0f;
  exec.intercept.detection.boundary_resolution_m = 50.0f;
  exec.intercept.detection.boundary_max_iterations = 32;
  exec.intercept.detection.min_dynamic_range_margin_db = -3.0f;

  // Intercept extension configs (keep defaults for now)
  exec.intercept.cluster.radius = 1.0f;
  exec.intercept.cluster.min_points = 1U;
  exec.intercept.cluster.rf_scale_hz = 5.0e6f;
  exec.intercept.cluster.pw_scale_sec = 1.0e-6f;
  exec.intercept.cluster.az_scale_deg = 2.0f;
  exec.intercept.cluster.el_scale_deg = 2.0f;
  exec.intercept.cluster.snr_scale_db = 8.0f;

  exec.intercept.spectral_analysis.enable = true;
  exec.intercept.spectral_analysis.min_sequence_length = 4U;
  exec.intercept.spectral_analysis.fft_length = 16U;
  exec.intercept.spectral_analysis.broadband_occupancy_threshold = 0.45f;
  exec.intercept.spectral_analysis.agile_stability_threshold_hz = 1.5e6f;
  exec.intercept.spectral_analysis.agile_peak_sparsity_threshold = 0.45f;
  exec.intercept.spectral_analysis.occupancy_peak_floor_ratio = 0.20f;

  exec.intercept.suppression.suppression_noise_scale = 1.0f;
  exec.intercept.suppression.suppression_mark_threshold_w = 1.0e-12f;

  exec.intercept.deception.false_alarm_probability_scale = 1.0f;
  exec.intercept.deception.confusion_probability_scale = 0.6f;
  exec.intercept.deception.max_false_observations_per_emitter = 1U;
  exec.intercept.deception.aoa_confusion_std_deg = 4.0f;
  exec.intercept.deception.rf_confusion_ratio = 0.02f;
  exec.intercept.deception.pw_confusion_ratio = 0.35f;
  exec.intercept.deception.cluster_confidence_penalty_scale = 0.55f;

  // Runtime sub-configs
  exec.runtime.integrator.integration_mode = extension::InterceptIntegrationMode::kNonCoherent;
  exec.runtime.track.gate_distance = 1.2f;
  exec.runtime.track.confirm_hits = 3U;
  exec.runtime.track.max_missed_cycles = 5U;
  exec.runtime.track.confidence_alpha = 0.3f;
  exec.runtime.track.output_tentative = true;

  // Environment: scenario → model config explicit mapping
  exec.environment = environment::BuildModelConfigFromScenario(session_config.environment.scenario_config);

  return exec;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
