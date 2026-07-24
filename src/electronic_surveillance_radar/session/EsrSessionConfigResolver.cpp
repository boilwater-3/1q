#include "electronic_surveillance_radar/session/EsrSessionConfigResolver.h"
#include "electronic_surveillance_radar/session/EsrResolutionRules.h"

namespace electronic_surveillance_radar {
namespace session {

using resolution_rules::ApplyScanPolicy;
using resolution_rules::ApplyWorkModeAdjustment;

EsrInternalExecutionConfig MapSessionToInternal(const config::EsrSessionConfig& session_config) {
  EsrInternalExecutionConfig exec;
  const config::EsrHardwareConfig& hardware = session_config.hardware;
  const config::EsrMissionConfig& mission = session_config.mission;
  const config::EsrScanPolicyConfig& scan_policy = session_config.mission.scan;

  // Hardware/Mission: using aliases, direct assignment
  exec.hardware = hardware;
  exec.mission = mission;

  // Detection: direct policy assignment (no profile abstraction)
  exec.base_detection = session_config.policy.detection;
  exec.detection = exec.base_detection;

  // Work mode adjustment
  ApplyWorkModeAdjustment(mission.work_mode, &exec.detection);

  // Scan resolution
  ApplyScanPolicy(hardware, scan_policy, &exec.resolved_scan);

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

  // Runtime sub-configs
  exec.runtime.integrator.integration_mode = extension::InterceptIntegrationMode::kNonCoherent;
  exec.runtime.track.gate_distance = 1.2f;
  exec.runtime.track.confirm_hits = 3U;
  exec.runtime.track.max_missed_cycles = 5U;
  exec.runtime.track.confidence_alpha = 0.3f;
  exec.runtime.track.output_tentative = true;

  // Environment: scenario → model config explicit mapping
  exec.environment = session_config.environment.scenario_config;

  return exec;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
