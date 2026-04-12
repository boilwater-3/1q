#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"

#include <algorithm>
#include <cmath>

#include "common/logging/ProjectLog.h"

namespace electronic_surveillance_radar {
namespace session {
namespace internal {
namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }
bool IsFinite(double value) { return std::isfinite(value) != 0; }

bool IsValidReceiverWindow(double lower_hz, double upper_hz) {
  return IsFinite(lower_hz) && IsFinite(upper_hz) && upper_hz > lower_hz;
}

EsrRuntimeConfigResolveResult RejectPatch(const ResolvedEsrSessionConfig& current_config,
                                          bool has_requested_update) {
  EsrRuntimeConfigResolveResult rejected;
  rejected.next_config = current_config;
  rejected.has_requested_update = has_requested_update;
  rejected.is_valid = false;
  return rejected;
}

}  // namespace

EsrRuntimeConfigResolveResult ResolveEsrRuntimeConfigPatch(
    const ResolvedEsrSessionConfig& current_config, const EsrRuntimeConfigPatch& patch) {
  EsrRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  bool has_requested_update = false;

  const bool has_fixed_receiver_window_patch = patch.has_fixed_receiver_window_hz;
  bool fixed_receiver_window_bounds_valid = false;
  if (has_fixed_receiver_window_patch) {
    has_requested_update = true;
    fixed_receiver_window_bounds_valid =
        IsValidReceiverWindow(patch.receiver_lower_hz, patch.receiver_upper_hz);
    if (!fixed_receiver_window_bounds_valid) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to invalid fixed receiver window "
          "bounds: lower_hz={} upper_hz={}.",
          patch.receiver_lower_hz, patch.receiver_upper_hz);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.runtime_config.receiver_lower_hz = patch.receiver_lower_hz;
    resolved.next_config.runtime_config.receiver_upper_hz = patch.receiver_upper_hz;
    resolved.runtime_config_changed = true;
  }

  if (patch.has_sensor_enabled) {
    resolved.next_config.runtime_config.sensor_enabled = patch.sensor_enabled;
    resolved.runtime_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_scan_rate_hz) {
    has_requested_update = true;
    if (!IsFinite(patch.scan_rate_hz) || patch.scan_rate_hz <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to invalid scan_rate_hz={}; "
          "must be finite and positive.",
          patch.scan_rate_hz);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.runtime_config.scan_rate_hz = patch.scan_rate_hz;
    resolved.runtime_config_changed = true;
  }
  if (patch.has_integrated_receive_loss_db) {
    has_requested_update = true;
    if (!IsFinite(patch.integrated_receive_loss_db)) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to non-finite "
          "integrated_receive_loss_db={}.",
          patch.integrated_receive_loss_db);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.runtime_config.integrated_receive_loss_db =
        std::max(0.0f, patch.integrated_receive_loss_db);
    resolved.runtime_config_changed = true;
  }

  const bool current_fixed_receiver_window_bounds_valid =
      IsValidReceiverWindow(resolved.next_config.runtime_config.receiver_lower_hz,
                            resolved.next_config.runtime_config.receiver_upper_hz);
  if (patch.has_use_fixed_receiver_window) {
    has_requested_update = true;
    if (patch.use_fixed_receiver_window && !current_fixed_receiver_window_bounds_valid) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch because fixed receiver window is enabled "
          "without valid bounds: lower_hz={} upper_hz={}.",
          resolved.next_config.runtime_config.receiver_lower_hz,
          resolved.next_config.runtime_config.receiver_upper_hz);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.runtime_config.use_fixed_receiver_window = patch.use_fixed_receiver_window;
    resolved.runtime_config_changed = true;
  } else if (has_fixed_receiver_window_patch) {
    resolved.next_config.runtime_config.use_fixed_receiver_window = true;
    resolved.runtime_config_changed = true;
  }

  if (patch.has_enable_statistical_detection) {
    resolved.next_config.pipeline_config.statistical_detection.enable_statistical_detection =
        patch.enable_statistical_detection;
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_enable_spectral_analysis) {
    resolved.next_config.pipeline_config.spectral_analysis.enable = patch.enable_spectral_analysis;
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_detection_min_snr_db) {
    has_requested_update = true;
    if (!IsFinite(patch.detection_min_snr_db)) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to non-finite detection_min_snr_db={}.",
          patch.detection_min_snr_db);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.pipeline_config.detection.min_detect_snr_db = patch.detection_min_snr_db;
    resolved.pipeline_config_changed = true;
  }

  if (patch.has_environment_runtime_config) {
    has_requested_update = true;
    const environment::EsrEnvironmentRuntimeConfigPatch& environment_patch =
        patch.environment_runtime_config;
    if (environment_patch.has_model_config) {
      resolved.next_config.environment_model_config = environment_patch.model_config;
      resolved.environment_model_config_changed = true;
    }
    if (environment_patch.has_jamming_detection_threshold_w) {
      if (!IsFinite(environment_patch.jamming_detection_threshold_w)) {
        PROJECT_LOG_ERROR(
            "[EsrSession] Rejecting runtime config patch due to non-finite "
            "jamming_detection_threshold_w={}.",
            environment_patch.jamming_detection_threshold_w);
        return RejectPatch(current_config, true);
      }
      resolved.next_config.environment_model_config.jamming_detection_threshold_w =
          std::max(0.0f, environment_patch.jamming_detection_threshold_w);
      resolved.environment_model_config_changed = true;
    }
  }

  if (patch.has_observation_jam_mark_threshold_w) {
    has_requested_update = true;
    if (!IsFinite(patch.observation_jam_mark_threshold_w)) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to non-finite "
          "observation_jam_mark_threshold_w={}.",
          patch.observation_jam_mark_threshold_w);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.pipeline_config.suppression_model.suppression_mark_threshold_w =
        std::max(0.0f, patch.observation_jam_mark_threshold_w);
    resolved.pipeline_config_changed = true;
  }

  resolved.has_requested_update = has_requested_update;
  return resolved;
}

}  // namespace internal
}  // namespace session

}  // namespace electronic_surveillance_radar
