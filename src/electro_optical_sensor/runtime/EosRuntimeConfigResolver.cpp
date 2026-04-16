#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"

#include <cmath>

#include "common/logging/ProjectLog.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace internal {
namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }

EosRuntimeConfigResolveResult RejectPatch(
    const ::electro_optical_sensor::session::EosSessionConfig& current_config,
    bool has_requested_update) {
  EosRuntimeConfigResolveResult rejected;
  rejected.next_config = current_config;
  rejected.has_requested_update = has_requested_update;
  rejected.is_valid = false;
  PROJECT_LOG_ERROR(
      "[EosSession] Runtime config patch rejected due to invalid fields; no changes applied.");
  return rejected;
}

}  // namespace

EosRuntimeConfigResolveResult ResolveEosRuntimeConfigPatch(
    const ::electro_optical_sensor::session::EosSessionConfig& current_config,
    const ::electro_optical_sensor::session::EosRuntimeConfigPatch& patch) {
  EosRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  bool has_requested_update = false;

  if (patch.has_work_mode) {
    resolved.next_config.scan.work_mode = patch.work_mode;
    has_requested_update = true;
  }

  if (patch.has_scan_rate_deg_per_sec) {
    has_requested_update = true;
    if (!IsFinite(patch.scan_rate_deg_per_sec) || patch.scan_rate_deg_per_sec <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting invalid scan_rate_deg_per_sec={}; "
          "must be finite and positive.",
          patch.scan_rate_deg_per_sec);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.scan.scan_rate_deg_per_sec = patch.scan_rate_deg_per_sec;
    resolved.reset_scan_phase = true;
  }

  if (patch.has_frame_rate_hz) {
    has_requested_update = true;
    if (!IsFinite(patch.frame_rate_hz) || patch.frame_rate_hz <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting invalid frame_rate_hz={}; "
          "must be finite and positive.",
          patch.frame_rate_hz);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.scan.frame_rate_hz = patch.frame_rate_hz;
  }

  if (patch.has_detection_profile) {
    resolved.next_config.detection.profile = patch.detection_profile;
    has_requested_update = true;
  }

  if (patch.has_stray_light_profile) {
    resolved.next_config.stray_light.profile = patch.stray_light_profile;
    has_requested_update = true;
  }

  if (patch.has_environment_model_type) {
    resolved.next_config.environment.model_type = patch.environment_model_type;
    has_requested_update = true;
  }

  if (patch.has_environment_preset) {
    resolved.next_config.environment.preset = patch.environment_preset;
    has_requested_update = true;
  }

  resolved.has_requested_update = has_requested_update;
  return resolved;
}

}  // namespace internal
}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
