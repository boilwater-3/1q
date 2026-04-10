#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"

#include <cmath>

#include "common/logging/ProjectLog.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {
namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }

EosRuntimeConfigResolveResult RejectPatch(const EosSessionConfig& current_config,
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

EosRuntimeConfigResolveResult ResolveEosRuntimeConfigPatch(const EosSessionConfig& current_config,
                                                           const EosRuntimeConfigPatch& patch) {
  EosRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  bool has_requested_update = false;

  if (patch.has_work_mode) {
    resolved.next_config.work_mode = patch.work_mode;
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
    resolved.next_config.scan_rate_deg_per_sec = patch.scan_rate_deg_per_sec;
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
    resolved.next_config.frame_rate_hz = patch.frame_rate_hz;
  }
  if (patch.has_minimum_snr_db) {
    resolved.next_config.minimum_snr_db = patch.minimum_snr_db;
    has_requested_update = true;
  }
  if (patch.has_enable_straylight_filter) {
    resolved.next_config.enable_straylight_filter = patch.enable_straylight_filter;
    has_requested_update = true;
  }
  if (patch.has_visible_reference_irradiance_w_m2) {
    has_requested_update = true;
    if (!IsFinite(patch.visible_reference_irradiance_w_m2) ||
        patch.visible_reference_irradiance_w_m2 <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting invalid visible_reference_irradiance_w_m2={}; "
          "must be finite and positive.",
          patch.visible_reference_irradiance_w_m2);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.visible_reference_irradiance_w_m2 =
        patch.visible_reference_irradiance_w_m2;
  }
  if (patch.has_environment_runtime_config) {
    has_requested_update = true;
    const environment::EosEnvironmentRuntimeConfigPatch& environment_patch =
        patch.environment_runtime_config;
    if (environment_patch.has_model_type) {
      resolved.next_config.environment_default_config.model_type = environment_patch.model_type;
    }
    if (environment_patch.has_radiative_transfer_model) {
      resolved.next_config.environment_default_config.radiative_transfer_model =
          environment_patch.radiative_transfer_model;
    }
    if (environment_patch.has_aerosol_density_factor) {
      if (!IsFinite(environment_patch.aerosol_density_factor) ||
          environment_patch.aerosol_density_factor < 1.0f) {
        PROJECT_LOG_ERROR(
            "[EosSession] Rejecting invalid aerosol_density_factor={}; must be finite and >= 1.",
            environment_patch.aerosol_density_factor);
        return RejectPatch(current_config, true);
      }
      resolved.next_config.environment_default_config.aerosol_density_factor =
          environment_patch.aerosol_density_factor;
    }
    if (environment_patch.has_turbulence_factor) {
      if (!IsFinite(environment_patch.turbulence_factor) ||
          environment_patch.turbulence_factor < 1.0f) {
        PROJECT_LOG_ERROR(
            "[EosSession] Rejecting invalid turbulence_factor={}; must be finite and >= 1.",
            environment_patch.turbulence_factor);
        return RejectPatch(current_config, true);
      }
      resolved.next_config.environment_default_config.turbulence_factor =
          environment_patch.turbulence_factor;
    }
  }

  resolved.has_requested_update = has_requested_update;
  return resolved;
}

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
