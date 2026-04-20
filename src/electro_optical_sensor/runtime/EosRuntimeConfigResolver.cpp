#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"

#include <cmath>

#include "common/logging/ProjectLog.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace internal {
namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsValidMission(const config::EosMissionConfig& mission) {
  return IsFinite(mission.scan_rate_deg_per_sec) && mission.scan_rate_deg_per_sec > 0.0f &&
         IsFinite(mission.frame_rate_hz) && mission.frame_rate_hz > 0.0f &&
         IsFinite(mission.horizontal_fov_deg) && mission.horizontal_fov_deg > 0.0f &&
         IsFinite(mission.vertical_fov_deg) && mission.vertical_fov_deg > 0.0f &&
         IsFinite(mission.scan_start_az_deg) && IsFinite(mission.scan_end_az_deg) &&
         IsFinite(mission.scan_center_el_deg) && IsFinite(mission.boresight_depression_deg);
}

bool IsValidDetectionPolicy(const config::EosDetectionPolicyConfig& detection) {
  if (detection.use_profile_defaults) {
    return true;
  }
  return IsFinite(detection.minimum_snr_db) &&
         IsFinite(detection.detection_sensitivity_w) &&
         detection.detection_sensitivity_w > 0.0f &&
         IsFinite(detection.visible_reference_irradiance_w_m2) &&
         detection.visible_reference_irradiance_w_m2 > 0.0f;
}

bool IsValidStrayLightPolicy(const config::EosStrayLightPolicyConfig& stray_light) {
  if (stray_light.use_profile_defaults) {
    return true;
  }
  if (!IsFinite(stray_light.hood_inner_half_angle_deg) ||
      !IsFinite(stray_light.hood_outer_half_angle_deg) ||
      !IsFinite(stray_light.hood_min_suppression_ratio) ||
      !IsFinite(stray_light.hood_max_suppression_ratio)) {
    return false;
  }
  if (stray_light.hood_inner_half_angle_deg <= 0.0f ||
      stray_light.hood_outer_half_angle_deg <= stray_light.hood_inner_half_angle_deg) {
    return false;
  }
  if (stray_light.hood_min_suppression_ratio < 0.0f ||
      stray_light.hood_min_suppression_ratio > 1.0f ||
      stray_light.hood_max_suppression_ratio < 0.0f ||
      stray_light.hood_max_suppression_ratio > 1.0f) {
    return false;
  }
  return stray_light.hood_max_suppression_ratio >= stray_light.hood_min_suppression_ratio;
}

bool IsValidPolicy(const config::EosPolicyConfig& policy) {
  return IsValidDetectionPolicy(policy.detection) &&
         IsValidStrayLightPolicy(policy.stray_light);
}

bool IsValidEnvironmentPatch(
    const environment::EosEnvironmentRuntimeConfigPatch& environment_patch) {
  if (environment_patch.has_aerosol_density_factor &&
      (!IsFinite(environment_patch.aerosol_density_factor) ||
       environment_patch.aerosol_density_factor <= 0.0f)) {
    return false;
  }
  if (environment_patch.has_turbulence_factor &&
      (!IsFinite(environment_patch.turbulence_factor) ||
       environment_patch.turbulence_factor <= 0.0f)) {
    return false;
  }
  return true;
}

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
  const bool has_requested_update =
      patch.has_mission || patch.has_policy || patch.has_environment ||
      patch.has_work_mode || patch.has_scan_rate_deg_per_sec || patch.has_frame_rate_hz;

  if (patch.has_mission) {
    if (!IsValidMission(patch.mission)) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting mission patch because mission values are invalid.");
      return RejectPatch(current_config, true);
    }
    resolved.next_config.mission = patch.mission;
    resolved.reset_scan_phase = true;
  }

  if (patch.has_policy) {
    if (!IsValidPolicy(patch.policy)) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting policy patch because policy values are invalid.");
      return RejectPatch(current_config, true);
    }
    resolved.next_config.policy = patch.policy;
  }

  if (patch.has_environment) {
    if (!IsValidEnvironmentPatch(patch.environment)) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting environment patch because environment values are invalid.");
      return RejectPatch(current_config, true);
    }

    environment::EosEnvironmentScenarioConfig& scenario_config =
        resolved.next_config.environment.scenario_config;
    const environment::EosEnvironmentRuntimeConfigPatch& environment_patch =
        patch.environment;

    if (environment_patch.has_model_type) {
      scenario_config.model_type = environment_patch.model_type;
    }

    const bool has_model_leaf_override =
        environment_patch.has_radiative_transfer_model ||
        environment_patch.has_aerosol_density_factor ||
        environment_patch.has_turbulence_factor ||
        environment_patch.has_enable_optical_countermeasure_extension;
    if (has_model_leaf_override) {
      scenario_config.has_custom_overrides = true;
      if (environment_patch.has_radiative_transfer_model) {
        scenario_config.custom_overrides.radiative_transfer_model =
            environment_patch.radiative_transfer_model;
      }
      if (environment_patch.has_aerosol_density_factor) {
        scenario_config.custom_overrides.aerosol_density_factor =
            environment_patch.aerosol_density_factor;
      }
      if (environment_patch.has_turbulence_factor) {
        scenario_config.custom_overrides.turbulence_factor =
            environment_patch.turbulence_factor;
      }
      if (environment_patch.has_enable_optical_countermeasure_extension) {
        scenario_config.custom_overrides.enable_optical_countermeasure_extension =
            environment_patch.enable_optical_countermeasure_extension;
      }
    }
  }

  if (patch.has_work_mode) {
    resolved.next_config.mission.work_mode = patch.work_mode;
  }

  if (patch.has_scan_rate_deg_per_sec) {
    if (!IsFinite(patch.scan_rate_deg_per_sec) || patch.scan_rate_deg_per_sec <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting invalid scan_rate_deg_per_sec={}; "
          "must be finite and positive.",
          patch.scan_rate_deg_per_sec);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.mission.scan_rate_deg_per_sec = patch.scan_rate_deg_per_sec;
    resolved.reset_scan_phase = true;
  }

  if (patch.has_frame_rate_hz) {
    if (!IsFinite(patch.frame_rate_hz) || patch.frame_rate_hz <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting invalid frame_rate_hz={}; "
          "must be finite and positive.",
          patch.frame_rate_hz);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.mission.frame_rate_hz = patch.frame_rate_hz;
  }

  resolved.has_requested_update = has_requested_update;
  return resolved;
}

}  // namespace internal
}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
