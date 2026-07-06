/**
 * @file EosRuntimeConfigResolver.cpp
 * @brief 实现 EOS 运行期补丁解析，直接操作内部执行配置。
 */

#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"

#include <cmath>

#include "common/logging/ProjectLog.h"
#include "common/validation/ValidationUtils.h"
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace {

bool IsValidMission(const config::EosMissionConfig& mission) {
  return oneq::common::validation::IsFinite(mission.scan_rate_deg_per_sec) &&
         mission.scan_rate_deg_per_sec > 0.0f &&
         oneq::common::validation::IsFinite(mission.frame_rate_hz) &&
         mission.frame_rate_hz > 0.0f &&
         oneq::common::validation::IsFinite(mission.horizontal_fov_deg) &&
         mission.horizontal_fov_deg > 0.0f &&
         oneq::common::validation::IsFinite(mission.vertical_fov_deg) &&
         mission.vertical_fov_deg > 0.0f &&
         oneq::common::validation::IsFinite(mission.scan_start_az_deg) &&
         oneq::common::validation::IsFinite(mission.scan_end_az_deg) &&
         oneq::common::validation::IsFinite(mission.scan_center_el_deg) &&
         oneq::common::validation::IsFinite(mission.boresight_depression_deg);
}

bool IsValidDetectionPolicy(const config::EosDetectionPolicyConfig& detection) {
  return oneq::common::validation::IsFinite(detection.minimum_snr_db) &&
         oneq::common::validation::IsFinite(detection.detection_sensitivity_w) &&
         detection.detection_sensitivity_w > 0.0f &&
         oneq::common::validation::IsFinite(detection.visible_reference_irradiance_w_m2) &&
         detection.visible_reference_irradiance_w_m2 > 0.0f;
}

bool IsValidStrayLightPolicy(const config::EosStrayLightPolicyConfig& stray_light) {
  if (!oneq::common::validation::IsFinite(stray_light.hood_inner_half_angle_deg) ||
      !oneq::common::validation::IsFinite(stray_light.hood_outer_half_angle_deg) ||
      !oneq::common::validation::IsFinite(stray_light.hood_min_suppression_ratio) ||
      !oneq::common::validation::IsFinite(stray_light.hood_max_suppression_ratio)) {
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
    const config::EosEnvironmentRuntimeConfigPatch& environment_patch) {
  if (environment_patch.has_scenario_config) {
    if (environment_patch.scenario_config.has_custom_overrides) {
      if (!oneq::common::validation::IsFinite(
              environment_patch.scenario_config.custom_overrides.aerosol_density_factor) ||
          environment_patch.scenario_config.custom_overrides.aerosol_density_factor <= 0.0f) {
        return false;
      }
      if (!oneq::common::validation::IsFinite(
              environment_patch.scenario_config.custom_overrides.turbulence_factor) ||
          environment_patch.scenario_config.custom_overrides.turbulence_factor <= 0.0f) {
        return false;
      }
    }
  }
  return true;
}

EosRuntimeConfigResolveResult RejectPatch(
    const config::execution::EosInternalExecutionConfig& current_config,
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
    const config::execution::EosInternalExecutionConfig& current_config,
    const ::electro_optical_sensor::config::EosRuntimeConfigPatch& patch) {
  EosRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  const bool has_requested_update =
      patch.has_mission || patch.has_policy || patch.has_environment ||
      patch.has_work_mode || patch.has_scan_rate_deg_per_sec || patch.has_frame_rate_hz ||
      patch.has_sensor_enabled;

  if (patch.has_mission) {
    if (!IsValidMission(patch.mission)) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting mission patch because mission values are invalid.");
      return RejectPatch(current_config, true);
    }
    resolved.next_config.scan = patch.mission;
    resolved.reset_scan_phase = true;
  }

  if (patch.has_policy) {
    if (!IsValidPolicy(patch.policy)) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting policy patch because policy values are invalid.");
      return RejectPatch(current_config, true);
    }
    resolved.next_config.detection = patch.policy.detection;
    resolved.next_config.stray_light = patch.policy.stray_light;
  }

  if (patch.has_environment) {
    if (!IsValidEnvironmentPatch(patch.environment)) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting environment patch because environment values are invalid.");
      return RejectPatch(current_config, true);
    }

    if (patch.environment.has_scenario_config) {
      const config::EosEnvironmentModelConfig model_config =
          config::BuildModelConfigFromScenario(patch.environment.scenario_config);
      ApplyEnvironmentModelToInternal(model_config, &resolved.next_config);
    }
  }

  if (patch.has_work_mode) {
    resolved.next_config.scan.work_mode = patch.work_mode;
  }

  if (patch.has_scan_rate_deg_per_sec) {
    if (!oneq::common::validation::IsFinite(patch.scan_rate_deg_per_sec) ||
        patch.scan_rate_deg_per_sec <= 0.0f) {
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
    if (!oneq::common::validation::IsFinite(patch.frame_rate_hz) ||
        patch.frame_rate_hz <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting invalid frame_rate_hz={}; "
          "must be finite and positive.",
          patch.frame_rate_hz);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.scan.frame_rate_hz = patch.frame_rate_hz;
  }

  if (patch.has_sensor_enabled) {
    resolved.next_config.sensor_enabled = patch.sensor_enabled;
  }

  resolved.has_requested_update = has_requested_update;
  if (has_requested_update) {
    PROJECT_LOG_INFO(
        "[EosSession] runtime config patch applied: mission={} policy={} environment={} "
        "work_mode={} scan_rate={} frame_rate={}",
        patch.has_mission, patch.has_policy, patch.has_environment,
        patch.has_work_mode, patch.has_scan_rate_deg_per_sec, patch.has_frame_rate_hz);
  }
  return resolved;
}

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
