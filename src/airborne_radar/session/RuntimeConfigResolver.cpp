#include "airborne_radar/session/RuntimeConfigResolver.h"

#include <cmath>
#include <vector>

#include "airborne_radar/config/internal/ExpertToEngineeringMapping.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace session {
namespace internal {
namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }

RuntimeConfigResolveResult RejectPatch(const RuntimeConfigState& current_state,
                                       bool has_requested_update) {
  RuntimeConfigResolveResult rejected;
  rejected.next_state = current_state;
  rejected.has_requested_update = has_requested_update;
  rejected.is_valid = false;
  return rejected;
}

}  // namespace

RuntimeConfigResolveResult ResolveRuntimeConfigPatch(const RuntimeConfigState& current_state,
                                                     const config::RadarRuntimeConfigPatch& patch) {
  RuntimeConfigResolveResult resolved;
  resolved.next_state = current_state;
  config::execution::InternalExecutionConfig next_execution_config = current_state.execution_config;
  bool has_requested_update = false;
  bool execution_config_changed = false;
  bool policy_changed = false;

  if (patch.has_mission) {
    next_execution_config.mission_orientation = patch.mission.orientation;
    execution_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_policy) {
    next_execution_config.policy_beam_control = patch.policy.beam_control;
    next_execution_config.policy_association = patch.policy.association;
    next_execution_config.policy_tracking = patch.policy.tracking;
    next_execution_config.policy_lifecycle = patch.policy.lifecycle;
    next_execution_config.policy_imm = patch.policy.imm;
    policy_changed = true;
    execution_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_work_sub_mode) {
    next_execution_config.mission_orientation.work_sub_mode = patch.work_sub_mode;
    execution_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_scan_center_deg) {
    if (!IsFinite(patch.scan_center_deg.az_deg) || !IsFinite(patch.scan_center_deg.el_deg)) {
      PROJECT_LOG_ERROR(
          "[RadarSession] Rejecting runtime config patch due to non-finite scan_center_deg "
          "(az_deg={}, el_deg={}).",
          patch.scan_center_deg.az_deg, patch.scan_center_deg.el_deg);
      return RejectPatch(current_state, true);
    }
    next_execution_config.mission_orientation.scan_center_deg = patch.scan_center_deg;
    execution_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_dwell_center_deg) {
    if (!IsFinite(patch.dwell_center_deg.az_deg) || !IsFinite(patch.dwell_center_deg.el_deg)) {
      PROJECT_LOG_ERROR(
          "[RadarSession] Rejecting runtime config patch due to non-finite dwell_center_deg "
          "(az_deg={}, el_deg={}).",
          patch.dwell_center_deg.az_deg, patch.dwell_center_deg.el_deg);
      return RejectPatch(current_state, true);
    }
    resolved.next_state.dwell_center_deg = patch.dwell_center_deg;
    execution_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_commanded_beamwidth_deg) {
    if (!IsFinite(patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg) ||
        !IsFinite(patch.commanded_beamwidth_deg.commanded_el_beamwidth_deg)) {
      PROJECT_LOG_ERROR(
          "[RadarSession] Rejecting runtime config patch due to non-finite "
          "commanded_beamwidth_deg (az_deg={}, el_deg={}).",
          patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg,
          patch.commanded_beamwidth_deg.commanded_el_beamwidth_deg);
      return RejectPatch(current_state, true);
    }
    next_execution_config.mission_orientation.commanded_beamwidth_deg = patch.commanded_beamwidth_deg;
    execution_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_commanded_beamwidth_enabled) {
    next_execution_config.mission_orientation.commanded_beamwidth_enabled =
        patch.commanded_beamwidth_enabled;
    execution_config_changed = true;
    has_requested_update = true;
  }

  if (policy_changed) {
    next_execution_config.tracking_engineering =
        config::internal::ResolveTrackingEngineering(next_execution_config.policy_tracking);
    next_execution_config.lifecycle_engineering =
        config::internal::ResolveLifecycleEngineering(next_execution_config.policy_lifecycle);
    next_execution_config.tracking_speed_decay_ratio_on_loss =
        next_execution_config.policy_tracking.speed_decay_ratio_on_loss;
    next_execution_config.tracking_rcs_decay_ratio_on_loss =
        next_execution_config.policy_tracking.rcs_decay_ratio_on_loss;
    next_execution_config.association_unassigned_cost =
        next_execution_config.policy_association.unassigned_cost;
    if (next_execution_config.lifecycle_engineering.enable_imm_lifecycle) {
      next_execution_config.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 4.0f};
    } else {
      next_execution_config.imm_model_noise_diff_coeffs.clear();
      next_execution_config.imm_initial_weights.clear();
      next_execution_config.imm_transition_probability.clear();
    }
  }

  if (execution_config_changed) {
    resolved.next_state.execution_config = next_execution_config;
    resolved.execution_config_changed = true;
  }

  if (patch.has_environment_runtime_config) {
    has_requested_update = true;
    if (patch.environment_runtime_config.has_scenario_config) {
      resolved.next_state.environment_scenario_config =
          patch.environment_runtime_config.scenario_config;
      resolved.environment_scenario_config_changed = true;
    }
    if (patch.environment_runtime_config.has_jamming_sensitivity_profile) {
      resolved.next_state.jamming_sensitivity_profile =
          patch.environment_runtime_config.jamming_sensitivity_profile;
      resolved.jamming_sensitivity_profile_changed = true;
    }
  }

  resolved.has_requested_update = has_requested_update;
  return resolved;
}

}  // namespace internal
}  // namespace session
}  // namespace airborne_radar
