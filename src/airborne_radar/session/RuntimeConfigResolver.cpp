#include "airborne_radar/session/RuntimeConfigResolver.h"

#include <cmath>
#include <utility>

#include "common/logging/ProjectLog.h"
#include "airborne_radar/session/SessionConfigBridge.h"

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
  session::RadarSessionConfig session_config =
      BuildSessionConfigFromExecutionConfig(current_state.execution_config);
  bool has_requested_update = false;
  bool execution_config_changed = false;

  if (patch.has_mission) {
    session_config.mission = patch.mission;
    execution_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_policy) {
    session_config.policy = patch.policy;
    execution_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_work_sub_mode) {
    session_config.mission.orientation.work_sub_mode = patch.work_sub_mode;
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
    session_config.mission.orientation.scan_center_deg = patch.scan_center_deg;
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
    session_config.mission.orientation.commanded_beamwidth_deg = patch.commanded_beamwidth_deg;
    execution_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_commanded_beamwidth_enabled) {
    session_config.mission.orientation.commanded_beamwidth_enabled = patch.commanded_beamwidth_enabled;
    execution_config_changed = true;
    has_requested_update = true;
  }

  if (execution_config_changed) {
    config::execution::InternalExecutionConfig next_execution_config =
        BuildExecutionConfigFromSessionConfig(session_config);
    next_execution_config.platform_attitude_deg =
        current_state.execution_config.platform_attitude_deg;
    resolved.next_state.execution_config = std::move(next_execution_config);
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
