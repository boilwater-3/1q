#include "airborne_radar/session/RuntimeConfigResolver.h"

#include "airborne_radar/config/mapping/RuntimePatchMapper.h"

namespace airborne_radar {
namespace session {
namespace internal {
namespace {

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
  const config::mapping::RuntimePatchMappingResult mapped_execution =
      config::mapping::ApplyRuntimePatch(current_state.execution_config, patch);

  if (!mapped_execution.is_valid) {
    return RejectPatch(current_state, mapped_execution.has_requested_update);
  }

  bool has_requested_update = mapped_execution.has_requested_update;

  if (mapped_execution.execution_config_changed) {
    resolved.next_state.execution_config = mapped_execution.next_execution_config;
    resolved.execution_config_changed = true;
  }

  if (mapped_execution.dwell_center_changed) {
    resolved.next_state.dwell_center_deg = mapped_execution.next_dwell_center_deg;
  }

  if (mapped_execution.environment_scenario_config_changed) {
    resolved.next_state.environment_scenario_config =
        mapped_execution.next_environment_scenario_config;
    resolved.environment_scenario_config_changed = true;
  }

  if (mapped_execution.jamming_sensitivity_profile_changed) {
    resolved.next_state.jamming_sensitivity_profile =
        mapped_execution.next_jamming_sensitivity_profile;
    resolved.jamming_sensitivity_profile_changed = true;
  }

  resolved.has_requested_update = has_requested_update;
  return resolved;
}

}  // namespace internal
}  // namespace session
}  // namespace airborne_radar
