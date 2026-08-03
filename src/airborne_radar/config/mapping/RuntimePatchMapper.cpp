#include "airborne_radar/config/mapping/RuntimePatchMapper.h"

#include <cmath>
#include <vector>

#include "airborne_radar/config/mapping/MappingTransforms.h"
#include "common/logging/ProjectLog.h"
#include "common/validation/ValidationUtils.h"

namespace airborne_radar {
namespace config {
namespace mapping {
namespace {

RuntimeConfigResolveResult RejectPatch(const RuntimeConfigState& current_state,
                                       bool has_requested_update) {
  RuntimeConfigResolveResult rejected;
  rejected.next_state = current_state;
  rejected.has_requested_update = has_requested_update;
  rejected.is_valid = false;
  return rejected;
}

/// @brief 验证 AzimuthElevationDeg 的两个分量是否有限。非法时记录日志。
bool ValidateFiniteAzEl(const AzimuthElevationDeg& v, const char* field_name) {
  if (!oneq::common::validation::IsFinite(v.az_deg) ||
      !oneq::common::validation::IsFinite(v.el_deg)) {
    PROJECT_LOG_ERROR("[ArSession] Rejecting runtime config patch due to non-finite {} "
                      "(az_deg={}, el_deg={}).",
                      field_name, v.az_deg, v.el_deg);
    return false;
  }
  return true;
}

/// @brief 验证 CommandedBeamwidthDeg 的两个分量是否有限。非法时记录日志。
bool ValidateFiniteBeamwidth(const CommandedBeamwidthDeg& v) {
  if (!oneq::common::validation::IsFinite(v.commanded_az_beamwidth_deg) ||
      !oneq::common::validation::IsFinite(v.commanded_el_beamwidth_deg)) {
    PROJECT_LOG_ERROR("[ArSession] Rejecting runtime config patch due to non-finite "
                      "commanded_beamwidth_deg (az_deg={}, el_deg={}).",
                      v.commanded_az_beamwidth_deg, v.commanded_el_beamwidth_deg);
    return false;
  }
  return true;
}

}  // namespace

RuntimeConfigResolveResult ApplyRuntimePatch(const RuntimeConfigState& current_state,
                                             const ArRuntimeConfigPatch& patch) {
  RuntimeConfigResolveResult resolved;
  resolved.next_state = current_state;

  execution::InternalExecutionConfig next_execution_config = current_state.execution_config;
  bool execution_config_changed = false;
  bool has_requested_update = false;
  bool policy_changed = false;

  if (patch.has_dwell_center_deg) {
    has_requested_update = true;
    if (!ValidateFiniteAzEl(patch.dwell_center_deg, "dwell_center_deg")) {
      return RejectPatch(current_state, true);
    }
    resolved.next_state.dwell_center_deg = patch.dwell_center_deg;
    execution_config_changed = true;
  }

  if (patch.has_environment) {
    has_requested_update = true;
    if (patch.environment.has_scenario_config) {
      resolved.next_state.environment_scenario_config = patch.environment.scenario_config;
      resolved.environment_scenario_config_changed = true;
    }
  }

  if (patch.has_mission) {
    // 电源单源：mission 域无电源字段（COMMON-OQ-4，见 contract.md §电源状态单源契约）。
    next_execution_config.detection.orientation = patch.mission.orientation;
    execution_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_policy) {
    next_execution_config.decision_control = patch.policy.decision_control;
    next_execution_config.anti_vgpo_max_acceleration_mps2 =
        patch.policy.decision_control.anti_vgpo_max_acceleration_mps2;
    next_execution_config.detection.beam_control = patch.policy.beam_control;
    next_execution_config.detection.engineering.detection_policy.cfar_pfa =
        patch.policy.detection.pfa;
    next_execution_config.detection.engineering.detection_policy.min_snr_db =
        patch.policy.detection.minimum_snr_db;
    next_execution_config.detection.engineering.pulse_count = patch.policy.detection.pulse_count;
    next_execution_config.detection.engineering.min_detection_margin_db =
        patch.policy.detection.minimum_detection_margin_db;
    next_execution_config.association.policy.unassigned_cost =
        SigmaToSquaredCost(patch.policy.association.distance_gate_sigma);
    next_execution_config.tracking.policy = patch.policy.tracking;
    next_execution_config.lifecycle.policy = patch.policy.lifecycle;
    policy_changed = true;
    execution_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_work_mode) {
    next_execution_config.detection.orientation.work_mode = patch.work_mode;
    execution_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_scan_center_deg) {
    if (!ValidateFiniteAzEl(patch.scan_center_deg, "scan_center_deg")) {
      return RejectPatch(current_state, true);
    }
    next_execution_config.detection.orientation.scan_center_deg = patch.scan_center_deg;
    execution_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_commanded_beamwidth_deg) {
    if (!ValidateFiniteBeamwidth(patch.commanded_beamwidth_deg)) {
      return RejectPatch(current_state, true);
    }
    next_execution_config.detection.orientation.commanded_beamwidth_deg =
        patch.commanded_beamwidth_deg;
    execution_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_commanded_beamwidth_enabled) {
    next_execution_config.detection.orientation.commanded_beamwidth_enabled =
        patch.commanded_beamwidth_enabled;
    execution_config_changed = true;
    has_requested_update = true;
  }

  const config::ArOrientationConfig& next_orientation = next_execution_config.detection.orientation;
  if (next_orientation.commanded_beamwidth_enabled &&
      (next_orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg <= 0.0f ||
       next_orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg <= 0.0f)) {
    PROJECT_LOG_ERROR(
        "[ArSession] Rejecting runtime config patch because enabled commanded beamwidth "
        "must be positive (az_deg={}, el_deg={}).",
        next_orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg,
        next_orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg);
    return RejectPatch(current_state, has_requested_update);
  }

  if (patch.has_sensor_enabled) {
    next_execution_config.sensor_enabled = patch.sensor_enabled;
    execution_config_changed = true;
    has_requested_update = true;
  }

  if (policy_changed) {
    ReconcilePolicyToEngineering(next_execution_config);
  }

  resolved.next_state.execution_config = next_execution_config;
  resolved.has_requested_update = has_requested_update;
  resolved.is_valid = true;
  resolved.execution_config_changed = execution_config_changed;
  return resolved;
}

config::ArSessionConfig MapExecutionToSession(
    const execution::InternalExecutionConfig& execution_config) {
  config::ArSessionConfig config;
  config.sensor_enabled = execution_config.sensor_enabled;
  config.hardware.transmitter = execution_config.detection.engineering.transmitter;
  config.hardware.antenna = execution_config.detection.engineering.antenna;
  config.hardware.receiver = execution_config.detection.engineering.receiver;
  config.hardware.rcs_physics = execution_config.detection.engineering.rcs_physics;
  config.mission.orientation = execution_config.detection.orientation;
  config.policy.decision_control = execution_config.decision_control;
  config.policy.decision_control.anti_vgpo_max_acceleration_mps2 =
      execution_config.anti_vgpo_max_acceleration_mps2;
  config.policy.beam_control = execution_config.detection.beam_control;
  config.policy.detection.pfa = execution_config.detection.engineering.detection_policy.cfar_pfa;
  config.policy.detection.minimum_snr_db =
      execution_config.detection.engineering.detection_policy.min_snr_db;
  config.policy.detection.pulse_count = execution_config.detection.engineering.pulse_count;
  config.policy.detection.minimum_detection_margin_db =
      execution_config.detection.engineering.min_detection_margin_db;
  config.policy.association.distance_gate_sigma =
      SquaredCostToSigma(execution_config.association.policy.unassigned_cost);
  config.policy.tracking = execution_config.tracking.policy;
  config.policy.lifecycle = execution_config.lifecycle.policy;
  return config;
}

config::ArSessionConfig MapRuntimeStateToPipelineSession(const RuntimeConfigState& runtime_state) {
  config::ArSessionConfig config = MapExecutionToSession(runtime_state.execution_config);
  config.mission.orientation.scan_center_deg.az_deg += runtime_state.dwell_center_deg.az_deg;
  config.mission.orientation.scan_center_deg.el_deg += runtime_state.dwell_center_deg.el_deg;
  return config;
}

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar
