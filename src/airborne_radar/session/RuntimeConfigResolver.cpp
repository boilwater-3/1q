#include "airborne_radar/session/RuntimeConfigResolver.h"

#include <cmath>

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
  bool has_requested_update = false;

  if (patch.has_signal_pipeline_config) {
    resolved.next_state.signal_pipeline_config = patch.signal_pipeline_config;
    resolved.signal_pipeline_config_changed = true;
    has_requested_update = true;
  }

  config::SignalPipelineConfig* signal_pipeline_config = &resolved.next_state.signal_pipeline_config;
  if (patch.has_work_sub_mode) {
    signal_pipeline_config->beam_control.radar_orientation.work_sub_mode = patch.work_sub_mode;
    resolved.signal_pipeline_config_changed = true;
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
    signal_pipeline_config->beam_control.radar_orientation.scan_center_deg = patch.scan_center_deg;
    resolved.signal_pipeline_config_changed = true;
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
    signal_pipeline_config->beam_control.radar_orientation.dwell_center_deg = patch.dwell_center_deg;
    resolved.signal_pipeline_config_changed = true;
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
    signal_pipeline_config->beam_control.radar_orientation.commanded_beamwidth_deg =
        patch.commanded_beamwidth_deg;
    resolved.signal_pipeline_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_commanded_beamwidth_enabled) {
    signal_pipeline_config->beam_control.radar_orientation.commanded_beamwidth_enabled =
        patch.commanded_beamwidth_enabled;
    resolved.signal_pipeline_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_environment_runtime_config) {
    has_requested_update = true;
    if (patch.environment_runtime_config.has_model_config) {
      resolved.next_state.environment_model_config = patch.environment_runtime_config.model_config;
      resolved.environment_model_config_changed = true;
    }
    if (patch.environment_runtime_config.has_jamming_detection_threshold_db) {
      if (!IsFinite(patch.environment_runtime_config.jamming_detection_threshold_db)) {
        PROJECT_LOG_ERROR(
            "[RadarSession] Rejecting runtime config patch due to non-finite "
            "jamming_detection_threshold_db={}.",
            patch.environment_runtime_config.jamming_detection_threshold_db);
        return RejectPatch(current_state, true);
      }
      resolved.next_state.jamming_detection_threshold_db =
          patch.environment_runtime_config.jamming_detection_threshold_db;
      resolved.jamming_detection_threshold_changed = true;
    }
  }

  resolved.has_requested_update = has_requested_update;
  return resolved;
}

}  // namespace internal
}  // namespace session
}  // namespace airborne_radar
