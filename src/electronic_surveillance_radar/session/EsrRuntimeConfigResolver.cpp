#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"

#include <algorithm>
#include <cmath>

#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/session/EsrResolutionRules.h"
#include "common/logging/ProjectLog.h"
#include "common/validation/ValidationUtils.h"

namespace electronic_surveillance_radar {
namespace session {

using resolution_rules::ApplyScanPolicy;
using resolution_rules::ApplyWorkModeAdjustment;
using resolution_rules::NormalizeScanBounds;

namespace {

EsrRuntimeConfigResolveResult RejectPatch(const EsrInternalExecutionConfig& current_config,
                                          bool has_requested_update,
                                          EsrRuntimeConfigApplyStatus status) {
  EsrRuntimeConfigResolveResult rejected;
  rejected.next_config = current_config;
  rejected.status = status;
  rejected.has_requested_update = has_requested_update;
  rejected.is_valid = false;
  return rejected;
}

void ApplyEnvironmentRuntimePatch(const config::EsrEnvironmentRuntimeConfigPatch& env_patch,
                                  EsrInternalExecutionConfig* resolved, bool* env_changed) {
  if (env_patch.has_atmospheric_physics) {
    resolved->environment.atmospheric_physics = env_patch.atmospheric_physics;
    *env_changed = true;
  }
  if (env_patch.has_atmospheric_context) {
    resolved->environment.atmospheric_context = env_patch.atmospheric_context;
    *env_changed = true;
  }
}

}  // namespace

EsrRuntimeConfigResolveResult ResolveEsrRuntimeConfigPatch(
    const EsrInternalExecutionConfig& current_config,
    const config::EsrRuntimeConfigPatch& patch) {
  EsrRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  resolved.status = EsrRuntimeConfigApplyStatus::kNoRequestedUpdate;
  bool has_requested_update = false;

  // ---- Phase 1: 整块域覆盖（先于叶子） ----

  if (patch.has_mission) {
    has_requested_update = true;
    if (!oneq::common::validation::IsFinite(patch.mission.scan.scan_rate_hz) ||
        patch.mission.scan.scan_rate_hz <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting mission runtime patch due to invalid scan_rate_hz={}; "
          "must be finite and positive.",
          patch.mission.scan.scan_rate_hz);
      return RejectPatch(current_config, true,
                         EsrRuntimeConfigApplyStatus::kRejectedInvalidScanRate);
    }
    resolved.next_config.mission = patch.mission;
    ApplyScanPolicy(resolved.next_config.hardware, resolved.next_config.mission.scan,
                    &resolved.next_config.resolved_scan);
    resolved.runtime_config_changed = true;
    resolved.pipeline_config_changed = true;
  }

  if (patch.has_policy) {
    has_requested_update = true;
    resolved.next_config.detection = patch.policy.detection;
    resolved.pipeline_config_changed = true;
  }

  if (patch.has_mission || patch.has_policy) {
    ApplyWorkModeAdjustment(resolved.next_config.mission.work_mode,
                            &resolved.next_config.detection);
  }

  if (patch.has_environment) {
    has_requested_update = true;
    ApplyEnvironmentRuntimePatch(patch.environment, &resolved.next_config,
                                 &resolved.environment_model_config_changed);
  }

  // ---- Phase 2: 叶子快捷覆盖（覆盖整块域的结果） ----

  if (patch.has_sensor_enabled) {
    resolved.next_config.mission.power_on = patch.sensor_enabled;
    resolved.runtime_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_work_mode) {
    resolved.next_config.mission.work_mode = patch.work_mode;
    ApplyWorkModeAdjustment(patch.work_mode, &resolved.next_config.detection);
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_scan_rate_hz) {
    has_requested_update = true;
    if (!oneq::common::validation::IsFinite(patch.scan_rate_hz) || patch.scan_rate_hz <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to invalid scan_rate_hz={}; "
          "must be finite and positive.",
          patch.scan_rate_hz);
      return RejectPatch(current_config, true,
                         EsrRuntimeConfigApplyStatus::kRejectedInvalidScanRate);
    }
    resolved.next_config.mission.scan.scan_rate_hz = patch.scan_rate_hz;
    resolved.runtime_config_changed = true;
  }

  if (patch.has_scan_start_position) {
    resolved.next_config.mission.scan.scan_start_position = patch.scan_start_position;
    resolved.next_config.resolved_scan.scan_start_pos =
        static_cast<int>(patch.scan_start_position);
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_scan_sequence) {
    resolved.next_config.mission.scan.scan_sequence = patch.scan_sequence;
    resolved.next_config.resolved_scan.scan_sequence =
        static_cast<int>(patch.scan_sequence);
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_scan_center_az_deg) {
    has_requested_update = true;
    if (!oneq::common::validation::IsFinite(patch.scan_center_az_deg)) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to non-finite scan_center_az_deg={} .",
          patch.scan_center_az_deg);
      return RejectPatch(current_config, true,
                         EsrRuntimeConfigApplyStatus::kRejectedInvalidScanCenterAz);
    }
    const float half_az_span =
        0.5f * std::fabs(resolved.next_config.resolved_scan.scan_end_az_deg -
                         resolved.next_config.resolved_scan.scan_start_az_deg);
    resolved.next_config.mission.scan.scan_center_az_deg = patch.scan_center_az_deg;
    resolved.next_config.mission.scan.use_explicit_scan_bounds = false;
    resolved.next_config.resolved_scan.scan_start_az_deg =
        patch.scan_center_az_deg - half_az_span;
    resolved.next_config.resolved_scan.scan_end_az_deg =
        patch.scan_center_az_deg + half_az_span;
    resolved.pipeline_config_changed = true;
  }
  if (patch.has_scan_center_el_deg) {
    has_requested_update = true;
    if (!oneq::common::validation::IsFinite(patch.scan_center_el_deg)) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to non-finite scan_center_el_deg={} .",
          patch.scan_center_el_deg);
      return RejectPatch(current_config, true,
                         EsrRuntimeConfigApplyStatus::kRejectedInvalidScanCenterEl);
    }
    const float half_el_span =
        0.5f * std::fabs(resolved.next_config.resolved_scan.scan_end_el_deg -
                         resolved.next_config.resolved_scan.scan_start_el_deg);
    resolved.next_config.mission.scan.scan_center_el_deg = patch.scan_center_el_deg;
    resolved.next_config.mission.scan.use_explicit_scan_bounds = false;
    resolved.next_config.resolved_scan.scan_start_el_deg =
        patch.scan_center_el_deg - half_el_span;
    resolved.next_config.resolved_scan.scan_end_el_deg =
        patch.scan_center_el_deg + half_el_span;
    resolved.pipeline_config_changed = true;
  }
  if (patch.has_explicit_scan_bounds) {
    has_requested_update = true;
    if (patch.explicit_scan_bounds.enabled) {
      const auto& sb = patch.explicit_scan_bounds;
      if (!oneq::common::validation::IsFinite(sb.scan_start_az_deg) ||
          !oneq::common::validation::IsFinite(sb.scan_end_az_deg) ||
          !oneq::common::validation::IsFinite(sb.scan_start_el_deg) ||
          !oneq::common::validation::IsFinite(sb.scan_end_el_deg)) {
        PROJECT_LOG_ERROR(
            "[EsrSession] Rejecting runtime config patch due to invalid explicit scan bounds "
            "payload.");
        return RejectPatch(current_config, true,
                           EsrRuntimeConfigApplyStatus::kRejectedInvalidExplicitScanBounds);
      }
      float start_az = sb.scan_start_az_deg;
      float end_az = sb.scan_end_az_deg;
      float start_el = sb.scan_start_el_deg;
      float end_el = sb.scan_end_el_deg;
      NormalizeScanBounds(&start_az, &end_az);
      NormalizeScanBounds(&start_el, &end_el);
      resolved.next_config.mission.scan.use_explicit_scan_bounds = true;
      resolved.next_config.mission.scan.scan_start_az_deg = start_az;
      resolved.next_config.mission.scan.scan_end_az_deg = end_az;
      resolved.next_config.mission.scan.scan_start_el_deg = start_el;
      resolved.next_config.mission.scan.scan_end_el_deg = end_el;
      resolved.next_config.resolved_scan.scan_start_az_deg = start_az;
      resolved.next_config.resolved_scan.scan_end_az_deg = end_az;
      resolved.next_config.resolved_scan.scan_start_el_deg = start_el;
      resolved.next_config.resolved_scan.scan_end_el_deg = end_el;
      resolved.pipeline_config_changed = true;
    }
  }

  resolved.has_requested_update = has_requested_update;
  if (has_requested_update) {
    resolved.status = EsrRuntimeConfigApplyStatus::kApplied;
    PROJECT_LOG_INFO(
        "[EsrSession] runtime config patch applied: mission={} policy={} env={} sensor_enabled={} "
        "work_mode={} scan_rate={} scan_bounds={}",
        patch.has_mission, patch.has_policy, patch.has_environment,
        patch.has_sensor_enabled, patch.has_work_mode, patch.has_scan_rate_hz,
        patch.has_explicit_scan_bounds);
  }
  return resolved;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
