#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "common/logging/ProjectLog.h"
#include "common/validation/ValidationUtils.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

constexpr std::uint32_t kActiveScanPulseMultiplier = 4U;
constexpr std::uint32_t kMaxPulseCount = 4096U;
constexpr float kMinimumThresholdScale = 0.1f;
constexpr float kHgesmThresholdScale = 0.85f;
constexpr float kRwrThresholdScale = 1.25f;

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

void NormalizeScanBounds(float* start, float* end) {
  if (start == nullptr || end == nullptr) {
    return;
  }
  if (*start > *end) {
    std::swap(*start, *end);
  }
}

void ApplyWorkModeAdjustment(config::EsrWorkMode mode,
                             DetectionConfig* detection_config) {
  if (detection_config == nullptr) {
    return;
  }
  detection_config->pulse_count = std::max<std::uint32_t>(1U, detection_config->pulse_count);
  detection_config->threshold_scale =
      oneq::internal::validation::IsFinite(detection_config->threshold_scale) &&
              detection_config->threshold_scale > 0.0f
          ? detection_config->threshold_scale
          : 1.0f;
  switch (mode) {
    case config::EsrWorkMode::kHgesm:
      detection_config->pulse_count = std::min<std::uint32_t>(
          detection_config->pulse_count * kActiveScanPulseMultiplier, kMaxPulseCount);
      detection_config->threshold_scale =
          std::max(kMinimumThresholdScale, detection_config->threshold_scale * kHgesmThresholdScale);
      break;
    case config::EsrWorkMode::kRwr:
      detection_config->pulse_count = std::max<std::uint32_t>(1U, detection_config->pulse_count / 2U);
      detection_config->threshold_scale =
          std::max(kMinimumThresholdScale, detection_config->threshold_scale * kRwrThresholdScale);
      break;
    case config::EsrWorkMode::kEsm:
    default:
      break;
  }
}

void ApplyScanPolicy(const config::EsrHardwareConfig& hardware,
                     const config::EsrScanPolicyConfig& scan_policy,
                     extension::InterceptScanConfig* scan_config) {
  if (scan_config == nullptr) {
    return;
  }

  const float mount_az = oneq::internal::validation::IsFinite(hardware.antenna_mount_az_deg)
                             ? hardware.antenna_mount_az_deg
                             : 0.0f;
  const float mount_el = oneq::internal::validation::IsFinite(hardware.antenna_mount_el_deg)
                             ? hardware.antenna_mount_el_deg
                             : 0.0f;
  scan_config->scan_start_pos = static_cast<int>(scan_policy.scan_start_position);
  scan_config->scan_sequence = static_cast<int>(scan_policy.scan_sequence);

  if (oneq::internal::validation::IsFinite(hardware.beam_az_width_deg) &&
      hardware.beam_az_width_deg > 0.0f) {
    scan_config->az_step_deg = hardware.beam_az_width_deg;
  }
  if (oneq::internal::validation::IsFinite(hardware.beam_el_width_deg) &&
      hardware.beam_el_width_deg > 0.0f) {
    scan_config->el_step_deg = hardware.beam_el_width_deg;
  }

  const bool explicit_bounds_valid =
      scan_policy.use_explicit_scan_bounds &&
      oneq::internal::validation::IsFinite(scan_policy.scan_start_az_deg) &&
      oneq::internal::validation::IsFinite(scan_policy.scan_end_az_deg) &&
      oneq::internal::validation::IsFinite(scan_policy.scan_start_el_deg) &&
      oneq::internal::validation::IsFinite(scan_policy.scan_end_el_deg);
  if (explicit_bounds_valid) {
    float start_az = scan_policy.scan_start_az_deg - mount_az;
    float end_az = scan_policy.scan_end_az_deg - mount_az;
    float start_el = scan_policy.scan_start_el_deg - mount_el;
    float end_el = scan_policy.scan_end_el_deg - mount_el;
    NormalizeScanBounds(&start_az, &end_az);
    NormalizeScanBounds(&start_el, &end_el);
    scan_config->scan_start_az_deg = start_az;
    scan_config->scan_end_az_deg = end_az;
    scan_config->scan_start_el_deg = start_el;
    scan_config->scan_end_el_deg = end_el;
    return;
  }

  const bool has_center_az = oneq::internal::validation::IsFinite(scan_policy.scan_center_az_deg);
  const bool has_center_el = oneq::internal::validation::IsFinite(scan_policy.scan_center_el_deg);
  if (has_center_az) {
    float half_az_span =
        0.5f * std::fabs(scan_config->scan_end_az_deg - scan_config->scan_start_az_deg);
    if (oneq::internal::validation::IsFinite(hardware.az_scan_range_deg) &&
        hardware.az_scan_range_deg > 0.0f) {
      half_az_span = 0.5f * hardware.az_scan_range_deg;
    }
    const float center_az = scan_policy.scan_center_az_deg - mount_az;
    scan_config->scan_start_az_deg = center_az - half_az_span;
    scan_config->scan_end_az_deg = center_az + half_az_span;
  }
  if (has_center_el) {
    float half_el_span =
        0.5f * std::fabs(scan_config->scan_end_el_deg - scan_config->scan_start_el_deg);
    if (oneq::internal::validation::IsFinite(hardware.el_scan_range_deg) &&
        hardware.el_scan_range_deg > 0.0f) {
      half_el_span = 0.5f * hardware.el_scan_range_deg;
    }
    const float center_el = scan_policy.scan_center_el_deg - mount_el;
    scan_config->scan_start_el_deg = center_el - half_el_span;
    scan_config->scan_end_el_deg = center_el + half_el_span;
  }
  NormalizeScanBounds(&scan_config->scan_start_az_deg, &scan_config->scan_end_az_deg);
  NormalizeScanBounds(&scan_config->scan_start_el_deg, &scan_config->scan_end_el_deg);
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
    resolved.next_config.mission = patch.mission;
    if (!oneq::internal::validation::IsFinite(
            resolved.next_config.mission.scan.scan_rate_hz) ||
        resolved.next_config.mission.scan.scan_rate_hz <= 0.0f) {
      resolved.next_config.mission.scan.scan_rate_hz = 1.0f;
    }
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
    if (!oneq::internal::validation::IsFinite(patch.scan_rate_hz) || patch.scan_rate_hz <= 0.0f) {
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
    if (!oneq::internal::validation::IsFinite(patch.scan_center_az_deg)) {
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
    if (!oneq::internal::validation::IsFinite(patch.scan_center_el_deg)) {
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
      if (!oneq::internal::validation::IsFinite(sb.scan_start_az_deg) ||
          !oneq::internal::validation::IsFinite(sb.scan_end_az_deg) ||
          !oneq::internal::validation::IsFinite(sb.scan_start_el_deg) ||
          !oneq::internal::validation::IsFinite(sb.scan_end_el_deg)) {
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
