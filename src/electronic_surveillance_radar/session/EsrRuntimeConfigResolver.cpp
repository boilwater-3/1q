#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
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

EsrRuntimeConfigResolveResult RejectPatch(const ResolvedEsrSessionConfig& current_config,
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
                             extension::InterceptStatisticalDetectionConfig* config_value) {
  if (config_value == nullptr) {
    return;
  }
  config_value->pulse_count = std::max<std::uint32_t>(1U, config_value->pulse_count);
  config_value->threshold_scale =
      oneq::internal::validation::IsFinite(config_value->threshold_scale) && config_value->threshold_scale > 0.0f
          ? config_value->threshold_scale
          : 1.0f;
  switch (mode) {
    case config::EsrWorkMode::kHgesm:
      config_value->pulse_count = std::min<std::uint32_t>(
          config_value->pulse_count * kActiveScanPulseMultiplier, kMaxPulseCount);
      config_value->threshold_scale =
          std::max(kMinimumThresholdScale, config_value->threshold_scale * kHgesmThresholdScale);
      break;
    case config::EsrWorkMode::kRwr:
      config_value->pulse_count = std::max<std::uint32_t>(1U, config_value->pulse_count / 2U);
      config_value->threshold_scale =
          std::max(kMinimumThresholdScale, config_value->threshold_scale * kRwrThresholdScale);
      break;
    case config::EsrWorkMode::kEsm:
    default:
      break;
  }
}

void ApplyEnvironmentRuntimePatch(const environment::EsrEnvironmentRuntimeConfigPatch& env_patch,
                                  ResolvedEsrSessionConfig* resolved, bool* env_changed) {
  if (env_patch.has_atmospheric_physics) {
    resolved->environment_model_config.atmospheric_physics = env_patch.atmospheric_physics;
    *env_changed = true;
  }
  if (env_patch.has_atmospheric_context) {
    resolved->environment_model_config.atmospheric_context = env_patch.atmospheric_context;
    *env_changed = true;
  }
}

void ApplyDetectionPolicy(const config::EsrDetectionPolicyConfig& detection,
                          extension::InterceptPipelineConfig* pipeline_config) {
  if (pipeline_config == nullptr) {
    return;
  }
  if (detection.use_profile_defaults) {
    switch (detection.profile) {
      case config::EsrDetectionProfile::kConservative:
        pipeline_config->detection.min_detect_snr_db = 10.0f;
        break;
      case config::EsrDetectionProfile::kSensitive:
        pipeline_config->detection.min_detect_snr_db = 3.0f;
        break;
      case config::EsrDetectionProfile::kBalanced:
      default:
        break;
    }
  } else {
    pipeline_config->detection.min_detect_snr_db = detection.min_detect_snr_db;
    pipeline_config->statistical_detection.pfa = detection.pfa;
    pipeline_config->statistical_detection.pulse_count = detection.pulse_count;
    pipeline_config->statistical_detection.threshold_scale = detection.threshold_scale;
    pipeline_config->statistical_detection.enable_statistical_detection =
        detection.enable_statistical_detection;
  }
}

}  // namespace

EsrRuntimeConfigResolveResult ResolveEsrRuntimeConfigPatch(
    const ResolvedEsrSessionConfig& current_config, const config::EsrRuntimeConfigPatch& patch) {
  EsrRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  resolved.status = EsrRuntimeConfigApplyStatus::kNoRequestedUpdate;
  bool has_requested_update = false;

  // ---- Phase 1: 整块域覆盖（先于叶子） ----

  if (patch.has_mission) {
    has_requested_update = true;
    resolved.next_config.runtime_config.sensor_enabled = patch.mission.power_on;
    resolved.next_config.runtime_config.scan_rate_hz =
        (oneq::internal::validation::IsFinite(patch.mission.scan.scan_rate_hz) && patch.mission.scan.scan_rate_hz > 0.0f)
            ? patch.mission.scan.scan_rate_hz
            : 1.0f;
    ApplyWorkModeAdjustment(patch.mission.work_mode,
                            &resolved.next_config.pipeline_config.statistical_detection);
    resolved.runtime_config_changed = true;
    resolved.pipeline_config_changed = true;
  }

  if (patch.has_policy) {
    has_requested_update = true;
    ApplyDetectionPolicy(patch.policy.detection, &resolved.next_config.pipeline_config);
    resolved.pipeline_config_changed = true;
  }

  if (patch.has_environment_runtime_config) {
    has_requested_update = true;
    if (patch.environment_runtime_config.has_preset) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch because environment preset hot update is "
          "not allowed.");
      return RejectPatch(current_config, true,
                         EsrRuntimeConfigApplyStatus::kRejectedUnsupportedEnvironmentPresetPatch);
    }
    ApplyEnvironmentRuntimePatch(patch.environment_runtime_config, &resolved.next_config,
                                 &resolved.environment_model_config_changed);
  }

  // ---- Phase 2: 叶子快捷覆盖（覆盖整块域的结果） ----

  if (patch.has_sensor_enabled) {
    resolved.next_config.runtime_config.sensor_enabled = patch.sensor_enabled;
    resolved.runtime_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_work_mode) {
    ApplyWorkModeAdjustment(patch.work_mode,
                            &resolved.next_config.pipeline_config.statistical_detection);
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
    resolved.next_config.runtime_config.scan_rate_hz = patch.scan_rate_hz;
    resolved.runtime_config_changed = true;
  }

  if (patch.has_scan_start_position) {
    resolved.next_config.pipeline_config.scan.scan_start_pos =
        static_cast<int>(patch.scan_start_position);
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_scan_sequence) {
    resolved.next_config.pipeline_config.scan.scan_sequence = static_cast<int>(patch.scan_sequence);
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
        0.5f * std::fabs(resolved.next_config.pipeline_config.scan.scan_end_az_deg -
                         resolved.next_config.pipeline_config.scan.scan_start_az_deg);
    resolved.next_config.pipeline_config.scan.scan_start_az_deg =
        patch.scan_center_az_deg - half_az_span;
    resolved.next_config.pipeline_config.scan.scan_end_az_deg =
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
        0.5f * std::fabs(resolved.next_config.pipeline_config.scan.scan_end_el_deg -
                         resolved.next_config.pipeline_config.scan.scan_start_el_deg);
    resolved.next_config.pipeline_config.scan.scan_start_el_deg =
        patch.scan_center_el_deg - half_el_span;
    resolved.next_config.pipeline_config.scan.scan_end_el_deg =
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
      resolved.next_config.pipeline_config.scan.scan_start_az_deg = start_az;
      resolved.next_config.pipeline_config.scan.scan_end_az_deg = end_az;
      resolved.next_config.pipeline_config.scan.scan_start_el_deg = start_el;
      resolved.next_config.pipeline_config.scan.scan_end_el_deg = end_el;
      resolved.pipeline_config_changed = true;
    }
  }

  resolved.has_requested_update = has_requested_update;
  if (has_requested_update) {
    resolved.status = EsrRuntimeConfigApplyStatus::kApplied;
    PROJECT_LOG_INFO(
        "[EsrSession] runtime config patch applied: mission={} policy={} env={} sensor_enabled={} "
        "work_mode={} scan_rate={} scan_bounds={}",
        patch.has_mission, patch.has_policy, patch.has_environment_runtime_config,
        patch.has_sensor_enabled, patch.has_work_mode, patch.has_scan_rate_hz,
        patch.has_explicit_scan_bounds);
  }
  return resolved;
}

}  // namespace session

}  // namespace electronic_surveillance_radar
