#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
#include "common/logging/ProjectLog.h"

namespace electronic_surveillance_radar {
namespace session {
namespace internal {
namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }
constexpr std::uint32_t kActiveScanPulseMultiplier = 4U;
constexpr std::uint32_t kMaxPulseCount = 4096U;
constexpr float kMinimumThresholdScale = 0.1f;
constexpr float kHgesmThresholdScale = 0.85f;
constexpr float kRwrThresholdScale = 1.25f;

EsrRuntimeConfigResolveResult RejectPatch(const ResolvedEsrSessionConfig& current_config,
                                          bool has_requested_update) {
  EsrRuntimeConfigResolveResult rejected;
  rejected.next_config = current_config;
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
      IsFinite(config_value->threshold_scale) && config_value->threshold_scale > 0.0f
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

void ApplyEnvironmentPreset(config::EsrEnvironmentPreset preset,
                            environment::EsrEnvironmentModelConfig* model_config) {
  if (model_config == nullptr) {
    return;
  }
  switch (preset) {
    case config::EsrEnvironmentPreset::kLowClutter:
      model_config->clutter_baseline_policy = environment::EsrClutterBaselinePolicy::kLow;
      model_config->jamming_sensitivity_policy = environment::EsrJammingSensitivityPolicy::kStrict;
      break;
    case config::EsrEnvironmentPreset::kDenseClutter:
      model_config->clutter_baseline_policy = environment::EsrClutterBaselinePolicy::kHigh;
      model_config->jamming_sensitivity_policy = environment::EsrJammingSensitivityPolicy::kBalanced;
      break;
    case config::EsrEnvironmentPreset::kJammed:
      model_config->clutter_baseline_policy = environment::EsrClutterBaselinePolicy::kHigh;
      model_config->jamming_sensitivity_policy = environment::EsrJammingSensitivityPolicy::kRelaxed;
      break;
    case config::EsrEnvironmentPreset::kStandard:
    default:
      break;
  }
}

}  // namespace

EsrRuntimeConfigResolveResult ResolveEsrRuntimeConfigPatch(
    const ResolvedEsrSessionConfig& current_config, const EsrRuntimeConfigPatch& patch) {
  EsrRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  bool has_requested_update = false;

  if (patch.has_sensor_enabled) {
    resolved.next_config.runtime_config.sensor_enabled = patch.sensor_enabled;
    resolved.runtime_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_work_mode) {
    ApplyWorkModeAdjustment(patch.work_mode, &resolved.next_config.pipeline_config.statistical_detection);
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_scan_rate_hz) {
    has_requested_update = true;
    if (!IsFinite(patch.scan_rate_hz) || patch.scan_rate_hz <= 0.0f) {
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to invalid scan_rate_hz={}; "
          "must be finite and positive.",
          patch.scan_rate_hz);
      return RejectPatch(current_config, true);
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
    if (!IsFinite(patch.scan_center_az_deg)) {
      PROJECT_LOG_ERROR("[EsrSession] Rejecting runtime config patch due to non-finite scan_center_az_deg={} .",
                        patch.scan_center_az_deg);
      return RejectPatch(current_config, true);
    }
    const float half_az_span = 0.5f * std::fabs(resolved.next_config.pipeline_config.scan.scan_end_az_deg -
                                                resolved.next_config.pipeline_config.scan.scan_start_az_deg);
    resolved.next_config.pipeline_config.scan.scan_start_az_deg = patch.scan_center_az_deg - half_az_span;
    resolved.next_config.pipeline_config.scan.scan_end_az_deg = patch.scan_center_az_deg + half_az_span;
    resolved.pipeline_config_changed = true;
  }
  if (patch.has_scan_center_el_deg) {
    has_requested_update = true;
    if (!IsFinite(patch.scan_center_el_deg)) {
      PROJECT_LOG_ERROR("[EsrSession] Rejecting runtime config patch due to non-finite scan_center_el_deg={} .",
                        patch.scan_center_el_deg);
      return RejectPatch(current_config, true);
    }
    const float half_el_span = 0.5f * std::fabs(resolved.next_config.pipeline_config.scan.scan_end_el_deg -
                                                resolved.next_config.pipeline_config.scan.scan_start_el_deg);
    resolved.next_config.pipeline_config.scan.scan_start_el_deg = patch.scan_center_el_deg - half_el_span;
    resolved.next_config.pipeline_config.scan.scan_end_el_deg = patch.scan_center_el_deg + half_el_span;
    resolved.pipeline_config_changed = true;
  }
  if (patch.has_use_explicit_scan_bounds) {
    has_requested_update = true;
    if (patch.use_explicit_scan_bounds) {
      if (!patch.has_scan_start_az_deg || !patch.has_scan_end_az_deg || !patch.has_scan_start_el_deg ||
          !patch.has_scan_end_el_deg || !IsFinite(patch.scan_start_az_deg) ||
          !IsFinite(patch.scan_end_az_deg) || !IsFinite(patch.scan_start_el_deg) ||
          !IsFinite(patch.scan_end_el_deg)) {
        PROJECT_LOG_ERROR(
            "[EsrSession] Rejecting runtime config patch due to invalid explicit scan bounds payload.");
        return RejectPatch(current_config, true);
      }
      float start_az = patch.scan_start_az_deg;
      float end_az = patch.scan_end_az_deg;
      float start_el = patch.scan_start_el_deg;
      float end_el = patch.scan_end_el_deg;
      NormalizeScanBounds(&start_az, &end_az);
      NormalizeScanBounds(&start_el, &end_el);
      resolved.next_config.pipeline_config.scan.scan_start_az_deg = start_az;
      resolved.next_config.pipeline_config.scan.scan_end_az_deg = end_az;
      resolved.next_config.pipeline_config.scan.scan_start_el_deg = start_el;
      resolved.next_config.pipeline_config.scan.scan_end_el_deg = end_el;
      resolved.pipeline_config_changed = true;
    }
  }
  if (patch.has_environment_preset) {
    ApplyEnvironmentPreset(patch.environment_preset, &resolved.next_config.environment_model_config);
    resolved.environment_model_config_changed = true;
    has_requested_update = true;
  }

  resolved.has_requested_update = has_requested_update;
  return resolved;
}

}  // namespace internal
}  // namespace session

}  // namespace electronic_surveillance_radar
