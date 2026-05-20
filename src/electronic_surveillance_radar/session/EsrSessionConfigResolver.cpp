#include "electronic_surveillance_radar/session/EsrSessionConfigResolver.h"
#include "common/validation/ValidationUtils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace electronic_surveillance_radar {
namespace session {
namespace internal {
namespace {



constexpr std::uint32_t kActiveScanPulseMultiplier = 4U;
constexpr std::uint32_t kMaxPulseCount = 4096U;
constexpr float kMinimumThresholdScale = 0.1f;
constexpr float kHgesmThresholdScale = 0.85f;
constexpr float kRwrThresholdScale = 1.25f;
constexpr float kConservativeDetectSnrDb = 10.0f;
constexpr float kSensitiveDetectSnrDb = 3.0f;

void NormalizeScanBounds(float* start, float* end) {
  if (start == nullptr || end == nullptr) {
    return;
  }
  if (*start > *end) {
    std::swap(*start, *end);
  }
}

void ApplyWorkModeAdjustment(EsrWorkMode mode,
                             extension::InterceptStatisticalDetectionConfig* config) {
  if (config == nullptr) {
    return;
  }
  config->pulse_count = std::max<std::uint32_t>(1U, config->pulse_count);
  config->threshold_scale = oneq::internal::validation::IsFinite(config->threshold_scale) && config->threshold_scale > 0.0f
                                ? config->threshold_scale
                                : 1.0f;
  switch (mode) {
    case EsrWorkMode::kHgesm:
      config->pulse_count =
          std::min<std::uint32_t>(config->pulse_count * kActiveScanPulseMultiplier, kMaxPulseCount);
      config->threshold_scale =
          std::max(kMinimumThresholdScale, config->threshold_scale * kHgesmThresholdScale);
      break;
    case EsrWorkMode::kRwr:
      config->pulse_count = std::max<std::uint32_t>(1U, config->pulse_count / 2U);
      config->threshold_scale =
          std::max(kMinimumThresholdScale, config->threshold_scale * kRwrThresholdScale);
      break;
    case EsrWorkMode::kEsm:
    default:
      break;
  }
}

void ResolveScanPolicy(const config::EsrHardwareConfig& hardware,
                       const config::EsrScanPolicyConfig& scan_policy,
                       const extension::InterceptRuntimeConfig& runtime_config,
                       extension::InterceptScanConfig* scan_config) {
  if (scan_config == nullptr) {
    return;
  }

  const float mount_az = runtime_config.antenna_mount_az_deg;
  const float mount_el = runtime_config.antenna_mount_el_deg;
  scan_config->scan_start_pos = static_cast<int>(scan_policy.scan_start_position);
  scan_config->scan_sequence = static_cast<int>(scan_policy.scan_sequence);

  if (oneq::internal::validation::IsFinite(hardware.beam_az_width_deg) && hardware.beam_az_width_deg > 0.0f) {
    scan_config->az_step_deg = hardware.beam_az_width_deg;
  }
  if (oneq::internal::validation::IsFinite(hardware.beam_el_width_deg) && hardware.beam_el_width_deg > 0.0f) {
    scan_config->el_step_deg = hardware.beam_el_width_deg;
  }

  const bool explicit_bounds_valid =
      scan_policy.use_explicit_scan_bounds && oneq::internal::validation::IsFinite(scan_policy.scan_start_az_deg) &&
      oneq::internal::validation::IsFinite(scan_policy.scan_end_az_deg) && oneq::internal::validation::IsFinite(scan_policy.scan_start_el_deg) &&
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
    if (oneq::internal::validation::IsFinite(hardware.az_scan_range_deg) && hardware.az_scan_range_deg > 0.0f) {
      half_az_span = 0.5f * hardware.az_scan_range_deg;
    }
    const float center_az = scan_policy.scan_center_az_deg - mount_az;
    scan_config->scan_start_az_deg = center_az - half_az_span;
    scan_config->scan_end_az_deg = center_az + half_az_span;
  }
  if (has_center_el) {
    float half_el_span =
        0.5f * std::fabs(scan_config->scan_end_el_deg - scan_config->scan_start_el_deg);
    if (oneq::internal::validation::IsFinite(hardware.el_scan_range_deg) && hardware.el_scan_range_deg > 0.0f) {
      half_el_span = 0.5f * hardware.el_scan_range_deg;
    }
    const float center_el = scan_policy.scan_center_el_deg - mount_el;
    scan_config->scan_start_el_deg = center_el - half_el_span;
    scan_config->scan_end_el_deg = center_el + half_el_span;
  }
  NormalizeScanBounds(&scan_config->scan_start_az_deg, &scan_config->scan_end_az_deg);
  NormalizeScanBounds(&scan_config->scan_start_el_deg, &scan_config->scan_end_el_deg);
}

void ApplyDetectionPolicy(const config::EsrDetectionPolicyConfig& detection,
                          extension::InterceptPipelineConfig* pipeline_config) {
  if (pipeline_config == nullptr) {
    return;
  }
  if (detection.use_profile_defaults) {
    switch (detection.profile) {
      case config::EsrDetectionProfile::kConservative:
        pipeline_config->detection.min_detect_snr_db = kConservativeDetectSnrDb;
        break;
      case config::EsrDetectionProfile::kSensitive:
        pipeline_config->detection.min_detect_snr_db = kSensitiveDetectSnrDb;
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

void ApplyEnvironmentConfig(const config::EsrEnvironmentConfig& env_config,
                            environment::EsrEnvironmentModelConfig* model_config) {
  if (model_config == nullptr) {
    return;
  }
  *model_config = environment::BuildModelConfigFromScenario(env_config.scenario_config);
}

}  // namespace

ResolvedEsrSessionConfig ResolveEsrSessionConfig(const EsrSessionConfig& session_config) {
  ResolvedEsrSessionConfig resolved;
  const config::EsrHardwareConfig& hardware = session_config.hardware;
  const config::EsrMissionConfig& mission = session_config.mission;
  const config::EsrScanPolicyConfig& scan_policy = session_config.mission.scan;

  resolved.runtime_config.sensor_enabled = mission.power_on;
  resolved.runtime_config.antenna_mount_az_deg =
      oneq::internal::validation::IsFinite(hardware.antenna_mount_az_deg) ? hardware.antenna_mount_az_deg : 0.0f;
  resolved.runtime_config.antenna_mount_el_deg =
      oneq::internal::validation::IsFinite(hardware.antenna_mount_el_deg) ? hardware.antenna_mount_el_deg : 0.0f;
  resolved.runtime_config.integrated_receive_loss_db =
      (oneq::internal::validation::IsFinite(hardware.integrated_receive_loss_db) && hardware.integrated_receive_loss_db > 0.0f)
          ? hardware.integrated_receive_loss_db
          : 0.0f;
  resolved.runtime_config.scan_rate_hz =
      (oneq::internal::validation::IsFinite(scan_policy.scan_rate_hz) && scan_policy.scan_rate_hz > 0.0f)
          ? scan_policy.scan_rate_hz
          : 1.0f;

  if (oneq::internal::validation::IsFinite(hardware.receiver_band_lower_hz) && oneq::internal::validation::IsFinite(hardware.receiver_band_upper_hz) &&
      hardware.receiver_band_upper_hz > hardware.receiver_band_lower_hz) {
    resolved.runtime_config.use_fixed_receiver_window = true;
    resolved.runtime_config.receiver_lower_hz = hardware.receiver_band_lower_hz;
    resolved.runtime_config.receiver_upper_hz = hardware.receiver_band_upper_hz;
  }

  if (oneq::internal::validation::IsFinite(hardware.receiver_sensitivity_w) && hardware.receiver_sensitivity_w > 0.0f) {
    resolved.pipeline_config.detection.receiver_noise_floor_w = hardware.receiver_sensitivity_w;
  }

  ApplyDetectionPolicy(session_config.policy.detection, &resolved.pipeline_config);
  ApplyEnvironmentConfig(session_config.environment, &resolved.environment_model_config);
  ResolveScanPolicy(hardware, scan_policy, resolved.runtime_config, &resolved.pipeline_config.scan);
  ApplyWorkModeAdjustment(mission.work_mode, &resolved.pipeline_config.statistical_detection);
  return resolved;
}

}  // namespace internal
}  // namespace session

}  // namespace electronic_surveillance_radar
