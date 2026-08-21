/**
 * @file EsrResolutionRules.cpp
 * @brief ESR 配置解析规则的单一实现（见 EsrResolutionRules.h）。
 */

#include "electronic_surveillance_radar/session/EsrResolutionRules.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "common/validation/ValidationUtils.h"

namespace electronic_surveillance_radar {
namespace session {
namespace resolution_rules {

namespace {

constexpr std::uint32_t kActiveScanPulseMultiplier = 4U;
constexpr std::uint32_t kMaxPulseCount = 4096U;
constexpr float kMinimumThresholdScale = 0.1f;
constexpr float kHgesmThresholdScale = 0.85f;
constexpr float kRwrThresholdScale = 1.25f;

}  // namespace

void NormalizeScanBounds(float* start, float* end) {
  if (start == nullptr || end == nullptr) {
    return;
  }
  if (*start > *end) {
    std::swap(*start, *end);
  }
}

void ApplyWorkModeAdjustment(config::EsrWorkMode mode, DetectionConfig* detection_config) {
  if (detection_config == nullptr) {
    return;
  }
  detection_config->pulse_count = std::max<std::uint32_t>(1U, detection_config->pulse_count);
  detection_config->threshold_scale =
      oneq::common::validation::IsFinite(detection_config->threshold_scale) &&
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
                     const config::EsrOrientationConfig& orientation,
                     const config::EsrScanPolicyConfig& scan_policy,
                     extension::InterceptScanConfig* scan_config) {
  if (scan_config == nullptr) {
    return;
  }

  const float mount_az = oneq::common::validation::IsFinite(orientation.antenna_mount_az_deg)
                             ? orientation.antenna_mount_az_deg
                             : 0.0f;
  const float mount_el = oneq::common::validation::IsFinite(orientation.antenna_mount_el_deg)
                             ? orientation.antenna_mount_el_deg
                             : 0.0f;
  scan_config->scan_start_pos = static_cast<int>(scan_policy.scan_start_position);
  scan_config->scan_sequence = static_cast<int>(scan_policy.scan_sequence);

  if (oneq::common::validation::IsFinite(hardware.beam_az_width_deg) && hardware.beam_az_width_deg > 0.0f) {
    scan_config->az_step_deg = hardware.beam_az_width_deg;
  }
  if (oneq::common::validation::IsFinite(hardware.beam_el_width_deg) && hardware.beam_el_width_deg > 0.0f) {
    scan_config->el_step_deg = hardware.beam_el_width_deg;
  }

  const bool explicit_bounds_valid =
      scan_policy.use_explicit_scan_bounds && oneq::common::validation::IsFinite(scan_policy.scan_start_az_deg) &&
      oneq::common::validation::IsFinite(scan_policy.scan_end_az_deg) && oneq::common::validation::IsFinite(scan_policy.scan_start_el_deg) &&
      oneq::common::validation::IsFinite(scan_policy.scan_end_el_deg);
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

  const bool has_center_az = oneq::common::validation::IsFinite(scan_policy.scan_center_az_deg);
  const bool has_center_el = oneq::common::validation::IsFinite(scan_policy.scan_center_el_deg);
  if (has_center_az) {
    float half_az_span =
        0.5f * std::fabs(scan_config->scan_end_az_deg - scan_config->scan_start_az_deg);
    if (oneq::common::validation::IsFinite(hardware.az_scan_range_deg) && hardware.az_scan_range_deg > 0.0f) {
      half_az_span = 0.5f * hardware.az_scan_range_deg;
    }
    const float center_az = scan_policy.scan_center_az_deg - mount_az;
    scan_config->scan_start_az_deg = center_az - half_az_span;
    scan_config->scan_end_az_deg = center_az + half_az_span;
  }
  if (has_center_el) {
    float half_el_span =
        0.5f * std::fabs(scan_config->scan_end_el_deg - scan_config->scan_start_el_deg);
    if (oneq::common::validation::IsFinite(hardware.el_scan_range_deg) && hardware.el_scan_range_deg > 0.0f) {
      half_el_span = 0.5f * hardware.el_scan_range_deg;
    }
    const float center_el = scan_policy.scan_center_el_deg - mount_el;
    scan_config->scan_start_el_deg = center_el - half_el_span;
    scan_config->scan_end_el_deg = center_el + half_el_span;
  }
  NormalizeScanBounds(&scan_config->scan_start_az_deg, &scan_config->scan_end_az_deg);
  NormalizeScanBounds(&scan_config->scan_start_el_deg, &scan_config->scan_end_el_deg);
}

}  // namespace resolution_rules
}  // namespace session
}  // namespace electronic_surveillance_radar
