#include "airborne_radar/signal/pipeline/ScanScheduleResolver.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

bool IsFinitePositive(float value) { return std::isfinite(value) && value > 0.0f; }

detection::EffectiveBeamwidthDeg ResolveSchedulingBeamwidth(const ExecutionConfig& runtime_config) {
  detection::EffectiveBeamwidthDeg beamwidth;
  const model::CommandedBeamwidthDeg& expert_nominal =
      runtime_config.detection.beam_control.pointing.nominal_beamwidth_deg;
  if (IsFinitePositive(expert_nominal.commanded_az_beamwidth_deg) &&
      IsFinitePositive(expert_nominal.commanded_el_beamwidth_deg)) {
    beamwidth.az_beamwidth_deg = expert_nominal.commanded_az_beamwidth_deg;
    beamwidth.el_beamwidth_deg = expert_nominal.commanded_el_beamwidth_deg;
    return beamwidth;
  }

  if (runtime_config.detection.orientation.commanded_beamwidth_enabled &&
      IsFinitePositive(
          runtime_config.detection.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg) &&
      IsFinitePositive(
          runtime_config.detection.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg)) {
    beamwidth.az_beamwidth_deg =
        runtime_config.detection.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg;
    beamwidth.el_beamwidth_deg =
        runtime_config.detection.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg;
    return beamwidth;
  }

  beamwidth.az_beamwidth_deg = runtime_config.detection.hardware.antenna.nominal_az_beamwidth_deg;
  beamwidth.el_beamwidth_deg = runtime_config.detection.hardware.antenna.nominal_el_beamwidth_deg;
  return beamwidth;
}

float ResolveAxisStepDeg(float min_deg, float max_deg, float default_step_deg,
                         std::uint32_t step_count_hint) {
  if (step_count_hint <= 1U) {
    return default_step_deg;
  }
  const float span_deg = std::max(0.0f, max_deg - min_deg);
  const float hint_step_deg = span_deg / static_cast<float>(step_count_hint - 1U);
  return IsFinitePositive(hint_step_deg) ? hint_step_deg : default_step_deg;
}

}  // namespace

model::AzimuthElevationDeg ResolveFiniteScanCenter(
    const model::RadarOrientationConfig& orientation_config) {
  model::AzimuthElevationDeg center = orientation_config.scan_center_deg;
  if (!std::isfinite(center.az_deg)) {
    center.az_deg = 0.0f;
  }
  if (!std::isfinite(center.el_deg)) {
    center.el_deg = 0.0f;
  }
  return center;
}

float ResolveScanStepScale(model::RadarWorkSubMode mode) {
  switch (mode) {
    case model::RadarWorkSubMode::kTas:
      return 0.5f;
    case model::RadarWorkSubMode::kTws:
      return 1.0f;
    case model::RadarWorkSubMode::kStby:
    case model::RadarWorkSubMode::kStt:
    default:
      return 1.0f;
  }
}

std::vector<model::AzimuthElevationDeg> BuildScheduledScanPattern(
    const model::AzimuthElevationLimitsDeg& limits, float az_step_deg, float el_step_deg,
    oneq::foundation::ScanStartPosition start_position, oneq::foundation::ScanSequence sequence) {
  const bool finite_limits = std::isfinite(limits.az_min_deg) && std::isfinite(limits.az_max_deg) &&
                             std::isfinite(limits.el_min_deg) && std::isfinite(limits.el_max_deg);
  if (!finite_limits || limits.az_min_deg > limits.az_max_deg ||
      limits.el_min_deg > limits.el_max_deg || !std::isfinite(az_step_deg) ||
      !std::isfinite(el_step_deg) || az_step_deg <= 0.0f || el_step_deg <= 0.0f) {
    return std::vector<model::AzimuthElevationDeg>();
  }

  const std::size_t kMaxAxisSamples = 4096U;
  std::vector<float> az_values;
  std::vector<float> el_values;
  for (float az = limits.az_min_deg;
       az <= limits.az_max_deg + 0.5f * az_step_deg && az_values.size() < kMaxAxisSamples;
       az += az_step_deg) {
    const float clamped = az > limits.az_max_deg ? limits.az_max_deg : az;
    if (az_values.empty() || std::fabs(az_values.back() - clamped) > 1.0e-4f) {
      az_values.push_back(clamped);
    }
  }
  for (float el = limits.el_min_deg;
       el <= limits.el_max_deg + 0.5f * el_step_deg && el_values.size() < kMaxAxisSamples;
       el += el_step_deg) {
    const float clamped = el > limits.el_max_deg ? limits.el_max_deg : el;
    if (el_values.empty() || std::fabs(el_values.back() - clamped) > 1.0e-4f) {
      el_values.push_back(clamped);
    }
  }
  if (az_values.empty()) {
    az_values.push_back(limits.az_min_deg);
  }
  if (el_values.empty()) {
    el_values.push_back(limits.el_min_deg);
  }
  if (std::fabs(az_values.back() - limits.az_max_deg) > 1.0e-4f &&
      az_values.size() < kMaxAxisSamples) {
    az_values.push_back(limits.az_max_deg);
  }
  if (std::fabs(el_values.back() - limits.el_max_deg) > 1.0e-4f &&
      el_values.size() < kMaxAxisSamples) {
    el_values.push_back(limits.el_max_deg);
  }

  const bool start_from_right = start_position == oneq::foundation::ScanStartPosition::kRightTop ||
                                start_position == oneq::foundation::ScanStartPosition::kRightBottom;
  const bool start_from_bottom =
      start_position == oneq::foundation::ScanStartPosition::kRightBottom ||
      start_position == oneq::foundation::ScanStartPosition::kLeftBottom;
  if (start_from_right) {
    std::reverse(az_values.begin(), az_values.end());
  }
  if (!start_from_bottom) {
    std::reverse(el_values.begin(), el_values.end());
  }

  std::vector<model::AzimuthElevationDeg> pattern;
  pattern.reserve(az_values.size() * el_values.size());
  if (sequence == oneq::foundation::ScanSequence::kAzimuthFirst) {
    for (std::size_t el_index = 0; el_index < el_values.size(); ++el_index) {
      const bool reverse_row = (el_index % 2U) == 1U;
      for (std::size_t az_order = 0; az_order < az_values.size(); ++az_order) {
        const std::size_t az_index = reverse_row ? (az_values.size() - 1U - az_order) : az_order;
        model::AzimuthElevationDeg pointing;
        pointing.az_deg = az_values[az_index];
        pointing.el_deg = el_values[el_index];
        pattern.push_back(pointing);
      }
    }
    return pattern;
  }

  for (std::size_t az_index = 0; az_index < az_values.size(); ++az_index) {
    const bool reverse_column = (az_index % 2U) == 1U;
    for (std::size_t el_order = 0; el_order < el_values.size(); ++el_order) {
      const std::size_t el_index = reverse_column ? (el_values.size() - 1U - el_order) : el_order;
      model::AzimuthElevationDeg pointing;
      pointing.az_deg = az_values[az_index];
      pointing.el_deg = el_values[el_index];
      pattern.push_back(pointing);
    }
  }
  return pattern;
}

model::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const model::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg,
    const config::BeamSchedulerConfig& scheduler_config, std::uint32_t cycle_index) {
  model::AzimuthElevationLimitsDeg effective_limits = utils::IntersectScanLimits(
      orientation_config.mechanical_scan_limits_deg, orientation_config.electronic_scan_limits_deg);
  const bool limits_valid =
      std::isfinite(effective_limits.az_min_deg) && std::isfinite(effective_limits.az_max_deg) &&
      std::isfinite(effective_limits.el_min_deg) && std::isfinite(effective_limits.el_max_deg) &&
      effective_limits.az_min_deg <= effective_limits.az_max_deg &&
      effective_limits.el_min_deg <= effective_limits.el_max_deg;

  const model::AzimuthElevationDeg normalized_scan_center =
      ResolveFiniteScanCenter(orientation_config);
  model::AzimuthElevationDeg fallback_center = normalized_scan_center;
  if (limits_valid) {
    fallback_center = utils::ClampAzimuthElevation(fallback_center, effective_limits);
  }

  if (orientation_config.work_sub_mode == model::RadarWorkSubMode::kStby) {
    model::AzimuthElevationDeg boresight;
    if (limits_valid) {
      boresight = utils::ClampAzimuthElevation(boresight, effective_limits);
    }
    return boresight;
  }

  if (orientation_config.work_sub_mode == model::RadarWorkSubMode::kStt) {
    return normalized_scan_center;
  }

  if (!limits_valid || !std::isfinite(effective_beamwidth_deg.az_beamwidth_deg) ||
      !std::isfinite(effective_beamwidth_deg.el_beamwidth_deg) ||
      effective_beamwidth_deg.az_beamwidth_deg <= 0.0f ||
      effective_beamwidth_deg.el_beamwidth_deg <= 0.0f) {
    return fallback_center;
  }

  float step_scale = ResolveScanStepScale(orientation_config.work_sub_mode);
  if (orientation_config.work_sub_mode == model::RadarWorkSubMode::kTas &&
      scheduler_config.prefer_dense_tas_sampling) {
    step_scale *= 0.5f;
  }
  const float default_az_step_deg = effective_beamwidth_deg.az_beamwidth_deg * step_scale;
  const float default_el_step_deg = effective_beamwidth_deg.el_beamwidth_deg * step_scale;
  const float az_step_deg =
      ResolveAxisStepDeg(effective_limits.az_min_deg, effective_limits.az_max_deg,
                         default_az_step_deg, scheduler_config.azimuth_step_count_hint);
  const float el_step_deg =
      ResolveAxisStepDeg(effective_limits.el_min_deg, effective_limits.el_max_deg,
                         default_el_step_deg, scheduler_config.elevation_step_count_hint);
  const std::vector<model::AzimuthElevationDeg> pattern = BuildScheduledScanPattern(
      effective_limits, az_step_deg, el_step_deg, orientation_config.scan_start_position,
      orientation_config.scan_sequence);
  if (pattern.empty()) {
    return fallback_center;
  }

  const std::uint64_t zero_based_cycle =
      cycle_index > 0U ? static_cast<std::uint64_t>(cycle_index - 1U) : 0U;
  return pattern[static_cast<std::size_t>(zero_based_cycle %
                                          static_cast<std::uint64_t>(pattern.size()))];
}

model::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const model::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index) {
  return ResolveScheduledBeamPointing(orientation_config, effective_beamwidth_deg,
                                      config::BeamSchedulerConfig(), cycle_index);
}

model::AzimuthElevationDeg ResolveScheduledDwellCenter(
    const model::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index) {
  if (orientation_config.work_sub_mode == model::RadarWorkSubMode::kStt) {
    return model::AzimuthElevationDeg();
  }
  const model::AzimuthElevationDeg pointing =
      ResolveScheduledBeamPointing(orientation_config, effective_beamwidth_deg, cycle_index);
  const model::AzimuthElevationDeg normalized_scan_center =
      ResolveFiniteScanCenter(orientation_config);
  model::AzimuthElevationDeg dwell;
  dwell.az_deg = pointing.az_deg - normalized_scan_center.az_deg;
  dwell.el_deg = pointing.el_deg - normalized_scan_center.el_deg;
  return dwell;
}

void ApplyScanScheduleToRuntimeConfig(std::uint32_t cycle_index,
                                      ExecutionConfig* runtime_config) {
  if (runtime_config == nullptr) {
    return;
  }

  runtime_config->detection.orientation.scan_center_deg = ResolveScheduledBeamPointing(
      runtime_config->detection.orientation, ResolveSchedulingBeamwidth(*runtime_config),
      runtime_config->detection.beam_control.scheduler, cycle_index);
}



}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
