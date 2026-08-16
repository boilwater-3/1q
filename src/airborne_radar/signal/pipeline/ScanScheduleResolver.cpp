#include "airborne_radar/signal/pipeline/ScanScheduleResolver.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "common/numerics/Constants.h"
#include "common/radar/ScanScheduleRuntime.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

bool IsFinitePositive(float value) { return std::isfinite(value) && value > 0.0f; }

detection::EffectiveBeamwidthDeg ResolveSchedulingBeamwidth(const ExecutionConfig& runtime_config) {
  detection::EffectiveBeamwidthDeg beamwidth;
  const config::CommandedBeamwidthDeg& expert_nominal =
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

  beamwidth.az_beamwidth_deg = runtime_config.detection.engineering.antenna.nominal_az_beamwidth_deg;
  beamwidth.el_beamwidth_deg = runtime_config.detection.engineering.antenna.nominal_el_beamwidth_deg;
  return beamwidth;
}

}  // namespace

config::AzimuthElevationDeg ResolveFiniteScanCenter(
    const config::ArOrientationConfig& orientation_config) {
  config::AzimuthElevationDeg center = orientation_config.scan_center_deg;
  if (!std::isfinite(center.az_deg)) {
    center.az_deg = 0.0f;
  }
  if (!std::isfinite(center.el_deg)) {
    center.el_deg = 0.0f;
  }
  return center;
}

float ResolveScanStepScale(config::ArWorkMode mode) {
  switch (mode) {
    case config::ArWorkMode::kTas:
      return 0.5f;
    case config::ArWorkMode::kTws:
      return 1.0f;
    case config::ArWorkMode::kStby:
    case config::ArWorkMode::kStt:
    default:
      return 1.0f;
  }
}

std::vector<config::AzimuthElevationDeg> BuildScheduledScanPattern(
    const config::AzimuthElevationLimitsDeg& limits, float az_step_deg, float el_step_deg,
    oneq::foundation::ScanStartPosition start_position, oneq::foundation::ScanSequence sequence) {
  // 模式构建为 common 单源（ScanScheduleRuntime.h，AR/RIR 共用同一扫描策略）。
  const std::vector<oneq::common::radar::AzimuthElevationDeg> common_pattern =
      oneq::common::radar::BuildScanPattern(limits.az_min_deg, limits.az_max_deg,
                                            limits.el_min_deg, limits.el_max_deg, az_step_deg,
                                            el_step_deg, start_position, sequence);
  std::vector<config::AzimuthElevationDeg> pattern;
  pattern.reserve(common_pattern.size());
  for (const oneq::common::radar::AzimuthElevationDeg& wave_position : common_pattern) {
    pattern.push_back(config::AzimuthElevationDeg{wave_position.az_deg, wave_position.el_deg});
  }
  return pattern;
}

config::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const config::ArOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg,
    const config::BeamSchedulerConfig& scheduler_config, std::uint32_t cycle_index) {
  config::AzimuthElevationLimitsDeg effective_limits = utils::IntersectScanLimits(
      orientation_config.mechanical_scan_limits_deg, orientation_config.electronic_scan_limits_deg);
  const bool limits_valid =
      std::isfinite(effective_limits.az_min_deg) && std::isfinite(effective_limits.az_max_deg) &&
      std::isfinite(effective_limits.el_min_deg) && std::isfinite(effective_limits.el_max_deg) &&
      effective_limits.az_min_deg <= effective_limits.az_max_deg &&
      effective_limits.el_min_deg <= effective_limits.el_max_deg;

  const config::AzimuthElevationDeg normalized_scan_center =
      ResolveFiniteScanCenter(orientation_config);
  config::AzimuthElevationDeg fallback_center = normalized_scan_center;
  if (limits_valid) {
    fallback_center = utils::ClampAzimuthElevation(fallback_center, effective_limits);
  }

  if (orientation_config.work_mode == config::ArWorkMode::kStby) {
    config::AzimuthElevationDeg boresight;
    if (limits_valid) {
      boresight = utils::ClampAzimuthElevation(boresight, effective_limits);
    }
    return boresight;
  }

  if (orientation_config.work_mode == config::ArWorkMode::kStt) {
    return normalized_scan_center;
  }

  if (!limits_valid || !std::isfinite(effective_beamwidth_deg.az_beamwidth_deg) ||
      !std::isfinite(effective_beamwidth_deg.el_beamwidth_deg) ||
      effective_beamwidth_deg.az_beamwidth_deg <= 0.0f ||
      effective_beamwidth_deg.el_beamwidth_deg <= 0.0f) {
    return fallback_center;
  }

  float step_scale = ResolveScanStepScale(orientation_config.work_mode);
  if (orientation_config.work_mode == config::ArWorkMode::kTas &&
      scheduler_config.prefer_dense_tas_sampling) {
    step_scale *= 0.5f;
  }
  const float default_az_step_deg = effective_beamwidth_deg.az_beamwidth_deg * step_scale;
  const float default_el_step_deg = effective_beamwidth_deg.el_beamwidth_deg * step_scale;
  const float az_step_deg =
      oneq::common::radar::ResolveAxisStepDeg(effective_limits.az_min_deg,
                                              effective_limits.az_max_deg, default_az_step_deg,
                                              scheduler_config.azimuth_step_count_hint);
  const float el_step_deg =
      oneq::common::radar::ResolveAxisStepDeg(effective_limits.el_min_deg,
                                              effective_limits.el_max_deg, default_el_step_deg,
                                              scheduler_config.elevation_step_count_hint);
  const std::vector<config::AzimuthElevationDeg> pattern = BuildScheduledScanPattern(
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

config::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const config::ArOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index) {
  return ResolveScheduledBeamPointing(orientation_config, effective_beamwidth_deg,
                                      config::BeamSchedulerConfig(), cycle_index);
}

config::AzimuthElevationDeg ResolveScheduledDwellCenter(
    const config::ArOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index) {
  if (orientation_config.work_mode == config::ArWorkMode::kStt) {
    return config::AzimuthElevationDeg();
  }
  const config::AzimuthElevationDeg pointing =
      ResolveScheduledBeamPointing(orientation_config, effective_beamwidth_deg, cycle_index);
  const config::AzimuthElevationDeg normalized_scan_center =
      ResolveFiniteScanCenter(orientation_config);
  config::AzimuthElevationDeg dwell;
  dwell.az_deg = pointing.az_deg - normalized_scan_center.az_deg;
  dwell.el_deg = pointing.el_deg - normalized_scan_center.el_deg;
  return dwell;
}

config::AzimuthElevationDeg ResolveScheduledBeamPointingFromExecutionConfig(
    const ExecutionConfig& runtime_config, std::uint32_t cycle_index) {
  return ResolveScheduledBeamPointingFromExecutionConfig(
      runtime_config, cycle_index, runtime_config.detection.orientation.work_mode);
}

config::AzimuthElevationDeg ResolveScheduledBeamPointingFromExecutionConfig(
    const ExecutionConfig& runtime_config, std::uint32_t cycle_index,
    config::ArWorkMode effective_work_mode) {
  config::ArOrientationConfig orientation = runtime_config.detection.orientation;
  orientation.work_mode = effective_work_mode;
  return ResolveScheduledBeamPointing(
      orientation, ResolveSchedulingBeamwidth(runtime_config),
      runtime_config.detection.beam_control.scheduler, cycle_index);
}

void ApplyScanScheduleToRuntimeConfig(std::uint32_t cycle_index,
                                      ExecutionConfig* runtime_config) {
  if (runtime_config == nullptr) {
    return;
  }
  runtime_config->detection.orientation.scan_center_deg =
      ResolveScheduledBeamPointingFromExecutionConfig(*runtime_config, cycle_index);
}

bool TryTrackPositionToLookAnglesDeg(float position_x, float position_y, float position_z,
                                     config::AzimuthElevationDeg* pointing) {
  if (pointing == nullptr || !std::isfinite(position_x) || !std::isfinite(position_y) ||
      !std::isfinite(position_z)) {
    return false;
  }
  const float range_hypot = std::sqrt(position_x * position_x + position_y * position_y);
  const float range = std::sqrt(range_hypot * range_hypot + position_z * position_z);
  if (range <= 0.0f) {
    return false;
  }
  pointing->az_deg =
      static_cast<float>(oneq::common::numerics::RadToDeg(std::atan2(position_y, position_x)));
  pointing->el_deg =
      static_cast<float>(oneq::common::numerics::RadToDeg(std::atan2(position_z, range_hypot)));
  return true;
}

SttTrackFollowingResolution ResolveSttTrackFollowingPointing(
    const config::ArOrientationConfig& orientation_config,
    const config::AzimuthElevationDeg& dwell_center_deg, bool has_designated_target,
    bool designated_track_confirmed, const config::AzimuthElevationDeg& track_pointing_deg) {
  SttTrackFollowingResolution resolved;
  const bool explicit_dwell = dwell_center_deg.az_deg != 0.0f || dwell_center_deg.el_deg != 0.0f;
  const bool track_following =
      orientation_config.work_mode == config::ArWorkMode::kStt && !explicit_dwell &&
      has_designated_target && designated_track_confirmed &&
      std::isfinite(track_pointing_deg.az_deg) && std::isfinite(track_pointing_deg.el_deg);
  if (track_following) {
    // 优先级 2：指定航迹 confirmed → 指向跟随航迹，dwell 视为零偏移。
    resolved.track_following_active = true;
    resolved.scan_center_deg = track_pointing_deg;
    resolved.dwell_center_deg = config::AzimuthElevationDeg();
    return resolved;
  }
  // 优先级 1（显式 dwell）与 3（回退 scan_center）共用：扫描中心保持配置值，
  // dwell 仅显式非零时透传（现状语义 scan_center + dwell）。
  resolved.scan_center_deg = orientation_config.scan_center_deg;
  resolved.dwell_center_deg = explicit_dwell ? dwell_center_deg : config::AzimuthElevationDeg();
  return resolved;
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
