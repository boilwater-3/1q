/**
 * @file ScanScheduleResolver.h
 * @brief 定义机载雷达二维扫描调度解析工具。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/common/utils/RadarOrientationUtils.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"

namespace airborne_radar {
namespace signal {
namespace runtime {

using pipeline::SignalPipelineConfig;

namespace internal {

/**
 * @brief 归一化基准指向，确保输出有限值。
 * @note 这里的 `scan_center_deg` 表示波束基准指向，不是扫描体积中心。
 * @param orientation_config 雷达方向配置。
 * @return 有限值基准指向。
 */
inline common::config::AzimuthElevationDeg ResolveFiniteScanCenter(
    const common::config::RadarOrientationConfig& orientation_config) {
  common::config::AzimuthElevationDeg center = orientation_config.scan_center_deg;
  if (std::isfinite(center.az_deg) == 0) {
    center.az_deg = 0.0f;
  }
  if (std::isfinite(center.el_deg) == 0) {
    center.el_deg = 0.0f;
  }
  return center;
}

/**
 * @brief 解析工作子模式对应的扫描步进倍率。
 * @param mode 工作子模式。
 * @return 扫描步进倍率。
 */
inline float ResolveScanStepScale(common::config::RadarWorkSubMode mode) {
  switch (mode) {
    case common::config::RadarWorkSubMode::kTas:
      return 0.5f;
    case common::config::RadarWorkSubMode::kTws:
      return 1.0f;
    case common::config::RadarWorkSubMode::kStby:
    case common::config::RadarWorkSubMode::kStt:
    default:
      return 1.0f;
  }
}

/**
 * @brief 依据扫描限位与调度配置生成二维扫描序列。
 * @param limits 扫描限位。
 * @param az_step_deg 方位步进（单位：deg）。
 * @param el_step_deg 俯仰步进（单位：deg）。
 * @param start_position 扫描起始象限。
 * @param sequence 扫描推进顺序。
 * @return 扫描序列；若输入无效则返回空序列。
 */
inline std::vector<common::config::AzimuthElevationDeg> BuildScheduledScanPattern(
    const common::config::AzimuthElevationLimitsDeg& limits, float az_step_deg, float el_step_deg,
    oneq::common::ScanStartPosition start_position, oneq::common::ScanSequence sequence) {
  const bool finite_limits = std::isfinite(limits.az_min_deg) != 0 && std::isfinite(limits.az_max_deg) != 0 &&
                             std::isfinite(limits.el_min_deg) != 0 && std::isfinite(limits.el_max_deg) != 0;
  if (!finite_limits || limits.az_min_deg > limits.az_max_deg || limits.el_min_deg > limits.el_max_deg ||
      std::isfinite(az_step_deg) == 0 || std::isfinite(el_step_deg) == 0 || az_step_deg <= 0.0f ||
      el_step_deg <= 0.0f) {
    return std::vector<common::config::AzimuthElevationDeg>();
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
  if (std::fabs(az_values.back() - limits.az_max_deg) > 1.0e-4f && az_values.size() < kMaxAxisSamples) {
    az_values.push_back(limits.az_max_deg);
  }
  if (std::fabs(el_values.back() - limits.el_max_deg) > 1.0e-4f && el_values.size() < kMaxAxisSamples) {
    el_values.push_back(limits.el_max_deg);
  }

  const bool start_from_right = start_position == oneq::common::ScanStartPosition::kRightTop ||
                                start_position == oneq::common::ScanStartPosition::kRightBottom;
  const bool start_from_bottom = start_position == oneq::common::ScanStartPosition::kRightBottom ||
                                 start_position == oneq::common::ScanStartPosition::kLeftBottom;
  if (start_from_right) {
    std::reverse(az_values.begin(), az_values.end());
  }
  if (!start_from_bottom) {
    std::reverse(el_values.begin(), el_values.end());
  }

  std::vector<common::config::AzimuthElevationDeg> pattern;
  pattern.reserve(az_values.size() * el_values.size());
  if (sequence == oneq::common::ScanSequence::kAzimuthFirst) {
    for (std::size_t el_index = 0; el_index < el_values.size(); ++el_index) {
      const bool reverse_row = (el_index % 2U) == 1U;
      for (std::size_t az_order = 0; az_order < az_values.size(); ++az_order) {
        const std::size_t az_index = reverse_row ? (az_values.size() - 1U - az_order) : az_order;
        common::config::AzimuthElevationDeg pointing;
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
      common::config::AzimuthElevationDeg pointing;
      pointing.az_deg = az_values[az_index];
      pointing.el_deg = el_values[el_index];
      pattern.push_back(pointing);
    }
  }
  return pattern;
}

/**
 * @brief 解析当前周期扫描格点（挂架坐标系绝对方位/俯仰）。
 * @param orientation_config 雷达方向配置。
 * @param effective_beamwidth_deg 当前周期有效波束宽度（单位：deg）。
 * @param cycle_index 周期号（第 1 周期映射到序列第 0 点）。
 * @return 当前周期扫描格点；若参数非法则退化到扫描中心。
 */
inline common::config::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const common::config::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index) {
  common::config::AzimuthElevationLimitsDeg effective_limits = common::utils::IntersectScanLimits(
      orientation_config.mechanical_scan_limits_deg, orientation_config.electronic_scan_limits_deg);
  const bool limits_valid = std::isfinite(effective_limits.az_min_deg) != 0 &&
                            std::isfinite(effective_limits.az_max_deg) != 0 &&
                            std::isfinite(effective_limits.el_min_deg) != 0 &&
                            std::isfinite(effective_limits.el_max_deg) != 0 &&
                            effective_limits.az_min_deg <= effective_limits.az_max_deg &&
                            effective_limits.el_min_deg <= effective_limits.el_max_deg;

  const common::config::AzimuthElevationDeg normalized_scan_center =
      ResolveFiniteScanCenter(orientation_config);
  common::config::AzimuthElevationDeg fallback_center = normalized_scan_center;
  if (limits_valid) {
    fallback_center = common::utils::ClampAzimuthElevation(fallback_center, effective_limits);
  }

  if (orientation_config.work_sub_mode == common::config::RadarWorkSubMode::kStby) {
    common::config::AzimuthElevationDeg boresight;
    if (limits_valid) {
      boresight = common::utils::ClampAzimuthElevation(boresight, effective_limits);
    }
    return boresight;
  }

  if (orientation_config.work_sub_mode == common::config::RadarWorkSubMode::kStt) {
    return normalized_scan_center;
  }

  if (!limits_valid || std::isfinite(effective_beamwidth_deg.az_beamwidth_deg) == 0 ||
      std::isfinite(effective_beamwidth_deg.el_beamwidth_deg) == 0 ||
      effective_beamwidth_deg.az_beamwidth_deg <= 0.0f || effective_beamwidth_deg.el_beamwidth_deg <= 0.0f) {
    return fallback_center;
  }

  const float step_scale = ResolveScanStepScale(orientation_config.work_sub_mode);
  const std::vector<common::config::AzimuthElevationDeg> pattern = BuildScheduledScanPattern(
      effective_limits, effective_beamwidth_deg.az_beamwidth_deg * step_scale,
      effective_beamwidth_deg.el_beamwidth_deg * step_scale, orientation_config.scan_start_position,
      orientation_config.scan_sequence);
  if (pattern.empty()) {
    return fallback_center;
  }

  const std::uint64_t zero_based_cycle = cycle_index > 0U ? static_cast<std::uint64_t>(cycle_index - 1U) : 0U;
  return pattern[static_cast<std::size_t>(zero_based_cycle % static_cast<std::uint64_t>(pattern.size()))];
}

/**
 * @brief 将扫描格点转换为运行期 `dwell_center_deg` 偏置。
 * @param orientation_config 雷达方向配置。
 * @param effective_beamwidth_deg 当前周期有效波束宽度（单位：deg）。
 * @param cycle_index 周期号（第 1 周期映射到序列第 0 点）。
 * @return 运行期 dwell 偏置。
 */
inline common::config::AzimuthElevationDeg ResolveScheduledDwellCenter(
    const common::config::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index) {
  if (orientation_config.work_sub_mode == common::config::RadarWorkSubMode::kStt) {
    return common::config::AzimuthElevationDeg();
  }
  const common::config::AzimuthElevationDeg pointing =
      ResolveScheduledBeamPointing(orientation_config, effective_beamwidth_deg, cycle_index);
  const common::config::AzimuthElevationDeg normalized_scan_center =
      ResolveFiniteScanCenter(orientation_config);
  common::config::AzimuthElevationDeg dwell;
  dwell.az_deg = pointing.az_deg - normalized_scan_center.az_deg;
  dwell.el_deg = pointing.el_deg - normalized_scan_center.el_deg;
  return dwell;
}

/**
 * @brief 按周期扫描调度更新运行时 `dwell_center_deg`。
 * @param cycle_index 周期号（第 1 周期映射到序列第 0 点）。
 * @param runtime_config 运行时配置。
 */
inline void ApplyScanScheduleToRuntimeConfig(std::uint32_t cycle_index,
                                             SignalPipelineConfig* runtime_config) {
  if (runtime_config == nullptr) {
    return;
  }
  common::config::RadarOrientationConfig& orientation = runtime_config->beam_control.radar_orientation;
  const detection::EffectiveBeamwidthDeg effective_beamwidth = detection::ResolveEffectiveBeamwidth(
      runtime_config->detection.radar_system.antenna, orientation);
  orientation.dwell_center_deg =
      ResolveScheduledDwellCenter(orientation, effective_beamwidth, cycle_index);
}

}  // namespace internal
}  // namespace runtime
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
