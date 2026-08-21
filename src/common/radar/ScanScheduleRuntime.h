/**
 * @file ScanScheduleRuntime.h
 * @brief 二维扫描调度数值内核（common 单源）。
 *
 * 扫描模式构建与逐轴步长解析的纯几何内核，供 AR（ScanScheduleResolver）与
 * RIR（驻留调度器）共用同一扫描策略口径：限位/步长/起点/顺序 → 波位序列。
 * 模块侧只保留模式语义（如 STT/TWS/TAS）与指向消费接线，不复制本内核。
 */

#ifndef ONEQ_COMMON_RADAR_SCAN_SCHEDULE_RUNTIME_H_
#define ONEQ_COMMON_RADAR_SCAN_SCHEDULE_RUNTIME_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/foundation/scan_schedule_types.h"
#include "common/radar/AntennaPatternRuntime.h"

namespace oneq {
namespace common {
namespace radar {

/**
 * @brief 将绝对方位角折算到 (-180, 180]（deg）。
 * @param[in] az_deg 输入方位角（deg）。
 * @return 折算后的方位角；非有限输入原样返回。
 */
inline float NormalizeAzimuthDeg(float az_deg) {
  if (!std::isfinite(az_deg)) {
    return az_deg;
  }
  float normalized = std::remainder(az_deg, 360.0f);
  if (normalized <= -180.0f) {
    normalized += 360.0f;
  }
  return normalized;
}

/**
 * @brief 将方位角差折算到 [-180, 180]（deg）。
 * @param[in] delta_az_deg 输入方位角差（deg）。
 * @return 折算后的方位角差；非有限输入原样返回。
 */
inline float NormalizeAzimuthDeltaDeg(float delta_az_deg) {
  if (!std::isfinite(delta_az_deg)) {
    return delta_az_deg;
  }
  return std::remainder(delta_az_deg, 360.0f);
}

/**
 * @brief 由跨度与步进数提示解析轴步长。
 * @param[in] min_deg 轴最小值（度）。
 * @param[in] max_deg 轴最大值（度）。
 * @param[in] default_step_deg 默认步长（度）。
 * @param[in] step_count_hint 步进数提示（<=1 时使用默认步长）。
 * @return 轴步长（度）；hint 派生值非法（非有限/非正）时回退默认步长。
 */
inline float ResolveAxisStepDeg(float min_deg, float max_deg, float default_step_deg,
                                std::uint32_t step_count_hint) {
  if (step_count_hint <= 1U) {
    return default_step_deg;
  }
  const float span_deg = std::max(0.0f, max_deg - min_deg);
  const float hint_step_deg = span_deg / static_cast<float>(step_count_hint - 1U);
  return (std::isfinite(hint_step_deg) && hint_step_deg > 0.0f) ? hint_step_deg
                                                                : default_step_deg;
}

/**
 * @brief 按扫描范围、步长、起点与顺序构建二维扫描波位序列。
 * @param[in] az_min_deg 方位最小值（度）。
 * @param[in] az_max_deg 方位最大值（度）。
 * @param[in] el_min_deg 俯仰最小值（度）。
 * @param[in] el_max_deg 俯仰最大值（度）。
 * @param[in] az_step_deg 方位步长（度）。
 * @param[in] el_step_deg 俯仰步长（度）。
 * @param[in] start_position 扫描起点位置。
 * @param[in] sequence 扫描顺序（方位优先或俯仰优先，蛇形往返）。
 * @return 波位序列；范围或步长非法时返回空向量。
 * @note 每轴采样数上限为 4096，超出后截断。
 */
inline std::vector<AzimuthElevationDeg> BuildScanPattern(
    float az_min_deg, float az_max_deg, float el_min_deg, float el_max_deg, float az_step_deg,
    float el_step_deg, oneq::foundation::ScanStartPosition start_position,
    oneq::foundation::ScanSequence sequence) {
  const bool finite_limits = std::isfinite(az_min_deg) && std::isfinite(az_max_deg) &&
                             std::isfinite(el_min_deg) && std::isfinite(el_max_deg);
  if (!finite_limits || az_min_deg > az_max_deg || el_min_deg > el_max_deg ||
      !std::isfinite(az_step_deg) || !std::isfinite(el_step_deg) || az_step_deg <= 0.0f ||
      el_step_deg <= 0.0f) {
    return std::vector<AzimuthElevationDeg>();
  }

  const std::size_t kMaxAxisSamples = 4096U;
  std::vector<float> az_values;
  std::vector<float> el_values;
  for (float az = az_min_deg;
       az <= az_max_deg + 0.5f * az_step_deg && az_values.size() < kMaxAxisSamples;
       az += az_step_deg) {
    const float clamped = az > az_max_deg ? az_max_deg : az;
    if (az_values.empty() || std::fabs(az_values.back() - clamped) > 1.0e-4f) {
      az_values.push_back(clamped);
    }
  }
  for (float el = el_min_deg;
       el <= el_max_deg + 0.5f * el_step_deg && el_values.size() < kMaxAxisSamples;
       el += el_step_deg) {
    const float clamped = el > el_max_deg ? el_max_deg : el;
    if (el_values.empty() || std::fabs(el_values.back() - clamped) > 1.0e-4f) {
      el_values.push_back(clamped);
    }
  }
  if (az_values.empty()) {
    az_values.push_back(az_min_deg);
  }
  if (el_values.empty()) {
    el_values.push_back(el_min_deg);
  }
  if (std::fabs(az_values.back() - az_max_deg) > 1.0e-4f && az_values.size() < kMaxAxisSamples) {
    az_values.push_back(az_max_deg);
  }
  if (std::fabs(el_values.back() - el_max_deg) > 1.0e-4f && el_values.size() < kMaxAxisSamples) {
    el_values.push_back(el_max_deg);
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

  std::vector<AzimuthElevationDeg> pattern;
  pattern.reserve(az_values.size() * el_values.size());
  if (sequence == oneq::foundation::ScanSequence::kAzimuthFirst) {
    for (std::size_t el_index = 0; el_index < el_values.size(); ++el_index) {
      const bool reverse_row = (el_index % 2U) == 1U;
      for (std::size_t az_order = 0; az_order < az_values.size(); ++az_order) {
        const std::size_t az_index = reverse_row ? (az_values.size() - 1U - az_order) : az_order;
        AzimuthElevationDeg pointing;
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
      AzimuthElevationDeg pointing;
      pointing.az_deg = az_values[az_index];
      pointing.el_deg = el_values[el_index];
      pattern.push_back(pointing);
    }
  }
  return pattern;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // ONEQ_COMMON_RADAR_SCAN_SCHEDULE_RUNTIME_H_
