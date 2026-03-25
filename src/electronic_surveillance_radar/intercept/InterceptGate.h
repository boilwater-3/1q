/**
 * @file InterceptGate.h
 * @brief 定义电子侦察截获门联合判定工具。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_INTERCEPT_GATE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_INTERCEPT_GATE_H_

#include <algorithm>
#include <cmath>

#include "common/geometry/GeometryTransform.h"

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief InterceptGateInput 描述截获门判定输入。
 */
struct InterceptGateInput {
  bool line_of_sight{true};                 /**< 是否满足通视条件 */
  float target_az_deg{0.0f};                /**< 目标方位角（单位：deg） */
  float target_el_deg{0.0f};                /**< 目标俯仰角（单位：deg） */
  float beam_az_deg{0.0f};                  /**< 接收波束中心方位（单位：deg） */
  float beam_el_deg{0.0f};                  /**< 接收波束中心俯仰（单位：deg） */
  float beam_az_width_deg{20.0f};           /**< 接收波束方位宽度（单位：deg） */
  float beam_el_width_deg{20.0f};           /**< 接收波束俯仰宽度（单位：deg） */
  double receiver_lower_hz{0.0};            /**< 接收机下边界频率（单位：Hz） */
  double receiver_upper_hz{0.0};            /**< 接收机上边界频率（单位：Hz） */
  double signal_center_hz{0.0};             /**< 信号中心频率（单位：Hz） */
  double signal_bandwidth_hz{0.0};          /**< 信号带宽（单位：Hz） */
  float range_m{0.0f};                      /**< 目标距离（单位：m） */
  float max_range_m{0.0f};                  /**< 可截获最大距离（单位：m） */
  float dynamic_range_margin_db{0.0f};      /**< 动态范围裕量（单位：dB） */
  float min_dynamic_range_margin_db{-3.0f}; /**< 动态范围最小裕量（单位：dB） */
  float min_frequency_overlap_ratio{0.0f};  /**< 频段覆盖最小重叠比例，范围 [0, 1] */
  float beam_guard_factor{1.0f};            /**< 波束守护因子（对半波束宽度的缩放） */
};

/**
 * @brief InterceptGateDecision 描述联合门判定结果。
 */
struct InterceptGateDecision {
  bool in_beam{false};                 /**< 是否落入当前接收波束 */
  bool frequency_covered{false};       /**< 是否满足接收频段覆盖 */
  bool in_range{false};                /**< 是否满足可截获距离 */
  bool dynamic_range_ok{false};        /**< 是否满足动态范围裕量 */
  bool passed{false};                  /**< 是否通过全部门判定 */
  float frequency_overlap_ratio{0.0f}; /**< 频段重叠度，范围 [0, 1] */
  float beam_overlap_ratio{0.0f};      /**< 波束重叠度近似指标，范围 [0, 1] */
};

/**
 * @brief InterceptGate 提供联合截获门判定。
 */
class InterceptGate final {
 public:
  /**
   * @brief 计算两段频谱区间重叠度。
   * @param[in] receiver_lower_hz 接收机频段下边界（单位：Hz）。
   * @param[in] receiver_upper_hz 接收机频段上边界（单位：Hz）。
   * @param[in] signal_center_hz 信号中心频率（单位：Hz）。
   * @param[in] signal_bandwidth_hz 信号带宽（单位：Hz）。
   * @return 频段重叠度，范围 [0, 1]。
   */
  static float ComputeFrequencyOverlapRatio(double receiver_lower_hz, double receiver_upper_hz,
                                            double signal_center_hz, double signal_bandwidth_hz) {
    if (!std::isfinite(receiver_lower_hz) || !std::isfinite(receiver_upper_hz) ||
        !std::isfinite(signal_center_hz) || !std::isfinite(signal_bandwidth_hz) ||
        receiver_upper_hz <= receiver_lower_hz || signal_bandwidth_hz <= 0.0) {
      return 0.0f;
    }
    const double signal_lower = signal_center_hz - 0.5 * signal_bandwidth_hz;
    const double signal_upper = signal_center_hz + 0.5 * signal_bandwidth_hz;
    const double overlap_lower = std::max(receiver_lower_hz, signal_lower);
    const double overlap_upper = std::min(receiver_upper_hz, signal_upper);
    if (overlap_upper <= overlap_lower) {
      return 0.0f;
    }
    const double overlap = overlap_upper - overlap_lower;
    const double ratio = overlap / signal_bandwidth_hz;
    if (ratio <= 0.0) {
      return 0.0f;
    }
    if (ratio >= 1.0) {
      return 1.0f;
    }
    return static_cast<float>(ratio);
  }

  /**
   * @brief 执行联合截获门判定。
   * @param[in] input 截获门输入。
   * @return 联合门判定结果。
   */
  static InterceptGateDecision Evaluate(const InterceptGateInput& input) {
    InterceptGateDecision decision;
    if (!IsFinite(input.target_az_deg) || !IsFinite(input.target_el_deg) ||
        !IsFinite(input.beam_az_deg) || !IsFinite(input.beam_el_deg) ||
        !IsFinite(input.beam_az_width_deg) || !IsFinite(input.beam_el_width_deg) ||
        !IsFinite(input.range_m) || !IsFinite(input.max_range_m) ||
        !IsFinite(input.dynamic_range_margin_db) || !IsFinite(input.min_dynamic_range_margin_db) ||
        !IsFinite(input.min_frequency_overlap_ratio) || !IsFinite(input.beam_guard_factor)) {
      return decision;
    }

    decision.frequency_overlap_ratio =
        ComputeFrequencyOverlapRatio(input.receiver_lower_hz, input.receiver_upper_hz,
                                     input.signal_center_hz, input.signal_bandwidth_hz);
    const float min_overlap_ratio = Clamp01(input.min_frequency_overlap_ratio);
    decision.frequency_covered = decision.frequency_overlap_ratio >= min_overlap_ratio;

    const float az_diff = std::fabs(oneq::internal::geometry::ComputeAzimuthDifferenceDeg(
        input.target_az_deg, input.beam_az_deg));
    const float el_diff = std::fabs(input.target_el_deg - input.beam_el_deg);
    const float guard_factor = std::max(0.0f, input.beam_guard_factor);
    const float half_az_width = std::max(1.0e-6f, 0.5f * input.beam_az_width_deg * guard_factor);
    const float half_el_width = std::max(1.0e-6f, 0.5f * input.beam_el_width_deg * guard_factor);
    const float normalized_az = az_diff / half_az_width;
    const float normalized_el = el_diff / half_el_width;
    const float normalized_distance =
        std::sqrt(normalized_az * normalized_az + normalized_el * normalized_el);

    decision.in_beam = normalized_distance <= 1.0f;
    decision.beam_overlap_ratio = normalized_distance >= 1.0f ? 0.0f : (1.0f - normalized_distance);
    decision.in_range =
        input.max_range_m > 0.0f && input.range_m >= 0.0f && input.range_m <= input.max_range_m;
    decision.dynamic_range_ok = input.dynamic_range_margin_db >= input.min_dynamic_range_margin_db;
    decision.passed = input.line_of_sight && decision.frequency_covered && decision.in_beam &&
                      decision.in_range && decision.dynamic_range_ok;
    return decision;
  }

 private:
  /**
   * @brief 将输入裁剪到 [0, 1] 区间。
   * @param[in] value 输入标量。
   * @return 裁剪后的结果。
   */
  static float Clamp01(float value) {
    if (value <= 0.0f) {
      return 0.0f;
    }
    if (value >= 1.0f) {
      return 1.0f;
    }
    return value;
  }

  /**
   * @brief 判断输入浮点值是否为有限数。
   * @param[in] value 输入值。
   * @return 有限数时返回 `true`。
   */
  static bool IsFinite(float value) { return std::isfinite(value) != 0; }
};

}  // namespace intercept
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_INTERCEPT_GATE_H_
