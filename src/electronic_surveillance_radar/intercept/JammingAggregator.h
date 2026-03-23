/**
 * @file JammingAggregator.h
 * @brief 定义电子侦察干扰聚合器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_JAMMING_AGGREGATOR_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_JAMMING_AGGREGATOR_H_

#include <algorithm>
#include <cstddef>
#include <cmath>

#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "electronic_surveillance_radar/intercept/InterceptGate.h"

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief JammingAggregateResult 描述干扰聚合输出。
 */
struct JammingAggregateResult {
  float jammer_power_w{0.0f}; /**< 聚合干扰功率（单位：W） */
  float weighted_overlap_ratio{0.0f}; /**< 置信度加权频段重叠度，范围 [0, 1] */
  float deception_risk{0.0f}; /**< 聚合欺骗风险，范围 [0, 1] */
  std::size_t active_source_count{0U}; /**< 参与聚合的有效干扰源数量 */
};

/**
 * @brief JammingAggregator 负责按目标频段聚合干扰源贡献。
 */
class JammingAggregator final {
 public:
  /**
   * @brief 聚合目标频段上的干扰功率。
   * @param[in] jammer_sources 干扰源列表。
   * @param[in] target_center_hz 目标中心频率（单位：Hz）。
   * @param[in] target_bandwidth_hz 目标带宽（单位：Hz）。
   * @return 干扰聚合结果。
   */
  static JammingAggregateResult Aggregate(
      const environment::EsrJammerSourceList& jammer_sources,
      double target_center_hz, double target_bandwidth_hz) {
    JammingAggregateResult result;
    if (!std::isfinite(target_center_hz) || !std::isfinite(target_bandwidth_hz) ||
        target_bandwidth_hz <= 0.0) {
      return result;
    }

    const double target_lower = target_center_hz - 0.5 * target_bandwidth_hz;
    const double target_upper = target_center_hz + 0.5 * target_bandwidth_hz;
    float overlap_weighted_sum = 0.0f;
    float power_weight_sum = 0.0f;
    float deception_clear_probability = 1.0f;
    for (std::size_t i = 0; i < jammer_sources.size(); ++i) {
      if (!jammer_sources[i].active || jammer_sources[i].bandwidth_hz <= 0.0 ||
          jammer_sources[i].power_w <= 0.0f ||
          !std::isfinite(jammer_sources[i].bandwidth_hz) ||
          !std::isfinite(jammer_sources[i].power_w) ||
          !std::isfinite(jammer_sources[i].center_hz) ||
          !std::isfinite(jammer_sources[i].confidence) ||
          !std::isfinite(jammer_sources[i].deception_risk)) {
        continue;
      }
      const float overlap_ratio = InterceptGate::ComputeFrequencyOverlapRatio(
          target_lower, target_upper, jammer_sources[i].center_hz,
          jammer_sources[i].bandwidth_hz);
      if (overlap_ratio <= 0.0f) {
        continue;
      }
      const float confidence = Clamp01(jammer_sources[i].confidence);
      const float deception_risk = Clamp01(jammer_sources[i].deception_risk);
      const float effective_power =
          jammer_sources[i].power_w * overlap_ratio * confidence;
      result.jammer_power_w += effective_power;
      overlap_weighted_sum += overlap_ratio * effective_power;
      power_weight_sum += effective_power;
      const float deception_effect = Clamp01(deception_risk * overlap_ratio *
                                             confidence);
      deception_clear_probability *= (1.0f - deception_effect);
      if (deception_clear_probability < 0.0f) {
        deception_clear_probability = 0.0f;
      }
      ++result.active_source_count;
    }

    if (power_weight_sum > 0.0f) {
      result.weighted_overlap_ratio = overlap_weighted_sum / power_weight_sum;
    }
    result.deception_risk = Clamp01(1.0f - deception_clear_probability);
    return result;
  }

 private:
  /**
   * @brief 将输入裁剪到 [0, 1]。
   * @param[in] value 输入值。
   * @return 裁剪结果。
   */
  static float Clamp01(float value) {
    return std::max(0.0f, std::min(1.0f, value));
  }
};

}  // namespace intercept
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_JAMMING_AGGREGATOR_H_
