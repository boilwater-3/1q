/**
 * @file JammingAggregator.h
 * @brief 定义电子侦察干扰聚合器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_JAMMING_AGGREGATOR_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_JAMMING_AGGREGATOR_H_

#include <cmath>
#include <cstddef>

#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "common/numerics/ClampUtils.h"
#include "electronic_surveillance_radar/intercept/InterceptGate.h"

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief JammingAggregateResult 描述干扰聚合输出。
 */
struct JammingAggregateResult {
  float suppression_power_w{0.0f};                /**< 聚合压制干扰功率（单位：W） */
  float suppression_weighted_overlap_ratio{0.0f}; /**< 压制分量置信度加权频段重叠度，范围 [0, 1] */
  float deception_weighted_overlap_ratio{0.0f};   /**< 欺骗分量置信度加权频段重叠度，范围 [0, 1] */
  float deception_risk{0.0f};                     /**< 聚合欺骗风险，范围 [0, 1] */
  std::size_t suppression_source_count{0U};       /**< 参与压制聚合的有效干扰源数量 */
  std::size_t deception_source_count{0U};         /**< 参与欺骗聚合的有效干扰源数量 */
  std::size_t active_source_count{0U};            /**< 参与任一分量聚合的有效干扰源数量 */
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
  static JammingAggregateResult Aggregate(const environment::EsrJammerSourceList& jammer_sources,
                                          double target_center_hz, double target_bandwidth_hz) {
    JammingAggregateResult result;
    if (!std::isfinite(target_center_hz) || !std::isfinite(target_bandwidth_hz) ||
        target_bandwidth_hz <= 0.0) {
      return result;
    }

    const double target_lower = target_center_hz - 0.5 * target_bandwidth_hz;
    const double target_upper = target_center_hz + 0.5 * target_bandwidth_hz;
    float suppression_overlap_weighted_sum = 0.0f;
    float suppression_power_weight_sum = 0.0f;
    float deception_overlap_weighted_sum = 0.0f;
    float deception_effect_weight_sum = 0.0f;
    float deception_clear_probability = 1.0f;
    for (std::size_t i = 0; i < jammer_sources.size(); ++i) {
      if (!jammer_sources[i].active || jammer_sources[i].bandwidth_hz <= 0.0 ||
          jammer_sources[i].power_w <= 0.0f || !std::isfinite(jammer_sources[i].bandwidth_hz) ||
          !std::isfinite(jammer_sources[i].power_w) ||
          !std::isfinite(jammer_sources[i].center_hz) ||
          !std::isfinite(jammer_sources[i].confidence) ||
          !std::isfinite(jammer_sources[i].deception_risk)) {
        continue;
      }
      const float overlap_ratio = InterceptGate::ComputeFrequencyOverlapRatio(
          target_lower, target_upper, jammer_sources[i].center_hz, jammer_sources[i].bandwidth_hz);
      if (overlap_ratio <= 0.0f) {
        continue;
      }
      const float confidence =
          oneq::internal::numerics::Clamp01(jammer_sources[i].confidence);
      const float deception_risk =
          oneq::internal::numerics::Clamp01(jammer_sources[i].deception_risk);
      const environment::EsrJammingTechnique technique = ResolveTechnique(jammer_sources[i]);

      bool contributed = false;
      if (HasSuppressionEffect(technique)) {
        const float effective_power = jammer_sources[i].power_w * overlap_ratio * confidence;
        result.suppression_power_w += effective_power;
        suppression_overlap_weighted_sum += overlap_ratio * effective_power;
        suppression_power_weight_sum += effective_power;
        ++result.suppression_source_count;
        contributed = true;
      }

      if (HasDeceptionEffect(technique)) {
        const float deception_effect =
            oneq::internal::numerics::Clamp01(deception_risk * overlap_ratio * confidence);
        const float safe_deception_effect =
            std::isfinite(deception_effect) ? deception_effect : 0.0f;
        deception_clear_probability *= (1.0f - safe_deception_effect);
        deception_clear_probability =
            oneq::internal::numerics::Clamp01(deception_clear_probability);
        deception_overlap_weighted_sum += overlap_ratio * deception_effect;
        deception_effect_weight_sum += deception_effect;
        ++result.deception_source_count;
        contributed = true;
      }

      if (contributed) {
        ++result.active_source_count;
      }
    }

    if (suppression_power_weight_sum > 0.0f) {
      result.suppression_weighted_overlap_ratio =
          suppression_overlap_weighted_sum / suppression_power_weight_sum;
    }
    if (deception_effect_weight_sum > 0.0f) {
      result.deception_weighted_overlap_ratio =
          deception_overlap_weighted_sum / deception_effect_weight_sum;
    }
    result.deception_risk =
        oneq::internal::numerics::Clamp01(1.0f - deception_clear_probability);
    return result;
  }

 private:
  /**
   * @brief 解析干扰技术类型并应用兼容推断。
   * @param[in] source 干扰源输入。
   * @return 解析后的干扰技术类型。
   */
  static environment::EsrJammingTechnique ResolveTechnique(
      const environment::EsrJammerSource& source) {
    if (source.technique != environment::EsrJammingTechnique::kUnknown) {
      return source.technique;
    }
    if (source.deception_risk > 0.0f) {
      return environment::EsrJammingTechnique::kMixed;
    }
    return environment::EsrJammingTechnique::kNoiseSuppression;
  }

  /**
   * @brief 判断技术类型是否包含压制分量。
   * @param[in] technique 干扰技术类型。
   * @return 包含压制分量时返回 `true`。
   */
  static bool HasSuppressionEffect(environment::EsrJammingTechnique technique) {
    return technique == environment::EsrJammingTechnique::kNoiseSuppression ||
           technique == environment::EsrJammingTechnique::kMixed;
  }

  /**
   * @brief 判断技术类型是否包含欺骗分量。
   * @param[in] technique 干扰技术类型。
   * @return 包含欺骗分量时返回 `true`。
   */
  static bool HasDeceptionEffect(environment::EsrJammingTechnique technique) {
    return technique == environment::EsrJammingTechnique::kDeception ||
           technique == environment::EsrJammingTechnique::kMixed;
  }

};

}  // namespace intercept
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_JAMMING_AGGREGATOR_H_
