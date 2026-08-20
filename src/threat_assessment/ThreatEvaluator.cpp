/**
 * @file ThreatEvaluator.cpp
 * @brief 归一化加权和威胁评估器实现。
 */

#include "1q/threat_assessment/ThreatEvaluator.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "1q/threat_assessment/ThreatEvaluationInput.h"
#include "1q/threat_assessment/ThreatResult.h"
#include "common/numerics/ClampUtils.h"

namespace threat_assessment {

namespace {

// 权重槽位顺序：range / speed / acceleration / rcs / target_probability /
// fusion_confidence。
constexpr std::size_t kWeightCount = 6U;

// 属性清洗：NaN 按缺失（归一化 0）；负值钳制 0；inf 保留（由归一化断点钳制到 1 或 0）。
float Clean(float v) { return std::isnan(v) ? 0.0f : std::max(0.0f, v); }

// 线性上升归一化：x ≤ x_min → 0，x ≥ x_max → 1，区间内线性。
// 断点退化（x_max ≤ x_min）时为阶梯：x ≥ x_max → 1，其余 0（早退分支先命中）。
// 断点为 NaN 时区间判定恒不成立，回退分支（span 非正）返回退化端常量 1（防呆）。
float NormalizeUp(float x, float x_min, float x_max) {
  if (x <= x_min) {
    return 0.0f;
  }
  if (x >= x_max) {
    return 1.0f;
  }
  const float span = x_max - x_min;
  return span > 0.0f ? (x - x_min) / span : 1.0f;
}

// 线性下降归一化：x ≤ x_near → 1，x ≥ x_far → 0，区间内线性（越近越危险）。
// 断点退化（x_far ≤ x_near）时为阶梯：x ≤ x_near → 1，其余 0；NaN 断点回退
// 返回退化端常量 0（防呆）。
float NormalizeDown(float x, float x_near, float x_far) {
  if (x <= x_near) {
    return 1.0f;
  }
  if (x >= x_far) {
    return 0.0f;
  }
  const float span = x_far - x_near;
  return span > 0.0f ? (x_far - x) / span : 0.0f;
}

// 权重清洗与归一化：钳制非负后归一化（Σ = 1）；全零/非有限和退化为均分，
// 避免全零权重配置导致威胁分恒零（配置错误防呆）。
std::array<float, kWeightCount> NormalizeWeights(
    const ThreatEvaluatorConfig& config) {
  std::array<float, kWeightCount> weights = {
      config.weight_range, config.weight_speed, config.weight_acceleration,
      config.weight_rcs, config.weight_target_probability,
      config.weight_fusion_confidence};
  // double 累积求和，降低默认配置（和 ≈ 1）下归一化引入的浮点误差。
  double sum = 0.0;
  for (float& w : weights) {
    w = std::isnan(w) ? 0.0f : std::max(0.0f, w);
    sum += static_cast<double>(w);
  }
  // 非有限和（任一权重为 +inf）或非正和 → 均分，避免 NaN 权重污染威胁分。
  if (!std::isfinite(sum) || sum <= 0.0) {
    for (float& w : weights) {
      w = 1.0f / static_cast<float>(kWeightCount);
    }
    return weights;
  }
  for (float& w : weights) {
    w = static_cast<float>(static_cast<double>(w) / sum);
  }
  return weights;
}

}  // namespace

ThreatEvaluator::ThreatEvaluator(const ThreatEvaluatorConfig& config)
    : config_(config) {}

std::vector<ThreatResult> ThreatEvaluator::Evaluate(
    const std::vector<ThreatEvaluationInput>& inputs) const {
  const std::array<float, kWeightCount> weights = NormalizeWeights(config_);
  std::vector<ThreatResult> results;
  results.reserve(inputs.size());

  for (const ThreatEvaluationInput& input : inputs) {
    ThreatResult result;
    result.key = input.key;

    // 六属性归一化（属性缺失/非法值 → 0，贡献为 0）。
    // 距离特殊：NaN/负值按缺失（0 分）；0 是合法值（零距离 = 满分，最危险）。
    const float range = input.range_m;
    const float f_range =
        (std::isfinite(range) && range >= 0.0f)
            ? NormalizeDown(range, config_.range_near_m, config_.range_far_m)
            : 0.0f;
    const float f_speed =
        NormalizeUp(Clean(input.speed), config_.speed_min_mps,
                    config_.speed_max_mps);
    const float f_acceleration =
        NormalizeUp(Clean(input.acceleration), 0.0f,
                    config_.acceleration_max_mps2);
    const float f_rcs = NormalizeUp(Clean(input.rcs), config_.rcs_min_sqm,
                                    config_.rcs_max_sqm);
    const float f_probability = oneq::common::numerics::Clamp01(
        Clean(input.target_probability));
    // 融合置信度冻结公式不归一化、可 >1，作为威胁属性时钳制到 [0,1]。
    const float f_confidence = oneq::common::numerics::Clamp01(
        Clean(input.fusion_confidence));

    // 可解释分解：每属性贡献 = 归一化权重 × 属性归一化值；威胁分 = 贡献之和
    // （权重已归一化 Σ = 1 → 总分自然 ∈ [0,1]，再钳制兜底）。
    result.contributions.range = weights[0] * f_range;
    result.contributions.speed = weights[1] * f_speed;
    result.contributions.acceleration = weights[2] * f_acceleration;
    result.contributions.rcs = weights[3] * f_rcs;
    result.contributions.target_probability = weights[4] * f_probability;
    result.contributions.fusion_confidence = weights[5] * f_confidence;
    result.threat_score = oneq::common::numerics::Clamp01(
        result.contributions.range + result.contributions.speed +
        result.contributions.acceleration + result.contributions.rcs +
        result.contributions.target_probability +
        result.contributions.fusion_confidence);

    // 等级映射：先判 HIGH 再判 MEDIUM（阈值配置倒置时 MEDIUM 不触发，属配置语义）。
    if (result.threat_score >= config_.high_threshold) {
      result.level = ThreatLevel::kHigh;
    } else if (result.threat_score >= config_.medium_threshold) {
      result.level = ThreatLevel::kMedium;
    } else {
      result.level = ThreatLevel::kLow;
    }

    results.push_back(result);
  }
  return results;
}

}  // namespace threat_assessment
