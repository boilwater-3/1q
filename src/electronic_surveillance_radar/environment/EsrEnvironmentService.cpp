#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"

#include <algorithm>
#include <cstddef>

namespace electronic_surveillance_radar {
namespace environment {

namespace {

/**
 * @brief 将输入裁剪到 [0, 1]。
 * @param[in] value 输入值。
 * @return 裁剪后结果。
 */
float Clamp01(float value) { return std::max(0.0f, std::min(1.0f, value)); }

/**
 * @brief 将输入裁剪到非负区间。
 * @param[in] value 输入值。
 * @return 裁剪后结果。
 */
float ClampNonNegative(float value) { return std::max(0.0f, value); }

/**
 * @brief 解析干扰技术类型并应用兼容推断。
 * @param[in] source 干扰源输入。
 * @return 解析后的干扰技术类型。
 */
EsrJammingTechnique ResolveTechnique(const EsrJammerSource& source) {
  if (source.technique != EsrJammingTechnique::kUnknown) {
    return source.technique;
  }
  if (source.deception_risk > 0.0f) {
    return EsrJammingTechnique::kMixed;
  }
  return EsrJammingTechnique::kNoiseSuppression;
}

/**
 * @brief 判断技术类型是否包含压制分量。
 * @param[in] technique 干扰技术类型。
 * @return 包含压制分量时返回 `true`。
 */
bool HasSuppressionEffect(EsrJammingTechnique technique) {
  return technique == EsrJammingTechnique::kNoiseSuppression ||
         technique == EsrJammingTechnique::kMixed;
}

/**
 * @brief 判断技术类型是否包含欺骗分量。
 * @param[in] technique 干扰技术类型。
 * @return 包含欺骗分量时返回 `true`。
 */
bool HasDeceptionEffect(EsrJammingTechnique technique) {
  return technique == EsrJammingTechnique::kDeception || technique == EsrJammingTechnique::kMixed;
}

/**
 * @brief 规范化单个干扰源输入。
 * @param[in] raw_source 原始输入。
 * @return 规范化后的干扰源。
 */
EsrJammerSource NormalizeJammerSource(const EsrJammerSource& raw_source) {
  EsrJammerSource normalized = raw_source;
  normalized.power_w = ClampNonNegative(raw_source.power_w);
  normalized.bandwidth_hz = std::max(0.0, raw_source.bandwidth_hz);
  normalized.deception_risk = Clamp01(raw_source.deception_risk);
  normalized.confidence = Clamp01(raw_source.confidence);
  normalized.technique = ResolveTechnique(normalized);
  normalized.active =
      raw_source.active && normalized.power_w > 0.0f && normalized.bandwidth_hz > 0.0;
  return normalized;
}

/**
 * @brief 根据周期上下文构造冻结快照。
 * @param[in] cycle_context 周期上下文。
 * @param[in] config 环境配置。
 * @return 冻结环境快照。
 */
EsrEnvironmentSnapshot BuildSnapshot(const EsrEnvironmentCycleContext& cycle_context,
                                     const EsrEnvironmentModelConfig& config) {
  EsrEnvironmentSnapshot snapshot;
  snapshot.cycle_index = cycle_context.cycle_index;
  snapshot.dt_sec = cycle_context.dt_sec;
  snapshot.propagation_loss_db =
      ClampNonNegative(cycle_context.scene_state.base_propagation_loss_db +
                       cycle_context.scene_state.atmospheric_attenuation_db +
                       cycle_context.scene_state.terrain_reflection_db);

  const float clutter_noise = cycle_context.scene_state.clutter_noise_w > 0.0f
                                  ? cycle_context.scene_state.clutter_noise_w
                                  : config.default_clutter_noise_w;
  snapshot.clutter_noise_w = ClampNonNegative(clutter_noise);
  snapshot.spectrum_occupancy_ratio = Clamp01(cycle_context.scene_state.spectrum_occupancy_ratio);

  snapshot.jammer_sources.clear();
  snapshot.jammer_sources.reserve(cycle_context.scene_state.jammer_sources.size());
  snapshot.suppression_power_w = 0.0f;
  snapshot.jammer_power_w = 0.0f;
  snapshot.deception_risk = 0.0f;
  float deception_clear_probability = 1.0f;
  for (std::size_t i = 0; i < cycle_context.scene_state.jammer_sources.size(); ++i) {
    const EsrJammerSource source =
        NormalizeJammerSource(cycle_context.scene_state.jammer_sources[i]);
    snapshot.jammer_sources.push_back(source);
    if (!source.active) {
      continue;
    }

    const float weighted_power = source.power_w * source.confidence;
    if (HasSuppressionEffect(source.technique)) {
      snapshot.suppression_power_w += weighted_power;
    }
    if (HasDeceptionEffect(source.technique)) {
      const float source_risk = Clamp01(source.deception_risk * source.confidence);
      deception_clear_probability *= (1.0f - source_risk);
      if (deception_clear_probability < 0.0f) {
        deception_clear_probability = 0.0f;
      }
    }
  }
  snapshot.deception_risk = Clamp01(1.0f - deception_clear_probability);
  snapshot.jammer_power_w = snapshot.suppression_power_w;

  snapshot.jamming_detected = snapshot.suppression_power_w >= config.jamming_detection_threshold_w;
  return snapshot;
}

}  // namespace

EsrEnvironmentService::EsrEnvironmentService(EsrEnvironmentModelConfig config) : config_(config) {}

void EsrEnvironmentService::BeginCycle(const EsrEnvironmentCycleContext& cycle_context) {
  frozen_snapshot_ = BuildSnapshot(cycle_context, config_);
}

EsrEnvironmentSnapshot EsrEnvironmentService::SampleEnvironment() const { return frozen_snapshot_; }

void EsrEnvironmentService::UpdateModelConfig(EsrEnvironmentModelConfig config) {
  config_ = config;
}

}  // namespace environment
}  // namespace electronic_surveillance_radar
