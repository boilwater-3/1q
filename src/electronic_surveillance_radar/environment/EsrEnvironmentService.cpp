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
float Clamp01(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

/**
 * @brief 将输入裁剪到非负区间。
 * @param[in] value 输入值。
 * @return 裁剪后结果。
 */
float ClampNonNegative(float value) {
  return std::max(0.0f, value);
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
  normalized.active = raw_source.active && normalized.power_w > 0.0f &&
                      normalized.bandwidth_hz > 0.0;
  return normalized;
}

/**
 * @brief 根据周期上下文构造冻结快照。
 * @param[in] cycle_context 周期上下文。
 * @param[in] config 环境配置。
 * @return 冻结环境快照。
 */
EsrEnvironmentSnapshot BuildSnapshot(
    const EsrEnvironmentCycleContext& cycle_context,
    const EsrEnvironmentModelConfig& config) {
  EsrEnvironmentSnapshot snapshot;
  snapshot.cycle_index = cycle_context.cycle_index;
  snapshot.dt_sec = cycle_context.dt_sec;
  snapshot.propagation_loss_db = ClampNonNegative(
      cycle_context.scene_state.base_propagation_loss_db +
      cycle_context.scene_state.atmospheric_attenuation_db +
      cycle_context.scene_state.terrain_reflection_db);

  const float clutter_noise = cycle_context.scene_state.clutter_noise_w > 0.0f
                                  ? cycle_context.scene_state.clutter_noise_w
                                  : config.default_clutter_noise_w;
  snapshot.clutter_noise_w = ClampNonNegative(clutter_noise);
  snapshot.spectrum_occupancy_ratio =
      Clamp01(cycle_context.scene_state.spectrum_occupancy_ratio);

  snapshot.jammer_sources.clear();
  snapshot.jammer_sources.reserve(cycle_context.scene_state.jammer_sources.size());
  snapshot.jammer_power_w = 0.0f;
  snapshot.deception_risk = 0.0f;
  for (std::size_t i = 0; i < cycle_context.scene_state.jammer_sources.size();
       ++i) {
    const EsrJammerSource source =
        NormalizeJammerSource(cycle_context.scene_state.jammer_sources[i]);
    snapshot.jammer_sources.push_back(source);
    if (!source.active) {
      continue;
    }

    snapshot.jammer_power_w += source.power_w * source.confidence;
    const float source_risk = source.deception_risk * source.confidence;
    if (source_risk > snapshot.deception_risk) {
      snapshot.deception_risk = source_risk;
    }
  }

  snapshot.jamming_detected =
      snapshot.jammer_power_w >= config.jamming_detection_threshold_w;
  return snapshot;
}

}  // namespace

EsrEnvironmentService::EsrEnvironmentService(EsrEnvironmentModelConfig config)
    : config_(config) {}

void EsrEnvironmentService::BeginCycle(
    const EsrEnvironmentCycleContext& cycle_context) {
  frozen_snapshot_ = BuildSnapshot(cycle_context, config_);
}

EsrEnvironmentSnapshot EsrEnvironmentService::SampleEnvironment() const {
  return frozen_snapshot_;
}

void EsrEnvironmentService::UpdateModelConfig(EsrEnvironmentModelConfig config) {
  config_ = config;
}

}  // namespace environment
}  // namespace electronic_surveillance_radar
