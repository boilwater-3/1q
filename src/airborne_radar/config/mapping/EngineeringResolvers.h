/**
 * @file EngineeringResolvers.h
 * @brief 定义四域配置子类型到 engineering 配置的共享映射函数。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_MAPPING_ENGINEERING_RESOLVERS_H_
#define AIRBORNE_RADAR_SRC_CONFIG_MAPPING_ENGINEERING_RESOLVERS_H_

#include <cmath>
#include <vector>

#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "airborne_radar/config/SignalEngineeringConfig.h"

namespace airborne_radar {
namespace config {
namespace mapping {

inline engineering::DetectionConfig ResolveDetectionEngineering(
    const detection::DetectionConfig& hardware, const ArDetectionPolicyConfig& policy) {
  engineering::DetectionConfig resolved;
  resolved.transmitter = hardware.transmitter;
  resolved.antenna = hardware.antenna;
  resolved.receiver = hardware.receiver;
  resolved.rcs_physics = hardware.rcs_physics;
  resolved.detection_policy.cfar_pfa = policy.pfa;
  resolved.detection_policy.min_snr_db = policy.minimum_snr_db;
  resolved.pulse_count = policy.pulse_count;
  resolved.min_detection_margin_db = policy.minimum_detection_margin_db;
  return resolved;
}

/**
 * @brief 将公开 LifecycleConfig 字段拷贝到 engineering 侧运行态配置。
 * @param[in] lifecycle 会话侧的生命周期策略参数。
 * @return 字段一一对应的 engineering::LifecycleRuntimeConfig。
 */
inline engineering::LifecycleRuntimeConfig ResolveLifecycleEngineering(
    const LifecycleConfig& lifecycle) {
  engineering::LifecycleRuntimeConfig resolved;
  resolved.lifecycle_config.confirm_hits = lifecycle.confirm_hits;
  resolved.lifecycle_config.max_miss_before_lost = lifecycle.max_miss_before_lost;
  resolved.lifecycle_config.max_lost_cycles = lifecycle.max_lost_cycles;
  resolved.enable_imm_lifecycle = lifecycle.enable_imm_lifecycle;
  return resolved;
}

/**
 * @brief 根据 model_count_hint 生成默认 IMM 模型运动噪声差异系数。
 *
 * 采用对数等间距分布：从精细（低运动噪声）到粗略（高运动噪声），
 * 确保 model_count_hint >= 2 时每个模型的系数区分度合理。
 * @param[in] model_count_hint 期望的 IMM 模型数量提示，小于 2 时按 2 处理。
 * @return 长度为实际模型数的差异系数向量，按运动噪声由低到高排列。
 */
inline std::vector<float> BuildDefaultImmNoiseDiffCoeffs(std::uint32_t model_count_hint) {
  const std::size_t model_count =
      static_cast<std::size_t>(model_count_hint < 2U ? 2U : model_count_hint);
  std::vector<float> coeffs;
  coeffs.reserve(model_count);
  for (std::size_t i = 0; i < model_count; ++i) {
    coeffs.push_back(std::pow(10.0f, static_cast<float>(i) / static_cast<float>(model_count - 1)));
  }
  return coeffs;
}

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_MAPPING_ENGINEERING_RESOLVERS_H_
