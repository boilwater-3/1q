/**
 * @file VegetationClutterModel.h
 * @brief 定义植被散射杂波与最小传播损耗组合模型（common 单源）。
 */

#ifndef COMMON_RADAR_VEGETATION_CLUTTER_MODEL_H_
#define COMMON_RADAR_VEGETATION_CLUTTER_MODEL_H_

#include <cmath>

#include "common/numerics/ClampUtils.h"

namespace oneq {
namespace common {
namespace radar {

/**
 * @brief 地表植被覆盖档位。
 */
enum class VegetationCoverProfile {
  kDisabled = 0,
  kOpenGrassland,
  kSparseWoodland,
  kDeciduousForest,
  kConiferousForest,
  kTropicalDense
};

/**
 * @brief 植被散射物理配置。
 */
struct VegetationScatterPhysicsConfig {
  VegetationCoverProfile cover_profile{VegetationCoverProfile::kDisabled};
  bool enable_physical_model{false};
};

/**
 * @brief PropagationClutterResult 表示传播与杂波组合输出。
 */
struct PropagationClutterResult {
  float propagation_loss_db{0.0f};
  /**
   * 杂波等效功率，单位：相对热噪底的 dB（CNR 口径）——消费方须乘接收机热噪
   * 功率换算为瓦，不得解释为绝对 dBW。
   */
  float clutter_power_db{0.0f};
};

/**
 * @brief 根据植被散射配置计算传播损耗与杂波功率。
 * @param[in] config 植被散射物理配置。
 * @return 包含传播损耗与杂波功率的结果。
 */
PropagationClutterResult EvaluatePropagationClutter(const VegetationScatterPhysicsConfig& config);

/**
 * @brief 把相对热噪底的杂波 dB 换算为等效杂波噪声瓦数（AR/RIR 统一口径单源）。
 * @param[in] thermal_noise_w 接收机热噪功率基准（W），非有限或 <=0 时返回 0。
 * @param[in] clutter_power_db 相对热噪底的杂波 dB（钳制到 [-120, +120]）。
 * @return 等效杂波噪声功率（W）。
 */
inline float ComputeEquivalentClutterNoiseW(float thermal_noise_w, float clutter_power_db) {
  if (!std::isfinite(clutter_power_db) ||
      !std::isfinite(thermal_noise_w) || thermal_noise_w <= 0.0f) {
    return 0.0f;
  }
  const float kMinRelativeClutterDb = -120.0f;
  const float kMaxRelativeClutterDb = 120.0f;
  const float relative_clutter_db =
      numerics::Clamp(clutter_power_db, kMinRelativeClutterDb, kMaxRelativeClutterDb);
  return thermal_noise_w * std::pow(10.0f, relative_clutter_db / 10.0f);
}

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RADAR_VEGETATION_CLUTTER_MODEL_H_
