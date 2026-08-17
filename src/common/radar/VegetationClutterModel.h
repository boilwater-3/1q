/**
 * @file VegetationClutterModel.h
 * @brief 定义植被散射杂波与最小传播损耗组合模型（common 单源）。
 */

#ifndef COMMON_RADAR_VEGETATION_CLUTTER_MODEL_H_
#define COMMON_RADAR_VEGETATION_CLUTTER_MODEL_H_

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
  float clutter_power_db{0.0f};
};

/**
 * @brief 根据植被散射配置计算传播损耗与杂波功率。
 * @param[in] config 植被散射物理配置。
 * @return 包含传播损耗与杂波功率的结果。
 */
PropagationClutterResult EvaluatePropagationClutter(const VegetationScatterPhysicsConfig& config);

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RADAR_VEGETATION_CLUTTER_MODEL_H_
