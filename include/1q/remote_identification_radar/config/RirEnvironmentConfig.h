/**
 * @file RirEnvironmentConfig.h
 * @brief 远程识别雷达环境域主配置类型。
 *
 * 阶段 1 为空占位（"扩展环境输入须先有真实消费路径"）；阶段 2-M M5 激活：
 * `RirPropagationModel`（internal）消费植被散射场景事实产出传播损耗与杂波功率，
 * 驻留链路预算（阶段 2-S 接线）为其真实消费路径。
 *
 * 类型合约对齐 `ArEnvironmentConfig.h`（审计基线 96de367c）：
 * - 仅承载外部输入事实（植被场景），不含内部算法调参项
 *   （传播/杂波基线系数保留在模型实现内部，与 AR 合约一致）；
 * - 不包含运行期派生量。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ENVIRONMENT_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"

namespace remote_identification_radar {
namespace config {

/**
 * @brief 地表植被覆盖档位（副本：airborne_radar::config::VegetationCoverProfile）。
 *
 * 选择档位后自动填写叶片尺寸、介电常数、叶片密度、冠层半径和冠层高度等
 * 植被散射物理参数，影响近地传播路径上的多径散射和杂波估计。
 */
enum class ONEQ_API RirVegetationCoverProfile {
  kDisabled = 0,     /**< 不建模植被散射。 */
  kOpenGrassland,    /**< 开阔草地——低矮冠层（0.8 m），少量小叶片。 */
  kSparseWoodland,   /**< 稀疏林地——中等冠层（3 m），中等密度散射体。 */
  kDeciduousForest,  /**< 落叶林——高大冠层（6 m），高密度大叶片。 */
  kConiferousForest, /**< 针叶林——高冠层（8 m），密集细小针叶散射体。 */
  kTropicalDense     /**< 热带密林——高冠层（9 m），最高密度与介电常数。 */
};

/**
 * @brief RirVegetationScatterPhysicsConfig 描述可选植被散射杂波参数
 *        （副本：airborne_radar::config::VegetationScatterPhysicsConfig）。
 */
struct ONEQ_API RirVegetationScatterPhysicsConfig {
  RirVegetationCoverProfile cover_profile{
      RirVegetationCoverProfile::kDisabled}; /**< 植被覆盖语义档位 */
  bool enable_physical_model{false}; /**< 是否启用植被散射物理建模 */
};

/**
 * @brief RirEnvironmentConfig 远程识别雷达环境域配置（场景事实输入）。
 *
 * 环境事实在会话初始化（`RirSessionConfig.environment`）注入；运行期变更经
 * `RirRuntimeConfigPatch.has_environment` 整域覆盖。禁止经周期输入携带。
 */
struct ONEQ_API RirEnvironmentConfig {
  bool enable_environment_effects{false}; /**< 是否启用环境传播/杂波效应（默认关闭）。 */
  float weather_attenuation_db{0.0f};     /**< 天气附加双程传播损耗（dB），须有限且 ≥0。 */
  RirVegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 场景植被散射输入 */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ENVIRONMENT_CONFIG_H_
