/**
 * @file ArEnvironmentConfig.h
 * @brief 机载雷达环境域主配置类型集合。
 *
 * 环境域配置（气象、空间天气、植被散射）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_ENVIRONMENT_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"
#include "1q/environment/AtmosphericTypes.h"

namespace airborne_radar {
namespace config {

/** @brief AtmosphericPhysicsConfig 复用统一环境模块基础气象观测类型。 */
using AtmosphericPhysicsConfig = oneq::environment::AtmosphericObservation;

/**
 * @brief 地表植被覆盖档位。
 *
 * 选择档位后自动填写叶片尺寸、介电常数、叶片密度、
 * 冠层半径和冠层高度等植被散射物理参数，
 * 影响近地传播路径上的多径散射和杂波估计。
 */
enum class ONEQ_API VegetationCoverProfile {
  kDisabled = 0,     /**< 不建模植被散射。 */
  kOpenGrassland,    /**< 开阔草地——低矮冠层（0.8 m），少量小叶片。 */
  kSparseWoodland,   /**< 稀疏林地——中等冠层（3 m），中等密度散射体。 */
  kDeciduousForest,  /**< 落叶林——高大冠层（6 m），高密度大叶片。 */
  kConiferousForest, /**< 针叶林——高冠层（8 m），密集细小针叶散射体。 */
  kTropicalDense     /**< 热带密林——高冠层（9 m），最高密度与介电常数。 */
};

/**
 * @brief VegetationScatterPhysicsConfig 描述可选植被散射杂波参数。
 */
struct ONEQ_API VegetationScatterPhysicsConfig {
  VegetationCoverProfile cover_profile{VegetationCoverProfile::kDisabled}; /**< 植被覆盖语义档位 */
  bool enable_physical_model{false}; /**< 是否启用植被散射物理建模 */
};

/**
 * @brief EnvironmentScenarioConfig 描述对外场景输入（不暴露内部传播/杂波调参项）。
 *
 * @par 类型合约
 * - 仅承载外部输入事实（气象观测、植被场景）。
 * - 不包含内部算法调参字段（如传播损耗系数、杂波增益）。
 * - 不包含运行期派生量（如 effective_k_factor）。
 * - 不暴露 SpaceWeatherContext（空间天气上下文）：其字段（k_factor、day_of_year、
 *   solar_flux、geomagnetic_ap、simulation_unix_seconds）在当前 GTD7 大气模型
 *   退化为 ISA 标准大气的情况下全部未被消费，属未接入的死输入，故不对外开放。
 * - 新增字段须为外部可观测或可注入的场景事实，不得引入内部调参项。
 */
struct ONEQ_API EnvironmentScenarioConfig {
  AtmosphericPhysicsConfig atmospheric_physics{};              /**< 场景气象/电离层输入 */
  VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 场景植被散射输入 */
};

/**
 * @brief ArEnvironmentConfig 描述初始化阶段的默认环境配置。
 *
 * @par 类型合约
 * - 初始化阶段一次性构造，构造后不再变更。
 * - 仅承载 scenario_config（外部场景事实），不包含策略枚举或调参项。
 * - 不提供运行期热更新语义；运行期更新须通过 EnvironmentRuntimeConfigPatch。
 * - 无复杂校验逻辑，构造后即视为合法。
 */
struct ONEQ_API ArEnvironmentConfig {
  EnvironmentScenarioConfig scenario_config{}; /**< 默认环境场景输入 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_ENVIRONMENT_CONFIG_H_
