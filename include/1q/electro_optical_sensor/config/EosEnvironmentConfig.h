/**
 * @file EosEnvironmentConfig.h
 * @brief EOS 环境配置契约（Scenario/Model/Default）。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"
#include "1q/environment/AtmosphericTypes.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief RadiativeTransferModel 描述路径辐射传输模型类型。
 */
enum class ONEQ_API RadiativeTransferModel {
  kDerivedBeerLambert = 0,  /**< 派生 Beer-Lambert（默认，兼容现有逻辑） */
  kHumidityWeighted,        /**< 云湿主导模型 */
  kAdaptivePathRadiance     /**< 含路径辐射惩罚的自适应模型 */
};

/**
 * @brief EosEnvironmentModelType 描述环境模型策略。
 */
enum class ONEQ_API EosEnvironmentModelType {
  kSimplified = 0,
  kAdvanced
};

/**
 * @brief EosEnvironmentPreset 描述高层环境预设。
 */
enum class ONEQ_API EosEnvironmentPreset {
  kStandard = 0,
  kHumid,
  kDusty,
  kTurbulent,
  kMaritime
};

/**
 * @brief EosEnvironmentCustomOverrides 描述场景级显式自定义覆盖。
 */
struct ONEQ_API EosEnvironmentCustomOverrides {
  RadiativeTransferModel radiative_transfer_model{
      RadiativeTransferModel::kDerivedBeerLambert};
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
};

/**
 * @brief EosEnvironmentScenarioConfig 描述外部场景语义输入。
 *
 * 解析顺序固定为：先由 @ref preset 建立整组环境参数基线；当
 * @ref has_custom_overrides 为 `true` 时，再由 @ref custom_overrides 整组覆盖该基线；
 * 最后由 @ref model_type 决定运行时是否依据实时环境对这组参数进行动态修正。
 * 当 @ref has_custom_overrides 为 `false` 时，custom_overrides 中的值完全忽略。
 * 大气观测通过独立的 has/value 对传入，不参与 preset 与 custom 的优先级判定。
 */
struct ONEQ_API EosEnvironmentScenarioConfig {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified}; /**< 动态修正策略 */
  EosEnvironmentPreset preset{EosEnvironmentPreset::kStandard}; /**< 环境参数基线 */
  bool has_custom_overrides{false}; /**< 是否整组启用显式覆盖 */
  EosEnvironmentCustomOverrides custom_overrides{}; /**< 显式整组覆盖值 */
  bool has_atmospheric_observation{false}; /**< 是否提供独立的大气观测 */
  oneq::environment::AtmosphericObservation atmospheric_observation{};
};

/**
 * @brief EosEnvironmentModelConfig 描述 pipeline 直接消费参数。
 */
struct ONEQ_API EosEnvironmentModelConfig {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};
  RadiativeTransferModel radiative_transfer_model{
      RadiativeTransferModel::kDerivedBeerLambert};
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
  bool has_atmospheric_observation{false};
  oneq::environment::AtmosphericObservation atmospheric_observation{};
};

/**
 * @brief EosEnvironmentConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EosEnvironmentConfig {
  EosEnvironmentScenarioConfig scenario_config{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_
