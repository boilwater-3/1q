/**
 * @file EosEnvironmentConfig.h
 * @brief EOS 环境配置契约（Scenario/Model/Default）。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"
#include "1q/environment/AtmosphericTypes.h"

namespace electro_optical_sensor {
namespace config {

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
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
};

/**
 * @brief EosEnvironmentScenarioConfig 描述外部场景语义输入。
 */
struct ONEQ_API EosEnvironmentScenarioConfig {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};
  EosEnvironmentPreset preset{EosEnvironmentPreset::kStandard};
  bool has_custom_overrides{false};
  EosEnvironmentCustomOverrides custom_overrides{};
  bool has_atmospheric_observation{false};
  oneq::environment::AtmosphericObservation atmospheric_observation{};
};

/**
 * @brief EosEnvironmentModelConfig 描述 pipeline 直接消费参数。
 */
struct ONEQ_API EosEnvironmentModelConfig {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};
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
