/**
 * @file EosEnvironmentConfig.h
 * @brief EOS 环境配置契约（Scenario/Model/Default）与单入口映射。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief EosEnvironmentModelType 描述环境模型策略。
 */
enum class ONEQ_API EosEnvironmentModelType {
  kSimplified = 0, /**< 简化模型（固定环境参数） */
  kAdvanced        /**< 高级模型（高度/风速/云量驱动） */
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
  bool enable_optical_countermeasure_extension{false};
};

/**
 * @brief EosEnvironmentScenarioConfig 描述外部场景语义输入。
 */
struct ONEQ_API EosEnvironmentScenarioConfig {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};
  EosEnvironmentPreset preset{EosEnvironmentPreset::kStandard};
  bool has_custom_overrides{false};
  EosEnvironmentCustomOverrides custom_overrides{};
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
  bool enable_optical_countermeasure_extension{false};
};

/**
 * @brief EosEnvironmentDefaultConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EosEnvironmentDefaultConfig {
  EosEnvironmentScenarioConfig scenario_config{};
};

/**
 * @brief 将场景配置映射为环境模型配置（单入口）。
 */
inline EosEnvironmentModelConfig BuildModelConfigFromScenario(
    const EosEnvironmentScenarioConfig& scenario_config) {
  EosEnvironmentModelConfig model_config;
  model_config.model_type = scenario_config.model_type;

  using Model = foundation::radiative_transfer::RadiativeTransferModel;
  if (scenario_config.preset == EosEnvironmentPreset::kHumid) {
    model_config.radiative_transfer_model = Model::kHumidityWeighted;
    model_config.aerosol_density_factor = 1.1f;
    model_config.turbulence_factor = 1.1f;
  } else if (scenario_config.preset == EosEnvironmentPreset::kDusty) {
    model_config.radiative_transfer_model = Model::kAdaptivePathRadiance;
    model_config.aerosol_density_factor = 2.0f;
    model_config.turbulence_factor = 1.2f;
  } else if (scenario_config.preset == EosEnvironmentPreset::kTurbulent) {
    model_config.radiative_transfer_model = Model::kAdaptivePathRadiance;
    model_config.aerosol_density_factor = 1.3f;
    model_config.turbulence_factor = 1.8f;
  } else if (scenario_config.preset == EosEnvironmentPreset::kMaritime) {
    model_config.radiative_transfer_model = Model::kHumidityWeighted;
    model_config.aerosol_density_factor = 1.5f;
    model_config.turbulence_factor = 1.4f;
  } else {
    model_config.radiative_transfer_model = Model::kDerivedBeerLambert;
    model_config.aerosol_density_factor = 1.0f;
    model_config.turbulence_factor = 1.0f;
  }

  if (scenario_config.has_custom_overrides) {
    model_config.radiative_transfer_model =
        scenario_config.custom_overrides.radiative_transfer_model;
    model_config.aerosol_density_factor =
        scenario_config.custom_overrides.aerosol_density_factor;
    model_config.turbulence_factor = scenario_config.custom_overrides.turbulence_factor;
    model_config.enable_optical_countermeasure_extension =
        scenario_config.custom_overrides.enable_optical_countermeasure_extension;
  }

  return model_config;
}

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_H_
