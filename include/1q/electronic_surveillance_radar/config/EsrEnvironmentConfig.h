/**
 * @file EsrEnvironmentConfig.h
 * @brief ESR 环境配置契约（Scenario/Model/Default）。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/environment/AtmosphericTypes.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrEnvironmentPreset 描述对外环境预设。
 */
enum class ONEQ_API EsrEnvironmentPreset {
  kStandard = 0, /**< 标准环境 */
  kLowClutter,   /**< 低杂波环境 */
  kDenseClutter, /**< 高杂波环境 */
  kJammed        /**< 强干扰环境 */
};

/** @brief EsrAtmosphericPhysicsConfig 复用统一环境模块基础气象观测类型。 */
using EsrAtmosphericPhysicsConfig = oneq::environment::AtmosphericObservation;

/** @brief EsrAtmosphericDerivedContext 复用统一环境模块空间天气上下文类型。 */
using EsrAtmosphericDerivedContext = oneq::environment::SpaceWeatherContext;

/**
 * @brief EsrEnvironmentScenarioConfig 描述环境场景语义输入。
 */
struct ONEQ_API EsrEnvironmentScenarioConfig {
  EsrEnvironmentPreset preset{EsrEnvironmentPreset::kStandard};
  EsrAtmosphericPhysicsConfig atmospheric_physics{};
  EsrAtmosphericDerivedContext atmospheric_context{};
};

/**
 * @brief EsrEnvironmentModelConfig 描述环境服务直接消费的参数。
 */
struct ONEQ_API EsrEnvironmentModelConfig {
  EsrEnvironmentPreset preset{EsrEnvironmentPreset::kStandard};
  EsrAtmosphericPhysicsConfig atmospheric_physics{};
  EsrAtmosphericDerivedContext atmospheric_context{};
};

/**
 * @brief 将对外场景输入映射为内部环境模型配置。
 */
inline EsrEnvironmentModelConfig BuildModelConfigFromScenario(
    const EsrEnvironmentScenarioConfig& scenario_config) {
  EsrEnvironmentModelConfig model_config;
  model_config.preset = scenario_config.preset;
  model_config.atmospheric_physics = scenario_config.atmospheric_physics;
  model_config.atmospheric_context = scenario_config.atmospheric_context;
  return model_config;
}

/**
 * @brief EsrEnvironmentConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EsrEnvironmentConfig {
  EsrEnvironmentScenarioConfig scenario_config{};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_
