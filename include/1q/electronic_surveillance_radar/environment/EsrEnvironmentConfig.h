/**
 * @file EsrEnvironmentConfig.h
 * @brief ESR 环境配置契约（Scenario/Model/Default）。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/foundation/atmospheric_types.h"

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

}  // namespace config

namespace environment {

/** @brief EsrAtmosphericPhysicsConfig 复用 foundation 层统一基础气象观测类型。 */
using EsrAtmosphericPhysicsConfig = oneq::foundation::AtmosphericObservation;

/** @brief EsrAtmosphericDerivedContext 复用 foundation 层统一空间天气上下文类型。 */
using EsrAtmosphericDerivedContext = oneq::foundation::SpaceWeatherContext;

/**
 * @brief 推导有效 k_factor（优先显式输入，否则使用默认近似）。
 */
inline float ResolveEffectiveKFactor(const EsrAtmosphericDerivedContext& context) {
  return oneq::foundation::ResolveEffectiveKFactor(context);
}

/**
 * @brief 推导有效 day_of_year（优先显式输入，否则使用默认近似）。
 */
inline std::int32_t ResolveEffectiveDayOfYear(const EsrAtmosphericDerivedContext& context) {
  return oneq::foundation::ResolveEffectiveDayOfYear(context);
}

/**
 * @brief EsrEnvironmentScenarioConfig 描述环境场景语义输入。
 *
 * 仅承载场景事实与语义输入，不直接用于运行态执行。
 */
struct ONEQ_API EsrEnvironmentScenarioConfig {
  config::EsrEnvironmentPreset preset{config::EsrEnvironmentPreset::kStandard}; /**< 环境预设语义 */
  EsrAtmosphericPhysicsConfig atmospheric_physics{};  /**< 可选基础气象观测参数 */
  EsrAtmosphericDerivedContext atmospheric_context{}; /**< 可选时间/空间天气高级上下文 */
};

/**
 * @brief EsrEnvironmentModelConfig 描述环境模型执行输入。
 */
struct ONEQ_API EsrEnvironmentModelConfig {
  config::EsrEnvironmentPreset preset{config::EsrEnvironmentPreset::kStandard}; /**< 环境预设语义 */
  EsrAtmosphericPhysicsConfig atmospheric_physics{};  /**< 可选基础气象观测参数 */
  EsrAtmosphericDerivedContext atmospheric_context{}; /**< 可选时间/空间天气高级上下文 */
};

/**
 * @brief 将场景语义输入映射为模型执行输入。
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
 * @brief EsrEnvironmentDefaultConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EsrEnvironmentDefaultConfig {
  EsrEnvironmentScenarioConfig scenario_config{}; /**< 默认环境场景配置 */
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_
