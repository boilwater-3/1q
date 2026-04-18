/**
 * @file EsrEnvironmentConfig.h
 * @brief ESR 环境默认配置契约。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentPreset.h"
#include "1q/foundation/atmospheric_types.h"

namespace electronic_surveillance_radar {
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
 * @brief EsrEnvironmentModelConfig 描述环境策略配置。
 */
struct ONEQ_API EsrEnvironmentModelConfig {
  config::EsrEnvironmentPreset preset{config::EsrEnvironmentPreset::kStandard}; /**< 环境预设语义 */
  EsrAtmosphericPhysicsConfig atmospheric_physics{};  /**< 可选基础气象观测参数 */
  EsrAtmosphericDerivedContext atmospheric_context{}; /**< 可选时间/空间天气高级上下文 */
};

/**
 * @brief EsrEnvironmentDefaultConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EsrEnvironmentDefaultConfig {
  EsrEnvironmentModelConfig model_config{}; /**< 默认环境模型配置 */
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_
