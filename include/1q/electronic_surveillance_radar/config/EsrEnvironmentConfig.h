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
 * @brief 环境服务当前直接消费场景事实，复用唯一配置类型权威。
 *
 * 当前不存在执行态专属字段；若未来出现，应基于运行路径证据新增内部执行配置，
 * 而不是复制一套同型公开 DTO。
 */
using EsrEnvironmentModelConfig = EsrEnvironmentScenarioConfig;

/**
 * @brief 将对外场景输入映射为内部环境模型配置。
 */
inline EsrEnvironmentModelConfig BuildModelConfigFromScenario(
    const EsrEnvironmentScenarioConfig& scenario_config) {
  return scenario_config;
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
