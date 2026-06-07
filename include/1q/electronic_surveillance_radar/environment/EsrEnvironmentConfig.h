/**
 * @file EsrEnvironmentConfig.h
 * @brief ESR 环境配置契约（Scenario/Model/Default）。
 *
 * @par 迁移模板
 * 参考 AR 环境配置迁移标准模板（EnvironmentConfig.h）。
 * 三层结构：ScenarioConfig（外部输入）→ ModelConfig（内部消费）→ DefaultConfig（初始化）。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_

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

}  // namespace config

namespace environment {

/** @brief EsrAtmosphericPhysicsConfig 复用统一环境模块基础气象观测类型。 */
using EsrAtmosphericPhysicsConfig = oneq::environment::AtmosphericObservation;

/** @brief EsrAtmosphericDerivedContext 复用统一环境模块空间天气上下文类型。 */
using EsrAtmosphericDerivedContext = oneq::environment::SpaceWeatherContext;

// 注意：ResolveEffectiveKFactor / ResolveEffectiveDayOfYear 的 ESR 命名空间包装
// 已移除。类型统一后，调用方直接使用 oneq::environment 中的 inline 版本即可。
// 需要使用时 include "1q/environment/AtmosphericTypes.h"。

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
 * @brief EsrEnvironmentModelConfig 描述环境服务/算法执行直接消费的参数。
 *
 * @par 类型合约
 * - 独立于 EsrEnvironmentScenarioConfig 的 struct，禁止退化为 type alias。
 * - 字段与 ScenarioConfig 一致时，通过 BuildModelConfigFromScenario 显式字段映射构造。
 * - 若未来需差异化（增加派生字段或移除场景字段），直接修改本 struct 并更新映射函数。
 * - 调用方不得假设 ModelConfig 与 ScenarioConfig 同型。
 */
struct ONEQ_API EsrEnvironmentModelConfig {
  config::EsrEnvironmentPreset preset{config::EsrEnvironmentPreset::kStandard}; /**< 环境预设语义 */
  EsrAtmosphericPhysicsConfig atmospheric_physics{};  /**< 气象观测参数 */
  EsrAtmosphericDerivedContext atmospheric_context{}; /**< 时间/空间天气上下文 */
};

/**
 * @brief 将对外场景输入映射为内部环境模型配置。
 *
 * @par 映射合约
 * - 显式字段映射：逐字段从 ScenarioConfig 拷贝到 ModelConfig。
 * - 若未来引入非恒等映射（字段转换、派生、过滤），须同步添加映射单元测试。
 * - 无 fallback 语义：输入合法即输出合法。
 *
 * @param[in] scenario_config 外部场景输入。
 * @return 逐字段拷贝后的模型配置。
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

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_
