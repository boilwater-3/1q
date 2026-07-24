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

/**
 * @brief EsrEnvironmentScenarioConfig 描述环境场景语义输入。
 *
 * @note 不暴露 SpaceWeatherContext（空间天气上下文）：其字段（k_factor、
 *       day_of_year、solar_flux、geomagnetic_ap、simulation_unix_seconds）在当前
 *       GTD7 大气模型退化为 ISA 标准大气的情况下全部未被消费，属未接入的死输入。
 */
struct ONEQ_API EsrEnvironmentScenarioConfig {
  EsrEnvironmentPreset preset{EsrEnvironmentPreset::kStandard};
  EsrAtmosphericPhysicsConfig atmospheric_physics{};
};

/**
 * @brief EsrEnvironmentConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EsrEnvironmentConfig {
  EsrEnvironmentScenarioConfig scenario_config{};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_
