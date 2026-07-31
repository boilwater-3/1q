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
 * @brief EsrPropagationEnvironmentProfile 描述高层传播环境类型。
 */
enum class ONEQ_API EsrPropagationEnvironmentProfile { kOpen = 0, kTypical = 1, kComplex = 2 };

/**
 * @brief EsrClutterDensityLevel 描述高层杂波密度级别。
 */
enum class ONEQ_API EsrClutterDensityLevel { kLow = 0, kMedium = 1, kHigh = 2 };

/**
 * @brief EsrAtmosphericObservation 描述外部可观测天气事实。
 */
/**
 * @brief EsrAtmosphericObservation 描述外部可观测天气事实。
 *
 * @note 湿度统一由 EsrAtmosphericPhysicsConfig::relative_humidity 表达，
 *       本结构仅保留降水和能见度等物理模型未覆盖的观测。
 */
struct ONEQ_API EsrAtmosphericObservation {
  float precipitation_rate_mmph{0.0f}; /**< 降水率（单位：mm/h） */
  float visibility_km{20.0f};          /**< 能见度（单位：km） */
};

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
  EsrPropagationEnvironmentProfile propagation_profile{EsrPropagationEnvironmentProfile::kTypical};
  EsrClutterDensityLevel clutter_density{EsrClutterDensityLevel::kMedium};
  float spectrum_occupancy_ratio{0.0f};
  EsrAtmosphericObservation atmospheric_observation{};
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
