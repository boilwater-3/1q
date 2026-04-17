/**
 * @file EsrEnvironmentConfig.h
 * @brief ESR 环境默认配置契约。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentPolicyConfig.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrAtmosphericPhysicsConfig 描述可选物理传播参数。
 */
struct ONEQ_API EsrAtmosphericPhysicsConfig {
  bool enable_physical_model{false}; /**< 是否启用物理传播模型 */
  float pressure_hpa{1013.25f};      /**< 气压（单位：hPa） */
  float temperature_k{288.15f};      /**< 温度（单位：K） */
  float relative_humidity{0.5f};     /**< 相对湿度 [0, 1] */
  bool has_k_factor{false};          /**< 是否显式提供地球有效半径因子 */
  float k_factor{4.0f / 3.0f};       /**< 地球有效半径因子 */
  bool has_day_of_year{false};       /**< 是否显式提供年积日 */
  std::int32_t day_of_year{172};     /**< 年积日 [1, 366] */
  float solar_flux_f107a{150.0f};    /**< 平滑太阳流量指数 */
  float solar_flux_f107{150.0f};     /**< 当日太阳流量指数 */
  float geomagnetic_ap{4.0f};        /**< 地磁活动指数 */
};

/**
 * @brief 推导有效 k_factor（优先显式输入，否则使用默认近似）。
 */
inline float ResolveEffectiveKFactor(const EsrAtmosphericPhysicsConfig& config) {
  if (config.has_k_factor) {
    return config.k_factor;
  }
  return 4.0f / 3.0f;
}

/**
 * @brief 推导有效 day_of_year（优先显式输入，否则使用默认近似）。
 */
inline std::int32_t ResolveEffectiveDayOfYear(const EsrAtmosphericPhysicsConfig& config) {
  if (config.has_day_of_year) {
    return config.day_of_year;
  }
  return 172;
}

/**
 * @brief EsrEnvironmentModelConfig 描述环境策略配置。
 */
struct ONEQ_API EsrEnvironmentModelConfig {
  config::EsrEnvironmentPreset preset{config::EsrEnvironmentPreset::kStandard}; /**< 环境预设语义 */
  EsrAtmosphericPhysicsConfig atmospheric_physics{}; /**< 可选物理传播参数 */
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
