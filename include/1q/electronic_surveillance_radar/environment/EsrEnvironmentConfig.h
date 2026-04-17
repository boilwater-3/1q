/**
 * @file EsrEnvironmentConfig.h
 * @brief ESR 环境默认配置契约。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrAtmosphericPhysicsConfig 描述可选物理传播参数。
 */
struct ONEQ_API EsrAtmosphericPhysicsConfig {
  bool enable_physical_model{false}; /**< 是否启用物理传播模型 */
  float frequency_hz{10.0e9f};       /**< 雷达频率（单位：Hz） */
  float path_length_m{10.0e3f};      /**< 传播路径长度（单位：m） */
  float radar_altitude_m{1.0e3f};    /**< 雷达高度（单位：m） */
  float target_altitude_m{1.0e3f};   /**< 目标高度（单位：m） */
  float elevation_deg{5.0f};         /**< 传播仰角（单位：deg） */
  float pressure_hpa{1013.25f};      /**< 气压（单位：hPa） */
  float temperature_k{288.15f};      /**< 温度（单位：K） */
  float relative_humidity{0.5f};     /**< 相对湿度 [0, 1] */
  float k_factor{4.0f / 3.0f};       /**< 地球有效半径因子 */
  std::int32_t day_of_year{172};     /**< 年积日 [1, 366] */
  float solar_flux_f107a{150.0f};    /**< 平滑太阳流量指数 */
  float solar_flux_f107{150.0f};     /**< 当日太阳流量指数 */
  float geomagnetic_ap{4.0f};        /**< 地磁活动指数 */
};

/**
 * @brief EsrClutterBaselinePolicy 描述内部杂波基线策略。
 */
enum class ONEQ_API EsrClutterBaselinePolicy {
  kLow = 0,      /**< 低杂波基线 */
  kStandard = 1, /**< 标准杂波基线 */
  kHigh = 2      /**< 高杂波基线 */
};

/**
 * @brief EsrJammingSensitivityPolicy 描述干扰检测敏感性策略。
 */
enum class ONEQ_API EsrJammingSensitivityPolicy {
  kRelaxed = 0,  /**< 更保守，不易判定为干扰 */
  kBalanced = 1, /**< 平衡策略 */
  kStrict = 2    /**< 更敏感，容易判定为干扰 */
};

/**
 * @brief EsrEnvironmentModelConfig 描述环境策略配置。
 */
struct ONEQ_API EsrEnvironmentModelConfig {
  EsrClutterBaselinePolicy clutter_baseline_policy{
      EsrClutterBaselinePolicy::kStandard}; /**< 杂波基线策略 */
  EsrJammingSensitivityPolicy jamming_sensitivity_policy{
      EsrJammingSensitivityPolicy::kBalanced}; /**< 干扰检测敏感性策略 */
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
