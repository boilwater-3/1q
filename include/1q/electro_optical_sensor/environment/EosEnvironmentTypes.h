/**
 * @file EosEnvironmentTypes.h
 * @brief EOS 环境模型输入输出类型契约。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_TYPES_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_TYPES_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief DayNightType 表示昼夜环境类型。
 */
enum class ONEQ_API DayNightType {
  kDay = 0, /**< 白天 */
  kNight,   /**< 夜间 */
  kTwilight /**< 晨昏 */
};

/**
 * @brief EosEnvironmentObservation 描述 EOS 单周期环境高层观测输入。
 */
struct ONEQ_API EosEnvironmentObservation {
  float solar_altitude_deg{45.0f};                 /**< 太阳高度角（单位：deg） */
  float solar_azimuth_deg{180.0f};                 /**< 太阳方位角（单位：deg） */
  float solar_irradiance_w_m2{800.0f};             /**< 地表太阳辐照度（单位：W/m^2） */
  float cloud_coverage_ratio{0.2f};                /**< 云量，范围 [0, 1] */
  float ambient_wind_speed_mps{0.0f};              /**< 环境风速（单位：m/s，范围 [0, +inf)） */
  DayNightType day_night_type{DayNightType::kDay}; /**< 昼夜环境类型 */
  float background_temperature_k{290.0f};          /**< 背景温度（单位：K） */
};

/**
 * @brief EosEnvironmentModelInputs 描述环境模型输入。
 */
struct ONEQ_API EosEnvironmentModelInputs {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};
  float base_aerosol_density_factor{1.0f};              /**< 预设/自定义气溶胶密度因子 */
  float base_turbulence_factor{1.0f};                   /**< 预设/自定义湍流因子 */
  float platform_altitude_m{0.0f};
  float cloud_coverage_ratio{0.0f};
  float wind_speed_mps{0.0f};
  bool has_atmospheric_observation{false};              /**< 是否提供大气物理观测输入 */
  oneq::environment::AtmosphericObservation atmospheric_observation{}; /**< 可选大气物理观测 */
};

/**
 * @brief EosEnvironmentModelResult 描述环境模型输出。
 */
struct ONEQ_API EosEnvironmentModelResult {
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
  float path_radiance_scale_bias{1.0f};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_TYPES_H_
