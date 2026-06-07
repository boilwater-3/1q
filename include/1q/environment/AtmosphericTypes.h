/**
 * @file AtmosphericTypes.h
 * @brief 定义跨模块复用的大气观测与空间天气上下文类型。
 *
 * 统一 AR/ESR/EOS 各模块中重复定义的大气输入类型。
 * 字段与默认值与 foundation::AtmosphericObservation / foundation::SpaceWeatherContext 完全一致。
 */

#ifndef ONEQ_ENVIRONMENT_ATMOSPHERIC_TYPES_H_
#define ONEQ_ENVIRONMENT_ATMOSPHERIC_TYPES_H_

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "1q/api.hpp"

namespace oneq {
namespace environment {

/**
 * @brief AtmosphericObservation 描述基础气象观测输入。
 */
struct ONEQ_API AtmosphericObservation {
  bool enable_physical_model{false}; /**< 是否启用物理传播模型 */
  float pressure_hpa{1013.25f};      /**< 气压（单位：hPa） */
  float temperature_k{288.15f};      /**< 温度（单位：K） */
  float relative_humidity{0.5f};     /**< 相对湿度，范围 [0, 1] */
};

/**
 * @brief SpaceWeatherContext 描述时间/空间天气高级上下文输入。
 */
struct ONEQ_API SpaceWeatherContext {
  bool has_k_factor{false};        /**< 是否显式提供地球有效半径因子 */
  float k_factor{4.0f / 3.0f};     /**< 地球有效半径因子 */
  bool has_day_of_year{false};     /**< 是否显式提供年积日 */
  std::int32_t day_of_year{172};   /**< 年积日 [1, 366] */
  float solar_flux_f107a{150.0f};  /**< 平滑太阳流量指数 */
  float solar_flux_f107{150.0f};   /**< 当日太阳流量指数 */
  float geomagnetic_ap{4.0f};      /**< 地磁活动指数 */
  bool has_simulation_unix_seconds{false};   /**< 是否显式提供仿真 UTC 秒级时间戳 */
  std::int64_t simulation_unix_seconds{0};   /**< 仿真 UTC 秒级时间戳（Unix epoch） */
};

/**
 * @brief 推导有效 k_factor（优先显式输入，否则使用默认近似）。
 */
inline float ResolveEffectiveKFactor(const SpaceWeatherContext& context) {
  if (context.has_k_factor) {
    return context.k_factor;
  }
  return 4.0f / 3.0f;
}

/**
 * @brief 从基础气象观测推导有效 k_factor（基于近地层折射率梯度）。
 *
 * 基于 ITU-R P.453 近地层折射率公式，从温度、气压、湿度估计折射率梯度，
 * 映射到有效地球半径因子 k。
 *
 * @param[in] obs 基础气象观测（温度、气压、湿度）。
 * @return 有效地球半径因子 k。
 */
inline float ResolveEffectiveKFactor(const AtmosphericObservation& obs) {
  const float temperature_k = (obs.temperature_k > 1.0f) ? obs.temperature_k : 288.15f;
  const float pressure_hpa = (obs.pressure_hpa > 0.0f) ? obs.pressure_hpa : 1013.25f;
  const float relative_humidity = std::max(0.0f, std::min(1.0f, obs.relative_humidity));

  const float temperature_c = temperature_k - 273.15f;
  const float saturation_vapor_pressure_hpa =
      6.1121f * std::exp((17.502f * temperature_c) / (240.97f + temperature_c));
  const float water_vapor_pressure_hpa = relative_humidity * saturation_vapor_pressure_hpa;

  const float refractivity_n =
      77.6f * (pressure_hpa / temperature_k) +
      3.73e5f * (water_vapor_pressure_hpa / (temperature_k * temperature_k));

  const float refractivity_scale_height_km = 7.35f;
  const float dndh_n_per_km = -refractivity_n / refractivity_scale_height_km;

  const float denominator = 1.0f + dndh_n_per_km / 157.0f;
  if (denominator <= 0.1f) {
    return 4.0f / 3.0f;
  }
  const float derived_k_factor = 1.0f / denominator;
  if (derived_k_factor < 0.5f || derived_k_factor > 2.5f) {
    return 4.0f / 3.0f;
  }
  return derived_k_factor;
}

/**
 * @brief 从 Unix 秒级时间戳推导年积日。
 * @param[in] unix_seconds Unix epoch 秒数。
 * @return 年积日 [1, 366]。
 */
ONEQ_API std::int32_t ResolveEffectiveDayOfYearFromUnix(std::int64_t unix_seconds);

/**
 * @brief 推导有效 day_of_year。
 *
 * 优先级：has_day_of_year > has_simulation_unix_seconds（推导）> 默认 172。
 */
inline std::int32_t ResolveEffectiveDayOfYear(const SpaceWeatherContext& context) {
  if (context.has_day_of_year) {
    return context.day_of_year;
  }
  if (context.has_simulation_unix_seconds) {
    return ResolveEffectiveDayOfYearFromUnix(context.simulation_unix_seconds);
  }
  return 172;
}

}  // namespace environment
}  // namespace oneq

#endif  // ONEQ_ENVIRONMENT_ATMOSPHERIC_TYPES_H_
