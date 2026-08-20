/**
 * @file AtmosphericTypes.h
 * @brief 定义跨模块复用的大气观测类型。
 *
 * 统一 AR/ESR/EOS 各模块中重复定义的大气输入类型，作为唯一公开来源。
 */

#ifndef ONEQ_ENVIRONMENT_ATMOSPHERIC_TYPES_H_
#define ONEQ_ENVIRONMENT_ATMOSPHERIC_TYPES_H_

#include <algorithm>
#include <cmath>

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

}  // namespace environment
}  // namespace oneq

#endif  // ONEQ_ENVIRONMENT_ATMOSPHERIC_TYPES_H_
