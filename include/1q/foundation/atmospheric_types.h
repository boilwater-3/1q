/**
 * @file atmospheric_types.h
 * @brief 定义跨模块复用的大气观测与空间天气上下文类型。
 */

#ifndef ONEQ_FOUNDATION_ATMOSPHERIC_TYPES_H_
#define ONEQ_FOUNDATION_ATMOSPHERIC_TYPES_H_

#include <cstdint>

#include "1q/api.hpp"

namespace oneq {
namespace foundation {

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
 * @brief 推导有效 day_of_year（优先显式输入，否则使用默认近似）。
 */
inline std::int32_t ResolveEffectiveDayOfYear(const SpaceWeatherContext& context) {
  if (context.has_day_of_year) {
    return context.day_of_year;
  }
  return 172;
}

}  // namespace foundation
}  // namespace oneq

#endif  // ONEQ_FOUNDATION_ATMOSPHERIC_TYPES_H_
