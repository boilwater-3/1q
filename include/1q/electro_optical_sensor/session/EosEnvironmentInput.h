/**
 * @file EosEnvironmentInput.h
 * @brief 定义 EOS 单周期环境输入聚合类型。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief DayNightType 表示昼夜环境类型。
 */
enum class DayNightType {
  kDay = 0,  /**< 白天 */
  kNight,    /**< 夜间 */
  kTwilight  /**< 晨昏 */
};

/**
 * @brief EosEnvironmentInput 聚合 EOS 单周期环境事实输入。
 */
struct ONEQ_API EosEnvironmentInput {
  float solar_altitude_deg{45.0f};        /**< 太阳高度角（单位：deg） */
  float solar_azimuth_deg{180.0f};        /**< 太阳方位角（单位：deg） */
  float solar_irradiance_w_m2{800.0f};    /**< 地表太阳辐照度（单位：W/m^2） */
  float cloud_coverage_ratio{0.2f};       /**< 云量，范围 [0, 1] */
  float ambient_wind_speed_mps{0.0f};     /**< 环境风速（单位：m/s，范围 [0, +inf)） */
  DayNightType day_night_type{DayNightType::kDay}; /**< 昼夜环境类型 */
  float background_temperature_k{290.0f}; /**< 背景温度（单位：K） */
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_
