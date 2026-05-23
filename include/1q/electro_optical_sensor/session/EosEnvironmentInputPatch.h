/**
 * @file EosEnvironmentInputPatch.h
 * @brief 定义 EOS 单周期环境输入状态的局部更新载荷。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_PATCH_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_PATCH_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosEnvironmentInputPatch 表示调用方侧环境事实状态的局部更新。
 *
 * @note 本类型不直接进入 EosSession::StepWithResult()。调用方应先用
 *       EosEnvironmentInputState 合成完整 EosEnvironmentInput 快照，再写入
 *       EosCycleInput::environment。
 */
struct ONEQ_API EosEnvironmentInputPatch {
  bool has_solar_altitude_deg{false};              /**< 是否更新太阳高度角 */
  float solar_altitude_deg{45.0f};                 /**< 新太阳高度角（单位：deg） */
  bool has_solar_azimuth_deg{false};               /**< 是否更新太阳方位角 */
  float solar_azimuth_deg{180.0f};                 /**< 新太阳方位角（单位：deg） */
  bool has_solar_irradiance_w_m2{false};           /**< 是否更新太阳辐照度 */
  float solar_irradiance_w_m2{800.0f};             /**< 新太阳辐照度（单位：W/m^2） */
  bool has_cloud_coverage_ratio{false};            /**< 是否更新云量 */
  float cloud_coverage_ratio{0.2f};                /**< 新云量，范围 [0, 1] */
  bool has_ambient_wind_speed_mps{false};          /**< 是否更新环境风速 */
  float ambient_wind_speed_mps{0.0f};              /**< 新环境风速（单位：m/s） */
  bool has_day_night_type{false};                  /**< 是否更新昼夜类型 */
  DayNightType day_night_type{DayNightType::kDay}; /**< 新昼夜类型 */
  bool has_background_temperature_k{false};        /**< 是否更新背景温度 */
  float background_temperature_k{290.0f};          /**< 新背景温度（单位：K） */
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_PATCH_H_
