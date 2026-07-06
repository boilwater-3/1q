/**
 * @file SbirsEnvironmentConfig.h
 * @brief 定义 SBIRS-inspired 环境和气象衰减参数。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ENVIRONMENT_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

enum class ONEQ_API SbirsWeatherType { kClear = 0, kCloudy, kRain, kFog };

enum class ONEQ_API SbirsSeaState { kLow = 0, kMedium, kHigh };

struct ONEQ_API SbirsEnvironmentConfig {
  SbirsWeatherType weather_type{SbirsWeatherType::kClear};
  SbirsSeaState sea_state{SbirsSeaState::kLow};
  float temperature_c{15.0f};
  float relative_humidity_percent{50.0f};
  float visibility_km{20.0f};
  float base_atmospheric_transmittance{0.8f};
  // 2.9 气象交互项权重：A_total 加权叠加中的交互项系数 k_j（默认 0 = 无交互项，
  // 保持向后兼容）。单位：无量纲。当前仅 humidity×visibility 与 rain×humidity 两项。
  float humidity_visibility_interaction_weight{0.0f};
  float rain_humidity_interaction_weight{0.0f};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ENVIRONMENT_CONFIG_H_
