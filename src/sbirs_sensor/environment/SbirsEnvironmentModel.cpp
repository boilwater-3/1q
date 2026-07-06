#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"

#include <algorithm>

namespace sbirs_sensor {
namespace environment {
namespace {

float WeatherAttenuation(config::SbirsWeatherType weather_type) {
  switch (weather_type) {
    case config::SbirsWeatherType::kCloudy:
      return 0.05f;
    case config::SbirsWeatherType::kRain:
      return 0.15f;
    case config::SbirsWeatherType::kFog:
      return 0.20f;
    case config::SbirsWeatherType::kClear:
    default:
      return 0.0f;
  }
}

float SeaAttenuation(config::SbirsSeaState sea_state) {
  switch (sea_state) {
    case config::SbirsSeaState::kMedium:
      return 0.10f;
    case config::SbirsSeaState::kHigh:
      return 0.15f;
    case config::SbirsSeaState::kLow:
    default:
      return 0.05f;
  }
}

}  // namespace

float ResolveWeatherAttenuation(const config::SbirsEnvironmentConfig& environment) {
  const float weather = WeatherAttenuation(environment.weather_type);
  const float sea = SeaAttenuation(environment.sea_state);
  const float humidity =
      std::max(0.0f, environment.relative_humidity_percent - 50.0f) / 20.0f * 0.05f;
  const float visibility =
      environment.visibility_km < 1.0f
          ? 0.20f
          : (environment.visibility_km < 5.0f ? 0.10f
                                              : (environment.visibility_km < 10.0f ? 0.05f : 0.0f));
  const float temperature_relief =
      std::max(0.0f, environment.temperature_c - 15.0f) / 10.0f * 0.02f;
  const float attenuation = 0.25f * weather + 0.15f * sea + 0.25f * humidity + 0.25f * visibility -
                            0.10f * temperature_relief;
  return std::max(0.0f, std::min(1.0f, attenuation));
}

float ResolveEffectiveTransmittance(const config::SbirsEnvironmentConfig& environment) {
  const float base = std::max(0.0f, std::min(1.0f, environment.base_atmospheric_transmittance));
  return base * (1.0f - ResolveWeatherAttenuation(environment));
}

}  // namespace environment
}  // namespace sbirs_sensor
