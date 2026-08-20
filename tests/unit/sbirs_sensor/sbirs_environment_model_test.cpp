#include <gtest/gtest.h>

#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"

namespace {

TEST(SbirsEnvironmentModelTest, WeatherReducesEffectiveTransmittance) {
  sbirs_sensor::config::SbirsEnvironmentConfig clear_config;
  clear_config.weather_type = sbirs_sensor::config::SbirsWeatherType::kClear;
  sbirs_sensor::config::SbirsEnvironmentConfig fog_config;
  fog_config.weather_type = sbirs_sensor::config::SbirsWeatherType::kFog;

  const float clear_transmittance =
      sbirs_sensor::environment::ResolveEffectiveTransmittance(clear_config);
  const float fog_transmittance =
      sbirs_sensor::environment::ResolveEffectiveTransmittance(fog_config);
  EXPECT_GT(clear_transmittance, fog_transmittance);
}

// design 2.9：交互项 k_j·A_p·A_q 应在湿度×能见度都贡献衰减时放大总衰减。
TEST(SbirsEnvironmentModelTest, HumidityVisibilityInteractionRaisesAttenuation) {
  sbirs_sensor::config::SbirsEnvironmentConfig base;
  base.relative_humidity_percent = 90.0f;   // 湿度贡献 > 0
  base.visibility_km = 0.5f;                // 能见度贡献 > 0
  const float base_attenuation = sbirs_sensor::environment::ResolveWeatherAttenuation(base);

  sbirs_sensor::config::SbirsEnvironmentConfig with_interaction = base;
  with_interaction.humidity_visibility_interaction_weight = 5.0f;
  const float interaction_attenuation =
      sbirs_sensor::environment::ResolveWeatherAttenuation(with_interaction);

  EXPECT_GT(interaction_attenuation, base_attenuation);
}

// design 2.9：雨×湿度交互项仅在雨天才生效。
TEST(SbirsEnvironmentModelTest, RainHumidityInteractionOnlyAppliesInRain) {
  sbirs_sensor::config::SbirsEnvironmentConfig clear_humid;
  clear_humid.weather_type = sbirs_sensor::config::SbirsWeatherType::kClear;
  clear_humid.relative_humidity_percent = 90.0f;
  clear_humid.rain_humidity_interaction_weight = 5.0f;
  const float clear_att = sbirs_sensor::environment::ResolveWeatherAttenuation(clear_humid);

  sbirs_sensor::config::SbirsEnvironmentConfig rain_humid = clear_humid;
  rain_humid.weather_type = sbirs_sensor::config::SbirsWeatherType::kRain;
  const float rain_att = sbirs_sensor::environment::ResolveWeatherAttenuation(rain_humid);

  EXPECT_GT(rain_att, clear_att);
}

}  // namespace
