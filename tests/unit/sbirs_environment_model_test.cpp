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

}  // namespace
