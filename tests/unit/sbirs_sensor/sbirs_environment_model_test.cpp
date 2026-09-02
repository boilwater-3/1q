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

// ===== 壳段气团（冻结契约 2026-09-02）：X = 穿壳弦长÷垂直壳厚，τ_geo = τ_eff^X =====
// 几何基准：地球平均半径 6371 km、壳顶 +100 km（kAtmosphereShellThicknessM）。

using sbirs_sensor::environment::ComputeShellAirmassFactor;
using sbirs_sensor::environment::ResolveGeometricTransmittance;
using sbirs_sensor::session::SbirsVector3M;

// 纯空间路径（双端在壳外、近地点高于壳顶）不穿大气：X=0，τ_geo=1，雾天也不变。
TEST(SbirsEnvironmentModelTest, SpaceToSpacePathSkipsAtmosphere) {
  const SbirsVector3M satellite(0.0, 0.0, 6371000.0 + 35778000.0);  // GEO 天顶方向
  const SbirsVector3M target(0.0, 0.0, 6371000.0 + 507000.0);       // LEO，近地点=自身
  EXPECT_NEAR(ComputeShellAirmassFactor(satellite, target), 0.0, 1.0e-12);

  sbirs_sensor::config::SbirsEnvironmentConfig fog;
  fog.weather_type = sbirs_sensor::config::SbirsWeatherType::kFog;
  EXPECT_FLOAT_EQ(ResolveGeometricTransmittance(fog, satellite, target), 1.0f);
}

// 地面目标正对天顶：穿壳弦长=垂直壳厚 → X=1，τ_geo=τ_eff（与旧口径向后兼容基准）。
TEST(SbirsEnvironmentModelTest, GroundZenithPathMatchesVerticalAirmass) {
  const SbirsVector3M satellite(0.0, 0.0, 6371000.0 + 1000000.0);
  const SbirsVector3M target(0.0, 0.0, 6371000.0);
  EXPECT_NEAR(ComputeShellAirmassFactor(satellite, target), 1.0, 1.0e-9);

  const sbirs_sensor::config::SbirsEnvironmentConfig env;  // 默认晴：τ_eff=0.8×0.9925
  EXPECT_NEAR(ResolveGeometricTransmittance(env, satellite, target), 0.794f, 1.0e-6);
}

// 目标在壳内（10 km 高空）天顶方向：穿壳弦长=90 km → X=0.9（目标越高穿壳越短）。
TEST(SbirsEnvironmentModelTest, AirborneTargetZenithPartialShell) {
  const SbirsVector3M satellite(0.0, 0.0, 6471000.0 + 1000000.0);
  const SbirsVector3M target(0.0, 0.0, 6371000.0 + 10000.0);
  EXPECT_NEAR(ComputeShellAirmassFactor(satellite, target), 0.9, 1.0e-9);
}

// 双端在壳外但路径横穿大气壳（近地点 50 km、弦中点在段中央）：X 超 10 被封顶。
TEST(SbirsEnvironmentModelTest, GrazingPathAirmassClamped) {
  // 近地点 Q=(6421000, 0, 0)（50 km 高），切向（+y）两端各 1000 km：
  // 端点半径 √(6421²+1000²)≈6498 km 在壳外；穿半弦=√(6471²−6421²)≈802.9 km
  // → X=2×802.9/100≈16.1，超过 kMaxShellAirmassFactor=10 被夹住。
  const SbirsVector3M target(6421000.0, 1000000.0, 0.0);
  const SbirsVector3M satellite(6421000.0, -1000000.0, 0.0);
  EXPECT_DOUBLE_EQ(ComputeShellAirmassFactor(satellite, target),
                   sbirs_sensor::environment::kMaxShellAirmassFactor);
}

// 退化输入：星目重合（零长度段）→ X=0。
TEST(SbirsEnvironmentModelTest, CoincidentPositionsGiveZeroAirmass) {
  const SbirsVector3M satellite(7000000.0, 1.0, 2.0);
  EXPECT_DOUBLE_EQ(ComputeShellAirmassFactor(satellite, satellite), 0.0);
}

}  // namespace
