#include <gtest/gtest.h>

#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"
#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"

namespace {

TEST(SbirsFoundationTest, ReceivedPowerScalesLinearlyWithRadiantIntensity) {
  // P = I_t · A_ap · τ_opt · τ_atm · η / d²：辐射强度加倍 → 接收功率加倍。
  const double dim_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      1.0e4, 1.0e6, 0.5, 0.8, 0.8, 0.7);
  const double bright_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      2.0e4, 1.0e6, 0.5, 0.8, 0.8, 0.7);
  EXPECT_NEAR(bright_power, 2.0 * dim_power, 1.0e-12);
}

TEST(SbirsFoundationTest, ReceivedPowerAndSnrDecreaseWithRange) {
  sbirs_sensor::config::SbirsHardwareConfig hardware;
  hardware.noise_equivalent_power_w = 1.0e-15f;
  const double near_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      1.0e4, 1.0e6, 0.5, 0.8, 0.8, 0.7);
  const double far_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      1.0e4, 2.0e6, 0.5, 0.8, 0.8, 0.7);
  EXPECT_GT(near_power, far_power);
  EXPECT_GT(sbirs_sensor::foundation::ComputeInfraredSnrLinear(near_power, hardware),
            sbirs_sensor::foundation::ComputeInfraredSnrLinear(far_power, hardware));
}

TEST(SbirsFoundationTest, NegativeRadiantIntensityYieldsZeroPower) {
  EXPECT_DOUBLE_EQ(sbirs_sensor::foundation::ComputeReceivedPowerW(-1.0, 1.0e6, 0.5, 0.8, 0.8, 0.7),
                   0.0);
}

TEST(SbirsEnvironmentModelTest, WorseWeatherReducesEffectiveTransmittance) {
  sbirs_sensor::config::SbirsEnvironmentConfig clear;
  sbirs_sensor::config::SbirsEnvironmentConfig fog;
  fog.weather_type = sbirs_sensor::config::SbirsWeatherType::kFog;
  fog.relative_humidity_percent = 90.0f;
  fog.visibility_km = 0.5f;
  EXPECT_GT(sbirs_sensor::environment::ResolveEffectiveTransmittance(clear),
            sbirs_sensor::environment::ResolveEffectiveTransmittance(fog));
}

TEST(SbirsEarthOccultationTest, FiniteSegmentBehindEarthIsOcculted) {
  sbirs_sensor::session::SbirsVector3M sat;
  sat.x = 7000000.0;
  sbirs_sensor::session::SbirsVector3M target;
  target.x = -7000000.0;
  EXPECT_TRUE(sbirs_sensor::foundation::IsEarthOcculted(sat, target, 6371000.0));
}

TEST(SbirsEarthOccultationTest, OutwardLineOfSightIsNotOcculted) {
  sbirs_sensor::session::SbirsVector3M sat;
  sat.x = 7000000.0;
  sbirs_sensor::session::SbirsVector3M target;
  target.x = 8000000.0;
  EXPECT_FALSE(sbirs_sensor::foundation::IsEarthOcculted(sat, target, 6371000.0));
}

TEST(SbirsEarthOccultationTest, MarginIsNegativeWhenOcculted) {
  // 对侧地表上方（6.5e6 m < 地球半径 6.371e6 m）：LOS 穿过地球 → 余量为负（遮挡深度）。
  sbirs_sensor::session::SbirsVector3M sat;
  sat.x = 7000000.0;
  sbirs_sensor::session::SbirsVector3M target;
  target.x = -6500000.0;
  EXPECT_LT(sbirs_sensor::foundation::ComputeEarthOccultationMarginM(sat, target, 6371000.0),
            0.0);
}

TEST(SbirsEarthOccultationTest, MarginIsPositiveWhenClear) {
  // 同侧视线：余量为正。
  sbirs_sensor::session::SbirsVector3M sat;
  sat.x = 7000000.0;
  sbirs_sensor::session::SbirsVector3M target;
  target.x = 8000000.0;
  EXPECT_GT(sbirs_sensor::foundation::ComputeEarthOccultationMarginM(sat, target, 6371000.0),
            0.0);
}

}  // namespace
