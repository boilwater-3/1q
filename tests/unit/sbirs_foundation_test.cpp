#include <gtest/gtest.h>

#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"
#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"

namespace {

TEST(SbirsFoundationTest, PlanckRadianceIncreasesWithTemperature) {
  const double cool = sbirs_sensor::foundation::ComputePlanckRadiance(4.0, 900.0);
  const double hot = sbirs_sensor::foundation::ComputePlanckRadiance(4.0, 1800.0);
  EXPECT_GT(hot, cool);
}

TEST(SbirsFoundationTest, ReceivedPowerAndSnrDecreaseWithRange) {
  sbirs_sensor::config::SbirsHardwareConfig hardware;
  hardware.noise_equivalent_power_w = 1.0e-15f;
  const double radiance = sbirs_sensor::foundation::ComputeBandRadiance(3.0, 5.0, 1800.0);
  const double near_power =
      sbirs_sensor::foundation::ComputeReceivedPowerW(radiance, 100.0, 1.0e6, 0.5, 0.8, 0.8, 0.7);
  const double far_power =
      sbirs_sensor::foundation::ComputeReceivedPowerW(radiance, 100.0, 2.0e6, 0.5, 0.8, 0.8, 0.7);
  EXPECT_GT(near_power, far_power);
  EXPECT_GT(sbirs_sensor::foundation::ComputeInfraredSnrLinear(near_power, hardware),
            sbirs_sensor::foundation::ComputeInfraredSnrLinear(far_power, hardware));
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

}  // namespace
