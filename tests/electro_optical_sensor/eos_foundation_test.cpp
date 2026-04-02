/**
 * @file eos_foundation_test.cpp
 * @brief 验证光学传感器基础计算层核心公式的数值行为。
 */

#include <gtest/gtest.h>

#include "1q/electro_optical_sensor/foundation/EosOpticalCharacteristics.h"
#include "1q/electro_optical_sensor/foundation/EosPropagation.h"
#include "1q/electro_optical_sensor/foundation/EosRadiometry.h"

namespace electro_optical_sensor {
namespace foundation {
namespace {

TEST(EosFoundationTest, PlanckRadianceIncreasesWithTemperature) {
  const float low_temp = radiometry::ComputePlanckRadiance(4.0f, 280.0f);
  const float high_temp = radiometry::ComputePlanckRadiance(4.0f, 340.0f);
  EXPECT_GT(low_temp, 0.0f);
  EXPECT_GT(high_temp, low_temp);
}

TEST(EosFoundationTest, InfraredRadianceDeltaReflectsTemperatureContrast) {
  radiometry::InfraredRadianceInputs inputs;
  inputs.wavelength_um = 4.0f;
  inputs.target_temperature_k = 330.0f;
  inputs.background_temperature_k = 290.0f;
  inputs.emissivity = 0.95f;
  const float delta = radiometry::ComputeInfraredRadianceDelta(inputs);
  EXPECT_GT(delta, 0.0f);
}

TEST(EosFoundationTest, VisibleRadianceDayIsHigherThanNight) {
  radiometry::VisibleRadianceInputs base;
  base.solar_irradiance_w_m2 = 900.0f;
  base.solar_altitude_deg = 50.0f;
  base.atmospheric_transmittance = 0.8f;
  base.cloud_coverage_ratio = 0.2f;
  base.reflectance = 0.4f;
  base.projected_area_m2 = 2.0f;
  base.range_m = 1500.0f;

  base.illumination = radiometry::IlluminationCondition::kDay;
  const float day_radiance = radiometry::ComputeVisibleLambertianRadiance(base);
  base.illumination = radiometry::IlluminationCondition::kNight;
  const float night_radiance = radiometry::ComputeVisibleLambertianRadiance(base);

  EXPECT_GT(day_radiance, night_radiance);
}

TEST(EosFoundationTest, OpticalGeometryFunctionsProducePositiveValues) {
  const float fov_deg = optics::ComputeInstantaneousFovDeg(0.02f, 0.8f);
  const float scan_width_m = optics::ComputeGroundScanWidthM(2000.0f, fov_deg);
  const float angular_resolution_rad =
      optics::ComputeDiffractionLimitedAngularResolutionRad(4.0f, 0.2f);
  const float gsd_m = optics::ComputeGroundSampleDistanceM(2000.0f, angular_resolution_rad);
  EXPECT_GT(fov_deg, 0.0f);
  EXPECT_GT(scan_width_m, 0.0f);
  EXPECT_GT(angular_resolution_rad, 0.0f);
  EXPECT_GT(gsd_m, 0.0f);
}

TEST(EosFoundationTest, AtmosphericTransmittanceDecreasesWithDistance) {
  const float short_path =
      propagation::ComputeAtmosphericTransmittance(2.0e-5f, 1.0e-5f, 1000.0f);
  const float long_path =
      propagation::ComputeAtmosphericTransmittance(2.0e-5f, 1.0e-5f, 5000.0f);
  EXPECT_GT(short_path, long_path);
  EXPECT_GE(short_path, 0.0f);
  EXPECT_LE(short_path, 1.0f);
}

TEST(EosFoundationTest, ReceivedPowerAndSnrDecreaseWithRange) {
  const float near_power = propagation::ComputeReceivedPowerW(100.0f, 2.0f, 1000.0f, 0.03f, 0.8f, 0.85f);
  const float far_power = propagation::ComputeReceivedPowerW(100.0f, 2.0f, 3000.0f, 0.03f, 0.8f, 0.85f);
  EXPECT_GT(near_power, far_power);

  const float near_snr_linear = propagation::ComputeSnrLinear(near_power, 1.0e-12f);
  const float far_snr_linear = propagation::ComputeSnrLinear(far_power, 1.0e-12f);
  EXPECT_GT(near_snr_linear, far_snr_linear);
  EXPECT_GT(propagation::ComputeSnrDb(near_snr_linear), propagation::ComputeSnrDb(far_snr_linear));
}

}  // namespace
}  // namespace foundation
}  // namespace electro_optical_sensor
