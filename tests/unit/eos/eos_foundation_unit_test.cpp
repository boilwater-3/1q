/**
 * @file eos_foundation_unit_test.cpp
 * @brief 验证光学传感器基础计算层核心公式的数值行为。
 */

#include <gtest/gtest.h>

#include <cmath>

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

TEST(EosFoundationTest, VisibleRadianceIgnoresGeometryAndPathTerms) {
  radiometry::VisibleRadianceInputs near_target;
  near_target.solar_irradiance_w_m2 = 900.0f;
  near_target.solar_altitude_deg = 50.0f;
  near_target.atmospheric_transmittance = 0.95f;
  near_target.cloud_coverage_ratio = 0.1f;
  near_target.reflectance = 0.35f;
  near_target.projected_area_m2 = 1.0f;
  near_target.range_m = 500.0f;
  near_target.illumination = radiometry::IlluminationCondition::kDay;

  radiometry::VisibleRadianceInputs far_target = near_target;
  far_target.atmospheric_transmittance = 0.4f;
  far_target.projected_area_m2 = 50.0f;
  far_target.range_m = 6000.0f;

  const float near_radiance = radiometry::ComputeVisibleLambertianRadiance(near_target);
  const float far_radiance = radiometry::ComputeVisibleLambertianRadiance(far_target);
  EXPECT_NEAR(near_radiance, far_radiance, 1.0e-6f);
}

TEST(EosFoundationTest, VisibleChannelResultProvidesBackgroundAndContrast) {
  radiometry::VisibleChannelInputs inputs;
  inputs.target.solar_irradiance_w_m2 = 850.0f;
  inputs.target.solar_altitude_deg = 40.0f;
  inputs.target.atmospheric_transmittance = 0.78f;
  inputs.target.cloud_coverage_ratio = 0.15f;
  inputs.target.reflectance = 0.45f;
  inputs.target.projected_area_m2 = 1.8f;
  inputs.target.range_m = 1700.0f;
  inputs.background_reflectance = 0.12f;
  inputs.background_patch_area_m2 = 20.0f;

  const radiometry::VisibleChannelResult result = radiometry::ComputeVisibleChannelResult(inputs);

  EXPECT_GT(result.target_radiance, 0.0f);
  EXPECT_GT(result.background_radiance, 0.0f);
  EXPECT_TRUE(std::isfinite(result.normalized_contrast));
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

TEST(EosFoundationTest, NepModelProducesReasonableSnrEvaluation) {
  propagation::NepNoiseModelInputs nep_inputs;
  nep_inputs.detector_detectivity_cm_sqrt_hz_per_w = 1.2e10f;
  nep_inputs.detector_area_cm2 = 0.36f;
  nep_inputs.electrical_bandwidth_hz = 2000.0f;
  nep_inputs.optical_transmittance = 0.82f;
  nep_inputs.integration_time_sec = 0.02f;
  nep_inputs.system_noise_factor = 1.1f;

  const float nep_w = propagation::ComputeNepW(nep_inputs);
  const propagation::SnrEvaluationResult result =
      propagation::EvaluateSnrWithNep(5.0e-10f, nep_inputs);

  EXPECT_GT(nep_w, 0.0f);
  EXPECT_GT(result.snr_linear, 0.0f);
  EXPECT_TRUE(std::isfinite(result.snr_db));
}

TEST(EosFoundationTest, BandIntegratedRadianceScalesWithBandwidth) {
  const float spectral_radiance =
      radiometry::ComputePlanckRadiance(4.0f, 320.0f);
  const float narrow_band_radiance =
      radiometry::IntegrateSpectralRadianceOverBand(spectral_radiance, 1.0f);
  const float wide_band_radiance =
      radiometry::IntegrateSpectralRadianceOverBand(spectral_radiance, 4.0f);
  EXPECT_GT(wide_band_radiance, narrow_band_radiance);
  EXPECT_NEAR(wide_band_radiance / narrow_band_radiance, 4.0f, 1.0e-3f);
}

TEST(EosFoundationTest, DetectionRangeInterfaceReturnsOrderedRange) {
  optics::DetectionRangeInputs inputs;
  inputs.platform_altitude_m = 2500.0f;
  inputs.boresight_depression_deg = 40.0f;
  inputs.vertical_fov_deg = 8.0f;
  inputs.min_depression_deg = 2.0f;
  inputs.max_depression_deg = 85.0f;

  const float dmin_m = optics::ComputeMinimumDetectionRangeM(inputs);
  const float dmax_m = optics::ComputeMaximumDetectionRangeM(inputs);

  EXPECT_GT(dmin_m, 0.0f);
  EXPECT_GT(dmax_m, 0.0f);
  EXPECT_GE(dmax_m, dmin_m);
}

}  // namespace
}  // namespace foundation
}  // namespace electro_optical_sensor
