/**
 * @file atmosphere_physics_test.cpp
 * @brief 验证共享大气传播物理模型与 REOS 对齐入口的基础行为。
 */

#include <gtest/gtest.h>

#include "common/atmosphere/AtmospherePhysics.h"

namespace oneq {
namespace internal {
namespace atmosphere {
namespace {

TEST(AtmospherePhysicsTest, BlakeLossGrowsWithPathLength) {
  const float short_path_loss_db =
      blake_atmos_loss_r4_1(1000.0f, 10.0e9f, 8.0f, 2000.0f, 4.0f / 3.0f);
  const float long_path_loss_db =
      blake_atmos_loss_r4_1(1000.0f, 10.0e9f, 8.0f, 20000.0f, 4.0f / 3.0f);
  EXPECT_GE(short_path_loss_db, 0.0f);
  EXPECT_GT(long_path_loss_db, short_path_loss_db);
}

TEST(AtmospherePhysicsTest, BlakeLossUsesProvidedSlantPathWithoutLowElevationAmplification) {
  const float low_elevation_loss_db =
      blake_atmos_loss_r4_1(1000.0f, 10.0e9f, 0.0f, 50000.0f, 4.0f / 3.0f);
  const float moderate_elevation_loss_db =
      blake_atmos_loss_r4_1(1000.0f, 10.0e9f, 8.0f, 50000.0f, 4.0f / 3.0f);

  EXPECT_NEAR(low_elevation_loss_db, moderate_elevation_loss_db, 1.0e-5f);
}

TEST(AtmospherePhysicsTest, RefractivityIndexRemainsAboveUnity) {
  const float n_r4 = refractivity_index_n_r4(20.0f, 293.15f, 900.0f, 1013.25f, 0.65f, 0);
  const double n_r8 = refractivity_index_n_r8(20.0, 293.15, 900.0, 1013.25, 0.65, 0);
  EXPECT_GT(n_r4, 1.0f);
  EXPECT_GT(n_r8, 1.0);

  const float n_surface = refractivity_index_nh_r4(n_r4, 0.0f, 7350.0f);
  const float n_upper = refractivity_index_nh_r4(n_r4, 5000.0f, 7350.0f);
  EXPECT_GT(n_surface, n_upper);
}

TEST(AtmospherePhysicsTest, Gtd7DensityDecaysWithAltitude) {
  const Gtd7Profile low_alt_profile = GTD7(172, 0.0, 1000.0, 0.0, 0.0, 0.0, 150.0, 150.0, 4.0, 48);
  const Gtd7Profile high_alt_profile =
      GTD7(172, 0.0, 12000.0, 0.0, 0.0, 0.0, 150.0, 150.0, 4.0, 48);
  EXPECT_GT(low_alt_profile.temperature_k, 0.0);
  EXPECT_GT(low_alt_profile.density_kg_m3, high_alt_profile.density_kg_m3);
}

TEST(AtmospherePhysicsTest, PhysicsPropagationReturnsPositiveLossWhenEnabled) {
  AtmosphericPropagationInputs inputs;
  inputs.enable_physics = true;
  inputs.frequency_hz = 9.6e9f;
  inputs.path_length_m = 50.0e3f;
  inputs.radar_altitude_m = 1200.0f;
  inputs.target_altitude_m = 800.0f;
  inputs.elevation_deg = 4.0f;
  inputs.relative_humidity = 0.7f;
  inputs.day_of_year = 180;
  const AtmosphericPropagationResult result = EvaluateAtmosphericPropagation(inputs);
  EXPECT_GT(result.total_physics_loss_db, 0.0f);
  EXPECT_GT(result.blake_loss_db, 0.0f);
  EXPECT_GT(result.refractivity_index, 1.0f);
  EXPECT_GT(result.neutral_density_kg_m3, 0.0f);
}

TEST(AtmospherePhysicsTest, SameAltitudePathLosesLessAtHigherAbsoluteAltitude) {
  AtmosphericPropagationInputs sea_level_inputs;
  sea_level_inputs.enable_physics = true;
  sea_level_inputs.frequency_hz = 9.6e9f;
  sea_level_inputs.path_length_m = 50.0e3f;
  sea_level_inputs.radar_altitude_m = 0.0f;
  sea_level_inputs.target_altitude_m = 0.0f;
  sea_level_inputs.elevation_deg = 4.0f;
  sea_level_inputs.relative_humidity = 0.7f;
  sea_level_inputs.day_of_year = 180;

  AtmosphericPropagationInputs elevated_inputs = sea_level_inputs;
  elevated_inputs.radar_altitude_m = 1200.0f;
  elevated_inputs.target_altitude_m = 1200.0f;

  const AtmosphericPropagationResult sea_level_result =
      EvaluateAtmosphericPropagation(sea_level_inputs);
  const AtmosphericPropagationResult elevated_result =
      EvaluateAtmosphericPropagation(elevated_inputs);

  EXPECT_LT(elevated_result.total_physics_loss_db, sea_level_result.total_physics_loss_db);
}

TEST(AtmospherePhysicsTest, BuildPropagationInputsSetsAllFields) {
  AtmosphericObservationRef obs;
  obs.pressure_hpa = 950.0f;
  obs.temperature_k = 280.0f;
  obs.relative_humidity = 0.6f;
  obs.k_factor = 1.5f;
  obs.day_of_year = 200;
  obs.solar_flux_f107a = 140.0f;
  obs.solar_flux_f107 = 130.0f;
  obs.geomagnetic_ap = 5.0f;

  const auto inputs = BuildPropagationInputs(
      9.6e9f, 50.0e3f, 1200.0f, 800.0f, 4.0f, obs);

  EXPECT_TRUE(inputs.enable_physics);
  EXPECT_FLOAT_EQ(inputs.frequency_hz, 9.6e9f);
  EXPECT_FLOAT_EQ(inputs.path_length_m, 50.0e3f);
  EXPECT_FLOAT_EQ(inputs.radar_altitude_m, 1200.0f);
  EXPECT_FLOAT_EQ(inputs.target_altitude_m, 800.0f);
  EXPECT_FLOAT_EQ(inputs.elevation_deg, 4.0f);
  EXPECT_FLOAT_EQ(inputs.pressure_hpa, 950.0f);
  EXPECT_FLOAT_EQ(inputs.temperature_k, 280.0f);
  EXPECT_FLOAT_EQ(inputs.relative_humidity, 0.6f);
  EXPECT_FLOAT_EQ(inputs.k_factor, 1.5f);
  EXPECT_EQ(inputs.day_of_year, 200);
  EXPECT_FLOAT_EQ(inputs.solar_flux_f107a, 140.0f);
  EXPECT_FLOAT_EQ(inputs.solar_flux_f107, 130.0f);
  EXPECT_FLOAT_EQ(inputs.geomagnetic_ap, 5.0f);
}

TEST(AtmospherePhysicsTest, BuildPropagationInputsProducesPositiveLoss) {
  AtmosphericObservationRef obs;
  obs.pressure_hpa = 1013.25f;
  obs.temperature_k = 288.15f;
  obs.relative_humidity = 0.5f;

  const auto inputs = BuildPropagationInputs(
      10.0e9f, 10.0e3f, 1.0e3f, 1.0e3f, 5.0f, obs);
  const auto result = EvaluateAtmosphericPropagation(inputs);

  EXPECT_GT(result.total_physics_loss_db, 0.0f);
  EXPECT_GT(result.blake_loss_db, 0.0f);
}

}  // namespace
}  // namespace atmosphere
}  // namespace internal
}  // namespace oneq
