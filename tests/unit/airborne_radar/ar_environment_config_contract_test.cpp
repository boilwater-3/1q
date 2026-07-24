// Copyright 2026. All Rights Reserved.
//
// @file ar_environment_config_contract_test.cpp
// @brief 验证 AR 自然环境配置、派生量与运行期补丁合同。

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArEnvironmentInput.h"

namespace airborne_radar {
namespace environment {
namespace {

TEST(ArEnvironmentTypeContractTest, DefaultsContainOnlyNaturalEnvironmentFacts) {
  const config::ArEnvironmentConfig defaults;

  EXPECT_FALSE(
      defaults.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_EQ(defaults.scenario_config.vegetation_scatter_physics.cover_profile,
            config::VegetationCoverProfile::kDisabled);
  EXPECT_FALSE(
      defaults.scenario_config.vegetation_scatter_physics.enable_physical_model);
}

TEST(ArEnvironmentTypeContractTest, RuntimePatchDefaultsToUnset) {
  const config::EnvironmentRuntimeConfigPatch patch;
  EXPECT_FALSE(patch.has_scenario_config);
}

TEST(ArEnvironmentTypeContractTest, RuntimePatchPreservesExplicitScenario) {
  config::EnvironmentRuntimeConfigPatch patch;
  patch.has_scenario_config = true;
  patch.scenario_config.atmospheric_physics.enable_physical_model = true;
  patch.scenario_config.atmospheric_physics.temperature_k = 301.0f;
  patch.scenario_config.vegetation_scatter_physics.enable_physical_model =
      true;
  patch.scenario_config.vegetation_scatter_physics.cover_profile =
      config::VegetationCoverProfile::kSparseWoodland;

  EXPECT_TRUE(patch.has_scenario_config);
  EXPECT_FLOAT_EQ(patch.scenario_config.atmospheric_physics.temperature_k,
                  301.0f);
  EXPECT_EQ(
      patch.scenario_config.vegetation_scatter_physics.cover_profile,
      config::VegetationCoverProfile::kSparseWoodland);
}

TEST(ArEnvironmentDerivedValueTest, AtmosphericDefaultsRemainStable) {
  const config::AtmosphericPhysicsConfig physics;

  EXPECT_FLOAT_EQ(physics.pressure_hpa, 1013.25f);
  EXPECT_FLOAT_EQ(physics.temperature_k, 288.15f);
  EXPECT_FLOAT_EQ(physics.relative_humidity, 0.5f);
}

TEST(ArEnvironmentDerivedValueTest, DerivesKFactorFromObservation) {
  config::AtmosphericPhysicsConfig physics;
  physics.pressure_hpa = 950.0f;
  physics.temperature_k = 300.0f;
  physics.relative_humidity = 0.9f;

  const float k_factor =
      oneq::environment::ResolveEffectiveKFactor(physics);
  EXPECT_GT(k_factor, 1.0f);
  EXPECT_LT(k_factor, 2.0f);
}

TEST(ArEnvironmentInputStateTest, PatchUpdatesOnlySelectedNaturalFacts) {
  session::ArEnvironmentInput initial;
  initial.atmospheric_observation.temperature_k = 280.0f;
  initial.surface_observation.cover_profile =
      config::VegetationCoverProfile::kOpenGrassland;

  session::ArEnvironmentInputState state(initial);
  session::ArEnvironmentInputPatch patch;
  patch.has_atmospheric_observation = true;
  patch.atmospheric_observation.temperature_k = 305.0f;
  state.Update(patch);

  const session::ArEnvironmentInput snapshot = state.Snapshot();
  EXPECT_FLOAT_EQ(snapshot.atmospheric_observation.temperature_k, 305.0f);
  EXPECT_EQ(snapshot.surface_observation.cover_profile,
            config::VegetationCoverProfile::kOpenGrassland);
}

}  // namespace
}  // namespace environment
}  // namespace airborne_radar
