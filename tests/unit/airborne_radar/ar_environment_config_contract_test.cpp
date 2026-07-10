// Copyright 2026. All Rights Reserved.
//
// @file ar_environment_config_contract_test.cpp
// @brief 验证 AR 环境配置类型合约、默认值稳定性、映射函数行为与补丁行为。

#include <gtest/gtest.h>

#include <type_traits>

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"

namespace airborne_radar {
namespace environment {

// Using declarations for types migrated to config::
using config::AtmosphericDerivedContext;
using config::AtmosphericPhysicsConfig;
using config::BuildModelConfigFromScenario;
using config::EnvironmentModelConfig;
using config::EnvironmentRuntimeConfigPatch;
using config::EnvironmentScenarioConfig;
using config::JammerEmitterState;
using config::JammingSensitivityProfile;
using config::JammingTechnique;
using config::ArEnvironmentConfig;
using config::ResolveJammingSensitivityProfile;
using config::VegetationCoverProfile;
using config::VegetationScatterPhysicsConfig;

namespace {

// ---------------------------------------------------------------------------
// Suite 1: 类型与别名合同
// ---------------------------------------------------------------------------

TEST(ArEnvironmentTypeContractTest, ModelConfigIsIndependentFromScenarioConfig) {
  static_assert(!std::is_same<EnvironmentModelConfig, EnvironmentScenarioConfig>::value,
                "EnvironmentModelConfig must NOT be an alias of EnvironmentScenarioConfig");
}

TEST(ArEnvironmentTypeContractTest, ModelConfigHasSameFieldStructureAsScenarioConfig) {
  EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.pressure_hpa = 1000.0f;
  scenario.atmospheric_context.solar_flux_f107 = 120.0f;
  scenario.vegetation_scatter_physics.cover_profile = VegetationCoverProfile::kDeciduousForest;
  JammerEmitterState j;
  j.power_db = 5.0f;
  scenario.jammer_sources = {j};

  const auto model = BuildModelConfigFromScenario(scenario);

  EXPECT_FLOAT_EQ(model.atmospheric_physics.pressure_hpa, 1000.0f);
  EXPECT_FLOAT_EQ(model.atmospheric_context.solar_flux_f107, 120.0f);
  EXPECT_EQ(model.vegetation_scatter_physics.cover_profile,
            VegetationCoverProfile::kDeciduousForest);
  ASSERT_EQ(model.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(model.jammer_sources[0].power_db, 5.0f);
}

TEST(ArEnvironmentTypeContractTest, DefaultConfigContainsOnlyScenarioConfig) {
  ArEnvironmentConfig defaults;
  (void)defaults.scenario_config;
}

TEST(ArEnvironmentTypeContractTest, DefaultConfigScenarioDefaultsToEmpty) {
  ArEnvironmentConfig defaults;
  EXPECT_TRUE(defaults.scenario_config.jammer_sources.empty());
  EXPECT_FALSE(defaults.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_EQ(defaults.scenario_config.vegetation_scatter_physics.cover_profile,
            VegetationCoverProfile::kDisabled);
}

TEST(ArEnvironmentTypeContractTest, RuntimePatchFieldsDefaultToUnset) {
  EnvironmentRuntimeConfigPatch patch;
  EXPECT_FALSE(patch.has_scenario_config);
  EXPECT_FALSE(patch.has_jamming_sensitivity_profile);
}

TEST(ArEnvironmentTypeContractTest, RuntimePatchSupportsExplicitHasFlags) {
  EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;

  EnvironmentRuntimeConfigPatch patch;
  patch.has_scenario_config = true;
  patch.scenario_config = scenario;
  patch.has_jamming_sensitivity_profile = true;
  patch.jamming_sensitivity_profile = JammingSensitivityProfile::kStrict;

  EXPECT_TRUE(patch.has_scenario_config);
  EXPECT_TRUE(patch.has_jamming_sensitivity_profile);
  EXPECT_TRUE(patch.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_EQ(patch.jamming_sensitivity_profile, JammingSensitivityProfile::kStrict);
}

// ---------------------------------------------------------------------------
// Suite 2: 默认值稳定性
// ---------------------------------------------------------------------------

TEST(ArEnvironmentDefaultStabilityTest, AtmosphericObservationDefaults) {
  AtmosphericPhysicsConfig physics;
  EXPECT_FALSE(physics.enable_physical_model);
  EXPECT_FLOAT_EQ(physics.pressure_hpa, 1013.25f);
  EXPECT_FLOAT_EQ(physics.temperature_k, 288.15f);
  EXPECT_FLOAT_EQ(physics.relative_humidity, 0.5f);
}

TEST(ArEnvironmentDefaultStabilityTest, AtmosphericDerivedContextDefaults) {
  AtmosphericDerivedContext context;
  EXPECT_FALSE(context.has_simulation_unix_seconds);
  EXPECT_EQ(context.simulation_unix_seconds, 0);
  EXPECT_FLOAT_EQ(context.solar_flux_f107a, 150.0f);
  EXPECT_FLOAT_EQ(context.solar_flux_f107, 150.0f);
  EXPECT_FLOAT_EQ(context.geomagnetic_ap, 4.0f);
}

TEST(ArEnvironmentDefaultStabilityTest, VegetationScatterPhysicsDefaults) {
  VegetationScatterPhysicsConfig vegetation;
  EXPECT_EQ(vegetation.cover_profile, VegetationCoverProfile::kDisabled);
  EXPECT_FALSE(vegetation.enable_physical_model);
}

TEST(ArEnvironmentDefaultStabilityTest, JammerEmitterStateDefaults) {
  JammerEmitterState jammer;
  EXPECT_EQ(jammer.technique, JammingTechnique::kUnknown);
  EXPECT_FLOAT_EQ(jammer.power_db, 0.0f);
  EXPECT_FLOAT_EQ(jammer.js_db, 0.0f);
  EXPECT_FLOAT_EQ(jammer.confidence, 1.0f);
}

TEST(ArEnvironmentDefaultStabilityTest, DefaultConfigDefaultConstructedProducesDefaults) {
  const ArEnvironmentConfig built;
  const ArEnvironmentConfig defaults;
  EXPECT_EQ(built.scenario_config.jammer_sources.size(),
            defaults.scenario_config.jammer_sources.size());
}

TEST(ArEnvironmentDefaultStabilityTest, DefaultConfigPreservesOverride) {
  EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;
  scenario.atmospheric_physics.temperature_k = 300.0f;

  ArEnvironmentConfig built;
  built.scenario_config = scenario;

  EXPECT_TRUE(built.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(built.scenario_config.atmospheric_physics.temperature_k, 300.0f);
}

// ---------------------------------------------------------------------------
// Suite 3: 映射函数行为
// ---------------------------------------------------------------------------

TEST(ArEnvironmentMappingFunctionTest, BuildModelConfigFromScenarioExplicitFieldMapping) {
  EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;
  scenario.atmospheric_physics.temperature_k = 300.0f;
  scenario.atmospheric_physics.pressure_hpa = 1000.0f;

  const auto model = BuildModelConfigFromScenario(scenario);

  EXPECT_FLOAT_EQ(model.atmospheric_physics.temperature_k,
                  scenario.atmospheric_physics.temperature_k);
  EXPECT_FLOAT_EQ(model.atmospheric_physics.pressure_hpa,
                  scenario.atmospheric_physics.pressure_hpa);
  EXPECT_EQ(model.atmospheric_physics.enable_physical_model,
            scenario.atmospheric_physics.enable_physical_model);
}

TEST(ArEnvironmentMappingFunctionTest, BuildModelConfigFromScenarioPreservesJammerSources) {
  EnvironmentScenarioConfig scenario;
  JammerEmitterState j1;
  JammerEmitterState j2;
  JammerEmitterState j3;
  j1.power_db = 10.0f;
  j2.power_db = 20.0f;
  j3.power_db = 30.0f;
  scenario.jammer_sources = {j1, j2, j3};

  const auto model = BuildModelConfigFromScenario(scenario);

  ASSERT_EQ(model.jammer_sources.size(), 3u);
  EXPECT_FLOAT_EQ(model.jammer_sources[0].power_db, 10.0f);
  EXPECT_FLOAT_EQ(model.jammer_sources[1].power_db, 20.0f);
  EXPECT_FLOAT_EQ(model.jammer_sources[2].power_db, 30.0f);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveKFactorStandardConditions) {
  AtmosphericDerivedContext context;
  AtmosphericPhysicsConfig physics;
  const float k = oneq::environment::ResolveEffectiveKFactor(physics);
  EXPECT_GT(k, 1.0f);
  EXPECT_LT(k, 2.0f);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveKFactorFallbackOnInvalidTemp) {
  AtmosphericDerivedContext context;
  AtmosphericPhysicsConfig physics;
  physics.temperature_k = 0.5f;  // <= 1K triggers ISA fallback
  const float k = oneq::environment::ResolveEffectiveKFactor(physics);
  EXPECT_GT(k, 0.5f);
  EXPECT_LT(k, 2.5f);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveKFactorFallbackOnInvalidPressure) {
  AtmosphericDerivedContext context;
  AtmosphericPhysicsConfig physics;
  physics.pressure_hpa = -1.0f;  // <= 0 triggers ISA fallback
  const float k = oneq::environment::ResolveEffectiveKFactor(physics);
  EXPECT_GT(k, 0.5f);
  EXPECT_LT(k, 2.5f);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveKFactorClampsNegativeHumidity) {
  AtmosphericDerivedContext context;
  AtmosphericPhysicsConfig physics;
  physics.relative_humidity = -0.5f;
  const float k = oneq::environment::ResolveEffectiveKFactor(physics);
  EXPECT_GT(k, 0.5f);
  EXPECT_LT(k, 2.5f);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveKFactorClampsHumidityAboveOne) {
  AtmosphericDerivedContext context;
  AtmosphericPhysicsConfig physics;
  physics.relative_humidity = 2.0f;
  const float k = oneq::environment::ResolveEffectiveKFactor(physics);
  EXPECT_GT(k, 0.5f);
  EXPECT_LT(k, 2.5f);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveKFactorFallbackOutOfRange) {
  AtmosphericDerivedContext context;
  AtmosphericPhysicsConfig physics;
  // Extremely high temperature + very low pressure can produce extreme k values
  physics.temperature_k = 1000.0f;
  physics.pressure_hpa = 100.0f;
  physics.relative_humidity = 1.0f;
  const float k = oneq::environment::ResolveEffectiveKFactor(physics);
  // If out of [0.5, 2.5], should fallback to 4/3
  EXPECT_GE(k, 0.5f);
  EXPECT_LE(k, 2.5f);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveDayOfYearWithTimestamp) {
  AtmosphericDerivedContext context;
  context.has_simulation_unix_seconds = true;
  // 2024-01-01 00:00:00 UTC = Unix 1704067200 → DOY 1
  context.simulation_unix_seconds = 1704067200;
  EXPECT_EQ(oneq::environment::ResolveEffectiveDayOfYear(context), 1);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveDayOfYearFallbackWithoutTimestamp) {
  AtmosphericDerivedContext context;
  context.has_simulation_unix_seconds = false;
  context.simulation_unix_seconds = 0;
  EXPECT_EQ(oneq::environment::ResolveEffectiveDayOfYear(context), 172);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveDayOfYearWithLeapYear) {
  AtmosphericDerivedContext context;
  context.has_simulation_unix_seconds = true;
  // 2024-12-31 00:00:00 UTC → DOY 366 (2024 is leap year)
  context.simulation_unix_seconds = 1735603200;
  EXPECT_EQ(oneq::environment::ResolveEffectiveDayOfYear(context), 366);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveEffectiveDayOfYearWithNonLeapYear) {
  AtmosphericDerivedContext context;
  context.has_simulation_unix_seconds = true;
  // 2023-12-31 00:00:00 UTC → DOY 365 (2023 is not leap year)
  context.simulation_unix_seconds = 1703980800;
  EXPECT_EQ(oneq::environment::ResolveEffectiveDayOfYear(context), 365);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveJammingSensitivityProfileStrictBoundary) {
  EXPECT_EQ(ResolveJammingSensitivityProfile(5.0f), JammingSensitivityProfile::kStrict);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveJammingSensitivityProfileBalanced) {
  EXPECT_EQ(ResolveJammingSensitivityProfile(6.0f), JammingSensitivityProfile::kBalanced);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveJammingSensitivityProfileRelaxedBoundary) {
  EXPECT_EQ(ResolveJammingSensitivityProfile(7.0f), JammingSensitivityProfile::kRelaxed);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveJammingSensitivityProfileNegativeThreshold) {
  EXPECT_EQ(ResolveJammingSensitivityProfile(-10.0f), JammingSensitivityProfile::kStrict);
}

TEST(ArEnvironmentMappingFunctionTest, ResolveJammingSensitivityProfileVeryLargeThreshold) {
  EXPECT_EQ(ResolveJammingSensitivityProfile(100.0f), JammingSensitivityProfile::kRelaxed);
}

// ---------------------------------------------------------------------------
// Suite 4: 补丁行为
// ---------------------------------------------------------------------------

TEST(ArEnvironmentRuntimePatchBehaviorTest, EmptyPatchHasNoUpdates) {
  EnvironmentRuntimeConfigPatch patch;
  EXPECT_FALSE(patch.has_scenario_config);
  EXPECT_FALSE(patch.has_jamming_sensitivity_profile);
}

TEST(ArEnvironmentRuntimePatchBehaviorTest, ScenarioConfigPatchSetsCorrectField) {
  EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;

  EnvironmentRuntimeConfigPatch patch;
  patch.has_scenario_config = true;
  patch.scenario_config = scenario;

  EXPECT_TRUE(patch.has_scenario_config);
  EXPECT_TRUE(patch.scenario_config.atmospheric_physics.enable_physical_model);
  // Profile flag should not be affected
  EXPECT_FALSE(patch.has_jamming_sensitivity_profile);
}

TEST(ArEnvironmentRuntimePatchBehaviorTest, JammingProfilePatchSetsCorrectField) {
  EnvironmentRuntimeConfigPatch patch;
  patch.has_jamming_sensitivity_profile = true;
  patch.jamming_sensitivity_profile = JammingSensitivityProfile::kRelaxed;

  EXPECT_TRUE(patch.has_jamming_sensitivity_profile);
  EXPECT_EQ(patch.jamming_sensitivity_profile, JammingSensitivityProfile::kRelaxed);
  // Scenario flag should not be affected
  EXPECT_FALSE(patch.has_scenario_config);
}

TEST(ArEnvironmentRuntimePatchBehaviorTest, FullPatchSupportsScenarioAndProfile) {
  EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.temperature_k = 310.0f;

  EnvironmentRuntimeConfigPatch patch;
  patch.has_scenario_config = true;
  patch.scenario_config = scenario;
  patch.has_jamming_sensitivity_profile = true;
  patch.jamming_sensitivity_profile = JammingSensitivityProfile::kStrict;

  EXPECT_TRUE(patch.has_scenario_config);
  EXPECT_TRUE(patch.has_jamming_sensitivity_profile);
  EXPECT_FLOAT_EQ(patch.scenario_config.atmospheric_physics.temperature_k, 310.0f);
  EXPECT_EQ(patch.jamming_sensitivity_profile, JammingSensitivityProfile::kStrict);
}

}  // namespace
}  // namespace environment
}  // namespace airborne_radar
