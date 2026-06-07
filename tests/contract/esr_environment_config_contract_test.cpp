// Copyright 2026. All Rights Reserved.
//
// @file esr_environment_config_contract_test.cpp
// @brief ESR 环境配置分层契约测试。

#include <gtest/gtest.h>

#include <type_traits>

#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"

namespace electronic_surveillance_radar {
namespace tests {
namespace {

TEST(EsrEnvironmentConfigContractTest, ConfigAliasMatchesEnvironmentDefaultConfig) {
  const bool is_same_type =
      std::is_same<config::EsrEnvironmentConfig, environment::EsrEnvironmentDefaultConfig>::value;
  EXPECT_TRUE(is_same_type);
}

TEST(EsrEnvironmentConfigContractTest, BuildModelConfigFromScenarioMapsFields) {
  environment::EsrEnvironmentScenarioConfig scenario_config;
  scenario_config.preset = config::EsrEnvironmentPreset::kDenseClutter;
  scenario_config.atmospheric_physics.enable_physical_model = true;
  scenario_config.atmospheric_physics.relative_humidity = 0.73f;
  scenario_config.atmospheric_context.has_day_of_year = true;
  scenario_config.atmospheric_context.day_of_year = 215;

  const environment::EsrEnvironmentModelConfig model_config =
      environment::BuildModelConfigFromScenario(scenario_config);

  EXPECT_EQ(model_config.preset, config::EsrEnvironmentPreset::kDenseClutter);
  EXPECT_TRUE(model_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(model_config.atmospheric_physics.relative_humidity, 0.73f);
  EXPECT_TRUE(model_config.atmospheric_context.has_day_of_year);
  EXPECT_EQ(model_config.atmospheric_context.day_of_year, 215);
}

TEST(EsrEnvironmentConfigContractTest, DefaultConfigOwnsScenarioConfig) {
  const config::EsrEnvironmentConfig config_default;
  EXPECT_EQ(config_default.scenario_config.preset, config::EsrEnvironmentPreset::kStandard);
}

TEST(EsrEnvironmentConfigContractTest, ModelConfigIsDistinctFromScenarioConfig) {
  const bool is_same =
      std::is_same<environment::EsrEnvironmentModelConfig,
                   environment::EsrEnvironmentScenarioConfig>::value;
  EXPECT_FALSE(is_same);
}

}  // namespace
}  // namespace tests
}  // namespace electronic_surveillance_radar
