// Copyright 2026. All Rights Reserved.
//
// @file esr_environment_config_contract_test.cpp
// @brief ESR 环境配置分层契约测试。

#include <gtest/gtest.h>

#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"

namespace electronic_surveillance_radar {
namespace tests {
namespace {

TEST(EsrEnvironmentConfigContractTest, DefaultConfigOwnsScenarioConfig) {
  const config::EsrEnvironmentConfig config_default;
  EXPECT_EQ(config_default.scenario_config.preset, config::EsrEnvironmentPreset::kStandard);
}

}  // namespace
}  // namespace tests
}  // namespace electronic_surveillance_radar
