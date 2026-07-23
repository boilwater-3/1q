// Copyright 2026. All Rights Reserved.
//
// @file ar_primary_naming_contract_test.cpp
// @brief Verifies the preferred Ar* public API naming surface.

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/airborne_radar/session/ArSession.h"

namespace airborne_radar {
namespace {

session::ArCycleInput MakeMinimalArInput() {
  session::ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.platform.platform_entity_id = 42U;
  input.platform.platform_position_ecef_m.x_m = 6378137.0;
  return input;
}

TEST(ArPrimaryNamingContractTest, PreferredArNamesConstructAndStepSession) {
  config::ArSessionConfig config = config::ArSessionConfigBuilder().Build();
  session::ArSession session = session::ArSession::Create(config);

  const session::ArCycleInput input = MakeMinimalArInput();
  const session::ValidationIssueList issues = session::ValidateArCycleInput(input);
  EXPECT_FALSE(session::HasValidationError(issues));

  const session::ArCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(result.has_validation_error);
}

TEST(ArPrimaryNamingContractTest, PreferredArRuntimePatchAppliesThroughSession) {
  session::ArSession session = session::ArSession::Create(config::ArSessionConfig{});
  const config::ArRuntimeConfigPatch patch =
      config::ArRuntimeConfigBuilder().WithWorkMode(config::ArWorkMode::kTas).Build();

  EXPECT_TRUE(session.TryApplyRuntimeConfig(patch));

  const session::ArCycleResult result = session.StepWithResult(MakeMinimalArInput());
  EXPECT_FALSE(result.has_validation_error);
}

}  // namespace
}  // namespace airborne_radar
