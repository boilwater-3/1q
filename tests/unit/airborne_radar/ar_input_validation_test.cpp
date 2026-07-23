// Copyright 2026. All Rights Reserved.
//
// @file ar_input_validation_test.cpp
// @brief 验证 AR 内部目标与单周期用户输入的原子校验合同。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/coordinate/position_transform.h"

namespace airborne_radar {
namespace tests {

using session::HasValidationError;
using session::ValidateArCycleInput;
using session::ValidateArSceneTargets;
using session::ValidationCode;
using session::ValidationSeverity;

namespace {

session::ArSceneTarget MakeValidLocalTarget(std::uint64_t id = 1U) {
  session::ArSceneTarget target(100.0f, 0.0f, 0.0f, 1.0f);
  target.external_target_id = id;
  target.position_x = 1000.0f;
  target.range_m = 1000.0f;
  return target;
}

session::ArCycleInput MakeValidCycleInput() {
  session::ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 0.5;
  input.platform.platform_entity_id = 42U;

  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(
      platform_lla, &input.platform.platform_position_ecef_m));

  session::ArTargetInput target;
  target.target_id = 7U;
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  target.kinematics.position_ecef_m = input.platform.platform_position_ecef_m;
  target.kinematics.position_ecef_m.x_m += 5000.0;
  target.rcs = 2.0f;
  input.targets.push_back(target);
  return input;
}

const session::ValidationIssue* FindIssue(
    const std::vector<session::ValidationIssue>& issues, ValidationCode code) {
  for (const session::ValidationIssue& issue : issues) {
    if (issue.code == code) {
      return &issue;
    }
  }
  return nullptr;
}

void AddValidInterferenceEmission(session::ArCycleInput* input) {
  ASSERT_NE(input, nullptr);
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = 99U;
  emission.identity.equipment_id = 5U;
  emission.identity.emission_id = 1U;
  emission.position_ecef_m = input->platform.platform_position_ecef_m;
  emission.position_ecef_m.x_m += 10000.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      input->cycle_start_time_s, input->dt_sec, 10.0e9, 5.0e6, 100.0,
      &emission.waveform));
  input->interference.world_cycle_index = input->cycle_index;
  input->interference.window_start_time_s = input->cycle_start_time_s;
  input->interference.window_duration_s = input->dt_sec;
  input->interference.emissions.push_back(emission);
}

}  // namespace

TEST(ArSceneTargetValidationTest, OriginRequiresPositiveRange) {
  session::ArSceneTarget target;
  target.external_target_id = 1U;
  target.rcs = 1.0f;

  const session::ValidationIssueList issues = ValidateArSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kMissingRangeAndCartesianPosition), nullptr);
}

TEST(ArSceneTargetValidationTest, PositiveRangeAllowsOrigin) {
  session::ArSceneTarget target;
  target.external_target_id = 1U;
  target.range_m = 5000.0f;
  target.rcs = 1.0f;

  const session::ValidationIssueList issues = ValidateArSceneTargets({target});
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(ArSceneTargetValidationTest, CartesianPositionAllowsMissingRange) {
  session::ArSceneTarget target;
  target.external_target_id = 1U;
  target.position_x = 3000.0f;
  target.rcs = 1.0f;

  const session::ValidationIssueList issues = ValidateArSceneTargets({target});
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(ArSceneTargetValidationTest, NonFiniteFieldsReject) {
  session::ArSceneTarget target = MakeValidLocalTarget();
  target.velocity_y = std::numeric_limits<float>::infinity();

  const session::ValidationIssueList issues = ValidateArSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kNonFiniteTargetField), nullptr);
}

TEST(ArSceneTargetValidationTest, DuplicateIdentityRejects) {
  session::ArSceneTarget first = MakeValidLocalTarget(42U);
  session::ArSceneTarget second = MakeValidLocalTarget(42U);
  second.position_x = 2000.0f;
  second.range_m = 2000.0f;

  const session::ValidationIssueList issues = ValidateArSceneTargets({first, second});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kDuplicateExternalTargetId), nullptr);
}

TEST(ArSceneTargetValidationTest, UnknownIdentityIsInformational) {
  const session::ValidationIssueList issues =
      ValidateArSceneTargets({MakeValidLocalTarget(0U)});
  const session::ValidationIssue* issue =
      FindIssue(issues, ValidationCode::kUnknownExternalTargetId);
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, ValidationSeverity::kInfo);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(ArSceneTargetValidationTest, NegativeRcsIsWarning) {
  session::ArSceneTarget target = MakeValidLocalTarget();
  target.rcs = -0.5f;

  const session::ValidationIssueList issues = ValidateArSceneTargets({target});
  const session::ValidationIssue* issue = FindIssue(issues, ValidationCode::kNegativeRcs);
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, ValidationSeverity::kWarning);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(ArCycleInputValidationTest, FullyValidInputProducesNoErrors) {
  EXPECT_FALSE(HasValidationError(ValidateArCycleInput(MakeValidCycleInput())));
}

TEST(ArCycleInputValidationTest, CycleIndexMustBeNonZero) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.cycle_index = 0U;

  const session::ValidationIssueList issues = ValidateArCycleInput(input);
  EXPECT_NE(FindIssue(issues, ValidationCode::kInvalidCycleIndex), nullptr);
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(ArCycleInputValidationTest, CycleStartMustBeFiniteAndNonNegative) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.cycle_start_time_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), ValidationCode::kInvalidCycleStartTime),
            nullptr);

  input.cycle_start_time_s = -1.0;
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), ValidationCode::kInvalidCycleStartTime),
            nullptr);
}

TEST(ArCycleInputValidationTest, CycleDurationMustBeFiniteAndPositive) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.dt_sec = 0.0;
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), ValidationCode::kInvalidCycleDeltaTime),
            nullptr);

  input.dt_sec = -1.0;
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), ValidationCode::kInvalidCycleDeltaTime),
            nullptr);

  input.dt_sec = std::numeric_limits<double>::infinity();
  EXPECT_NE(FindIssue(ValidateArCycleInput(input),
                      ValidationCode::kNonFiniteCycleDeltaTime),
            nullptr);
}

TEST(ArCycleInputValidationTest, PlatformIdentityAndWorldKinematicsAreRequired) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.platform.platform_entity_id = 0U;
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), ValidationCode::kInvalidPlatformInput),
            nullptr);

  input = MakeValidCycleInput();
  input.platform.platform_velocity_mps.z_mps =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), ValidationCode::kInvalidPlatformInput),
            nullptr);
}

TEST(ArCycleInputValidationTest, InvalidOrDuplicateWorldTargetsReject) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.targets.front().kinematics.position_ecef_m.x_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(FindIssue(ValidateArCycleInput(input), ValidationCode::kInvalidTargetInput),
            nullptr);

  input = MakeValidCycleInput();
  input.targets.push_back(input.targets.front());
  EXPECT_NE(FindIssue(ValidateArCycleInput(input),
                      ValidationCode::kDuplicateExternalTargetId),
            nullptr);
}

TEST(ArCycleInputValidationTest, NaturalEnvironmentIsValidatedIndependently) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.environment.atmospheric_observation.temperature_k = 0.0f;
  EXPECT_NE(FindIssue(ValidateArCycleInput(input),
                      ValidationCode::kInvalidEnvironmentObservation),
            nullptr);

  input = MakeValidCycleInput();
  input.environment.atmospheric_observation.relative_humidity = 1.5f;
  EXPECT_NE(FindIssue(ValidateArCycleInput(input),
                      ValidationCode::kInvalidEnvironmentObservation),
            nullptr);

  input = MakeValidCycleInput();
  input.environment.atmospheric_context.solar_flux_f107 =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_NE(FindIssue(ValidateArCycleInput(input),
                      ValidationCode::kInvalidEnvironmentObservation),
            nullptr);
}

TEST(ArCycleInputValidationTest, NonEmptyInterferenceMustMatchCycleWindow) {
  session::ArCycleInput input = MakeValidCycleInput();
  AddValidInterferenceEmission(&input);
  input.interference.world_cycle_index += 1U;

  const session::ValidationIssueList issues = ValidateArCycleInput(input);
  EXPECT_NE(FindIssue(issues, ValidationCode::kInterferenceFrameMismatch), nullptr);
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(ArCycleInputValidationTest, InvalidRfFactsRejectAtomically) {
  session::ArCycleInput input = MakeValidCycleInput();
  AddValidInterferenceEmission(&input);
  input.interference.emissions.front().identity.emission_id = 0U;

  const session::ValidationIssueList issues = ValidateArCycleInput(input);
  EXPECT_NE(FindIssue(issues, ValidationCode::kInvalidInterferenceInput), nullptr);
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(ArCycleInputValidationTest, EmptyInterferenceNeedsNoMetadata) {
  session::ArCycleInput input = MakeValidCycleInput();
  input.interference.world_cycle_index = 999U;
  input.interference.window_start_time_s = -10.0;
  input.interference.window_duration_s = -1.0;

  EXPECT_FALSE(HasValidationError(ValidateArCycleInput(input)));
}

}  // namespace tests
}  // namespace airborne_radar
