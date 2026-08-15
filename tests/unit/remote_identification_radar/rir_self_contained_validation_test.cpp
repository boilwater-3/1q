// Copyright 2026. All Rights Reserved.
//
// @file rir_self_contained_validation_test.cpp
// @brief 验证阶段 2-S 独立输入面校验：速度/起伏/环境/RF 输入。

#include <gtest/gtest.h>

#include <limits>

#include "1q/remote_identification_radar/session/RirInputValidation.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirSceneTarget;
using session::ValidateRirCycleInput;

RirCycleInput MakeValidInput() {
  RirCycleInput input;
  input.input_cycle_index = 1U;
  input.dt_sec = 0.5;
  input.sim_time_sec = 0.0f;
  input.platform_altitude_m = 1000.0f;
  RirSceneTarget target;
  target.external_target_id = 7U;
  target.position_x = 5000.0f;
  target.range_m = 5000.0f;
  target.rcs = 1.0f;
  input.scene_targets.push_back(target);
  return input;
}

bool HasCode(const session::RirIssueList& issues, const char* code) {
  for (const session::RirIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

TEST(RirSelfContainedValidationTest, SceneTargetMotionAndSwerlingFieldsAreValidated) {
  RirCycleInput input = MakeValidInput();
  input.scene_targets[0].velocity_y = std::numeric_limits<float>::infinity();
  const auto issues = ValidateRirCycleInput(input);
  EXPECT_TRUE(HasCode(issues, session::codes::kNonFiniteTargetField));

  input = MakeValidInput();
  input.scene_targets[0].target_swerling_type = static_cast<session::RirSwerlingType>(9);
  const auto swerling_issues = ValidateRirCycleInput(input);
  EXPECT_TRUE(HasCode(swerling_issues, session::codes::kInvalidTargetMotionField));
}

TEST(RirSelfContainedValidationTest, EnvironmentAndRfInputsAreValidated) {
  RirCycleInput input = MakeValidInput();
  input.environment_snapshot.has_environment_data = true;
  input.environment_snapshot.weather_attenuation_db = -1.0f;
  const auto env_issues = ValidateRirCycleInput(input);
  EXPECT_TRUE(HasCode(env_issues, session::codes::kInvalidEnvironmentSnapshot));

  input = MakeValidInput();
  oneq::electromagnetics::RfIncidentLinkResult link;
  link.identity.platform_id = 4U;
  link.identity.equipment_id = 5U;
  link.identity.emission_id = 6U;
  link.received_power_before_overlap_w = 1.0e-9;
  input.incident_links.push_back(link);
  const auto identity_issues = ValidateRirCycleInput(input);
  EXPECT_TRUE(HasCode(identity_issues, session::codes::kInvalidOwnEmissionIdentity));

  input.own_emission_identity = oneq::electromagnetics::RfEmissionIdentity{1U, 2U, 3U};
  input.incident_links[0].received_power_before_overlap_w = -1.0;
  const auto link_issues = ValidateRirCycleInput(input);
  EXPECT_TRUE(HasCode(link_issues, session::codes::kInvalidRfIncidentLink));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
