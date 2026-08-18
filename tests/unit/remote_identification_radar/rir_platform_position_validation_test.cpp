// Copyright 2026. All Rights Reserved.
//
// @file rir_platform_position_validation_test.cpp
// @brief 验证平台 ECEF 位置可选输入的 fail-closed 校验（冻结契约 §7 修订 1a）。
//
// 覆盖：合法位置通过；非有限分量/地心位置拒绝（整周期拒绝）；
// has=false 但分量非默认 → 存在性标志与数据不一致拒绝（实现安全规则 2）；
// 缺省输入零影响（既有调用方兼容）。

#include <gtest/gtest.h>

#include <cmath>

#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirInputValidation.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirIssueList;
using session::RirSceneTarget;

RirCycleInput MakeValidInput() {
  RirCycleInput input;
  input.input_cycle_index = 1U;
  input.batch_id = 1U;
  input.dt_sec = 0.5;
  input.sim_time_sec = 0.0f;
  input.platform_altitude_m = 1000.0f;
  RirSceneTarget target;
  target.external_target_id = 7U;
  target.position_x = 5000.0f;
  target.position_z = 2000.0f;
  target.rcs = 1.0f;
  target.range_m = 5000.0f;
  input.scene_targets.push_back(target);
  return input;
}

bool HasCode(const RirIssueList& issues, const char* code) {
  for (const session::RirIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

/// @brief 合法 ECEF 位置通过（分量有限且模长 > 0）。
TEST(RirPlatformPositionValidationTest, ValidEcefPositionPasses) {
  RirCycleInput input = MakeValidInput();
  input.has_platform_position = true;
  input.platform_position.x_m = 6378137.0;
  input.platform_position.y_m = 0.0;
  input.platform_position.z_m = 0.0;

  const RirIssueList issues = session::ValidateRirCycleInput(input);

  EXPECT_FALSE(HasCode(issues, session::codes::kInvalidPlatformPosition));
  EXPECT_FALSE(HasCode(issues, session::codes::kInconsistentPlatformPosition));
  EXPECT_FALSE(session::HasValidationError(issues));
}

/// @brief 非有限分量拒绝：fail-closed，整周期拒绝。
TEST(RirPlatformPositionValidationTest, NonFiniteComponentRejected) {
  RirCycleInput input = MakeValidInput();
  input.has_platform_position = true;
  input.platform_position.x_m = std::nan("");

  const RirIssueList issues = session::ValidateRirCycleInput(input);

  EXPECT_TRUE(HasCode(issues, session::codes::kInvalidPlatformPosition));
  EXPECT_TRUE(session::HasValidationError(issues));
}

/// @brief 地心位置（模长 0）拒绝。
TEST(RirPlatformPositionValidationTest, GeocenterPositionRejected) {
  RirCycleInput input = MakeValidInput();
  input.has_platform_position = true;  // 分量保持默认 (0,0,0)。

  const RirIssueList issues = session::ValidateRirCycleInput(input);

  EXPECT_TRUE(HasCode(issues, session::codes::kInvalidPlatformPosition));
  EXPECT_TRUE(session::HasValidationError(issues));
}

/// @brief has=false 但分量非默认：存在性标志与数据不一致拒绝。
TEST(RirPlatformPositionValidationTest, InconsistentFlagRejected) {
  RirCycleInput input = MakeValidInput();
  input.platform_position.z_m = 100.0;

  const RirIssueList issues = session::ValidateRirCycleInput(input);

  EXPECT_TRUE(HasCode(issues, session::codes::kInconsistentPlatformPosition));
  EXPECT_TRUE(session::HasValidationError(issues));
}

/// @brief 缺省输入（has=false + 默认分量）通过：既有调用方零影响。
TEST(RirPlatformPositionValidationTest, DefaultAbsencePasses) {
  const RirCycleInput input = MakeValidInput();

  const RirIssueList issues = session::ValidateRirCycleInput(input);

  EXPECT_FALSE(HasCode(issues, session::codes::kInvalidPlatformPosition));
  EXPECT_FALSE(HasCode(issues, session::codes::kInconsistentPlatformPosition));
  EXPECT_FALSE(session::HasValidationError(issues));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
