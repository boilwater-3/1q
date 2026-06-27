// Copyright 2026. All Rights Reserved.
//
// @file radar_input_validation_test.cpp
// @brief 验证雷达周期输入校验器的边界行为，覆盖以下缺口：
//   - 笛卡尔位置与斜距的组合有效性语义
//   - 负距离 / 零距离的错误判定
//   - 非有限数（NaN / Inf）全字段检测
//   - 重复外部 ID、零 ID、负 RCS 的级别判定
//   - 周期步长 (dt) 的有效性校验

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"

namespace airborne_radar {
namespace tests {

using session::HasValidationError;
using session::ValidateRadarCycleInput;
using session::ValidateRadarSceneTargets;
using session::ValidationCode;
using session::ValidationSeverity;

namespace {

// 构造一个最简有效目标（位于 X 轴方向 1000m，有速度有 ID）
session::RadarSceneTarget MakeValidTarget(std::uint64_t id = 1u) {
  session::RadarSceneTarget t(100.0f, 0.0f, 0.0f, 1.0f);
  t.external_target_id = id;
  t.position_x = 1000.0f;
  t.position_y = 0.0f;
  t.position_z = 0.0f;
  t.range_m = 1000.0f;
  return t;
}

// 在 issues 列表中查找指定编码的第一条记录
const session::ValidationIssue* FindIssue(
    const std::vector<session::ValidationIssue>& issues, ValidationCode code) {
  for (const auto& issue : issues) {
    if (issue.code == code) {
      return &issue;
    }
  }
  return nullptr;
}

}  // namespace

// ===========================================================================
// 笛卡尔位置与斜距组合语义
// ===========================================================================

/// @brief 目标位于原点 (0,0,0) 且 range_m <= 0 → 必须报 kMissingRangeAndCartesianPosition。
TEST(RadarInputValidationTest, OriginWithNoRangeIsError) {
  session::RadarSceneTarget target;
  target.external_target_id = 1u;
  target.position_x = 0.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 0.0f;  // 无有效斜距
  target.rcs = 1.0f;

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kMissingRangeAndCartesianPosition), nullptr);
}

/// @brief 目标位于原点 (0,0,0) 但 range_m > 0 → 斜距有效，不报位置缺失错误。
TEST(RadarInputValidationTest, OriginWithPositiveRangeIsValid) {
  session::RadarSceneTarget target;
  target.external_target_id = 1u;
  target.position_x = 0.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 5000.0f;  // 斜距有效
  target.rcs = 1.0f;

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_EQ(FindIssue(issues, ValidationCode::kMissingRangeAndCartesianPosition), nullptr);
  EXPECT_FALSE(HasValidationError(issues));
}

/// @brief 目标具有有效笛卡尔位置且 range_m <= 0 → 不报位置缺失错误。
TEST(RadarInputValidationTest, FlaggedCartesianPositionWithNonPositiveRangeIsValid) {
  session::RadarSceneTarget target;
  target.external_target_id = 1u;
  target.position_x = 3000.0f;  // 有笛卡尔位置
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 0.0f;  // 无斜距
  target.rcs = 1.0f;

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_EQ(FindIssue(issues, ValidationCode::kMissingRangeAndCartesianPosition), nullptr);
}

/// @brief 非零坐标自动视为有效笛卡尔位置，即使 range_m <= 0 也不报位置缺失错误。
TEST(RadarInputValidationTest, NonZeroCoordinatesImpliesCartesianPositionAndIsValid) {
  session::RadarSceneTarget target;
  target.external_target_id = 1u;
  target.position_x = 3000.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 0.0f;
  target.rcs = 1.0f;

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_EQ(FindIssue(issues, ValidationCode::kMissingRangeAndCartesianPosition), nullptr);
  EXPECT_FALSE(HasValidationError(issues));
}

/// @brief 原点 (0,0,0) 且 range_m <= 0 → 报位置缺失错误。
TEST(RadarInputValidationTest, OriginWithNoRangeIsErrorEvenWithoutPositionFlag) {
  session::RadarSceneTarget target;
  target.external_target_id = 1u;
  target.position_x = 0.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 0.0f;
  target.rcs = 1.0f;

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kMissingRangeAndCartesianPosition), nullptr);
}

/// @brief 目标 range_m 为负值且无有效笛卡尔位置 → 同样报错。
TEST(RadarInputValidationTest, NegativeRangeWithNoPositionIsError) {
  session::RadarSceneTarget target;
  target.external_target_id = 1u;
  target.position_x = 0.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = -100.0f;
  target.rcs = 1.0f;

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kMissingRangeAndCartesianPosition), nullptr);
}

// ===========================================================================
// 非有限数字段检测
// ===========================================================================

/// @brief 位置字段含 NaN → 报 kNonFiniteTargetField（Error 级别）。
TEST(RadarInputValidationTest, NanPositionFieldIsError) {
  session::RadarSceneTarget target = MakeValidTarget();
  target.position_x = std::numeric_limits<float>::quiet_NaN();

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kNonFiniteTargetField), nullptr);
}

/// @brief 速度字段含 Inf → 报 kNonFiniteTargetField。
TEST(RadarInputValidationTest, InfVelocityFieldIsError) {
  session::RadarSceneTarget target = MakeValidTarget();
  target.velocity_y = std::numeric_limits<float>::infinity();

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kNonFiniteTargetField), nullptr);
}

/// @brief RCS 字段含 NaN → 报 kNonFiniteTargetField。
TEST(RadarInputValidationTest, NanRcsFieldIsError) {
  session::RadarSceneTarget target = MakeValidTarget();
  target.rcs = std::numeric_limits<float>::quiet_NaN();

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kNonFiniteTargetField), nullptr);
}

/// @brief range_m 字段含 -Inf → 报 kNonFiniteTargetField。
TEST(RadarInputValidationTest, NegativeInfRangeFieldIsError) {
  session::RadarSceneTarget target = MakeValidTarget();
  target.range_m = -std::numeric_limits<float>::infinity();

  const auto issues = ValidateRadarSceneTargets({target});
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kNonFiniteTargetField), nullptr);
}

// ===========================================================================
// 外部 ID 与 RCS 级别判定
// ===========================================================================

/// @brief external_target_id == 0 → kInfo 级别（不阻断执行）。
TEST(RadarInputValidationTest, ZeroExternalIdIsInfo) {
  session::RadarSceneTarget target = MakeValidTarget(0u);

  const auto issues = ValidateRadarSceneTargets({target});
  const session::ValidationIssue* issue =
      FindIssue(issues, ValidationCode::kUnknownExternalTargetId);
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, ValidationSeverity::kInfo);
  EXPECT_FALSE(HasValidationError(issues));
}

/// @brief 同一 external_target_id 出现两次 → Error 级别。
TEST(RadarInputValidationTest, DuplicateExternalIdIsError) {
  session::RadarSceneTarget t1 = MakeValidTarget(42u);
  session::RadarSceneTarget t2 = MakeValidTarget(42u);
  t2.position_x = 2000.0f;
  t2.range_m = 2000.0f;

  const auto issues = ValidateRadarSceneTargets({t1, t2});
  const session::ValidationIssue* issue =
      FindIssue(issues, ValidationCode::kDuplicateExternalTargetId);
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, ValidationSeverity::kError);
  EXPECT_TRUE(HasValidationError(issues));
}

/// @brief 负 RCS → Warning 级别（不阻断执行）。
TEST(RadarInputValidationTest, NegativeRcsIsWarning) {
  session::RadarSceneTarget target = MakeValidTarget();
  target.rcs = -0.5f;

  const auto issues = ValidateRadarSceneTargets({target});
  const session::ValidationIssue* issue = FindIssue(issues, ValidationCode::kNegativeRcs);
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, ValidationSeverity::kWarning);
  EXPECT_FALSE(HasValidationError(issues));
}

// ===========================================================================
// 周期步长 (dt) 校验
// ===========================================================================

/// @brief 正常正值 dt → 无问题。
TEST(RadarInputValidationTest, PositiveDtIsValid) {
  session::RadarCycleInput input;
  input.dt_sec = 0.5f;
  input.scene.push_back(MakeValidTarget());

  const auto issues = ValidateRadarCycleInput(input);
  EXPECT_FALSE(HasValidationError(issues));
  EXPECT_EQ(FindIssue(issues, ValidationCode::kInvalidCycleDeltaTime), nullptr);
  EXPECT_EQ(FindIssue(issues, ValidationCode::kNonFiniteCycleDeltaTime), nullptr);
}

/// @brief dt == 0 → Error 级别（kInvalidCycleDeltaTime）。
TEST(RadarInputValidationTest, ZeroDtIsError) {
  session::RadarCycleInput input;
  input.dt_sec = 0.0f;
  input.scene.push_back(MakeValidTarget());

  const auto issues = ValidateRadarCycleInput(input);
  const session::ValidationIssue* issue =
      FindIssue(issues, ValidationCode::kInvalidCycleDeltaTime);
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, ValidationSeverity::kError);
  EXPECT_TRUE(HasValidationError(issues));
}

/// @brief dt < 0 → Error 级别（kInvalidCycleDeltaTime）。
TEST(RadarInputValidationTest, NegativeDtIsError) {
  session::RadarCycleInput input;
  input.dt_sec = -1.0f;
  input.scene.push_back(MakeValidTarget());

  const auto issues = ValidateRadarCycleInput(input);
  const session::ValidationIssue* issue =
      FindIssue(issues, ValidationCode::kInvalidCycleDeltaTime);
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, ValidationSeverity::kError);
  EXPECT_TRUE(HasValidationError(issues));
}

/// @brief dt 为 NaN → Error 级别（kNonFiniteCycleDeltaTime）。
TEST(RadarInputValidationTest, NanDtIsError) {
  session::RadarCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  input.scene.push_back(MakeValidTarget());

  const auto issues = ValidateRadarCycleInput(input);
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kNonFiniteCycleDeltaTime), nullptr);
}

/// @brief dt 为 Inf → Error 级别（kNonFiniteCycleDeltaTime）。
TEST(RadarInputValidationTest, InfDtIsError) {
  session::RadarCycleInput input;
  input.dt_sec = std::numeric_limits<float>::infinity();
  input.scene.push_back(MakeValidTarget());

  const auto issues = ValidateRadarCycleInput(input);
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kNonFiniteCycleDeltaTime), nullptr);
}

/// @brief platform_pose 任意字段为非有限数时应报 kNonFinitePlatformNumericField。
TEST(RadarInputValidationTest, NonFinitePlatformPoseIsError) {
  session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose.attitude_deg.yaw_deg = std::numeric_limits<float>::quiet_NaN();
  input.scene.push_back(MakeValidTarget());

  const auto issues = ValidateRadarCycleInput(input);
  EXPECT_TRUE(HasValidationError(issues));
  EXPECT_NE(FindIssue(issues, ValidationCode::kNonFinitePlatformNumericField), nullptr);
}

/// @brief 未提供环境快照时，不应校验默认构造的 environment 字段。
TEST(RadarInputValidationTest, OmittedEnvironmentSnapshotDoesNotValidateEnvironmentDefaults) {
  session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  input.has_environment = false;
  input.environment.atmospheric_observation.pressure_hpa = -1.0f;
  input.scene.push_back(MakeValidTarget());

  const auto issues = ValidateRadarCycleInput(input);
  EXPECT_EQ(FindIssue(issues, ValidationCode::kInvalidEnvironmentObservation), nullptr);
  EXPECT_FALSE(HasValidationError(issues));
}

/// @brief 显式提供环境快照时，环境字段仍应按原规则校验。
TEST(RadarInputValidationTest, ExplicitEnvironmentSnapshotIsValidated) {
  session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  input.has_environment = true;
  input.environment.atmospheric_observation.pressure_hpa = -1.0f;
  input.scene.push_back(MakeValidTarget());

  const auto issues = ValidateRadarCycleInput(input);
  EXPECT_NE(FindIssue(issues, ValidationCode::kInvalidEnvironmentObservation), nullptr);
  EXPECT_TRUE(HasValidationError(issues));
}

// ===========================================================================
// 完全有效输入 — 基线 Smoke Test
// ===========================================================================

/// @brief 完全合法输入 → 无任何问题。
TEST(RadarInputValidationTest, FullyValidInputProducesNoIssues) {
  session::RadarCycleInput input;
  input.dt_sec = 0.5f;
  input.scene.push_back(MakeValidTarget(101u));
  input.scene.push_back(MakeValidTarget(102u));

  const auto issues = ValidateRadarCycleInput(input);
  EXPECT_TRUE(issues.empty());
}

}  // namespace tests
}  // namespace airborne_radar
