#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

EsrCycleInput MakeValidInput() {
  EsrCycleInput input;
  input.cycle_index = 4U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  return input;
}

TEST(EsrInputValidationTest, AcceptsEmptyRfV2EmissionFrame) {
  EXPECT_FALSE(HasValidationError(ValidateEsrCycleInput(MakeValidInput())));
}

TEST(EsrInputValidationTest, RejectsFrameOutsideCycleWindow) {
  EsrCycleInput input = MakeValidInput();
  input.rf_emissions.window_start_time_s += 0.1;
  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  ASSERT_TRUE(HasValidationError(issues));
  EXPECT_EQ(issues.front().code, ValidationCode::kInvalidRfEmissionFrame);
}

TEST(EsrInputValidationTest, RejectsMissingReceiverIdentityOrEcefKinematics) {
  EsrCycleInput input = MakeValidInput();
  input.platform_entity_id = 0U;
  input.has_platform_ecef_kinematics = false;
  EXPECT_TRUE(HasValidationError(ValidateEsrCycleInput(input)));
}

TEST(EsrInputValidationTest, RejectsUnlocatablePlatformEcef) {
  // 地心点 (0,0,0) 是 finite 的，能通过既有 finite 校验，但 TryEcefToLla 因范数过小而失败；
  // 此类输入必须在输入校验即被拒绝，而不是延迟到 pipeline 运行期被伪装成 RF-link 拒绝。
  EsrCycleInput input = MakeValidInput();
  input.platform_position_ecef_m.x_m = 0.0;
  input.platform_position_ecef_m.y_m = 0.0;
  input.platform_position_ecef_m.z_m = 0.0;
  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  ASSERT_TRUE(HasValidationError(issues));
  const auto it = std::find_if(
      issues.begin(), issues.end(),
      [](const ValidationIssue& issue) { return issue.code == ValidationCode::kUnlocatablePlatformEcef; });
  ASSERT_NE(it, issues.end());
  EXPECT_EQ(it->location.kind, ValidationLocationKind::kPlatform);
}

TEST(EsrInputValidationTest, RejectsNonFiniteWorldTimeAndEnvironment) {
  EsrCycleInput input = MakeValidInput();
  input.cycle_start_time_s = std::numeric_limits<double>::quiet_NaN();
  input.environment.atmospheric_observation.visibility_km =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(HasValidationError(ValidateEsrCycleInput(input)));
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
