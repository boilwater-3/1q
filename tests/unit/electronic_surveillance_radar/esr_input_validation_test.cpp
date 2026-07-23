#include <gtest/gtest.h>

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
  input.interference.world_cycle_index = input.cycle_index;
  input.interference.window_start_time_s = input.cycle_start_time_s;
  input.interference.window_duration_s = input.dt_sec;
  return input;
}

TEST(EsrInputValidationTest, AcceptsEmptyRfV2EmissionFrame) {
  EXPECT_FALSE(HasValidationError(ValidateEsrCycleInput(MakeValidInput())));
}

TEST(EsrInputValidationTest, RejectsFrameOutsideCycleWindow) {
  EsrCycleInput input = MakeValidInput();
  input.interference.window_start_time_s += 0.1;
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
