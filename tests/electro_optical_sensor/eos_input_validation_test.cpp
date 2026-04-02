/**
 * @file eos_input_validation_test.cpp
 * @brief 验证光学传感器输入契约、扫描视场和探测判决链路的最小行为。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

#include "1q/electro_optical_sensor/core/context/EosCycleInput.h"
#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"
#include "1q/electro_optical_sensor/core/session/EosSession.h"

namespace electro_optical_sensor {
namespace core {
namespace context {
namespace {

bool ContainsCode(const EosValidationIssueList& issues, EosValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

EosTargetState MakeValidTarget() {
  EosTargetState target;
  target.target_id = 1001U;
  target.range_m = 2200.0f;
  target.azimuth_deg = 2.0f;
  target.elevation_deg = 0.0f;
  target.apparent_temperature_k = 335.0f;
  target.emissivity = 0.93f;
  target.reflectance = 0.45f;
  target.projected_area_m2 = 2.8f;
  return target;
}

EosCycleInput MakeValidInput() {
  EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.5f;
  input.solar_altitude_deg = 42.0f;
  input.solar_azimuth_deg = 165.0f;
  input.solar_irradiance_w_m2 = 900.0f;
  input.atmospheric_transmittance = 0.8f;
  input.cloud_coverage_ratio = 0.25f;
  input.day_night_type = DayNightType::kDay;
  input.background_temperature_k = 287.0f;
  input.scene_targets.push_back(MakeValidTarget());
  return input;
}

TEST(EosInputValidationTest, InvalidCycleDeltaTimeIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.dt_sec = 0.0f;

  const EosValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, EosValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(HasEosValidationError(issues));
}

TEST(EosInputValidationTest, NonFinitePlatformNumericFieldIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.platform_pose.attitude_deg.yaw_deg = std::numeric_limits<float>::infinity();

  const EosValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, EosValidationCode::kNonFinitePlatformNumericField));
  EXPECT_TRUE(HasEosValidationError(issues));
}

TEST(EosInputValidationTest, InvalidTargetEmissivityIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.scene_targets[0].emissivity = 1.2f;

  const EosValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, EosValidationCode::kInvalidTargetEmissivity));
  EXPECT_TRUE(HasEosValidationError(issues));
}

TEST(EosInputValidationTest, InvalidAmbientWindSpeedIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.ambient_wind_speed_mps = -1.0f;

  const EosValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, EosValidationCode::kInvalidAmbientWindSpeed));
  EXPECT_TRUE(HasEosValidationError(issues));
}

TEST(EosInputValidationTest, SessionProducesInFovDetectionsOnly) {
  session::EosSessionConfig config;
  config.work_mode = session::EosWorkMode::kFused;
  config.minimum_snr_db = 0.0f;
  config.scan_start_az_deg = -5.0f;
  config.scan_end_az_deg = 5.0f;
  config.horizontal_fov_deg = 6.0f;
  config.vertical_fov_deg = 4.0f;
  config.scan_rate_deg_per_sec = 2.0f;

  session::EosSession eos_session(config);
  EosCycleInput input = MakeValidInput();
  input.scene_targets[0].azimuth_deg = -3.0f;
  EosTargetState out_of_fov_target = MakeValidTarget();
  out_of_fov_target.target_id = 1002U;
  out_of_fov_target.azimuth_deg = 30.0f;
  input.scene_targets.push_back(out_of_fov_target);

  const session::EosCycleResult result = eos_session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  ASSERT_EQ(result.output_frame.detections.size(), 1U);
  EXPECT_EQ(result.output_frame.detections[0].target_id, 1001U);
}

TEST(EosInputValidationTest, SessionReturnsValidationErrorsForInvalidInput) {
  session::EosSession eos_session;
  EosCycleInput input = MakeValidInput();
  input.scene_targets[0].range_m = 0.0f;

  const session::EosCycleResult result = eos_session.StepWithResult(input);

  EXPECT_TRUE(result.has_validation_error);
  EXPECT_TRUE(ContainsCode(result.validation_issues, EosValidationCode::kInvalidTargetRange));
  EXPECT_TRUE(result.output_frame.detections.empty());
}

}  // namespace
}  // namespace context
}  // namespace core
}  // namespace electro_optical_sensor
