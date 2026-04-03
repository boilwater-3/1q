/**
 * @file eos_input_validation_unit_test.cpp
 * @brief 验证光学传感器输入契约、扫描视场和探测判决链路的最小行为。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

#include "1q/electro_optical_sensor/core/context/EosCycleInput.h"
#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"
#include "1q/electro_optical_sensor/core/session/EosSession.h"
#include "1q/electro_optical_sensor/foundation/EosRadiometry.h"

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

float ResolveFirstCycleScanAzimuthDeg(const session::EosSessionConfig& config, float dt_sec) {
  const float scan_width_deg = config.scan_end_az_deg - config.scan_start_az_deg;
  if (scan_width_deg <= 0.0f) {
    return config.scan_start_az_deg;
  }

  float wrapped_offset_deg = config.scan_rate_deg_per_sec * dt_sec;
  while (wrapped_offset_deg >= scan_width_deg) {
    wrapped_offset_deg -= scan_width_deg;
  }
  while (wrapped_offset_deg < 0.0f) {
    wrapped_offset_deg += scan_width_deg;
  }
  return config.scan_start_az_deg + wrapped_offset_deg;
}

float ResolveNextScanAzimuthDeg(const session::EosSessionConfig& config, float current_azimuth_deg,
                                float dt_sec) {
  const float scan_width_deg = config.scan_end_az_deg - config.scan_start_az_deg;
  if (scan_width_deg <= 0.0f) {
    return config.scan_start_az_deg;
  }

  float wrapped_offset_deg =
      (current_azimuth_deg - config.scan_start_az_deg) + config.scan_rate_deg_per_sec * dt_sec;
  while (wrapped_offset_deg >= scan_width_deg) {
    wrapped_offset_deg -= scan_width_deg;
  }
  while (wrapped_offset_deg < 0.0f) {
    wrapped_offset_deg += scan_width_deg;
  }
  return config.scan_start_az_deg + wrapped_offset_deg;
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

TEST(EosInputValidationTest, SessionConfigBuilderCanStartFromExternalConfig) {
  session::EosSessionConfig base_config;
  base_config.work_mode = session::EosWorkMode::kInfraredOnly;
  base_config.scan_rate_deg_per_sec = 12.0f;
  base_config.frame_rate_hz = 20.0f;
  base_config.minimum_snr_db = 2.0f;
  base_config.visible_reference_irradiance_w_m2 = 620.0f;

  const session::EosSessionConfig built_config = session::EosSessionConfigBuilder(base_config)
                                                     .WithFrameRateHz(25.0f)
                                                     .EnableStraylightFilter(true)
                                                     .Build();
  EXPECT_EQ(built_config.work_mode, session::EosWorkMode::kInfraredOnly);
  EXPECT_FLOAT_EQ(built_config.scan_rate_deg_per_sec, 12.0f);
  EXPECT_FLOAT_EQ(built_config.frame_rate_hz, 25.0f);
  EXPECT_FLOAT_EQ(built_config.minimum_snr_db, 2.0f);
  EXPECT_FLOAT_EQ(built_config.visible_reference_irradiance_w_m2, 620.0f);
  EXPECT_TRUE(built_config.enable_straylight_filter);
}

TEST(EosInputValidationTest, RuntimeConfigBuilderCanTightenDetectionThresholdAtRuntime) {
  session::EosSessionConfig config;
  config.work_mode = session::EosWorkMode::kFused;
  config.minimum_snr_db = -120.0f;
  config.scan_start_az_deg = -20.0f;
  config.scan_end_az_deg = 20.0f;
  config.horizontal_fov_deg = 12.0f;
  config.vertical_fov_deg = 6.0f;

  session::EosSession eos_session(config);
  EosCycleInput input = MakeValidInput();
  input.platform_pose.position_m.z = 1200.0f;
  input.scene_targets[0].range_m = 1700.0f;
  // Keep target aligned to first-cycle scan azimuth so this test isolates threshold behavior.
  input.scene_targets[0].azimuth_deg = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);

  const session::EosCycleResult baseline = eos_session.StepWithResult(input);
  ASSERT_FALSE(baseline.has_validation_error);
  ASSERT_FALSE(baseline.output_frame.detections.empty());
  EXPECT_TRUE(baseline.output_frame.detections[0].detected);
  const float tightened_threshold_db = baseline.output_frame.detections[0].fused_snr_db + 3.0f;

  const session::EosRuntimeConfigPatch patch =
      session::EosRuntimeConfigBuilder().WithMinimumSnrDb(tightened_threshold_db).Build();
  eos_session.ApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  input.scene_targets[0].azimuth_deg =
      ResolveNextScanAzimuthDeg(config, baseline.output_frame.scan_azimuth_deg, input.dt_sec);
  const session::EosCycleResult updated = eos_session.StepWithResult(input);
  EXPECT_FALSE(updated.has_validation_error);
  ASSERT_EQ(updated.output_frame.detections.size(), 1U);
  EXPECT_FALSE(updated.output_frame.detections[0].detected);
}

TEST(EosInputValidationTest, RuntimePatchPreservesScanPhaseUnlessScanRateChanges) {
  session::EosSessionConfig config;
  config.work_mode = session::EosWorkMode::kInfraredOnly;
  config.minimum_snr_db = -120.0f;
  config.scan_start_az_deg = -20.0f;
  config.scan_end_az_deg = 20.0f;
  config.horizontal_fov_deg = 12.0f;
  config.vertical_fov_deg = 6.0f;
  config.scan_rate_deg_per_sec = 4.0f;

  session::EosSession eos_session(config);
  EosCycleInput input = MakeValidInput();
  input.dt_sec = 0.5f;
  input.scene_targets[0].azimuth_deg = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);

  const session::EosCycleResult first = eos_session.StepWithResult(input);
  ASSERT_FALSE(first.has_validation_error);
  const float first_scan_azimuth_deg = first.output_frame.scan_azimuth_deg;

  const session::EosRuntimeConfigPatch non_geometry_patch =
      session::EosRuntimeConfigBuilder().WithMinimumSnrDb(-60.0f).Build();
  eos_session.ApplyRuntimeConfig(non_geometry_patch);

  input.cycle_index += 1U;
  const session::EosCycleResult after_non_geometry_patch = eos_session.StepWithResult(input);
  ASSERT_FALSE(after_non_geometry_patch.has_validation_error);
  const float expected_after_non_geometry_patch =
      ResolveNextScanAzimuthDeg(config, first_scan_azimuth_deg, input.dt_sec);
  EXPECT_NEAR(after_non_geometry_patch.output_frame.scan_azimuth_deg,
              expected_after_non_geometry_patch, 1.0e-5f);

  const session::EosRuntimeConfigPatch scan_rate_patch =
      session::EosRuntimeConfigBuilder().WithScanRateDegPerSec(12.0f).Build();
  eos_session.ApplyRuntimeConfig(scan_rate_patch);

  input.cycle_index += 1U;
  const session::EosCycleResult after_scan_rate_patch = eos_session.StepWithResult(input);
  ASSERT_FALSE(after_scan_rate_patch.has_validation_error);

  session::EosSessionConfig expected_scan_rate_config = config;
  expected_scan_rate_config.scan_rate_deg_per_sec = 12.0f;
  const float expected_after_scan_rate_patch =
      ResolveFirstCycleScanAzimuthDeg(expected_scan_rate_config, input.dt_sec);
  EXPECT_NEAR(after_scan_rate_patch.output_frame.scan_azimuth_deg, expected_after_scan_rate_patch,
              1.0e-5f);
}

TEST(EosInputValidationTest, NonFiniteSolarAnglesAreReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.solar_altitude_deg = std::numeric_limits<float>::quiet_NaN();

  const EosValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, EosValidationCode::kNonFiniteSolarAngles));
  EXPECT_TRUE(HasEosValidationError(issues));
}

TEST(EosInputValidationTest, SolarAltitudeOutOfRangeIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.solar_altitude_deg = 100.0f;

  const EosValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, EosValidationCode::kInvalidSolarAltitudeRange));
  EXPECT_TRUE(HasEosValidationError(issues));
}

TEST(EosInputValidationTest, RuntimePatchRejectsInvalidFrameRateHz) {
  session::EosSessionConfig config;
  config.work_mode = session::EosWorkMode::kInfraredOnly;
  config.minimum_snr_db = -120.0f;
  config.scan_start_az_deg = -20.0f;
  config.scan_end_az_deg = 20.0f;
  config.horizontal_fov_deg = 12.0f;
  config.vertical_fov_deg = 6.0f;

  session::EosSession eos_session(config);
  EosCycleInput input = MakeValidInput();
  input.scene_targets[0].azimuth_deg = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);

  const session::EosCycleResult baseline = eos_session.StepWithResult(input);
  ASSERT_FALSE(baseline.has_validation_error);

  const session::EosRuntimeConfigPatch patch =
      session::EosRuntimeConfigBuilder().WithFrameRateHz(0.0f).Build();
  eos_session.ApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  input.scene_targets[0].azimuth_deg =
      ResolveNextScanAzimuthDeg(config, baseline.output_frame.scan_azimuth_deg, input.dt_sec);
  const session::EosCycleResult after_patch = eos_session.StepWithResult(input);
  EXPECT_FALSE(after_patch.has_validation_error);

  const session::EosRuntimeConfigPatch valid_patch =
      session::EosRuntimeConfigBuilder().WithFrameRateHz(-5.0f).WithMinimumSnrDb(-80.0f).Build();
  eos_session.ApplyRuntimeConfig(valid_patch);

  input.cycle_index += 1U;
  const session::EosCycleResult after_mixed_patch = eos_session.StepWithResult(input);
  EXPECT_FALSE(after_mixed_patch.has_validation_error);
}

TEST(EosInputValidationTest, RuntimePatchRejectsInvalidScanRate) {
  session::EosSessionConfig config;
  config.work_mode = session::EosWorkMode::kInfraredOnly;
  config.minimum_snr_db = -120.0f;
  config.scan_start_az_deg = -20.0f;
  config.scan_end_az_deg = 20.0f;
  config.horizontal_fov_deg = 12.0f;
  config.vertical_fov_deg = 6.0f;
  config.scan_rate_deg_per_sec = 4.0f;

  session::EosSession eos_session(config);
  EosCycleInput input = MakeValidInput();
  input.scene_targets[0].azimuth_deg = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);

  const session::EosCycleResult baseline = eos_session.StepWithResult(input);
  ASSERT_FALSE(baseline.has_validation_error);
  const float baseline_azimuth = baseline.output_frame.scan_azimuth_deg;

  const session::EosRuntimeConfigPatch patch =
      session::EosRuntimeConfigBuilder().WithScanRateDegPerSec(-1.0f).Build();
  eos_session.ApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  input.scene_targets[0].azimuth_deg =
      ResolveNextScanAzimuthDeg(config, baseline_azimuth, input.dt_sec);
  const session::EosCycleResult after_patch = eos_session.StepWithResult(input);
  EXPECT_FALSE(after_patch.has_validation_error);

  const float expected_azimuth = ResolveNextScanAzimuthDeg(config, baseline_azimuth, input.dt_sec);
  EXPECT_NEAR(after_patch.output_frame.scan_azimuth_deg, expected_azimuth, 1.0e-5f);
}

}  // namespace
}  // namespace context
}  // namespace core
}  // namespace electro_optical_sensor
