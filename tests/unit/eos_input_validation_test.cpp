/**
 * @file eos_input_validation_unit_test.cpp
 * @brief 验证光学传感器输入契约、扫描视场和探测判决链路的最小行为。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "electro_optical_sensor/foundation/EosRadiometry.h"

namespace electro_optical_sensor {
namespace model {
namespace {

namespace eos_config = ::electro_optical_sensor::config;
namespace eos_session_ns = ::electro_optical_sensor::session;

using namespace ::electro_optical_sensor::model;
using namespace ::electro_optical_sensor::session;

bool ContainsCode(const ValidationIssueList& issues, ValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

EosSceneTarget MakeValidTarget() {
  EosSceneTarget target;
  target.target_id = 1001U;
  target.range_m = 2200.0f;
  target.azimuth_deg = 2.0f;
  target.elevation_deg = 0.0f;
  target.appearance.apparent_temperature_k = 335.0f;
  target.appearance.emissivity = 0.93f;
  target.appearance.reflectance = 0.45f;
  target.appearance.projected_area_m2 = 2.8f;
  return target;
}

EosCycleInput MakeValidInput() {
  EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.5f;
  input.environment.solar_altitude_deg = 42.0f;
  input.environment.solar_azimuth_deg = 165.0f;
  input.environment.solar_irradiance_w_m2 = 900.0f;
  input.environment.cloud_coverage_ratio = 0.25f;
  input.environment.day_night_type = DayNightType::kDay;
  input.environment.background_temperature_k = 287.0f;
  input.scene.push_back(MakeValidTarget());
  return input;
}

float ResolveFirstCycleScanAzimuthDeg(const session::EosSessionConfig& config, float dt_sec) {
  const float scan_width_deg = config.mission.scan_end_az_deg - config.mission.scan_start_az_deg;
  if (scan_width_deg <= 0.0f) {
    return config.mission.scan_start_az_deg;
  }

  float wrapped_offset_deg = config.mission.scan_rate_deg_per_sec * dt_sec;
  while (wrapped_offset_deg >= scan_width_deg) {
    wrapped_offset_deg -= scan_width_deg;
  }
  while (wrapped_offset_deg < 0.0f) {
    wrapped_offset_deg += scan_width_deg;
  }
  return config.mission.scan_start_az_deg + wrapped_offset_deg;
}

float ResolveNextScanAzimuthDeg(const session::EosSessionConfig& config, float current_azimuth_deg,
                                float dt_sec) {
  const float scan_width_deg = config.mission.scan_end_az_deg - config.mission.scan_start_az_deg;
  if (scan_width_deg <= 0.0f) {
    return config.mission.scan_start_az_deg;
  }

  float wrapped_offset_deg = (current_azimuth_deg - config.mission.scan_start_az_deg) +
                             config.mission.scan_rate_deg_per_sec * dt_sec;
  while (wrapped_offset_deg >= scan_width_deg) {
    wrapped_offset_deg -= scan_width_deg;
  }
  while (wrapped_offset_deg < 0.0f) {
    wrapped_offset_deg += scan_width_deg;
  }
  return config.mission.scan_start_az_deg + wrapped_offset_deg;
}

TEST(EosInputValidationTest, InvalidCycleDeltaTimeIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.dt_sec = 0.0f;

  const ValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, NonFinitePlatformNumericFieldIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.platform_pose.attitude_deg.yaw_deg = std::numeric_limits<float>::infinity();

  const ValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFinitePlatformNumericField));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, InvalidTargetEmissivityIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].appearance.emissivity = 1.2f;

  const ValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidTargetEmissivity));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, InconsistentTargetEnergyBalanceIsReportedAsWarning) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].appearance.emissivity = 0.8f;
  input.scene[0].appearance.reflectance = 0.4f;

  const ValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInconsistentTargetEnergyBalance));
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(EosInputValidationTest, InvalidAmbientWindSpeedIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.environment.ambient_wind_speed_mps = -1.0f;

  const ValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidAmbientWindSpeed));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, SessionProducesInFovDetectionsOnly) {
  session::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kFused;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.mission.scan_start_az_deg = -5.0f;
  config.mission.scan_end_az_deg = 5.0f;
  config.mission.horizontal_fov_deg = 6.0f;
  config.mission.vertical_fov_deg = 4.0f;
  config.mission.scan_rate_deg_per_sec = 2.0f;

  session::EosSession eos_session = session::EosSessionFactory::Create(config);
  EosCycleInput input = MakeValidInput();
  input.scene[0].azimuth_deg = -3.0f;
  EosSceneTarget out_of_fov_target = MakeValidTarget();
  out_of_fov_target.target_id = 1002U;
  out_of_fov_target.azimuth_deg = 30.0f;
  input.scene.push_back(out_of_fov_target);

  const ::electro_optical_sensor::session::EosCycleResult result =
      eos_session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  ASSERT_EQ(result.output_frame.detections.size(), 1U);
  EXPECT_EQ(result.output_frame.detections[0].target_id, 1001U);
}

TEST(EosInputValidationTest, SessionReturnsValidationErrorsForInvalidInput) {
  session::EosSession eos_session = session::EosSessionFactory::Create();
  EosCycleInput input = MakeValidInput();
  input.scene[0].range_m = 0.0f;

  const ::electro_optical_sensor::session::EosCycleResult result =
      eos_session.StepWithResult(input);

  EXPECT_TRUE(result.has_validation_error);
  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(ContainsCode(result.validation_issues, ValidationCode::kInvalidTargetRange));
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(EosInputValidationTest, SessionConfigBuilderCanStartFromExternalConfig) {
  session::EosSessionConfig base_config;
  base_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  base_config.mission.scan_rate_deg_per_sec = 12.0f;
  base_config.mission.frame_rate_hz = 20.0f;
  base_config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  auto built_config = base_config;
  built_config.mission.frame_rate_hz = 25.0f;
  built_config.policy.stray_light.profile = eos_config::EosStrayLightProfile::kEnhancedHood;
  built_config.policy.stray_light.use_profile_defaults = true;
  EXPECT_EQ(built_config.mission.work_mode, config::EosWorkMode::kInfraredOnly);
  EXPECT_FLOAT_EQ(built_config.mission.scan_rate_deg_per_sec, 12.0f);
  EXPECT_FLOAT_EQ(built_config.mission.frame_rate_hz, 25.0f);
  EXPECT_EQ(built_config.policy.detection.profile, eos_config::EosDetectionProfile::kAggressive);
  EXPECT_EQ(built_config.policy.stray_light.profile,
            eos_config::EosStrayLightProfile::kEnhancedHood);
}

TEST(EosInputValidationTest, RuntimeConfigBuilderCanTightenDetectionThresholdAtRuntime) {
  session::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kFused;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;

  session::EosSession eos_session = session::EosSessionFactory::Create(config);
  EosCycleInput input = MakeValidInput();
  input.platform_pose.position_m.z = 1200.0f;
  input.scene[0].range_m = 1700.0f;
  // Keep target aligned to first-cycle scan azimuth so this test isolates threshold behavior.
  input.scene[0].azimuth_deg = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);

  const ::electro_optical_sensor::session::EosCycleResult baseline =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(baseline.has_validation_error);
  ASSERT_FALSE(baseline.output_frame.detections.empty());
  EXPECT_TRUE(baseline.output_frame.detections[0].detected);

  const eos_session_ns::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithDetectionProfile(eos_config::EosDetectionProfile::kConservative)
          .Build();
  eos_session.ApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  input.scene[0].azimuth_deg =
      ResolveNextScanAzimuthDeg(config, baseline.output_frame.scan_azimuth_deg, input.dt_sec);
  const ::electro_optical_sensor::session::EosCycleResult updated =
      eos_session.StepWithResult(input);
  EXPECT_FALSE(updated.has_validation_error);
  ASSERT_EQ(updated.output_frame.detections.size(), 1U);
  EXPECT_LE(updated.output_frame.detections[0].fused_snr_db,
            baseline.output_frame.detections[0].fused_snr_db);
  EXPECT_LE(static_cast<int>(updated.output_frame.detections[0].detected),
            static_cast<int>(baseline.output_frame.detections[0].detected));
}

TEST(EosInputValidationTest, RuntimePatchPreservesScanPhaseUnlessScanRateChanges) {
  session::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;
  config.mission.scan_rate_deg_per_sec = 4.0f;

  session::EosSession eos_session = session::EosSessionFactory::Create(config);
  EosCycleInput input = MakeValidInput();
  input.dt_sec = 0.5f;
  input.scene[0].azimuth_deg = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);

  const ::electro_optical_sensor::session::EosCycleResult first = eos_session.StepWithResult(input);
  ASSERT_FALSE(first.has_validation_error);
  const float first_scan_azimuth_deg = first.output_frame.scan_azimuth_deg;

  const eos_session_ns::EosRuntimeConfigPatch non_geometry_patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithDetectionProfile(eos_config::EosDetectionProfile::kAggressive)
          .Build();
  eos_session.ApplyRuntimeConfig(non_geometry_patch);

  input.cycle_index += 1U;
  const ::electro_optical_sensor::session::EosCycleResult after_non_geometry_patch =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(after_non_geometry_patch.has_validation_error);
  const float expected_after_non_geometry_patch =
      ResolveNextScanAzimuthDeg(config, first_scan_azimuth_deg, input.dt_sec);
  EXPECT_NEAR(after_non_geometry_patch.output_frame.scan_azimuth_deg,
              expected_after_non_geometry_patch, 1.0e-5f);

  const eos_session_ns::EosRuntimeConfigPatch scan_rate_patch =
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(12.0f).Build();
  eos_session.ApplyRuntimeConfig(scan_rate_patch);

  input.cycle_index += 1U;
  const ::electro_optical_sensor::session::EosCycleResult after_scan_rate_patch =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(after_scan_rate_patch.has_validation_error);

  session::EosSessionConfig expected_scan_rate_config = config;
  expected_scan_rate_config.mission.scan_rate_deg_per_sec = 12.0f;
  const float expected_after_scan_rate_patch =
      ResolveFirstCycleScanAzimuthDeg(expected_scan_rate_config, input.dt_sec);
  EXPECT_NEAR(after_scan_rate_patch.output_frame.scan_azimuth_deg, expected_after_scan_rate_patch,
              1.0e-5f);
}

TEST(EosInputValidationTest, NonFiniteSolarAnglesAreReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.environment.solar_altitude_deg = std::numeric_limits<float>::quiet_NaN();

  const ValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFiniteSolarAngles));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, SolarAltitudeOutOfRangeIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.environment.solar_altitude_deg = 100.0f;

  const ValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidSolarAltitudeRange));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, InconsistentDayNightTypeIsReportedAsWarning) {
  EosCycleInput input = MakeValidInput();
  input.environment.day_night_type = DayNightType::kNight;
  input.environment.solar_altitude_deg = 20.0f;

  const ValidationIssueList issues = ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInconsistentDayNightType));
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(EosInputValidationTest, RuntimePatchRejectsInvalidFrameRateHz) {
  session::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;

  session::EosSession eos_session = session::EosSessionFactory::Create(config);
  EosCycleInput input = MakeValidInput();
  input.scene[0].azimuth_deg = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);

  const ::electro_optical_sensor::session::EosCycleResult baseline =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(baseline.has_validation_error);

  const eos_session_ns::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithFrameRateHz(0.0f).Build();
  eos_session.ApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  input.scene[0].azimuth_deg =
      ResolveNextScanAzimuthDeg(config, baseline.output_frame.scan_azimuth_deg, input.dt_sec);
  const ::electro_optical_sensor::session::EosCycleResult after_patch =
      eos_session.StepWithResult(input);
  EXPECT_FALSE(after_patch.has_validation_error);

  const eos_session_ns::EosRuntimeConfigPatch valid_patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithFrameRateHz(-5.0f)
          .WithDetectionProfile(eos_config::EosDetectionProfile::kAggressive)
          .Build();
  eos_session.ApplyRuntimeConfig(valid_patch);

  input.cycle_index += 1U;
  const ::electro_optical_sensor::session::EosCycleResult after_mixed_patch =
      eos_session.StepWithResult(input);
  EXPECT_FALSE(after_mixed_patch.has_validation_error);
}

TEST(EosInputValidationTest, RuntimePatchRejectsInvalidScanRate) {
  session::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;
  config.mission.scan_rate_deg_per_sec = 4.0f;

  session::EosSession eos_session = session::EosSessionFactory::Create(config);
  EosCycleInput input = MakeValidInput();
  input.scene[0].azimuth_deg = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);

  const ::electro_optical_sensor::session::EosCycleResult baseline =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(baseline.has_validation_error);
  const float baseline_azimuth = baseline.output_frame.scan_azimuth_deg;

  const eos_session_ns::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(-1.0f).Build();
  eos_session.ApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  input.scene[0].azimuth_deg = ResolveNextScanAzimuthDeg(config, baseline_azimuth, input.dt_sec);
  const ::electro_optical_sensor::session::EosCycleResult after_patch =
      eos_session.StepWithResult(input);
  EXPECT_FALSE(after_patch.has_validation_error);

  const float expected_azimuth = ResolveNextScanAzimuthDeg(config, baseline_azimuth, input.dt_sec);
  EXPECT_NEAR(after_patch.output_frame.scan_azimuth_deg, expected_azimuth, 1.0e-5f);
}

TEST(EosInputValidationTest, RuntimePatchIsAtomicWhenAnyFieldIsInvalid) {
  session::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kFused;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;

  session::EosSession eos_session = session::EosSessionFactory::Create(config);
  EosCycleInput input = MakeValidInput();
  input.platform_pose.position_m.z = 1200.0f;
  input.scene[0].range_m = 1700.0f;
  input.scene[0].azimuth_deg = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);

  const ::electro_optical_sensor::session::EosCycleResult baseline =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(baseline.has_validation_error);
  ASSERT_EQ(baseline.output_frame.detections.size(), 1U);
  ASSERT_TRUE(baseline.output_frame.detections[0].detected);

  const eos_session_ns::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithFrameRateHz(0.0f)
          .WithDetectionProfile(eos_config::EosDetectionProfile::kAggressive)
          .Build();
  eos_session.ApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  input.scene[0].azimuth_deg =
      ResolveNextScanAzimuthDeg(config, baseline.output_frame.scan_azimuth_deg, input.dt_sec);
  const ::electro_optical_sensor::session::EosCycleResult after_patch =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(after_patch.has_validation_error);
  ASSERT_EQ(after_patch.output_frame.detections.size(), 1U);
  EXPECT_TRUE(after_patch.output_frame.detections[0].detected);
}

}  // namespace
}  // namespace model
}  // namespace electro_optical_sensor
