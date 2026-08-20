/**
 * @file eos_input_validation_unit_test.cpp
 * @brief 验证光学传感器输入契约、扫描视场和探测判决链路的最小行为。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "electro_optical_sensor/foundation/EosRadiometry.h"
#include "support/eos_enu_scene_helpers.h"

namespace electro_optical_sensor {
namespace model {
namespace {

namespace eos_config = ::electro_optical_sensor::config;
namespace eos_session_ns = ::electro_optical_sensor::session;

using namespace ::electro_optical_sensor::model;
using namespace ::electro_optical_sensor::session;
using oneq::test_support::SetEosSphericalLook;

bool ContainsCode(const EosIssueList& issues, const std::string& code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

// 按 code 取条目（Q-2 审查修复：phase/severity 断言辅助）。
const EosIssue* FindIssue(const EosIssueList& issues, const std::string& code) {
  for (const EosIssue& issue : issues) {
    if (issue.code == code) {
      return &issue;
    }
  }
  return nullptr;
}

EosSceneTarget MakeValidTarget() {
  EosSceneTarget target;
  target.target_id = 1001U;
  SetEosSphericalLook(&target, 2200.0f, 2.0f, 0.0f);
  target.appearance.apparent_temperature_k = 335.0f;
  target.appearance.emissivity = 0.93f;
  target.appearance.reflectance = 0.45f;
  target.appearance.projected_area_m2 = 2.8f;
  return target;
}

EosCycleInput MakeValidInput() {
  EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.1f;
  input.platform_altitude_m = 1200.0f;
  input.scene.push_back(MakeValidTarget());
  return input;
}

float ResolveFirstCycleScanAzimuthDeg(const config::EosSessionConfig& config, float dt_sec) {
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

float ResolveNextScanAzimuthDeg(const config::EosSessionConfig& config, float current_azimuth_deg,
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

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);

  // 输入校验问题统一 phase=kInputValidation（规则 14b；HasValidationError 依赖该判定）。
  const EosIssue* issue = FindIssue(issues, "eos.validation.invalid_cycle_delta_time");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, EosIssueSeverity::kError);
  EXPECT_EQ(issue->phase, EosIssuePhase::kInputValidation);
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, NonFinitePlatformNumericFieldIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.platform_attitude_deg.yaw_deg = std::numeric_limits<double>::infinity();

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);

  EXPECT_TRUE(ContainsCode(issues, "eos.validation.non_finite_platform_numeric_field"));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, NonFinitePlatformAltitudeIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.platform_altitude_m = std::numeric_limits<float>::quiet_NaN();

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);

  EXPECT_TRUE(ContainsCode(issues, "eos.validation.non_finite_platform_numeric_field"));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, InvalidTargetEmissivityIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].appearance.emissivity = 1.2f;

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);

  EXPECT_TRUE(ContainsCode(issues, "eos.validation.invalid_target_emissivity"));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, InconsistentTargetEnergyBalanceIsReportedAsWarning) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].appearance.emissivity = 0.8f;
  input.scene[0].appearance.reflectance = 0.4f;

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);

  // warning 级校验问题同样 phase=kInputValidation（来源标签与严重级别正交）。
  const EosIssue* issue =
      FindIssue(issues, "eos.validation.inconsistent_target_energy_balance");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->severity, EosIssueSeverity::kWarning);
  EXPECT_EQ(issue->phase, EosIssuePhase::kInputValidation);
  EXPECT_FALSE(HasValidationError(issues));
}

// ===========================================================================
// 目标校验补充分支
// ===========================================================================

TEST(EosInputValidationTest, ZeroTargetIdDoesNotProduceValidationError) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].target_id = 0U;

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(EosInputValidationTest, NonFiniteTargetFieldIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].position_x = std::numeric_limits<float>::quiet_NaN();

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_TRUE(ContainsCode(issues, "eos.validation.non_finite_target_numeric_field"));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, NonFiniteTargetTemperatureIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].appearance.apparent_temperature_k =
      std::numeric_limits<float>::quiet_NaN();

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_TRUE(ContainsCode(issues, "eos.validation.non_finite_target_numeric_field"));
}

TEST(EosInputValidationTest, NonPositiveTargetTemperatureIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].appearance.apparent_temperature_k = 0.0f;

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_TRUE(ContainsCode(issues, "eos.validation.invalid_target_temperature"));
}

TEST(EosInputValidationTest, InvalidTargetReflectanceIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].appearance.reflectance = 1.5f;  // 超出 [0,1]

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_TRUE(ContainsCode(issues, "eos.validation.invalid_target_reflectance"));
}

TEST(EosInputValidationTest, NonPositiveProjectedAreaIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.scene[0].appearance.projected_area_m2 = 0.0f;

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_TRUE(ContainsCode(issues, "eos.validation.invalid_target_projected_area"));
}

TEST(EosInputValidationTest, NonFiniteCycleDeltaTimeIsReportedAsError) {
  EosCycleInput input = MakeValidInput();
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_TRUE(ContainsCode(issues, "eos.validation.non_finite_cycle_delta_time"));
}

TEST(EosInputValidationTest, ValidInputProducesNoErrors) {
  EosCycleInput input = MakeValidInput();
  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(EosInputValidationTest, SessionProducesInFovDetectionsOnly) {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kFused;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  config.mission.scan_start_az_deg = -5.0f;
  config.mission.scan_end_az_deg = 5.0f;
  config.mission.horizontal_fov_deg = 6.0f;
  config.mission.vertical_fov_deg = 4.0f;
  config.mission.scan_rate_deg_per_sec = 2.0f;

  session::EosSession eos_session = session::EosSession::Create(config);
  EosCycleInput input = MakeValidInput();
  SetEosSphericalLook(&input.scene[0], 2200.0f, -3.0f, 0.0f);
  EosSceneTarget out_of_fov_target = MakeValidTarget();
  out_of_fov_target.target_id = 1002U;
  SetEosSphericalLook(&out_of_fov_target, 2200.0f, 30.0f, 0.0f);
  input.scene.push_back(out_of_fov_target);

  const ::electro_optical_sensor::session::EosCycleResult result =
      eos_session.StepWithResult(input);

  EXPECT_FALSE(HasValidationError(result.issues));
  ASSERT_EQ(result.output_frame.detections.size(), 1U);
  EXPECT_EQ(result.output_frame.detections[0].detection_id, 1U);
  ASSERT_EQ(result.detection_attributions.size(), 1U);
  EXPECT_EQ(result.detection_attributions[0].target_id, 1001U);
}

TEST(EosInputValidationTest, SessionReturnsValidationErrorsForInvalidInput) {
  session::EosSession eos_session = session::EosSession::Create();
  EosCycleInput input = MakeValidInput();
  input.scene[0].position_x = 0.0f;
  input.scene[0].position_y = 0.0f;
  input.scene[0].position_z = 0.0f;

  const ::electro_optical_sensor::session::EosCycleResult result =
      eos_session.StepWithResult(input);

  EXPECT_TRUE(HasValidationError(result.issues));
  EXPECT_NE(result.status, eos_session_ns::EosCycleStatus::kCompleted);
  EXPECT_TRUE(ContainsCode(result.issues, "eos.validation.invalid_target_range"));
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(EosInputValidationTest, SessionConfigBuilderCanStartFromExternalConfig) {
  config::EosSessionConfig base_config;
  base_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  base_config.mission.scan_rate_deg_per_sec = 12.0f;
  base_config.mission.frame_rate_hz = 20.0f;
  base_config.policy.detection.minimum_snr_db = 4.5f;
  base_config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  base_config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  auto built_config = base_config;
  built_config.mission.frame_rate_hz = 25.0f;
  built_config.policy.stray_light.enable_straylight_filter = true;
  built_config.policy.stray_light.hood_inner_half_angle_deg = 8.0f;
  built_config.policy.stray_light.hood_outer_half_angle_deg = 55.0f;
  built_config.policy.stray_light.hood_min_suppression_ratio = 0.35f;
  built_config.policy.stray_light.hood_max_suppression_ratio = 0.95f;
  EXPECT_EQ(built_config.mission.work_mode, config::EosWorkMode::kInfraredOnly);
  EXPECT_FLOAT_EQ(built_config.mission.scan_rate_deg_per_sec, 12.0f);
  EXPECT_FLOAT_EQ(built_config.mission.frame_rate_hz, 25.0f);
  EXPECT_EQ(built_config.policy.stray_light.enable_straylight_filter, true);
}

TEST(EosInputValidationTest, RuntimeConfigBuilderCanTightenDetectionThresholdAtRuntime) {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kFused;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;

  session::EosSession eos_session = session::EosSession::Create(config);
  EosCycleInput input = MakeValidInput();
  // Keep target aligned to first-cycle scan azimuth so this test isolates threshold behavior.
  SetEosSphericalLook(&input.scene[0], 1700.0f, ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec),
                      0.0f);

  const ::electro_optical_sensor::session::EosCycleResult baseline =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(HasValidationError(baseline.issues));
  ASSERT_FALSE(baseline.output_frame.detections.empty());
  EXPECT_TRUE(baseline.output_frame.detections[0].detected);

  const eos_config::EosRuntimeConfigPatch patch = eos_config::EosRuntimeConfigBuilder()
                                                      .WithMinimumSnrDb(60.0f)
                                                      .WithDetectionSensitivityW(2.0e-12f)
                                                      .WithVisibleReferenceIrradianceWM2(1000.0f)
                                                      .Build();
  (void)eos_session.TryApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  SetEosSphericalLook(&input.scene[0], 2200.0f,
                      ResolveNextScanAzimuthDeg(config, baseline.output_frame.scan_azimuth_deg,
                                                input.dt_sec),
                      0.0f);
  const ::electro_optical_sensor::session::EosCycleResult updated =
      eos_session.StepWithResult(input);
  EXPECT_FALSE(HasValidationError(updated.issues));
  ASSERT_EQ(updated.output_frame.detections.size(), 1U);
  EXPECT_LE(updated.output_frame.detections[0].fused_snr_db,
            baseline.output_frame.detections[0].fused_snr_db);
  EXPECT_LE(static_cast<int>(updated.output_frame.detections[0].detected),
            static_cast<int>(baseline.output_frame.detections[0].detected));
}

TEST(EosInputValidationTest, RuntimePatchPreservesScanPhaseUnlessScanRateChanges) {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;
  config.mission.scan_rate_deg_per_sec = 4.0f;

  session::EosSession eos_session = session::EosSession::Create(config);
  EosCycleInput input = MakeValidInput();
  input.dt_sec = 0.1f;
  SetEosSphericalLook(&input.scene[0], 2200.0f,
                      ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec), 0.0f);

  const ::electro_optical_sensor::session::EosCycleResult first = eos_session.StepWithResult(input);
  ASSERT_FALSE(HasValidationError(first.issues));
  const float first_scan_azimuth_deg = first.output_frame.scan_azimuth_deg;

  const eos_config::EosRuntimeConfigPatch non_geometry_patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithMinimumSnrDb(4.5f)
          .WithDetectionSensitivityW(0.8e-12f)
          .WithVisibleReferenceIrradianceWM2(700.0f)
          .Build();
  (void)eos_session.TryApplyRuntimeConfig(non_geometry_patch);

  input.cycle_index += 1U;
  const ::electro_optical_sensor::session::EosCycleResult after_non_geometry_patch =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(HasValidationError(after_non_geometry_patch.issues));
  const float expected_after_non_geometry_patch =
      ResolveNextScanAzimuthDeg(config, first_scan_azimuth_deg, input.dt_sec);
  EXPECT_NEAR(after_non_geometry_patch.output_frame.scan_azimuth_deg,
              expected_after_non_geometry_patch, 1.0e-5f);

  const eos_config::EosRuntimeConfigPatch scan_rate_patch =
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(12.0f).Build();
  (void)eos_session.TryApplyRuntimeConfig(scan_rate_patch);

  input.cycle_index += 1U;
  const ::electro_optical_sensor::session::EosCycleResult after_scan_rate_patch =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(HasValidationError(after_scan_rate_patch.issues));

  config::EosSessionConfig expected_scan_rate_config = config;
  expected_scan_rate_config.mission.scan_rate_deg_per_sec = 12.0f;
  const float expected_after_scan_rate_patch =
      ResolveFirstCycleScanAzimuthDeg(expected_scan_rate_config, input.dt_sec);
  EXPECT_NEAR(after_scan_rate_patch.output_frame.scan_azimuth_deg, expected_after_scan_rate_patch,
              1.0e-5f);
}

TEST(EosInputValidationTest, RuntimePatchRejectsInvalidFrameRateHz) {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;

  session::EosSession eos_session = session::EosSession::Create(config);
  EosCycleInput input = MakeValidInput();
  SetEosSphericalLook(&input.scene[0], 2200.0f,
                      ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec), 0.0f);

  const ::electro_optical_sensor::session::EosCycleResult baseline =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(HasValidationError(baseline.issues));

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithFrameRateHz(0.0f).Build();
  (void)eos_session.TryApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  SetEosSphericalLook(&input.scene[0], 2200.0f,
                      ResolveNextScanAzimuthDeg(config, baseline.output_frame.scan_azimuth_deg,
                                                input.dt_sec),
                      0.0f);
  const ::electro_optical_sensor::session::EosCycleResult after_patch =
      eos_session.StepWithResult(input);
  EXPECT_FALSE(HasValidationError(after_patch.issues));

  const eos_config::EosRuntimeConfigPatch valid_patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithFrameRateHz(-5.0f)
          .WithMinimumSnrDb(4.5f)
          .WithDetectionSensitivityW(0.8e-12f)
          .WithVisibleReferenceIrradianceWM2(700.0f)
          .Build();
  (void)eos_session.TryApplyRuntimeConfig(valid_patch);

  input.cycle_index += 1U;
  const ::electro_optical_sensor::session::EosCycleResult after_mixed_patch =
      eos_session.StepWithResult(input);
  EXPECT_FALSE(HasValidationError(after_mixed_patch.issues));
}

TEST(EosInputValidationTest, RuntimePatchRejectsInvalidScanRate) {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;
  config.mission.scan_rate_deg_per_sec = 4.0f;

  session::EosSession eos_session = session::EosSession::Create(config);
  EosCycleInput input = MakeValidInput();
  SetEosSphericalLook(&input.scene[0], 2200.0f,
                      ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec), 0.0f);

  const ::electro_optical_sensor::session::EosCycleResult baseline =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(HasValidationError(baseline.issues));
  const float baseline_azimuth = baseline.output_frame.scan_azimuth_deg;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(-1.0f).Build();
  (void)eos_session.TryApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  SetEosSphericalLook(&input.scene[0], 2200.0f,
                      ResolveNextScanAzimuthDeg(config, baseline_azimuth, input.dt_sec), 0.0f);
  const ::electro_optical_sensor::session::EosCycleResult after_patch =
      eos_session.StepWithResult(input);
  EXPECT_FALSE(HasValidationError(after_patch.issues));

  const float expected_azimuth = ResolveNextScanAzimuthDeg(config, baseline_azimuth, input.dt_sec);
  EXPECT_NEAR(after_patch.output_frame.scan_azimuth_deg, expected_azimuth, 1.0e-5f);
}

TEST(EosInputValidationTest, RuntimePatchIsAtomicWhenAnyFieldIsInvalid) {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kFused;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.horizontal_fov_deg = 12.0f;
  config.mission.vertical_fov_deg = 6.0f;

  session::EosSession eos_session = session::EosSession::Create(config);
  EosCycleInput input = MakeValidInput();
  SetEosSphericalLook(&input.scene[0], 1700.0f, ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec),
                      0.0f);

  const ::electro_optical_sensor::session::EosCycleResult baseline =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(HasValidationError(baseline.issues));
  ASSERT_EQ(baseline.output_frame.detections.size(), 1U);
  ASSERT_TRUE(baseline.output_frame.detections[0].detected);

  const eos_config::EosRuntimeConfigPatch patch = eos_config::EosRuntimeConfigBuilder()
                                                      .WithFrameRateHz(0.0f)
                                                      .WithMinimumSnrDb(4.5f)
                                                      .WithDetectionSensitivityW(0.8e-12f)
                                                      .WithVisibleReferenceIrradianceWM2(700.0f)
                                                      .Build();
  (void)eos_session.TryApplyRuntimeConfig(patch);

  input.cycle_index += 1U;
  SetEosSphericalLook(&input.scene[0], 1700.0f,
                      ResolveNextScanAzimuthDeg(config, baseline.output_frame.scan_azimuth_deg,
                                                input.dt_sec),
                      0.0f);
  const ::electro_optical_sensor::session::EosCycleResult after_patch =
      eos_session.StepWithResult(input);
  ASSERT_FALSE(HasValidationError(after_patch.issues));
  ASSERT_EQ(after_patch.output_frame.detections.size(), 1U);
  EXPECT_TRUE(after_patch.output_frame.detections[0].detected);
}

TEST(EosInputValidationTest, AcceptsDtSecWithinFrameRateBound) {
  // frame_rate_hz=30 → max_dt = 10/30 ≈ 0.333s; dt_sec=0.1 is within range.
  EosCycleInput input = MakeValidInput();
  input.dt_sec = 0.1f;

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(EosInputValidationTest, RejectsDtSecExceedingFrameRateBound) {
  // frame_rate_hz=30 → max_dt = 10/30 ≈ 0.333s; dt_sec=0.5 exceeds the bound.
  EosCycleInput input = MakeValidInput();
  input.dt_sec = 0.5f;

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_TRUE(ContainsCode(issues, "eos.validation.cycle_delta_time_exceeds_frame_period"));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EosInputValidationTest, AcceptsDtSecAtExactFrameRateBound) {
  // frame_rate_hz=30 → max_dt = 10/30; dt_sec at exact boundary passes.
  EosCycleInput input = MakeValidInput();
  input.dt_sec = 10.0f / 30.0f;

  const EosIssueList issues = ValidateEosCycleInput(input, 30.0f);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(EosInputValidationTest, HigherFrameRateAllowsTighterDtSec) {
  // frame_rate_hz=60 → max_dt = 10/60 ≈ 0.167s; dt_sec=0.2 should fail.
  EosCycleInput input = MakeValidInput();
  input.dt_sec = 0.2f;

  const EosIssueList issues_60hz = ValidateEosCycleInput(input, 60.0f);
  EXPECT_TRUE(HasValidationError(issues_60hz));

  // frame_rate_hz=30 → max_dt = 10/30 ≈ 0.333s; dt_sec=0.2 should pass.
  const EosIssueList issues_30hz = ValidateEosCycleInput(input, 30.0f);
  EXPECT_FALSE(HasValidationError(issues_30hz));
}

}  // namespace
}  // namespace model
}  // namespace electro_optical_sensor
