// Copyright 2026. All Rights Reserved.
//
// @file esr_public_api_convenience_test.cpp
// @brief ESR 对外易用性 API 契约测试（高层语义配置版本）。

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace tests {
namespace {

bool ContainsEsrIssueCode(const std::vector<session::ValidationIssue>& issues,
                          session::ValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

config::EsrSessionConfig MakeSessionConfig() {
  config::EsrSessionConfig config =
      config::EsrSessionConfigBuilder()
          .Mission()
          .WithMissionProfile(config::EsrMissionProfile::kElectronicOrderOfBattle)
          .End()
          .Detection()
          .WithSensitivityProfile(config::EsrSensitivityProfile::kStandard)
          .End()
          .Environment()
          .WithEnvironmentPreset(config::EsrEnvironmentPreset::kStandard)
          .End()
          .Build();
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_az_deg = -60.0f;
  config.mission.scan.scan_end_az_deg = 60.0f;
  config.mission.scan.scan_start_el_deg = -20.0f;
  config.mission.scan.scan_end_el_deg = 20.0f;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;
  return config;
}

}  // namespace

TEST(EsrPublicApiConvenienceTest, SessionConfigBuilderDefaultsMatchDefaultConfig) {
  const config::EsrSessionConfig built = config::EsrSessionConfigBuilder().Build();
  const config::EsrSessionConfig defaults;

  EXPECT_EQ(built.mission.work_mode, defaults.mission.work_mode);
  EXPECT_EQ(built.environment.scenario_config.preset, defaults.environment.scenario_config.preset);
}

TEST(EsrPublicApiConvenienceTest, SessionConfigBuilderAppliesSemanticProfiles) {
  config::EsrSessionConfig cfg =
      config::EsrSessionConfigBuilder()
          .Mission()
          .WithMissionProfile(config::EsrMissionProfile::kThreatWarning)
          .End()
          .Detection()
          .WithSensitivityProfile(config::EsrSensitivityProfile::kHighSensitivity)
          .End()
          .Environment()
          .WithEnvironmentPreset(config::EsrEnvironmentPreset::kJammed)
          .End()
          .Build();
  cfg.mission.power_on = false;

  EXPECT_EQ(cfg.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FALSE(cfg.mission.power_on);
  EXPECT_FLOAT_EQ(cfg.mission.scan.scan_rate_hz, 5.0f);
  EXPECT_FLOAT_EQ(cfg.policy.detection.minimum_snr_db, 3.0f);
  EXPECT_EQ(cfg.environment.scenario_config.preset, config::EsrEnvironmentPreset::kJammed);
}

TEST(EsrPublicApiConvenienceTest, DetailedSessionConfigBuilderSupportsDetailedDetectionParams) {
  config::EsrSessionConfig details_cfg{};
  details_cfg.policy.detection.minimum_snr_db = 8.0f;
  details_cfg.policy.detection.pfa = 1.0e-5f;
  details_cfg.policy.detection.pulse_count = 16U;
  details_cfg.policy.detection.threshold_scale = 0.9f;
  details_cfg.policy.detection.enable_statistical_detection = true;
  details_cfg.mission.scan.scan_rate_hz = 4.0f;
  details_cfg.mission.scan.use_explicit_scan_bounds = true;
  details_cfg.mission.scan.scan_start_az_deg = -30.0f;
  details_cfg.mission.scan.scan_end_az_deg = 30.0f;
  details_cfg.mission.scan.scan_start_el_deg = -5.0f;
  details_cfg.mission.scan.scan_end_el_deg = 5.0f;
  details_cfg.environment.scenario_config.preset = config::EsrEnvironmentPreset::kLowClutter;

  EXPECT_FLOAT_EQ(details_cfg.policy.detection.minimum_snr_db, 8.0f);
  EXPECT_FLOAT_EQ(details_cfg.policy.detection.pfa, 1.0e-5f);
  EXPECT_EQ(details_cfg.policy.detection.pulse_count, 16U);
  EXPECT_FLOAT_EQ(details_cfg.policy.detection.threshold_scale, 0.9f);
  EXPECT_TRUE(details_cfg.policy.detection.enable_statistical_detection);
  EXPECT_FLOAT_EQ(details_cfg.mission.scan.scan_rate_hz, 4.0f);
  EXPECT_TRUE(details_cfg.mission.scan.use_explicit_scan_bounds);
  EXPECT_EQ(details_cfg.environment.scenario_config.preset,
            config::EsrEnvironmentPreset::kLowClutter);
}

TEST(EsrPublicApiConvenienceTest, SessionConfigBuilderProfilesOverrideWithSessionConfig) {
  config::EsrSessionConfig baseline;
  baseline.mission.work_mode = config::EsrWorkMode::kHgesm;
  baseline.mission.scan.scan_rate_hz = 0.25f;
  baseline.policy.detection.minimum_snr_db = 12.0f;

  const config::EsrSessionConfig config =
      config::EsrSessionConfigBuilder()
          .Mission()
          .WithMissionProfile(config::EsrMissionProfile::kThreatWarning)
          .End()
          .Detection()
          .WithSensitivityProfile(config::EsrSensitivityProfile::kHighSensitivity)
          .End()
          .WithSessionConfig(baseline)
          .Build();

  // WithSessionConfig() does not reset profiles; profiles override baseline fields.
  EXPECT_EQ(config.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FLOAT_EQ(config.mission.scan.scan_rate_hz, 5.0f);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 3.0f);
}

TEST(EsrPublicApiConvenienceTest, SessionConfigValidatorReportsFinalConfigIssues) {
  config::EsrSessionConfig invalid_config;
  invalid_config.mission.scan.scan_rate_hz = 0.0f;
  invalid_config.hardware.receiver_band_lower_hz = 10.0e9;
  invalid_config.hardware.receiver_band_upper_hz = 9.0e9;
  invalid_config.hardware.beam_az_width_deg = 0.0f;
  invalid_config.hardware.beam_el_width_deg = -1.0f;
  invalid_config.mission.scan.use_explicit_scan_bounds = true;
  invalid_config.mission.scan.scan_start_az_deg = 10.0f;
  invalid_config.mission.scan.scan_end_az_deg = 10.0f;
  invalid_config.mission.scan.scan_start_el_deg = 5.0f;
  invalid_config.mission.scan.scan_end_el_deg = 5.0f;

  const config::ValidationIssueList issues = config::ValidateEsrSessionConfig(invalid_config);

  ASSERT_EQ(issues.size(), 6U);
  EXPECT_EQ(issues[0].code, config::ConfigValidationCode::kScanRateNotPositive);
  EXPECT_EQ(issues[1].code, config::ConfigValidationCode::kReceiverBandLowerAboveUpper);
  EXPECT_EQ(issues[2].code, config::ConfigValidationCode::kBeamAzWidthNotPositive);
  EXPECT_EQ(issues[3].code, config::ConfigValidationCode::kBeamElWidthNotPositive);
  EXPECT_EQ(issues[4].code, config::ConfigValidationCode::kExplicitScanBoundsAzSwapped);
  EXPECT_EQ(issues[5].code, config::ConfigValidationCode::kExplicitScanBoundsElSwapped);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderDefaultsAreUnset) {
  const config::EsrRuntimeConfigPatch patch = config::EsrRuntimeConfigBuilder().Build();

  EXPECT_FALSE(patch.has_mission);
  EXPECT_FALSE(patch.has_policy);
  EXPECT_FALSE(patch.has_environment);
  EXPECT_FALSE(patch.has_sensor_enabled);
  EXPECT_FALSE(patch.has_work_mode);
  EXPECT_FALSE(patch.has_scan_rate_hz);
  EXPECT_FALSE(patch.has_scan_center_az_deg);
  EXPECT_FALSE(patch.has_explicit_scan_bounds);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderSetsSemanticFields) {
  config::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.65f;

  const config::EsrRuntimeConfigPatch patch =
      config::EsrRuntimeConfigBuilder()
          .WithSensorEnabled(false)
          .WithWorkMode(config::EsrWorkMode::kHgesm)
          .WithScanRateHz(3.0f)
          .WithScanCenterAzDeg(15.0f)
          .WithScanCenterElDeg(5.0f)
          .WithExplicitScanBoundsDeg(-30.0f, 30.0f, -10.0f, 10.0f)
          .WithAtmosphericPhysicsConfig(atmospheric_physics)
          .Build();

  EXPECT_TRUE(patch.has_sensor_enabled);
  EXPECT_FALSE(patch.sensor_enabled);
  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, config::EsrWorkMode::kHgesm);
  EXPECT_TRUE(patch.has_scan_rate_hz);
  EXPECT_FLOAT_EQ(patch.scan_rate_hz, 3.0f);
  EXPECT_TRUE(patch.has_scan_center_az_deg);
  EXPECT_TRUE(patch.has_scan_center_el_deg);
  EXPECT_TRUE(patch.has_explicit_scan_bounds);
  EXPECT_TRUE(patch.explicit_scan_bounds.enabled);
  EXPECT_TRUE(patch.has_environment);
  EXPECT_TRUE(patch.environment.has_atmospheric_physics);
  EXPECT_TRUE(patch.environment.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(patch.environment.atmospheric_physics.relative_humidity, 0.65f);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderSupportsDomainOverrides) {
  config::EsrMissionConfig mission;
  mission.power_on = false;
  mission.work_mode = config::EsrWorkMode::kRwr;
  mission.scan.scan_rate_hz = 5.0f;

  config::EsrPolicyConfig policy;
  policy.detection.minimum_snr_db = 5.0f;

  const config::EsrRuntimeConfigPatch patch =
      config::EsrRuntimeConfigBuilder().WithMission(mission).WithPolicy(policy).Build();

  EXPECT_TRUE(patch.has_mission);
  EXPECT_FALSE(patch.mission.power_on);
  EXPECT_EQ(patch.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FLOAT_EQ(patch.mission.scan.scan_rate_hz, 5.0f);
  EXPECT_TRUE(patch.has_policy);
  EXPECT_FLOAT_EQ(patch.policy.detection.minimum_snr_db, 5.0f);
}

TEST(EsrPublicApiConvenienceTest, InputValidationReportsErrorsForBoundaryCases) {
  session::EsrCycleInput input;
  input.dt_sec = 0.0f;
  input.platform_attitude_deg.yaw_deg = std::numeric_limits<double>::infinity();

  input.platform_entity_id = 0U;

  const session::ValidationIssueList issues = session::ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsEsrIssueCode(issues, session::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(
      ContainsEsrIssueCode(issues, session::ValidationCode::kNonFinitePlatformNumericField));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(EsrPublicApiConvenienceTest, SessionStepAndRuntimePatchWorkTogether) {
  session::EsrSession session = session::EsrSession::Create(MakeSessionConfig());

  session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;

  const session::EsrCycleResult baseline = session.StepWithResult(input);
  EXPECT_EQ(baseline.status, session::EsrCycleExecutionStatus::kCompleted);

  const config::EsrRuntimeConfigPatch patch =
      config::EsrRuntimeConfigBuilder().WithSensorEnabled(false).Build();
  session.ApplyRuntimeConfig(patch);

  const session::EsrCycleResult updated = session.StepWithResult(input);
  EXPECT_TRUE(updated.output_frame.observation_output.observations.empty());
}

TEST(EsrPublicApiConvenienceTest, TryApplyRuntimeConfigExposesRejectFeedback) {
  session::EsrSession session = session::EsrSession::Create(MakeSessionConfig());

  config::EsrRuntimeConfigPatch invalid_patch;
  invalid_patch.has_explicit_scan_bounds = true;
  invalid_patch.explicit_scan_bounds.enabled = true;
  invalid_patch.explicit_scan_bounds.scan_start_az_deg = std::numeric_limits<float>::quiet_NaN();
  invalid_patch.explicit_scan_bounds.scan_end_az_deg = 10.0f;
  invalid_patch.explicit_scan_bounds.scan_start_el_deg = -10.0f;
  invalid_patch.explicit_scan_bounds.scan_end_el_deg = 10.0f;

  const session::EsrRuntimeConfigApplyResult invalid_result =
      session.ApplyRuntimeConfigWithResult(invalid_patch);
  EXPECT_FALSE(invalid_result.applied);
  EXPECT_TRUE(invalid_result.has_requested_update);
  EXPECT_EQ(invalid_result.status,
            session::EsrRuntimeConfigApplyStatus::kRejectedInvalidExplicitScanBounds);
  EXPECT_FALSE(session.TryApplyRuntimeConfig(invalid_patch));

  const config::EsrRuntimeConfigPatch valid_patch =
      config::EsrRuntimeConfigBuilder().WithSensorEnabled(false).Build();
  const session::EsrRuntimeConfigApplyResult valid_result =
      session.ApplyRuntimeConfigWithResult(valid_patch);
  EXPECT_TRUE(valid_result.applied);
  EXPECT_TRUE(valid_result.has_requested_update);
  EXPECT_EQ(valid_result.status, session::EsrRuntimeConfigApplyStatus::kApplied);
  EXPECT_TRUE(session.TryApplyRuntimeConfig(valid_patch));
}

TEST(EsrPublicApiConvenienceTest, CreateWithValidationBuildsSessionAndReportsNoIssuesForHealthyConfig) {
  config::EsrSessionConfig config;

  config::ValidationIssueList issues;
  const session::EsrSession session = session::EsrSession::CreateWithValidation(config, &issues);

  EXPECT_TRUE(issues.empty());
  (void)session;
}

TEST(EsrPublicApiConvenienceTest, CreateWithValidationReportsIssuesButStillConstructsSession) {
  config::EsrSessionConfig invalid;
  invalid.mission.scan.scan_rate_hz = 0.0f;

  config::ValidationIssueList issues;
  const session::EsrSession session = session::EsrSession::CreateWithValidation(invalid, &issues);

  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues.front().code, config::ConfigValidationCode::kScanRateNotPositive);
  (void)session;  // 会话仍被构造，调用方据 issues 决策
}

TEST(EsrPublicApiConvenienceTest, CreateWithValidationAcceptsNullIssuesWithoutCrash) {
  config::EsrSessionConfig invalid;
  invalid.mission.scan.scan_rate_hz = 0.0f;

  const session::EsrSession session = session::EsrSession::CreateWithValidation(invalid, nullptr);
  (void)session;  // nullptr 时仅构造，不写回 issues
}

}  // namespace tests
}  // namespace electronic_surveillance_radar
