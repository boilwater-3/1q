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

#include "1q/coordinate/position_transform.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSessionFactory.h"

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

session::EsrSceneEmitter MakeEmitter(std::uint64_t id) {
  session::EsrSceneEmitter emitter;
  emitter.emitter_id = id;
  emitter.pose.position_m.x = 1200.0f;
  emitter.pose.position_m.z = 5200.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 5.0e7;
  emitter.pulse_width_s = 1.0e-6;
  emitter.pri_s = 1.0e-4;
  return emitter;
}

config::EsrSessionConfig MakeSessionConfig() {
  config::EsrSessionConfig config =
      config::EsrSessionConfigBuilder()
          .Mission()
          .WithWorkMode(config::EsrWorkMode::kEsm)
          .End()
          .Detection()
          .WithMinDetectSnrDb(6.0f)
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

TEST(EsrPublicApiConvenienceTest, SessionConfigBuilderOverridesDomainFields) {
  const config::EsrSessionConfig cfg =
      config::EsrSessionConfigBuilder()
          .Mission()
          .WithWorkMode(config::EsrWorkMode::kRwr)
          .WithPowerOn(false)
          .WithScanRateHz(2.0f)
          .End()
          .Detection()
          .WithMinDetectSnrDb(3.0f)
          .End()
          .Environment()
          .WithEnvironmentPreset(config::EsrEnvironmentPreset::kJammed)
          .End()
          .Build();

  EXPECT_EQ(cfg.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FALSE(cfg.mission.power_on);
  EXPECT_FLOAT_EQ(cfg.mission.scan.scan_rate_hz, 2.0f);
  EXPECT_FLOAT_EQ(cfg.policy.detection.min_detect_snr_db, 3.0f);
  EXPECT_EQ(cfg.environment.scenario_config.preset, config::EsrEnvironmentPreset::kJammed);
}

TEST(EsrPublicApiConvenienceTest, DetailedSessionConfigBuilderSupportsDetailedDetectionParams) {
  config::EsrSessionConfig details_cfg{};
  details_cfg.policy.detection.min_detect_snr_db = 8.0f;
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

  EXPECT_FLOAT_EQ(details_cfg.policy.detection.min_detect_snr_db, 8.0f);
  EXPECT_FLOAT_EQ(details_cfg.policy.detection.pfa, 1.0e-5f);
  EXPECT_EQ(details_cfg.policy.detection.pulse_count, 16U);
  EXPECT_FLOAT_EQ(details_cfg.policy.detection.threshold_scale, 0.9f);
  EXPECT_TRUE(details_cfg.policy.detection.enable_statistical_detection);
  EXPECT_FLOAT_EQ(details_cfg.mission.scan.scan_rate_hz, 4.0f);
  EXPECT_TRUE(details_cfg.mission.scan.use_explicit_scan_bounds);
  EXPECT_EQ(details_cfg.environment.scenario_config.preset,
            config::EsrEnvironmentPreset::kLowClutter);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderDefaultsAreUnset) {
  const config::EsrRuntimeConfigPatch patch = config::EsrRuntimeConfigBuilder().Build();

  EXPECT_FALSE(patch.has_mission);
  EXPECT_FALSE(patch.has_policy);
  EXPECT_FALSE(patch.has_environment_runtime_config);
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
  config::EsrAtmosphericDerivedContext atmospheric_context;
  atmospheric_context.has_day_of_year = true;
  atmospheric_context.day_of_year = 210;

  const config::EsrRuntimeConfigPatch patch =
      config::EsrRuntimeConfigBuilder()
          .WithSensorEnabled(false)
          .WithWorkMode(config::EsrWorkMode::kHgesm)
          .WithScanRateHz(3.0f)
          .WithScanCenterAzDeg(15.0f)
          .WithScanCenterElDeg(5.0f)
          .WithExplicitScanBoundsDeg(-30.0f, 30.0f, -10.0f, 10.0f)
          .WithAtmosphericPhysicsConfig(atmospheric_physics)
          .WithAtmosphericContext(atmospheric_context)
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
  EXPECT_TRUE(patch.has_environment_runtime_config);
  EXPECT_TRUE(patch.environment_runtime_config.has_atmospheric_physics);
  EXPECT_TRUE(patch.environment_runtime_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(patch.environment_runtime_config.atmospheric_physics.relative_humidity, 0.65f);
  EXPECT_TRUE(patch.environment_runtime_config.has_atmospheric_context);
  EXPECT_TRUE(patch.environment_runtime_config.atmospheric_context.has_day_of_year);
  EXPECT_EQ(patch.environment_runtime_config.atmospheric_context.day_of_year, 210);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderSupportsDomainOverrides) {
  config::EsrMissionConfig mission;
  mission.power_on = false;
  mission.work_mode = config::EsrWorkMode::kRwr;
  mission.scan.scan_rate_hz = 5.0f;

  config::EsrPolicyConfig policy;
  policy.detection.min_detect_snr_db = 5.0f;

  const config::EsrRuntimeConfigPatch patch =
      config::EsrRuntimeConfigBuilder().WithMission(mission).WithPolicy(policy).Build();

  EXPECT_TRUE(patch.has_mission);
  EXPECT_FALSE(patch.mission.power_on);
  EXPECT_EQ(patch.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FLOAT_EQ(patch.mission.scan.scan_rate_hz, 5.0f);
  EXPECT_TRUE(patch.has_policy);
  EXPECT_FLOAT_EQ(patch.policy.detection.min_detect_snr_db, 5.0f);
}

TEST(EsrPublicApiConvenienceTest, InputValidationReportsErrorsForBoundaryCases) {
  session::EsrCycleInput input;
  input.dt_sec = 0.0f;
  input.platform_pose.position_m.x = std::numeric_limits<float>::infinity();

  session::EsrSceneEmitter invalid_emitter;
  invalid_emitter.emitter_id = 0U;
  invalid_emitter.carrier_hz = -1.0;
  input.scene.push_back(invalid_emitter);

  const session::ValidationIssueList issues = session::ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsEsrIssueCode(issues, session::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(
      ContainsEsrIssueCode(issues, session::ValidationCode::kNonFinitePlatformNumericField));
  EXPECT_TRUE(ContainsEsrIssueCode(issues, session::ValidationCode::kEmptyEmitterId));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(EsrPublicApiConvenienceTest, CoordinateUtilsBuildsPoseFromExternalKinematics) {
  oneq::coordinate::LocalFrameReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  oneq::coordinate::EcefPositionM target_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  session::EsrExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = target_ecef;

  oneq::foundation::PoseState pose;
  ASSERT_TRUE(session::TryMakeEsrPoseFromExternalKinematics(pose_input, reference, &pose));
  EXPECT_GT(pose.position_m.x, 100.0f);
}

TEST(EsrPublicApiConvenienceTest, CoordinateUtilsBuildsSceneEmitterFromExternalInput) {
  oneq::coordinate::LocalFrameReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::coordinate::LlaPositionDegM emitter_lla;
  emitter_lla.latitude_deg = 0.0;
  emitter_lla.longitude_deg = 0.001;
  emitter_lla.altitude_m = 0.0;

  oneq::coordinate::EcefPositionM emitter_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(emitter_lla, &emitter_ecef));

  session::EsrExternalEmitterInput input;
  input.emitter_id = 1001U;
  input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  input.kinematics.position_ecef_m = emitter_ecef;
  input.kinematics.velocity_mps.x_mps = 3.0f;  // ECEF X → ENU Up at lat=0,lon=0
  input.kinematics.velocity_mps.y_mps = 1.0f;  // ECEF Y → ENU East
  input.kinematics.velocity_mps.z_mps = 2.0f;  // ECEF Z → ENU North
  input.carrier_hz = 10.0e9;
  input.bandwidth_hz = 2.0e6;
  input.tx_power_w = 5.0e7;
  input.pulse_width_s = 1.0e-6;
  input.pri_s = 1.0e-4;

  session::EsrSceneEmitter emitter;
  session::EsrCoordinateStatus status = session::EsrCoordinateStatus::kCoordinateTransformFail;
  ASSERT_TRUE(
      session::TryMakeEsrSceneEmitterFromExternalInput(input, reference, &emitter, &status));
  EXPECT_EQ(status, session::EsrCoordinateStatus::kOk);
  EXPECT_EQ(emitter.emitter_id, 1001U);
  EXPECT_GT(emitter.pose.position_m.x, 100.0f);
  EXPECT_NEAR(emitter.pose.velocity_mps.x, 1.0f, 1.0e-5f);
}

TEST(EsrPublicApiConvenienceTest, SessionStepAndRuntimePatchWorkTogether) {
  session::EsrSession session = session::EsrSessionFactory::Create(MakeSessionConfig());

  session::EsrCycleInput input;
  input.cycle_index = 0U;
  input.dt_sec = 1.0f;
  input.scene.push_back(MakeEmitter(1001U));

  const session::EsrCycleResult baseline = session.StepWithResult(input);
  EXPECT_FALSE(baseline.has_validation_error);

  const config::EsrRuntimeConfigPatch patch =
      config::EsrRuntimeConfigBuilder().WithSensorEnabled(false).Build();
  session.ApplyRuntimeConfig(patch);

  const session::EsrCycleResult updated = session.StepWithResult(input);
  EXPECT_TRUE(updated.output_frame.observation_output.observations.empty());
}

TEST(EsrPublicApiConvenienceTest, TryApplyRuntimeConfigExposesRejectFeedback) {
  session::EsrSession session = session::EsrSessionFactory::Create(MakeSessionConfig());

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

}  // namespace tests
}  // namespace electronic_surveillance_radar
