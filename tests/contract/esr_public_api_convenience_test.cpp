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

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/model/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/foundation/coordinate_transform.h"

namespace electronic_surveillance_radar {
namespace tests {
namespace {

bool ContainsEsrIssueCode(const std::vector<session::EsrValidationIssue>& issues,
                          session::EsrValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

model::EmitterTruthState MakeEmitter(const std::string& id) {
  model::EmitterTruthState emitter;
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

session::EsrSessionConfig MakeSessionConfig() {
  session::EsrSessionConfig config = config::EsrSessionConfigBuilder()
                                         .WithWorkMode(config::EsrWorkMode::kEsm)
                                         .WithDetectionProfile(config::EsrDetectionProfile::kBalanced)
                                         .WithEnvironmentPreset(config::EsrEnvironmentPreset::kStandard)
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
  const session::EsrSessionConfig built = config::EsrSessionConfigBuilder().Build();
  const session::EsrSessionConfig defaults;

  EXPECT_EQ(built.mission.work_mode, defaults.mission.work_mode);
  EXPECT_EQ(built.detection.profile, defaults.detection.profile);
  EXPECT_EQ(built.environment.preset, defaults.environment.preset);
}

TEST(EsrPublicApiConvenienceTest, SessionConfigBuilderOverridesDomainFields) {
  const session::EsrSessionConfig cfg = config::EsrSessionConfigBuilder()
                                            .WithWorkMode(config::EsrWorkMode::kRwr)
                                            .WithPowerOn(false)
                                            .WithScanRateHz(2.0f)
                                            .WithDetectionProfile(config::EsrDetectionProfile::kSensitive)
                                            .WithEnvironmentPreset(config::EsrEnvironmentPreset::kJammed)
                                            .Build();

  EXPECT_EQ(cfg.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FALSE(cfg.mission.power_on);
  EXPECT_FLOAT_EQ(cfg.mission.scan.scan_rate_hz, 2.0f);
  EXPECT_EQ(cfg.detection.profile, config::EsrDetectionProfile::kSensitive);
  EXPECT_EQ(cfg.environment.preset, config::EsrEnvironmentPreset::kJammed);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderDefaultsAreUnset) {
  const session::EsrRuntimeConfigPatch patch = config::EsrRuntimeConfigBuilder().Build();

  EXPECT_FALSE(patch.has_sensor_enabled);
  EXPECT_FALSE(patch.has_work_mode);
  EXPECT_FALSE(patch.has_scan_rate_hz);
  EXPECT_FALSE(patch.has_scan_center_az_deg);
  EXPECT_FALSE(patch.has_use_explicit_scan_bounds);
  EXPECT_FALSE(patch.has_preset);
  EXPECT_FALSE(patch.has_atmospheric_physics);
  EXPECT_FALSE(patch.has_atmospheric_context);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderSetsSemanticFields) {
  environment::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.65f;
  environment::EsrAtmosphericDerivedContext atmospheric_context;
  atmospheric_context.has_day_of_year = true;
  atmospheric_context.day_of_year = 210;

  const session::EsrRuntimeConfigPatch patch = config::EsrRuntimeConfigBuilder()
                                                   .WithSensorEnabled(false)
                                                   .WithWorkMode(config::EsrWorkMode::kHgesm)
                                                   .WithScanRateHz(3.0f)
                                                   .WithScanCenterAzDeg(15.0f)
                                                   .WithScanCenterElDeg(5.0f)
                                                   .WithExplicitScanBoundsDeg(-30.0f, 30.0f, -10.0f, 10.0f)
                                                   .WithEnvironmentPreset(
                                                       config::EsrEnvironmentPreset::kDenseClutter)
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
  EXPECT_TRUE(patch.has_use_explicit_scan_bounds);
  EXPECT_TRUE(patch.use_explicit_scan_bounds);
  EXPECT_TRUE(patch.has_preset);
  EXPECT_EQ(patch.preset, config::EsrEnvironmentPreset::kDenseClutter);
  EXPECT_TRUE(patch.has_atmospheric_physics);
  EXPECT_TRUE(patch.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(patch.atmospheric_physics.relative_humidity, 0.65f);
  EXPECT_TRUE(patch.has_atmospheric_context);
  EXPECT_TRUE(patch.atmospheric_context.has_day_of_year);
  EXPECT_EQ(patch.atmospheric_context.day_of_year, 210);
}

TEST(EsrPublicApiConvenienceTest, InputValidationReportsErrorsForBoundaryCases) {
  session::EsrCycleInput input;
  input.dt_sec = 0.0f;
  input.platform_pose.position_m.x = std::numeric_limits<float>::infinity();

  model::EmitterTruthState invalid_emitter;
  invalid_emitter.emitter_id = "";
  invalid_emitter.carrier_hz = -1.0;
  input.scene_emitters.push_back(invalid_emitter);

  const session::EsrValidationIssueList issues = session::ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsEsrIssueCode(issues, session::EsrValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(ContainsEsrIssueCode(issues, session::EsrValidationCode::kNonFinitePlatformNumericField));
  EXPECT_TRUE(ContainsEsrIssueCode(issues, session::EsrValidationCode::kEmptyEmitterId));
  EXPECT_TRUE(session::HasEsrValidationError(issues));
}

TEST(EsrPublicApiConvenienceTest, CoordinateUtilsBuildsPoseFromExternalKinematics) {
  session::EsrCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::foundation::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  oneq::foundation::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(target_lla, &target_ecef));

  session::EsrExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = target_ecef;

  model::EsrPoseState pose;
  ASSERT_TRUE(session::TryMakeEsrPoseFromExternalKinematics(pose_input, reference, &pose));
  EXPECT_GT(pose.position_m.x, 100.0f);
}

TEST(EsrPublicApiConvenienceTest, SessionStepAndRuntimePatchWorkTogether) {
  session::EsrSession session(MakeSessionConfig());

  session::EsrCycleInput input;
  input.cycle_index = 0U;
  input.dt_sec = 1.0f;
  input.scene_emitters.push_back(MakeEmitter("emitter-1"));

  const session::EsrCycleResult baseline = session.StepWithResult(input);
  EXPECT_FALSE(baseline.has_validation_error);

  const session::EsrRuntimeConfigPatch patch =
      config::EsrRuntimeConfigBuilder().WithSensorEnabled(false).Build();
  session.ApplyRuntimeConfig(patch);

  const session::EsrCycleResult updated = session.StepWithResult(input);
  EXPECT_TRUE(updated.output_frame.observation_output.observations.empty());
}

}  // namespace tests
}  // namespace electronic_surveillance_radar
