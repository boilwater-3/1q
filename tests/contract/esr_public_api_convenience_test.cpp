// Copyright 2026. All Rights Reserved.
//
// @file esr_public_api_convenience_test.cpp
// @brief 验证 ESR 对外易用性增强 API 的默认语义与集成行为。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "1q/foundation/coordinate_transform.h"
#include "1q/electronic_surveillance_radar/model/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/model/EsrCoordinateUtils.h"
#include "1q/electronic_surveillance_radar/output/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

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

model::EmitterTruthState MakeEmitter(const std::string& id, float pos_x, double carrier_hz,
                                      double bandwidth_hz, double tx_power_w, double pulse_width_s,
                                      double pri_s) {
  model::EmitterTruthState emitter;
  emitter.emitter_id = id;
  emitter.pose.position_m.x = pos_x;
  emitter.carrier_hz = carrier_hz;
  emitter.bandwidth_hz = bandwidth_hz;
  emitter.tx_power_w = tx_power_w;
  emitter.pulse_width_s = pulse_width_s;
  emitter.pri_s = pri_s;
  return emitter;
}

}  // namespace

TEST(EsrPublicApiConvenienceTest, SessionConfigBuilderDefaultsMatchEsrSessionConfig) {
  const session::EsrSessionConfig built = config::EsrSessionConfigBuilder().Build();
  const session::EsrSessionConfig default_config;

  EXPECT_EQ(built.enable_layered_config, default_config.enable_layered_config);
  EXPECT_NEAR(built.pipeline_config.detection.min_detect_snr_db,
              default_config.pipeline_config.detection.min_detect_snr_db, 1e-5f);
}

TEST(EsrPublicApiConvenienceTest, SessionConfigBuilderOverridesLayeredAndPipelineFields) {
  const session::EsrSessionConfig config = config::EsrSessionConfigBuilder()
                                                     .EnableLayeredConfig(true)
                                                     .WithWorkMode(session::EsrWorkMode::kRwr)
                                                     .WithScanRateHz(2.0f)
                                                     .EnableSpectralAnalysis(false)
                                                     .WithDetectionMinSnrDb(8.0f)
                                                     .WithJammingDetectionThresholdW(1.0e-8f)
                                                     .Build();

  EXPECT_TRUE(config.enable_layered_config);
  EXPECT_EQ(config.layered_config.mission.work_mode, session::EsrWorkMode::kRwr);
  EXPECT_NEAR(config.layered_config.mission.scan_rate_hz, 2.0f, 1e-5f);
  EXPECT_FALSE(config.pipeline_config.spectral_analysis.enable);
  EXPECT_NEAR(config.pipeline_config.detection.min_detect_snr_db, 8.0f, 1e-5f);
  EXPECT_NEAR(config.environment_default_config.model_config.jamming_detection_threshold_w, 1.0e-8f,
              1e-10f);
}

TEST(EsrPublicApiConvenienceTest, SessionConfigBuilderPreservesPreconfiguredSessionConfig) {
  session::EsrSessionConfig base;
  base.enable_layered_config = true;
  base.layered_config.hardware.receiver_band_lower_hz = 1.0e9;

  const session::EsrSessionConfig config =
      config::EsrSessionConfigBuilder(base).WithScanRateHz(5.0f).Build();

  EXPECT_TRUE(config.enable_layered_config);
  EXPECT_NEAR(config.layered_config.hardware.receiver_band_lower_hz, 1.0e9, 1.0);
  EXPECT_NEAR(config.layered_config.mission.scan_rate_hz, 5.0f, 1e-5f);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderDefaultsAreUnset) {
  const session::EsrRuntimeConfigPatch patch = config::EsrRuntimeConfigBuilder().Build();

  EXPECT_FALSE(patch.has_sensor_enabled);
  EXPECT_FALSE(patch.has_scan_rate_hz);
  EXPECT_FALSE(patch.has_integrated_receive_loss_db);
  EXPECT_FALSE(patch.has_fixed_receiver_window_hz);
  EXPECT_FALSE(patch.has_use_fixed_receiver_window);
  EXPECT_FALSE(patch.has_enable_statistical_detection);
  EXPECT_FALSE(patch.has_enable_spectral_analysis);
  EXPECT_FALSE(patch.has_detection_min_snr_db);
  EXPECT_FALSE(patch.has_environment_runtime_config);
  EXPECT_FALSE(patch.has_observation_jam_mark_threshold_w);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderSetsAllFields) {
  const session::EsrRuntimeConfigPatch patch = config::EsrRuntimeConfigBuilder()
                                                         .WithSensorEnabled(false)
                                                         .WithScanRateHz(3.0f)
                                                         .WithIntegratedReceiveLossDb(2.5f)
                                                         .WithFixedReceiverWindowHz(1.0e9, 2.0e9)
                                                         .SetFixedReceiverWindowEnabled(true)
                                                         .EnableStatisticalDetection(false)
                                                         .EnableSpectralAnalysis(false)
                                                         .WithDetectionMinSnrDb(10.0f)
                                                         .WithObservationJamMarkThresholdW(5.0e-10f)
                                                         .Build();

  EXPECT_TRUE(patch.has_sensor_enabled);
  EXPECT_FALSE(patch.sensor_enabled);
  EXPECT_TRUE(patch.has_scan_rate_hz);
  EXPECT_NEAR(patch.scan_rate_hz, 3.0f, 1e-5f);
  EXPECT_TRUE(patch.has_integrated_receive_loss_db);
  EXPECT_NEAR(patch.integrated_receive_loss_db, 2.5f, 1e-5f);
  EXPECT_TRUE(patch.has_fixed_receiver_window_hz);
  EXPECT_NEAR(patch.receiver_lower_hz, 1.0e9, 1.0);
  EXPECT_NEAR(patch.receiver_upper_hz, 2.0e9, 1.0);
  EXPECT_TRUE(patch.has_use_fixed_receiver_window);
  EXPECT_TRUE(patch.use_fixed_receiver_window);
  EXPECT_TRUE(patch.has_enable_statistical_detection);
  EXPECT_FALSE(patch.enable_statistical_detection);
  EXPECT_TRUE(patch.has_enable_spectral_analysis);
  EXPECT_FALSE(patch.enable_spectral_analysis);
  EXPECT_TRUE(patch.has_detection_min_snr_db);
  EXPECT_NEAR(patch.detection_min_snr_db, 10.0f, 1e-5f);
  EXPECT_TRUE(patch.has_observation_jam_mark_threshold_w);
  EXPECT_NEAR(patch.observation_jam_mark_threshold_w, 5.0e-10f, 1e-15f);
}

TEST(EsrPublicApiConvenienceTest, RuntimeConfigBuilderDisableFixedReceiverWindow) {
  const session::EsrRuntimeConfigPatch patch =
      config::EsrRuntimeConfigBuilder().DisableFixedReceiverWindow().Build();

  EXPECT_TRUE(patch.has_use_fixed_receiver_window);
  EXPECT_FALSE(patch.use_fixed_receiver_window);
}

TEST(EsrPublicApiConvenienceTest, InputValidationReportsErrorsForCommonBoundaryCases) {
  session::EsrCycleInput input;
  input.dt_sec = 0.0f;
  input.platform_pose.position_m.x = std::numeric_limits<float>::infinity();

  model::EmitterTruthState invalid_emitter;
  invalid_emitter.emitter_id = "";
  invalid_emitter.carrier_hz = -1.0;
  invalid_emitter.bandwidth_hz = -1.0;
  invalid_emitter.tx_power_w = -1.0;
  invalid_emitter.pulse_width_s = -1.0;
  invalid_emitter.pri_s = -1.0;
  input.scene_emitters.push_back(invalid_emitter);

  model::EmitterTruthState pri_less_emitter;
  pri_less_emitter.emitter_id = "pri-test";
  pri_less_emitter.carrier_hz = 10.0e9;
  pri_less_emitter.bandwidth_hz = 1.0e6;
  pri_less_emitter.tx_power_w = 1.0e3;
  pri_less_emitter.pulse_width_s = 1.0e-6;
  pri_less_emitter.pri_s = 0.5e-6;
  input.scene_emitters.push_back(pri_less_emitter);

  const session::EsrValidationIssueList issues = session::ValidateEsrCycleInput(input);

  EXPECT_TRUE(
      ContainsEsrIssueCode(issues, session::EsrValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(ContainsEsrIssueCode(
      issues, session::EsrValidationCode::kNonFinitePlatformNumericField));
  EXPECT_TRUE(ContainsEsrIssueCode(issues, session::EsrValidationCode::kEmptyEmitterId));
  EXPECT_TRUE(
      ContainsEsrIssueCode(issues, session::EsrValidationCode::kInvalidEmitterFrequency));
  EXPECT_TRUE(ContainsEsrIssueCode(issues, session::EsrValidationCode::kInvalidEmitterPower));
  EXPECT_TRUE(ContainsEsrIssueCode(
      issues, session::EsrValidationCode::kEmitterPriLessThanPulseWidth));
  EXPECT_TRUE(session::HasEsrValidationError(issues));
}

TEST(EsrPublicApiConvenienceTest, InputValidationFlagsNonFiniteDtAndEmitterFields) {
  session::EsrCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();

  model::EmitterTruthState emitter;
  emitter.emitter_id = "finite-test";
  emitter.carrier_hz = std::numeric_limits<double>::infinity();
  input.scene_emitters.push_back(emitter);

  const session::EsrValidationIssueList issues = session::ValidateEsrCycleInput(input);

  EXPECT_TRUE(
      ContainsEsrIssueCode(issues, session::EsrValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(ContainsEsrIssueCode(
      issues, session::EsrValidationCode::kNonFiniteEmitterNumericField));
  EXPECT_TRUE(session::HasEsrValidationError(issues));
}

TEST(EsrPublicApiConvenienceTest, InputValidationPassesForValidInput) {
  session::EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.scene_emitters.push_back(
      MakeEmitter("valid-1", 1200.0f, 10.0e9, 2.0e6, 5.0e7, 1.0e-6, 1.0e-4));

  const session::EsrValidationIssueList issues = session::ValidateEsrCycleInput(input);

  EXPECT_FALSE(session::HasEsrValidationError(issues));
}

TEST(EsrPublicApiConvenienceTest, CoordinateUtilsConvertsLlaAndEcefToEsrLocal) {
  model::EsrCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;
  reference.frame_attitude_deg.yaw_deg = 0.0f;
  reference.frame_attitude_deg.pitch_deg = 0.0f;
  reference.frame_attitude_deg.roll_deg = 0.0f;

  oneq::foundation::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  model::EsrVector3f local_from_lla;
  ASSERT_TRUE(model::TryConvertLlaToEsrLocal(target_lla, reference, &local_from_lla));
  EXPECT_GT(local_from_lla.x, 100.0f);
  EXPECT_NEAR(local_from_lla.y, 0.0f, 1.0e-2f);
  EXPECT_NEAR(local_from_lla.z, 0.0f, 1.0e-2f);

  oneq::foundation::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(target_lla, &target_ecef));
  model::EsrVector3f local_from_ecef;
  ASSERT_TRUE(model::TryConvertEcefToEsrLocal(target_ecef, reference, &local_from_ecef));
  EXPECT_NEAR(local_from_ecef.x, local_from_lla.x, 1.0e-3f);
  EXPECT_NEAR(local_from_ecef.y, local_from_lla.y, 1.0e-3f);
  EXPECT_NEAR(local_from_ecef.z, local_from_lla.z, 1.0e-3f);
}

TEST(EsrPublicApiConvenienceTest, CoordinateUtilsBuildsPoseFromEcefAndLla) {
  model::EsrCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::foundation::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 100.0;

  model::EsrVector3f velocity;
  velocity.x = 10.0f;
  velocity.y = 0.0f;
  velocity.z = 0.0f;
  model::EsrEulerAngleDeg attitude;
  attitude.yaw_deg = 5.0f;
  attitude.pitch_deg = -1.0f;
  attitude.roll_deg = 0.0f;

  model::EsrPoseState pose_from_lla;
  ASSERT_TRUE(
      model::TryMakeEsrPoseFromLla(target_lla, reference, velocity, attitude, &pose_from_lla));
  EXPECT_GT(pose_from_lla.position_m.x, 100.0f);
  EXPECT_NEAR(pose_from_lla.velocity_mps.x, 10.0f, 1e-5f);
  EXPECT_NEAR(pose_from_lla.attitude_deg.yaw_deg, 5.0f, 1e-5f);

  oneq::foundation::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(target_lla, &target_ecef));
  model::EsrPoseState pose_from_ecef;
  ASSERT_TRUE(
      model::TryMakeEsrPoseFromEcef(target_ecef, reference, velocity, attitude, &pose_from_ecef));
  EXPECT_NEAR(pose_from_ecef.position_m.x, pose_from_lla.position_m.x, 1.0e-3f);
}

TEST(EsrPublicApiConvenienceTest, EsrSessionStepProducesThreeChannelOutput) {
  session::EsrSessionConfig config;
  config.pipeline_config.scan.scan_start_az_deg = -60.0f;
  config.pipeline_config.scan.scan_end_az_deg = 60.0f;
  config.pipeline_config.scan.scan_start_el_deg = -20.0f;
  config.pipeline_config.scan.scan_end_el_deg = 20.0f;
  config.pipeline_config.scan.az_step_deg = 120.0f;
  config.pipeline_config.scan.el_step_deg = 40.0f;

  session::EsrSession session(config);

  session::EsrCycleInput input;
  input.cycle_index = 0U;
  input.dt_sec = 1.0f;
  input.scene_emitters.push_back(
      MakeEmitter("emitter-1", 1200.0f, 10.0e9, 2.0e6, 5.0e7, 1.0e-6, 1.0e-4));

  const output::EsrOutputFrame frame = session.Step(input);

  EXPECT_EQ(frame.observation_output.cycle_index, 0U);
  EXPECT_EQ(frame.emitter_output.cycle_index, 0U);
  EXPECT_EQ(frame.truth_evaluation_output.cycle_index, 0U);
}

TEST(EsrPublicApiConvenienceTest, EsrSessionStepWithResultAggregatesOutputAndValidation) {
  session::EsrSessionConfig config;
  config.pipeline_config.scan.scan_start_az_deg = -60.0f;
  config.pipeline_config.scan.scan_end_az_deg = 60.0f;
  config.pipeline_config.scan.az_step_deg = 120.0f;
  config.pipeline_config.scan.el_step_deg = 40.0f;

  session::EsrSession session(config);

  session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.scene_emitters.push_back(
      MakeEmitter("emitter-2", 5000.0f, 8.0e9, 5.0e6, 1.0e8, 2.0e-6, 5.0e-4));

  const session::EsrCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.validation_issues.empty());
  EXPECT_EQ(result.output_frame.observation_output.cycle_index, 1U);
}

TEST(EsrPublicApiConvenienceTest, EsrSessionStepWithResultSurfacesValidationErrors) {
  session::EsrSessionConfig session_config;
  session::EsrSession session(session_config);

  session::EsrCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  model::EmitterTruthState invalid_emitter;
  invalid_emitter.emitter_id = "";
  input.scene_emitters.push_back(invalid_emitter);

  const session::EsrCycleResult result = session.StepWithResult(input);

  EXPECT_TRUE(result.has_validation_error);
  EXPECT_TRUE(ContainsEsrIssueCode(result.validation_issues,
                                   session::EsrValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(ContainsEsrIssueCode(result.validation_issues,
                                   session::EsrValidationCode::kEmptyEmitterId));
}

TEST(EsrPublicApiConvenienceTest, EsrSessionAppliesRuntimeConfigPatch) {
  session::EsrSessionConfig session_config;
  session::EsrSession session(session_config);

  const session::EsrRuntimeConfigPatch patch =
      config::EsrRuntimeConfigBuilder().WithScanRateHz(10.0f).WithDetectionMinSnrDb(12.0f).Build();
  session.ApplyRuntimeConfig(patch);

  session::EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.scene_emitters.push_back(
      MakeEmitter("rt-emitter", 3000.0f, 9.0e9, 1.0e6, 1.0e7, 1.0e-6, 1.0e-4));

  const session::EsrCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.output_frame.observation_output.cycle_index, 0U);
}

TEST(EsrPublicApiConvenienceTest, EsrSessionMultiCycleProducesProgressiveCycleIndices) {
  session::EsrSessionConfig config;
  config.pipeline_config.scan.scan_start_az_deg = -60.0f;
  config.pipeline_config.scan.scan_end_az_deg = 60.0f;
  config.pipeline_config.scan.az_step_deg = 120.0f;
  config.pipeline_config.scan.el_step_deg = 40.0f;

  session::EsrSession session(config);

  for (std::uint32_t i = 0U; i < 3U; ++i) {
    session::EsrCycleInput input;
    input.cycle_index = i;
    input.dt_sec = 1.0f;
    input.scene_emitters.push_back(
        MakeEmitter("multi-emitter", 2000.0f, 10.0e9, 2.0e6, 5.0e7, 1.0e-6, 1.0e-4));

    const output::EsrOutputFrame frame = session.Step(input);
    EXPECT_EQ(frame.observation_output.cycle_index, i);
  }
}

TEST(EsrPublicApiConvenienceTest, EsrEnvironmentSceneStateDefaultsAreReasonable) {
  const environment::EsrEnvironmentSceneState default_scene;
  EXPECT_NEAR(default_scene.base_propagation_loss_db, 3.0f, 1e-5f);
  EXPECT_NEAR(default_scene.atmospheric_attenuation_db, 1.0f, 1e-5f);
  EXPECT_NEAR(default_scene.clutter_noise_w, 1.0e-12f, 1e-20f);
  EXPECT_TRUE(default_scene.jammer_sources.empty());
  EXPECT_FALSE(default_scene.atmospheric_physics.enable_physical_model);
}

TEST(EsrPublicApiConvenienceTest, EsrJammerSourcePopulation) {
  environment::EsrJammerSource noise;
  noise.technique = environment::EsrJammingTechnique::kNoiseSuppression;
  noise.active = true;
  noise.center_hz = 10.0e9;
  noise.bandwidth_hz = 100.0e6;
  noise.power_w = 1.0e-6f;
  noise.deception_risk = 0.0f;
  noise.confidence = 0.95f;

  environment::EsrJammerSource deception;
  deception.technique = environment::EsrJammingTechnique::kDeception;
  deception.active = true;
  deception.deception_risk = 0.8f;

  environment::EsrEnvironmentSceneState scene;
  scene.jammer_sources.push_back(noise);
  scene.jammer_sources.push_back(deception);

  ASSERT_EQ(scene.jammer_sources.size(), 2U);
  EXPECT_EQ(scene.jammer_sources[0].technique, environment::EsrJammingTechnique::kNoiseSuppression);
  EXPECT_NEAR(scene.jammer_sources[0].confidence, 0.95f, 1e-5f);
  EXPECT_EQ(scene.jammer_sources[1].technique, environment::EsrJammingTechnique::kDeception);
  EXPECT_NEAR(scene.jammer_sources[1].deception_risk, 0.8f, 1e-5f);
}

}  // namespace tests
}  // namespace electronic_surveillance_radar
