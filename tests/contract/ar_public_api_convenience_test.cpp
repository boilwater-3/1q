// Copyright 2026. All Rights Reserved.
//
// @file public_api_convenience_test.cpp
// @brief 验证对外易用性增强 API 的默认语义与集成行为。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/output/TrackOutputQueries.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "airborne_radar/session/MutableRadarContext.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/core/SignalPipeline.h"

namespace airborne_radar {
namespace tests {

namespace {

config::SignalPipelineConfig MakeConveniencePipelineConfig() {
  config::SignalPipelineConfig config;
  config.detection.min_detection_margin_db = -100.0f;
  config.lifecycle.enable_auto_lifecycle_manager = true;
  config.lifecycle.lifecycle_config.confirm_hits = 1U;
  config.tracking.kalman_measurement_noise_std = 1.0f;
  return config;
}

session::RadarSessionConfig MakeConvenienceSessionConfig() {
  session::RadarSessionConfig config;
  const auto pipeline_config = MakeConveniencePipelineConfig();
  config.detection = pipeline_config.detection;
  config.beam_control = pipeline_config.beam_control;
  config.tracking = pipeline_config.tracking;
  config.lifecycle = pipeline_config.lifecycle;
  return config;
}

session::RadarCycleInput MakeCycleInput(model::TargetFeatureList targets,
                                              float dt_sec = 1.0f) {
  session::RadarCycleInput input;
  input.target_features = std::move(targets);
  input.dt_sec = dt_sec;
  input.platform_attitude_deg.yaw_deg = 5.0f;
  input.platform_attitude_deg.pitch_deg = -1.0f;
  input.platform_attitude_deg.roll_deg = 2.0f;
  return input;
}

/// @brief 比较两组控制命令的类型和来源是否一致。
/// @param expected 期望命令列表。
/// @param actual 实际命令列表。
void ExpectEquivalentCommands(const std::vector<extension::control::RadarCommand>& expected,
                              const std::vector<extension::control::RadarCommand>& actual) {
  ASSERT_EQ(expected.size(), actual.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(expected[i].type, actual[i].type);
    EXPECT_EQ(expected[i].source, actual[i].source);
  }
}

/// @brief 比较两份控制真值的公开语义字段。
/// @param expected 期望控制真值。
/// @param actual 实际控制真值。
void ExpectEquivalentProfiles(const extension::control::RadarControlProfile& expected,
                              const extension::control::RadarControlProfile& actual) {
  EXPECT_EQ(expected.version, actual.version);
  EXPECT_EQ(expected.enable_lpi_power_control, actual.enable_lpi_power_control);
  EXPECT_NEAR(expected.lpi_power_scale, actual.lpi_power_scale, 1e-5f);
  EXPECT_EQ(expected.enable_lpi_beamforming, actual.enable_lpi_beamforming);
  EXPECT_NEAR(expected.lpi_dwell_scale, actual.lpi_dwell_scale, 1e-5f);
  EXPECT_EQ(expected.enable_agility_frequency, actual.enable_agility_frequency);
  EXPECT_EQ(expected.agility_frequency_hop_phase, actual.agility_frequency_hop_phase);
  EXPECT_EQ(expected.enable_sidelobe_canceller, actual.enable_sidelobe_canceller);
  EXPECT_EQ(expected.enable_adaptive_beamforming, actual.enable_adaptive_beamforming);
  EXPECT_EQ(expected.enable_eccm_rejitter, actual.enable_eccm_rejitter);
  EXPECT_NEAR(expected.eccm_burnthrough_gain, actual.eccm_burnthrough_gain, 1e-5f);
}

/// @brief 判断问题列表是否包含指定编码。
/// @param issues 校验问题列表。
/// @param code 目标编码。
/// @return 若存在则返回 true。
bool ContainsIssueCode(const std::vector<session::ValidationIssue>& issues,
                       session::ValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(PublicApiConvenienceTest, MutableRadarContextBeginsCycleAndResetsPerCycleCommands) {
  session::MutableRadarContext context;
  session::RadarCycleInput input = MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(101U, 120.0f, -5.0f, 18.0f, 62.0f, 0.0f, 0.0f, 1.0f),
      },
      0.5f);

  context.BeginCycle(input);
  ASSERT_EQ(context.GetTargetFeatures().size(), 1U);
  EXPECT_NEAR(context.GetPlatformAttitude().yaw_deg, 5.0f, 1e-5f);
  EXPECT_NEAR(context.GetCycleDeltaTimeSec(), 0.5f, 1e-5f);
  EXPECT_TRUE(context.GetSubmittedCommands().empty());

  context.SubmitControlCommand(extension::control::RadarCommand(extension::control::RadarCommandType::SET_AGILITY_FREQ,
                                                    extension::control::RadarCommandSource::ECCM));
  extension::control::RadarControlProfile profile;
  profile.enable_agility_frequency = true;
  profile.version = 4U;
  context.UpdateRadarControlProfile(profile);

  ASSERT_EQ(context.GetSubmittedCommands().size(), 1U);
  ASSERT_TRUE(context.HasLatestControlProfile());
  EXPECT_TRUE(context.GetLatestControlProfile().enable_agility_frequency);
  EXPECT_EQ(context.GetLatestControlProfile().version, 4U);

  session::RadarCycleInput second_input = MakeCycleInput({}, 1.25f);
  context.BeginCycle(second_input);
  EXPECT_TRUE(context.GetSubmittedCommands().empty());
  EXPECT_NEAR(context.GetCycleDeltaTimeSec(), 1.25f, 1e-5f);
  EXPECT_TRUE(context.HasLatestControlProfile());
  EXPECT_EQ(context.GetLatestControlProfile().version, 4U);
}

TEST(PublicApiConvenienceTest, TargetFeatureUtilsBuildCartesianGroundAndAirTargets) {
  const model::TargetFeature cartesian_target =
      model::MakeTargetFromCartesian(201U, 3.0f, 4.0f, 12.0f, 10.0f, -2.0f, 1.0f, 0.9f, 2);
  EXPECT_EQ(cartesian_target.external_target_id, 201U);
  EXPECT_TRUE(cartesian_target.has_cartesian_position);
  EXPECT_NEAR(cartesian_target.range_m, 13.0f, 1e-5f);
  EXPECT_NEAR(cartesian_target.current_track_speed, std::sqrt(105.0f), 1e-5f);
  EXPECT_EQ(cartesian_target.target_swerling_type, 2);

  const model::TargetFeature ground_target = model::MakeGroundTarget(202U, 20.0f, -15.0f, 0.8f);
  EXPECT_TRUE(ground_target.has_cartesian_position);
  EXPECT_NEAR(ground_target.position_z, 0.0f, 1e-5f);
  EXPECT_NEAR(ground_target.current_track_velocity_z, 0.0f, 1e-5f);
  EXPECT_NEAR(ground_target.range_m, 25.0f, 1e-5f);

  const model::TargetFeature air_target =
      model::MakeAirTarget(203U, 30.0f, 40.0f, 50.0f, 90.0f, -3.0f, 4.0f, 1.2f);
  EXPECT_TRUE(air_target.has_cartesian_position);
  EXPECT_NEAR(air_target.range_m, std::sqrt(5000.0f), 1e-5f);
  EXPECT_NEAR(air_target.current_track_speed, std::sqrt(8125.0f), 1e-5f);
}

TEST(PublicApiConvenienceTest, TargetFeatureUtilsNormalizeGeometryOnlyWhenCartesianPositionExists) {
  model::TargetFeature normalized =
      model::MakeAirTarget(301U, 6.0f, 8.0f, 0.0f, 50.0f, 0.0f, 0.0f, 1.0f);
  normalized.range_m = 0.0f;
  model::NormalizeTargetGeometry(&normalized);
  EXPECT_NEAR(normalized.range_m, 10.0f, 1e-5f);

  model::TargetFeature unresolved;
  unresolved.external_target_id = 302U;
  unresolved.range_m = 0.0f;
  model::NormalizeTargetGeometry(&unresolved);
  EXPECT_NEAR(unresolved.range_m, 0.0f, 1e-5f);

  model::TargetFeatureList targets;
  targets.push_back(normalized);
  targets.back().range_m = -1.0f;
  targets.push_back(unresolved);
  model::NormalizeTargetGeometry(&targets);
  EXPECT_NEAR(targets[0].range_m, 10.0f, 1e-5f);
  EXPECT_NEAR(targets[1].range_m, 0.0f, 1e-5f);
}

TEST(PublicApiConvenienceTest, TargetFeatureUtilsConvertsLlaAndEcefToRadarLocalFrame) {
  model::RadarLocalFrameReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;
  reference.radar_attitude_deg.yaw_deg = 0.0f;
  reference.radar_attitude_deg.pitch_deg = 0.0f;
  reference.radar_attitude_deg.roll_deg = 0.0f;

  oneq::common::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  oneq::common::Vector3f local_from_lla;
  ASSERT_TRUE(model::TryConvertLlaToRadarLocal(target_lla, reference, &local_from_lla));
  EXPECT_GT(local_from_lla.x, 100.0f);
  EXPECT_NEAR(local_from_lla.y, 0.0f, 1.0e-2f);
  EXPECT_NEAR(local_from_lla.z, 0.0f, 1.0e-2f);

  oneq::common::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::common::TryLlaToEcef(target_lla, &target_ecef));
  oneq::common::Vector3f local_from_ecef;
  ASSERT_TRUE(model::TryConvertEcefToRadarLocal(target_ecef, reference, &local_from_ecef));
  EXPECT_NEAR(local_from_ecef.x, local_from_lla.x, 1.0e-3f);
  EXPECT_NEAR(local_from_ecef.y, local_from_lla.y, 1.0e-3f);
  EXPECT_NEAR(local_from_ecef.z, local_from_lla.z, 1.0e-3f);
}

TEST(PublicApiConvenienceTest, TargetFeatureUtilsBuildsTargetFromExternalCoordinates) {
  model::RadarLocalFrameReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::common::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  model::TargetFeature target_from_lla;
  ASSERT_TRUE(model::TryMakeTargetFromLla(401U, target_lla, reference, 11.0f, 12.0f, 13.0f,
                                                   2.0f, 1, &target_from_lla));
  EXPECT_EQ(target_from_lla.external_target_id, 401U);
  EXPECT_TRUE(target_from_lla.has_cartesian_position);
  EXPECT_GT(target_from_lla.position_x, 100.0f);
  EXPECT_GT(target_from_lla.range_m, 100.0f);
  EXPECT_NEAR(target_from_lla.current_track_speed, std::sqrt(434.0f), 1.0e-5f);

  oneq::common::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::common::TryLlaToEcef(target_lla, &target_ecef));
  model::TargetFeature target_from_ecef;
  ASSERT_TRUE(model::TryMakeTargetFromEcef(402U, target_ecef, reference, 0.0f, 0.0f, 0.0f,
                                                    1.5f, 0, &target_from_ecef));
  EXPECT_EQ(target_from_ecef.external_target_id, 402U);
  EXPECT_NEAR(target_from_ecef.position_x, target_from_lla.position_x, 1.0e-3f);
  EXPECT_NEAR(target_from_ecef.position_y, target_from_lla.position_y, 1.0e-3f);
  EXPECT_NEAR(target_from_ecef.position_z, target_from_lla.position_z, 1.0e-3f);
}

TEST(PublicApiConvenienceTest, EnvironmentSceneBuilderDefaultsMatchEnvironmentSceneState) {
  const environment::EnvironmentSceneState built_scene =
      environment::EnvironmentSceneBuilder().Build();
  const environment::EnvironmentSceneState default_scene;

  EXPECT_NEAR(built_scene.base_propagation_loss_db, default_scene.base_propagation_loss_db, 1e-5f);
  EXPECT_NEAR(built_scene.atmospheric_attenuation_db, default_scene.atmospheric_attenuation_db,
              1e-5f);
  EXPECT_NEAR(built_scene.terrain_reflection_db, default_scene.terrain_reflection_db, 1e-5f);
  EXPECT_NEAR(built_scene.clutter_power_db, default_scene.clutter_power_db, 1e-5f);
  EXPECT_EQ(built_scene.atmospheric_physics.enable_physical_model,
            default_scene.atmospheric_physics.enable_physical_model);
  EXPECT_TRUE(built_scene.jammer_emitters.empty());
}

TEST(PublicApiConvenienceTest, EnvironmentSceneBuilderJammerHelpersPopulateTypedEmitters) {
  environment::JammerEmitterState noise_emitter_1;
  noise_emitter_1.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_emitter_1.power_db = 11.0f;
  noise_emitter_1.js_db = 8.0f;
  noise_emitter_1.frequency_overlap_ratio = 0.2f;
  noise_emitter_1.prf_lock_risk = 0.1f;
  noise_emitter_1.in_sidelobe = true;
  noise_emitter_1.azimuth_deg = 12.0f;
  noise_emitter_1.elevation_deg = 1.0f;
  noise_emitter_1.angular_span_deg = 5.0f;
  noise_emitter_1.confidence = 0.9f;

  environment::JammerEmitterState deception_emitter;
  deception_emitter.technique = environment::JammingTechnique::kDeception;
  deception_emitter.power_db = 9.0f;
  deception_emitter.js_db = 7.5f;
  deception_emitter.frequency_overlap_ratio = 0.9f;
  deception_emitter.prf_lock_risk = 0.8f;
  deception_emitter.in_sidelobe = false;
  deception_emitter.azimuth_deg = -8.0f;
  deception_emitter.elevation_deg = 2.5f;
  deception_emitter.angular_span_deg = 3.0f;
  deception_emitter.confidence = 0.8f;

  environment::JammerEmitterState repeater_emitter;
  repeater_emitter.technique = environment::JammingTechnique::kRepeater;
  repeater_emitter.power_db = 8.5f;
  repeater_emitter.js_db = 6.5f;
  repeater_emitter.frequency_overlap_ratio = 0.1f;
  repeater_emitter.prf_lock_risk = 0.95f;
  repeater_emitter.in_sidelobe = false;
  repeater_emitter.azimuth_deg = 4.0f;
  repeater_emitter.elevation_deg = -1.0f;
  repeater_emitter.angular_span_deg = 2.0f;
  repeater_emitter.confidence = 0.7f;

  const environment::EnvironmentSceneState scene = environment::EnvironmentSceneBuilder()
                                                       .SetBasePropagationLossDb(5.5f)
                                                       .SetAtmosphericAttenuationDb(2.0f)
                                                       .SetTerrainReflectionDb(1.2f)
                                                       .SetClutterPowerDb(4.5f)
                                                       .AddNoiseJammer(noise_emitter_1)
                                                       .AddDeceptionJammer(deception_emitter)
                                                       .AddRepeaterJammer(repeater_emitter)
                                                       .Build();

  ASSERT_EQ(scene.jammer_emitters.size(), 3U);
  EXPECT_EQ(scene.jammer_emitters[0].technique, environment::JammingTechnique::kNoiseSuppression);
  EXPECT_TRUE(scene.jammer_emitters[0].in_sidelobe);
  EXPECT_EQ(scene.jammer_emitters[1].technique, environment::JammingTechnique::kDeception);
  EXPECT_NEAR(scene.jammer_emitters[1].frequency_overlap_ratio, 0.9f, 1e-5f);
  EXPECT_EQ(scene.jammer_emitters[2].technique, environment::JammingTechnique::kRepeater);
  EXPECT_NEAR(scene.jammer_emitters[2].prf_lock_risk, 0.95f, 1e-5f);
  EXPECT_NEAR(scene.base_propagation_loss_db, 5.5f, 1e-5f);
  EXPECT_NEAR(scene.clutter_power_db, 4.5f, 1e-5f);
}

TEST(PublicApiConvenienceTest, TrackOutputQueriesSupportUniqueDuplicateAndJammingSearch) {
  output::TrackOutputFrame frame;
  frame.tracks.push_back(
      model::DecisionTrackSnapshot(10.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, false, 401U, 1U));
  frame.tracks.push_back(
      model::DecisionTrackSnapshot(20.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, true, 402U, 2U));
  frame.tracks.push_back(
      model::DecisionTrackSnapshot(21.0f, 0.0f, 0.0f, 1.1f, 0.0f, 0.0f, 0.0f, false, 402U, 3U));
  frame.tracks.push_back(
      model::DecisionTrackSnapshot(8.0f, 0.0f, 0.0f, 0.7f, 0.0f, 0.0f, 0.0f, true, 0U, 4U));

  const auto track_map = output::BuildTrackMapByExternalTargetId(frame);
  EXPECT_EQ(track_map.size(), 2U);
  ASSERT_NE(track_map.count(401U), 0U);
  ASSERT_NE(track_map.count(402U), 0U);
  EXPECT_EQ(track_map.at(402U).state.association_key, 3U);

  const model::DecisionTrackSnapshotList duplicate_tracks =
      output::CollectTracksByExternalTargetId(frame, 402U);
  EXPECT_EQ(duplicate_tracks.size(), 2U);
  EXPECT_TRUE(output::ContainsExternalTargetId(frame, 401U));
  EXPECT_TRUE(output::ContainsExternalTargetId(frame, 0U));
  EXPECT_FALSE(output::ContainsExternalTargetId(frame, 999U));
  EXPECT_EQ(output::CountJammingTracks(frame), 2U);
  EXPECT_EQ(output::CountTracksByStatus(frame, model::DecisionTrackStatus::kTentative), 4U);
}

TEST(PublicApiConvenienceTest, TrackOutputQueriesSupportAssociationKeyStatusAndJammingCollections) {
  output::TrackOutputFrame frame;
  model::DecisionTrackSnapshot confirmed(20.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, false, 410U,
                                          11U);
  confirmed.state.status = model::DecisionTrackStatus::kConfirmed;
  model::DecisionTrackSnapshot lost(5.0f, 0.0f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f, false, 411U, 12U);
  lost.state.status = model::DecisionTrackStatus::kLost;
  model::DecisionTrackSnapshot jammed(18.0f, 0.0f, 0.0f, 1.2f, 0.0f, 0.0f, 0.0f, true, 412U, 13U);
  jammed.state.status = model::DecisionTrackStatus::kConfirmed;
  frame.tracks.push_back(confirmed);
  frame.tracks.push_back(lost);
  frame.tracks.push_back(jammed);

  const auto track_map = output::BuildTrackMapByAssociationKey(frame);
  ASSERT_EQ(track_map.size(), 3U);
  EXPECT_EQ(track_map.at(13U).state.external_target_id, 412U);
  EXPECT_EQ(output::CollectConfirmedTracks(frame).size(), 2U);
  EXPECT_EQ(output::CollectLostTracks(frame).size(), 1U);
  EXPECT_EQ(output::CollectJammingTracks(frame).size(), 1U);
  EXPECT_EQ(output::CountTracksByStatus(frame, model::DecisionTrackStatus::kConfirmed), 2U);
}

TEST(PublicApiConvenienceTest, RadarInputValidationReportsWarningsAndErrorsForCommonBoundaryCases) {
  session::RadarCycleInput input = MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(0U, 120.0f, 0.0f, 10.0f, 55.0f, 0.0f, 0.0f, 1.0f),
          model::MakeAirTarget(800U, 150.0f, -2.0f, 12.0f, 62.0f, 0.2f, 0.0f, 1.0f),
          model::MakeAirTarget(800U, 155.0f, 2.0f, 12.0f, 63.0f, -0.2f, 0.0f, -1.1f),
      },
      0.0f);
  input.target_features[2].position_x = std::numeric_limits<float>::infinity();
  input.target_features[2].range_m = 0.0f;

  const std::vector<session::ValidationIssue> issues =
      session::ValidateRadarCycleInput(input);
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kUnknownExternalTargetId));
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kDuplicateExternalTargetId));
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kNegativeRcs));
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kNonFiniteTargetField));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(PublicApiConvenienceTest, RadarInputValidationFlagsMissingGeometryAndNonFiniteCycleDelta) {
  session::RadarCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  model::TargetFeature invalid_target;
  invalid_target.external_target_id = 900U;
  invalid_target.range_m = 0.0f;
  input.target_features.push_back(invalid_target);

  const std::vector<session::ValidationIssue> issues =
      session::ValidateRadarCycleInput(input);
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(
      ContainsIssueCode(issues, session::ValidationCode::kMissingRangeAndCartesianPosition));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(PublicApiConvenienceTest, ConfigPresetsProvideExpectedDetectionAndRobustnessDefaults) {
  const config::SignalPipelineConfig detection_config =
      config::MakeDetectionMissionSignalPipelineConfig();
  EXPECT_TRUE(detection_config.lifecycle.enable_auto_lifecycle_manager);
  EXPECT_EQ(detection_config.lifecycle.lifecycle_config.confirm_hits, 1U);
  EXPECT_NEAR(detection_config.detection.min_detection_margin_db, -100.0f, 1e-5f);

  const config::SignalPipelineConfig robust_config =
      config::MakeHighRobustnessSignalPipelineConfig();
  EXPECT_TRUE(robust_config.lifecycle.enable_auto_lifecycle_manager);
  EXPECT_GT(robust_config.lifecycle.lifecycle_config.max_miss_before_lost,
            detection_config.lifecycle.lifecycle_config.max_miss_before_lost);
  EXPECT_GT(robust_config.lifecycle.lifecycle_config.max_lost_cycles,
            detection_config.lifecycle.lifecycle_config.max_lost_cycles);
  EXPECT_GT(robust_config.tracking.kalman_measurement_noise_std,
            detection_config.tracking.kalman_measurement_noise_std);

  const session::RadarSessionConfig session_config =
      config::MakeDetectionMissionRadarSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(session_config);
  const output::TrackOutputFrame frame = session.Step(MakeCycleInput(model::TargetFeatureList{
      model::MakeGroundTarget(901U, 20.0f, 5.0f, 0.8f),
  }));
  EXPECT_EQ(frame.confirmed_track_count, 1U);
}

TEST(PublicApiConvenienceTest, DefaultSessionConfigUsesLifecycleManagedTracks) {
  const session::RadarSessionConfig default_config = config::MakeDefaultRadarSessionConfig();
  EXPECT_TRUE(default_config.lifecycle.enable_auto_lifecycle_manager);

  session::RadarSession session = session::RadarSessionFactory::Create();
  const output::TrackOutputFrame frame = session.Step(MakeCycleInput(model::TargetFeatureList{
      model::MakeGroundTarget(902U, 15.0f, -3.0f, 0.8f),
  }));

  ASSERT_EQ(frame.tracks.size(), 1U);
  EXPECT_EQ(frame.published_track_count, 1U);
  EXPECT_EQ(frame.confirmed_track_count, 0U);
  EXPECT_EQ(frame.tracks[0].state.status, model::DecisionTrackStatus::kTentative);
}

TEST(PublicApiConvenienceTest, RadarSessionStepProducesReadableOutputWithoutInterference) {
  session::RadarSession session = session::RadarSessionFactory::Create(MakeConvenienceSessionConfig());
  const session::RadarCycleInput input = MakeCycleInput(model::TargetFeatureList{
      model::MakeGroundTarget(501U, 25.0f, -5.0f, 0.7f),
      model::MakeAirTarget(502U, 150.0f, 6.0f, 18.0f, 72.0f, 1.0f, 0.0f, 1.0f),
  });

  const output::TrackOutputFrame frame = session.Step(input);
  EXPECT_EQ(frame.published_track_count, 2U);
  EXPECT_EQ(frame.confirmed_track_count, 2U);
  EXPECT_TRUE(output::ContainsExternalTargetId(frame, 501U));
  EXPECT_TRUE(output::ContainsExternalTargetId(frame, 502U));
  EXPECT_EQ(output::CountJammingTracks(frame), 0U);
  EXPECT_TRUE(session.GetSubmittedCommands().empty());
  EXPECT_TRUE(session.HasLatestControlProfile());
}

TEST(PublicApiConvenienceTest, RadarSessionSceneAwareStepMatchesManualControllerChain) {
  const session::RadarSessionConfig config = MakeConvenienceSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(config);

  session::MutableRadarContext manual_context;
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection = config.detection;
  pipeline_config.beam_control = config.beam_control;
  pipeline_config.tracking = config.tracking;
  pipeline_config.lifecycle = config.lifecycle;
  signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);
  environment::EnvironmentService environment_service(config.environment_default_config.model_config);
  environment_service.SetJammingDetectionThresholdDb(config.environment_default_config.jamming_detection_threshold_db);
  extension::RadarController controller(manual_context, signal_pipeline,
                                               environment_service);

  const session::RadarCycleInput input = MakeCycleInput(model::TargetFeatureList{
      model::MakeAirTarget(601U, 180.0f, -4.0f, 16.0f, 60.0f, 0.0f, 0.0f, 1.0f),
      model::MakeAirTarget(602U, 240.0f, 8.0f, 18.0f, 76.0f, 0.5f, 0.0f, 1.1f),
  });
  environment::JammerEmitterState noise_emitter_2;
  noise_emitter_2.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_emitter_2.power_db = 12.0f;
  noise_emitter_2.js_db = 8.0f;
  noise_emitter_2.frequency_overlap_ratio = 0.2f;
  noise_emitter_2.prf_lock_risk = 0.1f;
  noise_emitter_2.in_sidelobe = true;

  const environment::EnvironmentSceneState scene =
      environment::EnvironmentSceneBuilder().AddNoiseJammer(noise_emitter_2).Build();

  const output::TrackOutputFrame session_frame = session.Step(input, scene);

  environment_service.UpdateSceneState(scene);
  manual_context.BeginCycle(input);
  controller.RunOnce();
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame manual_frame = controller.GetLatestTrackOutputFrame();

  EXPECT_EQ(session_frame.published_track_count, manual_frame.published_track_count);
  EXPECT_EQ(session_frame.confirmed_track_count, manual_frame.confirmed_track_count);
  EXPECT_EQ(output::CountJammingTracks(session_frame),
            output::CountJammingTracks(manual_frame));

  const auto session_track_map = output::BuildTrackMapByExternalTargetId(session_frame);
  const auto manual_track_map = output::BuildTrackMapByExternalTargetId(manual_frame);
  ASSERT_EQ(session_track_map.size(), manual_track_map.size());
  for (std::size_t i = 0; i < input.target_features.size(); ++i) {
    const std::uint64_t target_id = input.target_features[i].external_target_id;
    ASSERT_NE(session_track_map.count(target_id), 0U);
    ASSERT_NE(manual_track_map.count(target_id), 0U);
    EXPECT_EQ(session_track_map.at(target_id).state.association_key,
              manual_track_map.at(target_id).state.association_key);
  }

  ExpectEquivalentCommands(manual_context.GetSubmittedCommands(), session.GetSubmittedCommands());
  ASSERT_TRUE(session.HasLatestControlProfile());
  ASSERT_TRUE(manual_context.HasLatestControlProfile());
  ExpectEquivalentProfiles(manual_context.GetLatestControlProfile(),
                           session.GetLatestControlProfile());

  const extension::AssociationQualityMetrics session_metrics =
      session.GetLastAssociationQualityMetrics();
  const extension::AssociationQualityMetrics manual_metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();
  EXPECT_EQ(session_metrics.detection_count, manual_metrics.detection_count);
  EXPECT_NEAR(session_metrics.match_rate, manual_metrics.match_rate, 1e-5f);
  EXPECT_NEAR(session_metrics.association_stress, manual_metrics.association_stress, 1e-5f);
}

TEST(PublicApiConvenienceTest,
     RadarSessionHandlesInvalidDeltaUnknownAndDuplicateIdsAcrossSceneSwitches) {
  session::RadarSession session = session::RadarSessionFactory::Create(MakeConvenienceSessionConfig());

  session::RadarCycleInput cycle_1 = MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(0U, 120.0f, 0.0f, 10.0f, 55.0f, 0.0f, 0.0f, 1.0f),
          model::MakeAirTarget(701U, 150.0f, -2.0f, 12.0f, 62.0f, 0.2f, 0.0f, 1.0f),
          model::MakeAirTarget(701U, 155.0f, 2.0f, 12.0f, 63.0f, -0.2f, 0.0f, 1.1f),
          model::MakeGroundTarget(702U, 40.0f, -6.0f, 0.8f),
      },
      1.0f);
  const session::RadarCycleResult result_1 = session.StepWithResult(cycle_1);
  const output::TrackOutputFrame& frame_1 = result_1.track_output_frame;
  EXPECT_GT(frame_1.published_track_count, 0U);
  EXPECT_TRUE(output::ContainsExternalTargetId(frame_1, 0U));
  EXPECT_EQ(output::CollectTracksByExternalTargetId(frame_1, 701U).size(), 2U);
  EXPECT_FALSE(result_1.has_validation_error);

  session::RadarCycleInput cycle_2 = cycle_1;
  cycle_2.dt_sec = -1.0f;
  for (std::size_t i = 0; i < cycle_2.target_features.size(); ++i) {
    cycle_2.target_features[i].position_x += 5.0f;
  }
  environment::JammerEmitterState noise_emitter_3;
  noise_emitter_3.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_emitter_3.power_db = 12.0f;
  noise_emitter_3.js_db = 8.0f;
  noise_emitter_3.frequency_overlap_ratio = 0.2f;
  noise_emitter_3.prf_lock_risk = 0.1f;
  noise_emitter_3.in_sidelobe = true;

  const session::RadarCycleResult result_2 = session.StepWithResult(
      cycle_2, environment::EnvironmentSceneBuilder().AddNoiseJammer(noise_emitter_3).Build());
  const output::TrackOutputFrame& frame_2 = result_2.track_output_frame;

  EXPECT_TRUE(result_2.has_validation_error);
  EXPECT_TRUE(
      ContainsIssueCode(result_2.validation_issues, session::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_EQ(frame_2.cycle_index, frame_1.cycle_index);
  EXPECT_EQ(frame_2.batch_id, frame_1.batch_id);
  EXPECT_EQ(frame_2.published_track_count, frame_1.published_track_count);
  EXPECT_EQ(output::CountJammingTracks(frame_2), output::CountJammingTracks(frame_1));
  EXPECT_TRUE(session.HasLatestControlProfile());

  session::RadarCycleInput cycle_3 = cycle_2;
  cycle_3.dt_sec = 1.0f;
  cycle_3.target_features[1].external_target_id = 703U;
  cycle_3.target_features[2].external_target_id = 704U;
  const output::TrackOutputFrame frame_3 =
      session.Step(cycle_3, environment::EnvironmentSceneBuilder().Build());
  EXPECT_GT(frame_3.published_track_count, 0U);
  EXPECT_EQ(output::CountJammingTracks(frame_3), 0U);
  EXPECT_TRUE(output::ContainsExternalTargetId(frame_3, 703U));
  EXPECT_TRUE(output::ContainsExternalTargetId(frame_3, 704U));
}

TEST(PublicApiConvenienceTest, RadarSessionStepWithResultAggregatesCurrentCycleObservations) {
  const session::RadarSessionConfig config = config::MakeDetectionMissionRadarSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(config);

  const session::RadarCycleInput input = MakeCycleInput(model::TargetFeatureList{
      model::MakeAirTarget(950U, 200.0f, -3.0f, 15.0f, 70.0f, 0.0f, 0.0f, 1.0f),
      model::MakeAirTarget(951U, 240.0f, 4.0f, 18.0f, 78.0f, 0.3f, 0.0f, 1.1f),
  });

  const session::RadarCycleResult result = session.StepWithResult(input);

  EXPECT_GT(result.track_output_frame.published_track_count, 0U);
  EXPECT_GE(result.track_output_frame.published_track_count,
            result.track_output_frame.confirmed_track_count);
  EXPECT_EQ(result.submitted_commands.size(), session.GetSubmittedCommands().size());
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.validation_issues.empty());
  EXPECT_EQ(result.has_control_profile, session.HasLatestControlProfile());
  if (result.has_control_profile) {
    ExpectEquivalentProfiles(result.control_profile, session.GetLatestControlProfile());
  }
  EXPECT_EQ(result.association_quality_metrics.detection_count,
            session.GetLastAssociationQualityMetrics().detection_count);
}

TEST(PublicApiConvenienceTest, RadarSessionStepWithResultSurfacesValidationErrors) {
  session::RadarSession session = session::RadarSessionFactory::Create(MakeConvenienceSessionConfig());

  session::RadarCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  model::TargetFeature invalid_target;
  invalid_target.external_target_id = 123U;
  invalid_target.range_m = 0.0f;
  input.target_features.push_back(invalid_target);

  const session::RadarCycleResult result = session.StepWithResult(input);

  EXPECT_TRUE(result.has_validation_error);
  EXPECT_TRUE(
      ContainsIssueCode(result.validation_issues, session::ValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(ContainsIssueCode(result.validation_issues,
                                session::ValidationCode::kMissingRangeAndCartesianPosition));
}

TEST(PublicApiConvenienceTest, RadarSessionStepWithResultMatchesManualChainUnderJammedScene) {
  const session::RadarSessionConfig config = config::MakeDetectionMissionRadarSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(config);

  session::MutableRadarContext manual_context;
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection = config.detection;
  pipeline_config.beam_control = config.beam_control;
  pipeline_config.tracking = config.tracking;
  pipeline_config.lifecycle = config.lifecycle;
  signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);
  environment::EnvironmentService environment_service(config.environment_default_config.model_config);
  environment_service.SetJammingDetectionThresholdDb(config.environment_default_config.jamming_detection_threshold_db);
  extension::RadarController controller(manual_context, signal_pipeline,
                                               environment_service);

  const session::RadarCycleInput input = MakeCycleInput(model::TargetFeatureList{
      model::MakeAirTarget(960U, 180.0f, -4.0f, 16.0f, 60.0f, 0.0f, 0.0f, 1.0f),
  });
  environment::JammerEmitterState noise_emitter_5;
  noise_emitter_5.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_emitter_5.power_db = 12.0f;
  noise_emitter_5.js_db = 8.0f;
  noise_emitter_5.frequency_overlap_ratio = 0.2f;
  noise_emitter_5.prf_lock_risk = 0.1f;
  noise_emitter_5.in_sidelobe = true;

  const environment::EnvironmentSceneState scene =
      environment::EnvironmentSceneBuilder().AddNoiseJammer(noise_emitter_5).Build();

  const session::RadarCycleResult session_result = session.StepWithResult(input, scene);

  environment_service.UpdateSceneState(scene);
  manual_context.BeginCycle(input);
  controller.RunOnce();

  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  EXPECT_EQ(session_result.track_output_frame.published_track_count,
            controller.GetLatestTrackOutputFrame().published_track_count);
  ExpectEquivalentCommands(manual_context.GetSubmittedCommands(),
                           session_result.submitted_commands);
  ASSERT_TRUE(manual_context.HasLatestControlProfile());
  ASSERT_TRUE(session_result.has_control_profile);
  ExpectEquivalentProfiles(manual_context.GetLatestControlProfile(),
                           session_result.control_profile);
  EXPECT_EQ(session_result.association_quality_metrics.detection_count,
            signal_pipeline.GetLastAssociationQualityMetrics().detection_count);
}

}  // namespace tests
}  // namespace airborne_radar
