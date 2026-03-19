// Copyright 2026. All Rights Reserved.
//
// @file public_api_convenience_test.cpp
// @brief 验证对外易用性增强 API 的默认语义与集成行为。

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/ConfigPresets.h"
#include "1q/airborne_radar/common/TargetFeatureUtils.h"
#include "airborne_radar/core/context/MutableRadarContext.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "1q/airborne_radar/core/context/RadarInputValidation.h"
#include "1q/airborne_radar/core/output/TrackOutputQueries.h"
#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/core/controller/RadarController.h"

namespace airborne_radar {
namespace tests {

namespace {

signal::pipeline::SignalPipelineConfig MakeConveniencePipelineConfig() {
  signal::pipeline::SignalPipelineConfig config;
  config.detection.min_detection_margin_db = -100.0f;
  config.lifecycle.enable_auto_lifecycle_manager = true;
  config.lifecycle.lifecycle_config.confirm_hits = 1U;
  config.tracking.kalman_measurement_noise_std = 1.0f;
  return config;
}

core::session::RadarSessionConfig MakeConvenienceSessionConfig() {
  core::session::RadarSessionConfig config;
  config.signal_pipeline_config = MakeConveniencePipelineConfig();
  return config;
}

core::context::RadarCycleInput MakeCycleInput(
    common::TargetFeatureList targets, float dt_sec = 1.0f) {
  core::context::RadarCycleInput input;
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
void ExpectEquivalentCommands(
    const std::vector<common::RadarCommand>& expected,
    const std::vector<common::RadarCommand>& actual) {
  ASSERT_EQ(expected.size(), actual.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(expected[i].type, actual[i].type);
    EXPECT_EQ(expected[i].source, actual[i].source);
  }
}

/// @brief 比较两份控制真值的公开语义字段。
/// @param expected 期望控制真值。
/// @param actual 实际控制真值。
void ExpectEquivalentProfiles(const common::RadarControlProfile& expected,
                              const common::RadarControlProfile& actual) {
  EXPECT_EQ(expected.version, actual.version);
  EXPECT_EQ(expected.enable_lpi_power_control, actual.enable_lpi_power_control);
  EXPECT_NEAR(expected.lpi_power_scale, actual.lpi_power_scale, 1e-5f);
  EXPECT_EQ(expected.enable_lpi_beamforming, actual.enable_lpi_beamforming);
  EXPECT_NEAR(expected.lpi_dwell_scale, actual.lpi_dwell_scale, 1e-5f);
  EXPECT_EQ(expected.enable_agility_frequency, actual.enable_agility_frequency);
  EXPECT_EQ(expected.enable_sidelobe_canceller,
            actual.enable_sidelobe_canceller);
  EXPECT_EQ(expected.enable_adaptive_beamforming,
            actual.enable_adaptive_beamforming);
  EXPECT_EQ(expected.enable_eccm_rejitter, actual.enable_eccm_rejitter);
  EXPECT_NEAR(expected.eccm_burnthrough_gain, actual.eccm_burnthrough_gain,
              1e-5f);
}

/// @brief 判断问题列表是否包含指定编码。
/// @param issues 校验问题列表。
/// @param code 目标编码。
/// @return 若存在则返回 true。
bool ContainsIssueCode(
    const std::vector<core::context::ValidationIssue>& issues,
    core::context::ValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

} // namespace

TEST(PublicApiConvenienceTest,
     MutableRadarContextBeginsCycleAndResetsPerCycleCommands) {
  core::context::MutableRadarContext context;
  core::context::RadarCycleInput input = MakeCycleInput(
      common::TargetFeatureList{
          common::MakeAirTarget(101U, 120.0f, -5.0f, 18.0f, 62.0f, 0.0f, 0.0f,
                                1.0f),
      },
      0.5f);

  context.BeginCycle(input);
  ASSERT_EQ(context.GetTargetFeatures().size(), 1U);
  EXPECT_NEAR(context.GetPlatformAttitude().yaw_deg, 5.0f, 1e-5f);
  EXPECT_NEAR(context.GetCycleDeltaTimeSec(), 0.5f, 1e-5f);
  EXPECT_TRUE(context.GetSubmittedCommands().empty());

  context.SubmitControlCommand(common::RadarCommand(
      common::RadarCommandType::SET_AGILITY_FREQ,
      common::RadarCommandSource::ECCM));
  common::RadarControlProfile profile;
  profile.enable_agility_frequency = true;
  profile.version = 4U;
  context.UpdateRadarControlProfile(profile);

  ASSERT_EQ(context.GetSubmittedCommands().size(), 1U);
  ASSERT_TRUE(context.HasLatestControlProfile());
  EXPECT_TRUE(context.GetLatestControlProfile().enable_agility_frequency);
  EXPECT_EQ(context.GetLatestControlProfile().version, 4U);

  core::context::RadarCycleInput second_input = MakeCycleInput({}, 1.25f);
  context.BeginCycle(second_input);
  EXPECT_TRUE(context.GetSubmittedCommands().empty());
  EXPECT_NEAR(context.GetCycleDeltaTimeSec(), 1.25f, 1e-5f);
  EXPECT_TRUE(context.HasLatestControlProfile());
  EXPECT_EQ(context.GetLatestControlProfile().version, 4U);
}

TEST(PublicApiConvenienceTest,
     TargetFeatureUtilsBuildCartesianGroundAndAirTargets) {
  const common::TargetFeature cartesian_target =
      common::MakeTargetFromCartesian(201U, 3.0f, 4.0f, 12.0f, 10.0f, -2.0f,
                                      1.0f, 0.9f, 0.0f, 3.0f, 4.0f, 2);
  EXPECT_EQ(cartesian_target.external_target_id, 201U);
  EXPECT_NEAR(cartesian_target.range_m, 13.0f, 1e-5f);
  EXPECT_NEAR(cartesian_target.current_track_speed,
              std::sqrt(105.0f), 1e-5f);
  EXPECT_EQ(cartesian_target.target_swerling_type, 2);

  const common::TargetFeature ground_target =
      common::MakeGroundTarget(202U, 20.0f, -15.0f, 0.8f);
  EXPECT_NEAR(ground_target.position_z, 0.0f, 1e-5f);
  EXPECT_NEAR(ground_target.current_track_velocity_z, 0.0f, 1e-5f);
  EXPECT_NEAR(ground_target.range_m, 25.0f, 1e-5f);

  const common::TargetFeature air_target =
      common::MakeAirTarget(203U, 30.0f, 40.0f, 50.0f, 90.0f, -3.0f, 4.0f,
                            1.2f);
  EXPECT_NEAR(air_target.range_m, std::sqrt(5000.0f), 1e-5f);
  EXPECT_NEAR(air_target.current_track_speed,
              std::sqrt(8125.0f), 1e-5f);
}

TEST(PublicApiConvenienceTest,
     TargetFeatureUtilsNormalizeGeometryOnlyWhenCartesianPositionExists) {
  common::TargetFeature normalized =
      common::MakeAirTarget(301U, 6.0f, 8.0f, 0.0f, 50.0f, 0.0f, 0.0f, 1.0f);
  normalized.range_m = 0.0f;
  common::NormalizeTargetGeometry(&normalized);
  EXPECT_NEAR(normalized.range_m, 10.0f, 1e-5f);

  common::TargetFeature unresolved;
  unresolved.external_target_id = 302U;
  unresolved.range_m = 0.0f;
  common::NormalizeTargetGeometry(&unresolved);
  EXPECT_NEAR(unresolved.range_m, 0.0f, 1e-5f);

  common::TargetFeatureList targets;
  targets.push_back(normalized);
  targets.back().range_m = -1.0f;
  targets.push_back(unresolved);
  common::NormalizeTargetGeometry(&targets);
  EXPECT_NEAR(targets[0].range_m, 10.0f, 1e-5f);
  EXPECT_NEAR(targets[1].range_m, 0.0f, 1e-5f);
}

TEST(PublicApiConvenienceTest,
     EnvironmentSceneBuilderDefaultsMatchEnvironmentSceneState) {
  const environment::EnvironmentSceneState built_scene =
      environment::EnvironmentSceneBuilder().Build();
  const environment::EnvironmentSceneState default_scene;

  EXPECT_NEAR(built_scene.base_propagation_loss_db,
              default_scene.base_propagation_loss_db, 1e-5f);
  EXPECT_NEAR(built_scene.atmospheric_attenuation_db,
              default_scene.atmospheric_attenuation_db, 1e-5f);
  EXPECT_NEAR(built_scene.terrain_reflection_db,
              default_scene.terrain_reflection_db, 1e-5f);
  EXPECT_NEAR(built_scene.clutter_power_db, default_scene.clutter_power_db,
              1e-5f);
  EXPECT_TRUE(built_scene.jammer_emitters.empty());
}

TEST(PublicApiConvenienceTest,
     EnvironmentSceneBuilderJammerHelpersPopulateTypedEmitters) {
  const environment::EnvironmentSceneState scene =
      environment::EnvironmentSceneBuilder()
          .SetBasePropagationLossDb(5.5f)
          .SetAtmosphericAttenuationDb(2.0f)
          .SetTerrainReflectionDb(1.2f)
          .SetClutterPowerDb(4.5f)
          .AddNoiseJammer(11.0f, 8.0f, 0.2f, 0.1f, true, 12.0f, 1.0f, 5.0f,
                          0.9f)
          .AddDeceptionJammer(9.0f, 7.5f, 0.9f, 0.8f, false, -8.0f, 2.5f, 3.0f,
                              0.8f)
          .AddRepeaterJammer(8.5f, 6.5f, 0.1f, 0.95f, false, 4.0f, -1.0f, 2.0f,
                             0.7f)
          .Build();

  ASSERT_EQ(scene.jammer_emitters.size(), 3U);
  EXPECT_EQ(scene.jammer_emitters[0].technique,
            environment::JammingTechnique::kNoiseSuppression);
  EXPECT_TRUE(scene.jammer_emitters[0].in_sidelobe);
  EXPECT_EQ(scene.jammer_emitters[1].technique,
            environment::JammingTechnique::kDeception);
  EXPECT_NEAR(scene.jammer_emitters[1].frequency_overlap_ratio, 0.9f, 1e-5f);
  EXPECT_EQ(scene.jammer_emitters[2].technique,
            environment::JammingTechnique::kRepeater);
  EXPECT_NEAR(scene.jammer_emitters[2].prf_lock_risk, 0.95f, 1e-5f);
  EXPECT_NEAR(scene.base_propagation_loss_db, 5.5f, 1e-5f);
  EXPECT_NEAR(scene.clutter_power_db, 4.5f, 1e-5f);
}

TEST(PublicApiConvenienceTest,
     TrackOutputQueriesSupportUniqueDuplicateAndJammingSearch) {
  common::TrackOutputFrame frame;
  frame.tracks.push_back(
      common::DecisionTrackSnapshot(10.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                                    false, 401U, 1U));
  frame.tracks.push_back(
      common::DecisionTrackSnapshot(20.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                                    true, 402U, 2U));
  frame.tracks.push_back(
      common::DecisionTrackSnapshot(21.0f, 0.0f, 0.0f, 1.1f, 0.0f, 0.0f, 0.0f,
                                    false, 402U, 3U));
  frame.tracks.push_back(
      common::DecisionTrackSnapshot(8.0f, 0.0f, 0.0f, 0.7f, 0.0f, 0.0f, 0.0f,
                                    true, 0U, 4U));

  const auto track_map =
      core::output::BuildTrackMapByExternalTargetId(frame);
  EXPECT_EQ(track_map.size(), 2U);
  ASSERT_NE(track_map.count(401U), 0U);
  ASSERT_NE(track_map.count(402U), 0U);
  EXPECT_EQ(track_map.at(402U).state.association_key, 3U);

  const common::DecisionTrackSnapshotList duplicate_tracks =
      core::output::CollectTracksByExternalTargetId(frame, 402U);
  EXPECT_EQ(duplicate_tracks.size(), 2U);
  EXPECT_TRUE(core::output::ContainsExternalTargetId(frame, 401U));
  EXPECT_TRUE(core::output::ContainsExternalTargetId(frame, 0U));
  EXPECT_FALSE(core::output::ContainsExternalTargetId(frame, 999U));
  EXPECT_EQ(core::output::CountJammingTracks(frame), 2U);
  EXPECT_EQ(core::output::CountTracksByStatus(
                frame, common::DecisionTrackStatus::kTentative),
            4U);
}

TEST(PublicApiConvenienceTest,
     TrackOutputQueriesSupportAssociationKeyStatusAndJammingCollections) {
  common::TrackOutputFrame frame;
  common::DecisionTrackSnapshot confirmed(
      20.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, false, 410U, 11U);
  confirmed.state.status = common::DecisionTrackStatus::kConfirmed;
  common::DecisionTrackSnapshot lost(
      5.0f, 0.0f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f, false, 411U, 12U);
  lost.state.status = common::DecisionTrackStatus::kLost;
  common::DecisionTrackSnapshot jammed(
      18.0f, 0.0f, 0.0f, 1.2f, 0.0f, 0.0f, 0.0f, true, 412U, 13U);
  jammed.state.status = common::DecisionTrackStatus::kConfirmed;
  frame.tracks.push_back(confirmed);
  frame.tracks.push_back(lost);
  frame.tracks.push_back(jammed);

  const auto track_map = core::output::BuildTrackMapByAssociationKey(frame);
  ASSERT_EQ(track_map.size(), 3U);
  EXPECT_EQ(track_map.at(13U).state.external_target_id, 412U);
  EXPECT_EQ(core::output::CollectConfirmedTracks(frame).size(), 2U);
  EXPECT_EQ(core::output::CollectLostTracks(frame).size(), 1U);
  EXPECT_EQ(core::output::CollectJammingTracks(frame).size(), 1U);
  EXPECT_EQ(core::output::CountTracksByStatus(
                frame, common::DecisionTrackStatus::kConfirmed),
            2U);
}

TEST(PublicApiConvenienceTest,
     RadarInputValidationReportsWarningsAndErrorsForCommonBoundaryCases) {
  core::context::RadarCycleInput input = MakeCycleInput(
      common::TargetFeatureList{
          common::MakeAirTarget(0U, 120.0f, 0.0f, 10.0f, 55.0f, 0.0f, 0.0f,
                                1.0f),
          common::MakeAirTarget(800U, 150.0f, -2.0f, 12.0f, 62.0f, 0.2f, 0.0f,
                                1.0f),
          common::MakeAirTarget(800U, 155.0f, 2.0f, 12.0f, 63.0f, -0.2f, 0.0f,
                                -1.1f),
      },
      0.0f);
  input.target_features[2].position_x =
      std::numeric_limits<float>::infinity();
  input.target_features[2].range_m = 0.0f;

  const std::vector<core::context::ValidationIssue> issues =
      core::context::ValidateRadarCycleInput(input);
  EXPECT_TRUE(ContainsIssueCode(
      issues, core::context::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(ContainsIssueCode(
      issues, core::context::ValidationCode::kUnknownExternalTargetId));
  EXPECT_TRUE(ContainsIssueCode(
      issues, core::context::ValidationCode::kDuplicateExternalTargetId));
  EXPECT_TRUE(ContainsIssueCode(issues,
                                core::context::ValidationCode::kNegativeRcs));
  EXPECT_TRUE(ContainsIssueCode(
      issues, core::context::ValidationCode::kNonFiniteTargetField));
  EXPECT_TRUE(core::context::HasValidationError(issues));
}

TEST(PublicApiConvenienceTest,
     RadarInputValidationFlagsMissingGeometryAndNonFiniteCycleDelta) {
  core::context::RadarCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  common::TargetFeature invalid_target;
  invalid_target.external_target_id = 900U;
  invalid_target.range_m = 0.0f;
  input.target_features.push_back(invalid_target);

  const std::vector<core::context::ValidationIssue> issues =
      core::context::ValidateRadarCycleInput(input);
  EXPECT_TRUE(ContainsIssueCode(
      issues, core::context::ValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(ContainsIssueCode(
      issues,
      core::context::ValidationCode::kMissingRangeAndCartesianPosition));
  EXPECT_TRUE(core::context::HasValidationError(issues));
}

TEST(PublicApiConvenienceTest,
     ConfigPresetsProvideExpectedDetectionAndRobustnessDefaults) {
  const signal::pipeline::SignalPipelineConfig detection_config =
      common::MakeDetectionMissionSignalPipelineConfig();
  EXPECT_TRUE(detection_config.lifecycle.enable_auto_lifecycle_manager);
  EXPECT_EQ(detection_config.lifecycle.lifecycle_config.confirm_hits, 1U);
  EXPECT_NEAR(detection_config.detection.min_detection_margin_db, -100.0f,
              1e-5f);

  const signal::pipeline::SignalPipelineConfig robust_config =
      common::MakeHighRobustnessSignalPipelineConfig();
  EXPECT_TRUE(robust_config.lifecycle.enable_auto_lifecycle_manager);
  EXPECT_GT(robust_config.association.unassigned_cost,
            detection_config.association.unassigned_cost);
  EXPECT_GT(robust_config.lifecycle.lifecycle_config.max_miss_before_lost,
            detection_config.lifecycle.lifecycle_config.max_miss_before_lost);
  EXPECT_GT(robust_config.tracking.speed_decay_ratio_on_loss,
            detection_config.tracking.speed_decay_ratio_on_loss);

  const core::session::RadarSessionConfig session_config =
      common::MakeDetectionMissionRadarSessionConfig();
  core::session::RadarSession session(session_config);
  const common::TrackOutputFrame frame = session.Step(MakeCycleInput(
      common::TargetFeatureList{
          common::MakeGroundTarget(901U, 20.0f, 5.0f, 0.8f),
      }));
  EXPECT_EQ(frame.confirmed_track_count, 1U);
}

TEST(PublicApiConvenienceTest,
     RadarSessionStepProducesReadableOutputWithoutInterference) {
  core::session::RadarSession session(MakeConvenienceSessionConfig());
  const core::context::RadarCycleInput input = MakeCycleInput(
      common::TargetFeatureList{
          common::MakeGroundTarget(501U, 25.0f, -5.0f, 0.7f),
          common::MakeAirTarget(502U, 150.0f, 6.0f, 18.0f, 72.0f, 1.0f, 0.0f,
                                1.0f),
      });

  const common::TrackOutputFrame frame = session.Step(input);
  EXPECT_EQ(frame.published_track_count, 2U);
  EXPECT_EQ(frame.confirmed_track_count, 2U);
  EXPECT_TRUE(core::output::ContainsExternalTargetId(frame, 501U));
  EXPECT_TRUE(core::output::ContainsExternalTargetId(frame, 502U));
  EXPECT_EQ(core::output::CountJammingTracks(frame), 0U);
  EXPECT_TRUE(session.GetSubmittedCommands().empty());
  EXPECT_TRUE(session.HasLatestControlProfile());
}

TEST(PublicApiConvenienceTest,
     RadarSessionSceneAwareStepMatchesManualControllerChain) {
  const core::session::RadarSessionConfig config = MakeConvenienceSessionConfig();
  core::session::RadarSession session(config);

  core::context::MutableRadarContext manual_context;
  signal::pipeline::SignalPipeline signal_pipeline(config.signal_pipeline_config);
  environment::EnvironmentService environment_service(
      config.environment_model_config);
  environment_service.SetJammingDetectionThresholdDb(
      config.jamming_detection_threshold_db);
  core::controller::RadarController controller(
      manual_context, signal_pipeline, environment_service);

  const core::context::RadarCycleInput input = MakeCycleInput(
      common::TargetFeatureList{
          common::MakeAirTarget(601U, 180.0f, -4.0f, 16.0f, 60.0f, 0.0f, 0.0f,
                                1.0f),
          common::MakeAirTarget(602U, 240.0f, 8.0f, 18.0f, 76.0f, 0.5f, 0.0f,
                                1.1f),
      });
  const environment::EnvironmentSceneState scene =
      environment::EnvironmentSceneBuilder()
          .AddNoiseJammer(12.0f, 8.0f, 0.2f, 0.1f, true)
          .Build();

  const common::TrackOutputFrame session_frame = session.Step(input, scene);

  environment_service.UpdateSceneState(scene);
  manual_context.BeginCycle(input);
  controller.RunOnce();
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const common::TrackOutputFrame manual_frame =
      controller.GetLatestTrackOutputFrame();

  EXPECT_EQ(session_frame.published_track_count, manual_frame.published_track_count);
  EXPECT_EQ(session_frame.confirmed_track_count, manual_frame.confirmed_track_count);
  EXPECT_EQ(core::output::CountJammingTracks(session_frame),
            core::output::CountJammingTracks(manual_frame));

  const auto session_track_map =
      core::output::BuildTrackMapByExternalTargetId(session_frame);
  const auto manual_track_map =
      core::output::BuildTrackMapByExternalTargetId(manual_frame);
  ASSERT_EQ(session_track_map.size(), manual_track_map.size());
  for (std::size_t i = 0; i < input.target_features.size(); ++i) {
    const std::uint64_t target_id = input.target_features[i].external_target_id;
    ASSERT_NE(session_track_map.count(target_id), 0U);
    ASSERT_NE(manual_track_map.count(target_id), 0U);
    EXPECT_EQ(session_track_map.at(target_id).state.association_key,
              manual_track_map.at(target_id).state.association_key);
  }

  ExpectEquivalentCommands(manual_context.GetSubmittedCommands(),
                           session.GetSubmittedCommands());
  ASSERT_TRUE(session.HasLatestControlProfile());
  ASSERT_TRUE(manual_context.HasLatestControlProfile());
  ExpectEquivalentProfiles(manual_context.GetLatestControlProfile(),
                           session.GetLatestControlProfile());

  const signal::pipeline::AssociationQualityMetrics session_metrics =
      session.GetLastAssociationQualityMetrics();
  const signal::pipeline::AssociationQualityMetrics manual_metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();
  EXPECT_EQ(session_metrics.detection_count, manual_metrics.detection_count);
  EXPECT_NEAR(session_metrics.match_rate, manual_metrics.match_rate, 1e-5f);
  EXPECT_NEAR(session_metrics.association_stress,
              manual_metrics.association_stress, 1e-5f);

}

TEST(PublicApiConvenienceTest,
     RadarSessionHandlesInvalidDeltaUnknownAndDuplicateIdsAcrossSceneSwitches) {
  core::session::RadarSession session(MakeConvenienceSessionConfig());

  core::context::RadarCycleInput cycle_1 = MakeCycleInput(
      common::TargetFeatureList{
          common::MakeAirTarget(0U, 120.0f, 0.0f, 10.0f, 55.0f, 0.0f, 0.0f,
                                1.0f),
          common::MakeAirTarget(701U, 150.0f, -2.0f, 12.0f, 62.0f, 0.2f, 0.0f,
                                1.0f),
          common::MakeAirTarget(701U, 155.0f, 2.0f, 12.0f, 63.0f, -0.2f, 0.0f,
                                1.1f),
          common::MakeGroundTarget(702U, 40.0f, -6.0f, 0.8f),
      },
      0.0f);
  const common::TrackOutputFrame frame_1 = session.Step(cycle_1);
  EXPECT_GT(frame_1.published_track_count, 0U);
  EXPECT_TRUE(core::output::ContainsExternalTargetId(frame_1, 0U));
  EXPECT_EQ(core::output::CollectTracksByExternalTargetId(frame_1, 701U).size(),
            2U);

  core::context::RadarCycleInput cycle_2 = cycle_1;
  cycle_2.dt_sec = -1.0f;
  for (std::size_t i = 0; i < cycle_2.target_features.size(); ++i) {
    cycle_2.target_features[i].position_x += 5.0f;
  }
  const common::TrackOutputFrame frame_2 = session.Step(
      cycle_2, environment::EnvironmentSceneBuilder()
                   .AddNoiseJammer(12.0f, 8.0f, 0.2f, 0.1f, true)
                   .Build());
  EXPECT_GT(frame_2.published_track_count, 0U);
  EXPECT_GT(core::output::CountJammingTracks(frame_2), 0U);
  EXPECT_TRUE(session.HasLatestControlProfile());

  core::context::RadarCycleInput cycle_3 = cycle_2;
  cycle_3.dt_sec = 1.0f;
  cycle_3.target_features[1].external_target_id = 703U;
  cycle_3.target_features[2].external_target_id = 704U;
  const common::TrackOutputFrame frame_3 = session.Step(
      cycle_3, environment::EnvironmentSceneBuilder().Build());
  EXPECT_GT(frame_3.published_track_count, 0U);
  EXPECT_TRUE(core::output::ContainsExternalTargetId(frame_3, 703U));
  EXPECT_TRUE(core::output::ContainsExternalTargetId(frame_3, 704U));
}

TEST(PublicApiConvenienceTest,
     RadarSessionStepWithResultAggregatesCurrentCycleObservations) {
  const core::session::RadarSessionConfig config =
      common::MakeDetectionMissionRadarSessionConfig();
  core::session::RadarSession session(config);

  const core::context::RadarCycleInput input = MakeCycleInput(
      common::TargetFeatureList{
          common::MakeAirTarget(950U, 200.0f, -3.0f, 15.0f, 70.0f, 0.0f, 0.0f,
                                1.0f),
          common::MakeAirTarget(951U, 240.0f, 4.0f, 18.0f, 78.0f, 0.3f, 0.0f,
                                1.1f),
      });

  const core::session::RadarCycleResult result = session.StepWithResult(input);

  EXPECT_GT(result.track_output_frame.published_track_count, 0U);
  EXPECT_GE(result.track_output_frame.published_track_count,
            result.track_output_frame.confirmed_track_count);
  EXPECT_EQ(result.submitted_commands.size(),
            session.GetSubmittedCommands().size());
  EXPECT_EQ(result.has_control_profile, session.HasLatestControlProfile());
  if (result.has_control_profile) {
    ExpectEquivalentProfiles(result.control_profile,
                             session.GetLatestControlProfile());
  }
  EXPECT_EQ(result.association_quality_metrics.detection_count,
            session.GetLastAssociationQualityMetrics().detection_count);
}

TEST(PublicApiConvenienceTest,
     RadarSessionStepWithResultMatchesManualChainUnderJammedScene) {
  const core::session::RadarSessionConfig config =
      common::MakeDetectionMissionRadarSessionConfig();
  core::session::RadarSession session(config);

  core::context::MutableRadarContext manual_context;
  signal::pipeline::SignalPipeline signal_pipeline(config.signal_pipeline_config);
  environment::EnvironmentService environment_service(
      config.environment_model_config);
  environment_service.SetJammingDetectionThresholdDb(
      config.jamming_detection_threshold_db);
  core::controller::RadarController controller(
      manual_context, signal_pipeline, environment_service);

  const core::context::RadarCycleInput input = MakeCycleInput(
      common::TargetFeatureList{
          common::MakeAirTarget(960U, 180.0f, -4.0f, 16.0f, 60.0f, 0.0f, 0.0f,
                                1.0f),
      });
  const environment::EnvironmentSceneState scene =
      environment::EnvironmentSceneBuilder()
          .AddNoiseJammer(12.0f, 8.0f, 0.2f, 0.1f, true)
          .Build();

  const core::session::RadarCycleResult session_result =
      session.StepWithResult(input, scene);

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

} // namespace tests
} // namespace airborne_radar
