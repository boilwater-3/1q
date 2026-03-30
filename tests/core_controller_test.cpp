// Copyright 2026. All Rights Reserved.
//
// @file core_controller_test.cpp
// @brief 验证核心调度器使用桩组件的最小集成流程。

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "1q/airborne_radar/common/control/RadarCommand.h"
#include "1q/airborne_radar/common/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/model/DecisionInputFrame.h"
#include "1q/airborne_radar/common/model/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "1q/airborne_radar/decision/pipeline/ControlReducerTypes.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "environment_test_fixture.h"

namespace airborne_radar {
namespace tests {

namespace {

common::model::TargetFeatureList BuildSingleTarget(float speed, float rcs, bool jamming) {
  (void)jamming;
  common::model::TargetFeature target(speed, 0.0f, 0.0f, rcs);
  target.has_cartesian_position = true;
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  return common::model::TargetFeatureList{target};
}

environment::EnvironmentModelConfig BuildJammingEnvironmentConfig(float jammer_power_db) {
  environment::EnvironmentModelConfig config;
  config.jammer_sources.push_back(environment_test::MakeJammerEmitter(
      environment::JammingTechnique::kUnknown, jammer_power_db));
  return config;
}

}  // namespace

/// @brief FakeRadarContext 提供最小化的雷达上下文实现。
class FakeRadarContext : public core::context::IRadarContext {
 public:
  /// @brief 构造函数，注入固定的输入状态。
  explicit FakeRadarContext(common::model::TargetFeatureList state) : state_(std::move(state)) {}

  /// @brief 获取当前雷达状态。
  const common::model::TargetFeatureList& GetTargetFeatures() const override { return state_; }

  /// @brief 获取当前搭载平台姿态角。
  common::config::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_deg_;
  }

  /// @brief 获取当前周期时间步长。
  float GetCycleDeltaTimeSec() const override { return cycle_dt_sec_; }

  /// @brief 收集控制指令。
  void SubmitControlCommand(common::control::RadarCommand cmd) override {
    submitted_commands_.push_back(cmd);
  }

  /// @brief 获取已提交的指令集合。
  const std::vector<common::control::RadarCommand>& SubmittedCommands() const {
    return submitted_commands_;
  }

  /// @brief 记录最新控制真值。
  void UpdateRadarControlProfile(const common::control::RadarControlProfile& profile) override {
    latest_control_profile_ = profile;
  }

  /// @brief 获取最新控制真值。
  const common::control::RadarControlProfile& LatestControlProfile() const {
    return latest_control_profile_;
  }

  /// @brief 设置测试上下文使用的平台姿态角。
  void SetPlatformAttitude(const common::config::PlatformAttitudeDeg& platform_attitude_deg) {
    platform_attitude_deg_ = platform_attitude_deg;
  }

  /// @brief 设置当前周期时间步长。
  void SetCycleDeltaTimeSec(float cycle_dt_sec) { cycle_dt_sec_ = cycle_dt_sec; }

 private:
  common::model::TargetFeatureList state_;
  common::config::PlatformAttitudeDeg platform_attitude_deg_{};
  float cycle_dt_sec_{1.0f};
  std::vector<common::control::RadarCommand> submitted_commands_;
  common::control::RadarControlProfile latest_control_profile_{};
};

class FixedDirectiveDecisionEngine : public decision::pipeline::ITacticalDecisionEngine {
 public:
  explicit FixedDirectiveDecisionEngine(common::control::ControlDirective directive)
      : directive_(std::move(directive)) {}

  decision::pipeline::TacticalDecisionResult Evaluate(
      const common::model::DecisionInputFrame&, decision::pipeline::TacticalStateStore&) override {
    decision::pipeline::TacticalDecisionResult result;
    result.proposals.push_back(
        decision::pipeline::TacticalProposal(directive_, 10, "test directive"));
    return result;
  }

 private:
  common::control::ControlDirective directive_;
};

class FixedProposalDecisionEngine : public decision::pipeline::ITacticalDecisionEngine {
 public:
  explicit FixedProposalDecisionEngine(std::vector<decision::pipeline::TacticalProposal> proposals)
      : proposals_(std::move(proposals)) {}

  decision::pipeline::TacticalDecisionResult Evaluate(
      const common::model::DecisionInputFrame&, decision::pipeline::TacticalStateStore&) override {
    decision::pipeline::TacticalDecisionResult result;
    result.proposals = proposals_;
    return result;
  }

 private:
  std::vector<decision::pipeline::TacticalProposal> proposals_;
};

class CapturingDecisionEngine : public decision::pipeline::ITacticalDecisionEngine {
 public:
  decision::pipeline::TacticalDecisionResult Evaluate(
      const common::model::DecisionInputFrame& frame,
      decision::pipeline::TacticalStateStore&) override {
    last_frame = frame;
    ++evaluate_count;
    return decision::pipeline::TacticalDecisionResult();
  }

  common::model::DecisionInputFrame last_frame{};
  std::size_t evaluate_count{0U};
};

/// @brief CoreControllerTest 覆盖核心调度与指令下发路径。
class CoreControllerTest : public ::testing::Test {};

TEST_F(CoreControllerTest, RunOnceSubmitsCommands) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service(BuildJammingEnvironmentConfig(12.0f));

  signal::pipeline::SignalPipeline signal_pipeline;

  core::controller::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  const auto& cmds = radar_context.SubmittedCommands();
  EXPECT_GT(cmds.size(), 0u);
}

TEST_F(CoreControllerTest, PushesPlatformAttitudeIntoSignalPipelineBeforeRun) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);
  common::config::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 18.0f;
  platform_attitude_deg.pitch_deg = -4.0f;
  platform_attitude_deg.roll_deg = 2.0f;
  radar_context.SetPlatformAttitude(platform_attitude_deg);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  core::controller::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  const common::config::PlatformAttitudeDeg cached_platform_attitude =
      signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 18.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -4.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 2.0f);
}

TEST_F(CoreControllerTest, ReusesFrozenEnvironmentSnapshotAcrossSignalAndDecision) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;
  environment::JammerEmitterState jammer_source =
      environment_test::MakeJammerEmitter(environment::JammingTechnique::kDeception, 10.0f);
  jammer_source.js_db = 7.0f;
  jammer_source.frequency_overlap_ratio = 0.85f;
  jammer_source.prf_lock_risk = 0.75f;
  jammer_source.in_sidelobe = false;
  environment_service.UpdateSceneState(environment_test::MemorySceneBuilder()
                                           .WithPropagation(20.0f, 6.0f, 4.0f)
                                           .WithClutter(12.0f)
                                           .AddJammer(jammer_source)
                                           .BuildSceneState());

  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;
  core::controller::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                               environment_service);

  controller.RunOnce();

  const environment::EnvironmentSnapshot frozen_snapshot = environment_service.SampleEnvironment();
  const std::vector<signal::tracking::TrackMeasurement> measurements =
      signal_pipeline.GetLastTrackMeasurements();

  EXPECT_TRUE(decision_engine.last_frame.environment_jamming_detected);
  EXPECT_EQ(decision_engine.last_frame.environment_jamming_detected,
            frozen_snapshot.jamming_detected);
  EXPECT_EQ(decision_engine.last_frame.eccm_source_info.has_jamming_signal,
            frozen_snapshot.jamming_detected);
  ASSERT_EQ(decision_engine.last_frame.eccm_source_info.jammer_sources.size(),
            frozen_snapshot.jammer_sources.size());
  ASSERT_FALSE(decision_engine.last_frame.eccm_source_info.jammer_sources.empty());
  ASSERT_FALSE(frozen_snapshot.jammer_sources.empty());
  EXPECT_FLOAT_EQ(decision_engine.last_frame.eccm_source_info.jammer_sources.front().jammer_power_db,
                  frozen_snapshot.jammer_sources.front().power_db);
  ASSERT_EQ(measurements.size(), 1U);
  EXPECT_EQ(measurements[0].filtered_feature.jamming_detected, frozen_snapshot.jamming_detected);
}

TEST_F(CoreControllerTest, AppliesUpdatedSceneOnNextControllerCycle) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;
  core::controller::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                               environment_service);

  controller.RunOnce();
  EXPECT_FALSE(decision_engine.last_frame.environment_jamming_detected);

  environment::JammerEmitterState jammer_source =
      environment_test::MakeJammerEmitter(environment::JammingTechnique::kNoiseSuppression, 9.0f);
  jammer_source.in_sidelobe = true;
  environment_service.UpdateSceneState(
      environment_test::MemorySceneBuilder().AddJammer(jammer_source).BuildSceneState());

  EXPECT_FALSE(environment_service.SampleEnvironment().jamming_detected);

  controller.RunOnce();

  EXPECT_TRUE(decision_engine.last_frame.environment_jamming_detected);
  EXPECT_TRUE(environment_service.SampleEnvironment().jamming_detected);
}

TEST_F(CoreControllerTest, MapsMultiSourceJammingFactsIntoDecisionFrame) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  environment::JammerSourceFact deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 9.0f;
  deception_source.js_db = 7.5f;
  deception_source.frequency_overlap_ratio = 0.85f;
  deception_source.prf_lock_risk = 0.90f;
  deception_source.azimuth_deg = 30.0f;
  deception_source.elevation_deg = 5.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.in_sidelobe = false;
  deception_source.confidence = 0.95f;
  env_config.jammer_sources.push_back(deception_source);
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;
  core::controller::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                               environment_service);

  controller.RunOnce();

  ASSERT_EQ(decision_engine.evaluate_count, 1u);
  EXPECT_TRUE(decision_engine.last_frame.environment_jamming_detected);
  ASSERT_EQ(decision_engine.last_frame.eccm_source_info.jammer_sources.size(), 1u);
  const common::model::EccmJammerSourceInfo& mapped_source =
      decision_engine.last_frame.eccm_source_info.jammer_sources.front();
  EXPECT_EQ(mapped_source.technique, common::model::JammingTechnique::kDeception);
  EXPECT_FLOAT_EQ(mapped_source.jammer_power_db, 9.0f);
  EXPECT_FLOAT_EQ(mapped_source.jammer_to_signal_db, 7.5f);
  EXPECT_FLOAT_EQ(mapped_source.frequency_overlap_ratio, 0.85f);
  EXPECT_FLOAT_EQ(mapped_source.prf_lock_risk, 0.90f);
  EXPECT_FLOAT_EQ(mapped_source.azimuth_deg, 30.0f);
  EXPECT_FLOAT_EQ(mapped_source.angular_span_deg, 8.0f);
  EXPECT_EQ(decision_engine.last_frame.association_quality_info.dominant_jamming_semantic,
            common::utils::JammingSemantic::kDeception);
  EXPECT_GT(decision_engine.last_frame.association_quality_info.jamming_severity, 0.0f);
  EXPECT_GT(decision_engine.last_frame.association_quality_info.association_stress, 0.0f);
  EXPECT_EQ(decision_engine.last_frame.perception_quality_info.input_target_count, 1u);
  EXPECT_EQ(decision_engine.last_frame.perception_quality_info.detection_count, 1u);
  EXPECT_FLOAT_EQ(decision_engine.last_frame.perception_quality_info.detection_rate, 1.0f);
  EXPECT_FLOAT_EQ(decision_engine.last_frame.perception_quality_info.detection_stress, 0.0f);
}

TEST_F(CoreControllerTest, NoLifecycleFallbackBuildsDecisionFrameFromSyntheticTrackOutput) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(640.0f, 1.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;

  core::controller::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                               environment_service);

  controller.RunOnce();

  EXPECT_TRUE(controller.HasLatestTrackOutputFrame());
  const common::output::TrackOutputFrame& latest_track_output_frame =
      controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(latest_track_output_frame.published_track_count, 1U);
  EXPECT_EQ(latest_track_output_frame.confirmed_track_count, 1U);
  EXPECT_FALSE(latest_track_output_frame.contains_lost_tracks);
  ASSERT_EQ(decision_engine.last_frame.tracks.size(), 1U);
  EXPECT_EQ(decision_engine.last_frame.tracks[0].state.status,
            common::model::DecisionTrackStatus::kConfirmed);
  EXPECT_EQ(decision_engine.last_frame.tracks[0].state.association_key,
            latest_track_output_frame.tracks[0].state.association_key);
}

TEST_F(CoreControllerTest, PublicOutputReaderApiExposesLatestTrackOutputFrame) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  core::controller::RadarController controller(radar_context, signal_pipeline, environment_service);

  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());

  controller.RunOnce();

  EXPECT_TRUE(controller.HasLatestTrackOutputFrame());
  const common::output::TrackOutputFrame& latest_track_output_frame =
      controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(latest_track_output_frame.cycle_index, 1U);
  EXPECT_EQ(latest_track_output_frame.batch_id, 1U);
  EXPECT_EQ(latest_track_output_frame.published_track_count, 1U);
  ASSERT_EQ(latest_track_output_frame.tracks.size(), 1U);
  EXPECT_EQ(latest_track_output_frame.tracks[0].state.status,
            common::model::DecisionTrackStatus::kConfirmed);
}

TEST_F(CoreControllerTest, RuntimeValidationErrorsAreExposedAndSkipCommandSubmission) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);
  radar_context.SetCycleDeltaTimeSec(std::numeric_limits<float>::quiet_NaN());

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  core::controller::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  EXPECT_TRUE(controller.HasValidationError());
  const core::context::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(
      std::find_if(issues.begin(), issues.end(), [](const core::context::ValidationIssue& issue) {
        return issue.code == core::context::ValidationCode::kNonFiniteCycleDeltaTime;
      }) != issues.end());
  EXPECT_TRUE(radar_context.SubmittedCommands().empty());
}

TEST_F(CoreControllerTest, NextCycleAppliesPendingControlProfileToSignalPipeline) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service(BuildJammingEnvironmentConfig(12.0f));

  signal::pipeline::SignalPipeline signal_pipeline;

  core::controller::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();
  EXPECT_EQ(signal_pipeline.GetControlProfile().version, 0u);
  const std::uint64_t first_pending_version = radar_context.LatestControlProfile().version;
  EXPECT_GT(first_pending_version, 0u);

  controller.RunOnce();
  EXPECT_EQ(signal_pipeline.GetControlProfile().version, first_pending_version);
}

TEST_F(CoreControllerTest, CustomReducerConfigChangesPendingControlProfile) {
  const common::model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  std::vector<decision::pipeline::TacticalProposal> proposals;
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::control::ControlDirective(
          common::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::control::ControlDirectiveSource::EMISSION_CONTROL),
      60, "reduce power"});
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::control::ControlDirective(
          common::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
          common::control::ControlDirectiveSource::SURVIVABILITY),
      82, "burnthrough"});
  FixedProposalDecisionEngine decision_engine(proposals);

  core::controller::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                               environment_service);

  decision::pipeline::ControlReducerConfig reducer_config;
  reducer_config.lpi_power_scale_on_reduction = 0.60f;
  reducer_config.eccm_burnthrough_gain = 1.8f;
  reducer_config.burnthrough_lpi_power_floor = 0.95f;
  controller.UpdateControlReducerConfig(reducer_config);

  controller.RunOnce();

  EXPECT_FLOAT_EQ(radar_context.LatestControlProfile().eccm_burnthrough_gain, 1.8f);
  EXPECT_FLOAT_EQ(radar_context.LatestControlProfile().lpi_power_scale, 0.95f);
}

}  // namespace tests
}  // namespace airborne_radar
