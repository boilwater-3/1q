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

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/extension/ControlReducerTypes.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/core/SignalPipeline.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace tests {

namespace {

model::TargetFeatureList BuildSingleTarget(float speed, float rcs, bool jamming) {
  (void)jamming;
  model::TargetFeature target(speed, 0.0f, 0.0f, rcs);
  target.has_cartesian_position = true;
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  return model::TargetFeatureList{target};
}

environment::EnvironmentModelConfig BuildJammingEnvironmentConfig(float jammer_power_db) {
  environment::EnvironmentModelConfig config;
  environment::JammerEmitterState jammer;
  jammer.technique = environment::JammingTechnique::kUnknown;
  jammer.power_db = jammer_power_db;
  jammer.confidence = 1.0f;
  config.jammer_sources.push_back(jammer);
  return config;
}

}  // namespace

/// @brief FakeRadarContext 提供最小化的雷达上下文实现。
class FakeRadarContext : public extension::IRadarContext {
 public:
  /// @brief 构造函数，注入固定的输入状态。
  explicit FakeRadarContext(model::TargetFeatureList state) : state_(std::move(state)) {}

  /// @brief 获取当前雷达状态。
  const model::TargetFeatureList& GetTargetFeatures() const override { return state_; }

  void BeginCycle(const session::RadarCycleInput& input) override {
    state_ = input.target_features;
    platform_attitude_deg_.yaw_deg = input.platform_pose.attitude_deg.yaw_deg;
    platform_attitude_deg_.pitch_deg = input.platform_pose.attitude_deg.pitch_deg;
    platform_attitude_deg_.roll_deg = input.platform_pose.attitude_deg.roll_deg;
    cycle_dt_sec_ = input.dt_sec;
    submitted_commands_.clear();
  }

  /// @brief 获取当前搭载平台姿态角。
  model::PlatformAttitudeDeg GetPlatformAttitude() const override { return platform_attitude_deg_; }

  /// @brief 获取当前周期时间步长。
  float GetCycleDeltaTimeSec() const override { return cycle_dt_sec_; }

  /// @brief 收集控制指令。
  void SubmitControlCommand(extension::control::RadarCommand cmd) override {
    submitted_commands_.push_back(cmd);
  }

  const std::vector<extension::control::RadarCommand>& GetSubmittedCommands() const override {
    return submitted_commands_;
  }

  /// @brief 获取已提交的指令集合。
  const std::vector<extension::control::RadarCommand>& SubmittedCommands() const {
    return submitted_commands_;
  }

  /// @brief 记录最新控制真值。
  void UpdateRadarControlProfile(const extension::control::RadarControlProfile& profile) override {
    latest_control_profile_ = profile;
    has_latest_control_profile_ = true;
  }

  bool HasLatestControlProfile() const override { return has_latest_control_profile_; }

  const extension::control::RadarControlProfile& GetLatestControlProfile() const override {
    return latest_control_profile_;
  }

  extension::RadarContextRuntimeState CaptureRuntimeState() const override {
    extension::RadarContextRuntimeState state;
    state.target_features = state_;
    state.platform_attitude_deg = platform_attitude_deg_;
    state.cycle_dt_sec = cycle_dt_sec_;
    state.submitted_commands = submitted_commands_;
    state.latest_control_profile = latest_control_profile_;
    state.has_latest_control_profile = has_latest_control_profile_;
    return state;
  }

  void RestoreRuntimeState(const extension::RadarContextRuntimeState& state) override {
    state_ = state.target_features;
    platform_attitude_deg_ = state.platform_attitude_deg;
    cycle_dt_sec_ = state.cycle_dt_sec;
    submitted_commands_ = state.submitted_commands;
    latest_control_profile_ = state.latest_control_profile;
    has_latest_control_profile_ = state.has_latest_control_profile;
  }

  /// @brief 获取最新控制真值。
  const extension::control::RadarControlProfile& LatestControlProfile() const {
    return latest_control_profile_;
  }

  /// @brief 设置测试上下文使用的平台姿态角。
  void SetPlatformAttitude(const model::PlatformAttitudeDeg& platform_attitude_deg) {
    platform_attitude_deg_ = platform_attitude_deg;
  }

  /// @brief 设置当前周期时间步长。
  void SetCycleDeltaTimeSec(float cycle_dt_sec) { cycle_dt_sec_ = cycle_dt_sec; }

  /// @brief 设置当前周期目标列表。
  void SetTargetFeatures(model::TargetFeatureList state) { state_ = std::move(state); }

 private:
  model::TargetFeatureList state_;
  model::PlatformAttitudeDeg platform_attitude_deg_{};
  float cycle_dt_sec_{1.0f};
  std::vector<extension::control::RadarCommand> submitted_commands_;
  extension::control::RadarControlProfile latest_control_profile_{};
  bool has_latest_control_profile_{false};
};

class FixedDirectiveDecisionEngine : public extension::ITacticalDecisionEngine {
 public:
  explicit FixedDirectiveDecisionEngine(extension::control::ControlDirective directive)
      : directive_(std::move(directive)) {}

  extension::TacticalDecisionResult Evaluate(const model::DecisionInputFrame&,
                                             extension::TacticalStateStore&) override {
    extension::TacticalDecisionResult result;
    result.proposals.push_back(extension::TacticalProposal(directive_, 10, "test directive"));
    return result;
  }

 private:
  extension::control::ControlDirective directive_;
};

class FixedProposalDecisionEngine : public extension::ITacticalDecisionEngine {
 public:
  explicit FixedProposalDecisionEngine(std::vector<extension::TacticalProposal> proposals)
      : proposals_(std::move(proposals)) {}

  extension::TacticalDecisionResult Evaluate(const model::DecisionInputFrame&,
                                             extension::TacticalStateStore&) override {
    extension::TacticalDecisionResult result;
    result.proposals = proposals_;
    return result;
  }

 private:
  std::vector<extension::TacticalProposal> proposals_;
};

class CapturingDecisionEngine : public extension::ITacticalDecisionEngine {
 public:
  extension::TacticalDecisionResult Evaluate(const model::DecisionInputFrame& frame,
                                             extension::TacticalStateStore&) override {
    last_frame = frame;
    ++evaluate_count;
    return extension::TacticalDecisionResult();
  }

  model::DecisionInputFrame last_frame{};
  std::size_t evaluate_count{0U};
};

/// @brief CoreControllerTest 覆盖核心调度与指令下发路径。
class CoreControllerTest : public ::testing::Test {};

class AbortingSignalPipeline : public extension::ISignalPipeline {
 public:
  extension::SignalCycleResult RunCycle(const model::TargetFeatureList&,
                                        const environment::IEnvironmentService&) override {
    extension::SignalCycleResult result;
    result.executed_this_cycle = should_execute_;
    result.abort_reason = should_execute_
                              ? extension::SignalCycleAbortReason::kNone
                              : extension::SignalCycleAbortReason::kRuntimePreparationFailed;
    return result;
  }

  void UpdatePlatformAttitude(const model::PlatformAttitudeDeg& platform_attitude_deg) override {
    platform_attitude_deg_ = platform_attitude_deg;
  }

  model::PlatformAttitudeDeg GetPlatformAttitude() const override { return platform_attitude_deg_; }

  void SetControlProfile(const extension::control::RadarControlProfile& control_profile) override {
    control_profile_ = control_profile;
  }

  extension::control::RadarControlProfile GetControlProfile() const override {
    return control_profile_;
  }

  bool UpdateConfig(const session::RadarSessionConfig& config) override {
    config_ = config;
    return true;
  }

  extension::AssociationQualityMetrics GetLastAssociationQualityMetrics() const override {
    return {};
  }

  void SetShouldExecute(bool should_execute) { should_execute_ = should_execute; }

  extension::SignalPipelineRuntimeState CaptureRuntimeState() const override {
    std::shared_ptr<RuntimeState> state(new RuntimeState());
    state->platform_attitude_deg = platform_attitude_deg_;
    state->control_profile = control_profile_;
    state->config = config_;
    state->should_execute = should_execute_;
    extension::SignalPipelineRuntimeState runtime_state;
    runtime_state.owner_identity = this;
    runtime_state.schema_version = 1U;
    runtime_state.opaque = state;
    return runtime_state;
  }

  void RestoreRuntimeState(const extension::SignalPipelineRuntimeState& state) override {
    if (state.owner_identity != this || state.schema_version != 1U) {
      return;
    }
    const std::shared_ptr<RuntimeState> snapshot =
        std::static_pointer_cast<RuntimeState>(state.opaque);
    if (snapshot == nullptr) {
      return;
    }
    platform_attitude_deg_ = snapshot->platform_attitude_deg;
    control_profile_ = snapshot->control_profile;
    config_ = snapshot->config;
    should_execute_ = snapshot->should_execute;
  }

 private:
  struct RuntimeState {
    model::PlatformAttitudeDeg platform_attitude_deg{};
    extension::control::RadarControlProfile control_profile{};
    session::RadarSessionConfig config{};
    bool should_execute{false};
  };

  model::PlatformAttitudeDeg platform_attitude_deg_{};
  extension::control::RadarControlProfile control_profile_{};
  session::RadarSessionConfig config_{};
  bool should_execute_{false};
};

TEST_F(CoreControllerTest, RunOnceSubmitsCommands) {
  const model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service(BuildJammingEnvironmentConfig(12.0f));

  signal::pipeline::SignalPipeline signal_pipeline;

  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  const auto& cmds = radar_context.SubmittedCommands();
  EXPECT_GT(cmds.size(), 0u);
}

TEST_F(CoreControllerTest, PushesPlatformAttitudeIntoSignalPipelineBeforeRun) {
  const model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);
  model::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 18.0f;
  platform_attitude_deg.pitch_deg = -4.0f;
  platform_attitude_deg.roll_deg = 2.0f;
  radar_context.SetPlatformAttitude(platform_attitude_deg);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  const model::PlatformAttitudeDeg cached_platform_attitude = signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 18.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -4.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 2.0f);
}

TEST_F(CoreControllerTest, ReusesFrozenEnvironmentSnapshotAcrossSignalAndDecision) {
  const model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;
  environment::JammerEmitterState jammer_source;
  jammer_source.technique = environment::JammingTechnique::kDeception;
  jammer_source.power_db = 10.0f;
  jammer_source.confidence = 1.0f;
  jammer_source.js_db = 7.0f;
  jammer_source.has_direction_deg = true;
  jammer_source.azimuth_deg = 0.0f;
  jammer_source.elevation_deg = 1.0f;
  jammer_source.angular_span_deg = 5.0f;
  environment_service.UpdateSceneState(
      environment::EnvironmentSceneBuilder().AddJammer(jammer_source).Build());

  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
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
  EXPECT_FLOAT_EQ(
      decision_engine.last_frame.eccm_source_info.jammer_sources.front().jammer_power_db,
      frozen_snapshot.jammer_sources.front().power_db);
  ASSERT_EQ(measurements.size(), 1U);
  EXPECT_EQ(measurements[0].filtered_feature.jamming_detected, frozen_snapshot.jamming_detected);
}

TEST_F(CoreControllerTest, AppliesUpdatedSceneOnNextControllerCycle) {
  const model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);

  controller.RunOnce();
  EXPECT_FALSE(decision_engine.last_frame.environment_jamming_detected);

  environment::JammerEmitterState jammer_source;
  jammer_source.technique = environment::JammingTechnique::kNoiseSuppression;
  jammer_source.power_db = 9.0f;
  jammer_source.confidence = 1.0f;
  jammer_source.has_direction_deg = true;
  jammer_source.azimuth_deg = 22.0f;
  jammer_source.elevation_deg = 0.0f;
  jammer_source.angular_span_deg = 12.0f;
  environment_service.UpdateSceneState(
      environment::EnvironmentSceneBuilder().AddJammer(jammer_source).Build());

  EXPECT_FALSE(environment_service.SampleEnvironment().jamming_detected);

  controller.RunOnce();

  EXPECT_TRUE(decision_engine.last_frame.environment_jamming_detected);
  EXPECT_TRUE(environment_service.SampleEnvironment().jamming_detected);
}

TEST_F(CoreControllerTest, MapsMultiSourceJammingFactsIntoDecisionFrame) {
  const model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  environment::JammerEmitterState deception_emitter;
  deception_emitter.technique = environment::JammingTechnique::kDeception;
  deception_emitter.power_db = 9.0f;
  deception_emitter.js_db = 7.5f;
  deception_emitter.has_direction_deg = true;
  deception_emitter.azimuth_deg = 30.0f;
  deception_emitter.elevation_deg = 5.0f;
  deception_emitter.angular_span_deg = 8.0f;
  deception_emitter.confidence = 0.95f;
  env_config.jammer_sources.push_back(deception_emitter);
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);

  controller.RunOnce();

  const environment::EnvironmentSnapshot frozen_snapshot = environment_service.SampleEnvironment();
  ASSERT_EQ(decision_engine.evaluate_count, 1u);
  EXPECT_TRUE(decision_engine.last_frame.environment_jamming_detected);
  ASSERT_EQ(decision_engine.last_frame.eccm_source_info.jammer_sources.size(), 1u);
  ASSERT_EQ(frozen_snapshot.jammer_sources.size(), 1u);
  const model::EccmJammerSourceInfo& mapped_source =
      decision_engine.last_frame.eccm_source_info.jammer_sources.front();
  const environment::JammerSourceFact& frozen_source = frozen_snapshot.jammer_sources.front();
  EXPECT_EQ(mapped_source.technique, model::JammingTechnique::kDeception);
  EXPECT_FLOAT_EQ(mapped_source.jammer_power_db, 9.0f);
  EXPECT_FLOAT_EQ(mapped_source.jammer_to_signal_db, 7.5f);
  EXPECT_FLOAT_EQ(mapped_source.frequency_overlap_ratio, frozen_source.frequency_overlap_ratio);
  EXPECT_FLOAT_EQ(mapped_source.prf_lock_risk, frozen_source.prf_lock_risk);
  EXPECT_FLOAT_EQ(mapped_source.direction_deg.azimuth_deg, 30.0f);
  EXPECT_FLOAT_EQ(mapped_source.angular_span_deg, 8.0f);
  EXPECT_EQ(decision_engine.last_frame.association_quality_info.dominant_jamming_semantic,
            model::JammingSemantic::kDeception);
  EXPECT_GT(decision_engine.last_frame.association_quality_info.jamming_severity, 0.0f);
  EXPECT_GT(decision_engine.last_frame.association_quality_info.association_stress, 0.0f);
  EXPECT_EQ(decision_engine.last_frame.perception_quality_info.input_target_count, 1u);
  EXPECT_EQ(decision_engine.last_frame.perception_quality_info.detection_count, 1u);
  EXPECT_FLOAT_EQ(decision_engine.last_frame.perception_quality_info.detection_rate, 1.0f);
  EXPECT_FLOAT_EQ(decision_engine.last_frame.perception_quality_info.detection_stress, 0.0f);
}

TEST_F(CoreControllerTest, DefaultLifecyclePathBuildsTentativeDecisionFrameOnFirstCycle) {
  const model::TargetFeatureList input_state = BuildSingleTarget(640.0f, 1.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;

  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);

  controller.RunOnce();

  EXPECT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame& latest_track_output_frame =
      controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(latest_track_output_frame.published_track_count, 1U);
  EXPECT_EQ(latest_track_output_frame.confirmed_track_count, 0U);
  EXPECT_FALSE(latest_track_output_frame.contains_lost_tracks);
  ASSERT_EQ(decision_engine.last_frame.tracks.size(), 1U);
  EXPECT_EQ(decision_engine.last_frame.tracks[0].state.status,
            model::DecisionTrackStatus::kTentative);
  EXPECT_EQ(decision_engine.last_frame.tracks[0].state.association_key,
            latest_track_output_frame.tracks[0].state.association_key);
}

TEST_F(CoreControllerTest, PublicOutputReaderApiExposesLatestTrackOutputFrame) {
  const model::TargetFeatureList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());

  controller.RunOnce();

  EXPECT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame& latest_track_output_frame =
      controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(latest_track_output_frame.cycle_index, 1U);
  EXPECT_EQ(latest_track_output_frame.batch_id, 1U);
  EXPECT_EQ(latest_track_output_frame.published_track_count, 1U);
  ASSERT_EQ(latest_track_output_frame.tracks.size(), 1U);
  EXPECT_EQ(latest_track_output_frame.tracks[0].state.status,
            model::DecisionTrackStatus::kTentative);
}

TEST_F(CoreControllerTest, RuntimeValidationErrorsAreExposedAndSkipCommandSubmission) {
  const model::TargetFeatureList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);
  radar_context.SetCycleDeltaTimeSec(std::numeric_limits<float>::quiet_NaN());

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  EXPECT_TRUE(controller.HasValidationError());
  const session::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(std::find_if(issues.begin(), issues.end(), [](const session::ValidationIssue& issue) {
                return issue.code == session::ValidationCode::kNonFiniteCycleDeltaTime;
              }) != issues.end());
  EXPECT_TRUE(radar_context.SubmittedCommands().empty());
  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());
}

TEST_F(CoreControllerTest, FirstCycleSignalPipelineAbortDoesNotPublishSyntheticLatestFrame) {
  const model::TargetFeatureList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);
  environment::EnvironmentService environment_service;
  AbortingSignalPipeline signal_pipeline;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  EXPECT_FALSE(controller.HasValidationError());
  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());
}

TEST_F(CoreControllerTest, FailedCycleDoesNotAdvanceEnvironmentStampOrOutputSequence) {
  const model::TargetFeatureList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);
  environment::EnvironmentService environment_service;
  AbortingSignalPipeline signal_pipeline;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  EXPECT_FLOAT_EQ(environment_service.SampleEnvironment().cycle_dt_sec, 0.0f);

  controller.RunOnce();

  EXPECT_FALSE(controller.ExecutedLatestCycle());
  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());
  EXPECT_FLOAT_EQ(environment_service.SampleEnvironment().cycle_dt_sec, 0.0f);

  signal_pipeline.SetShouldExecute(true);
  controller.RunOnce();

  EXPECT_TRUE(controller.ExecutedLatestCycle());
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame& latest_track_output_frame =
      controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(latest_track_output_frame.cycle_index, 1U);
  EXPECT_EQ(latest_track_output_frame.batch_id, 1U);
}

TEST_F(CoreControllerTest, InvalidDeltaTimeRetainsPreviousValidOutputFrame) {
  const model::TargetFeatureList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  const session::RadarSessionConfig session_config =
      config::presets::MakeDetectionMissionRadarSessionConfig();
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame previous_frame = controller.GetLatestTrackOutputFrame();
  ASSERT_GT(previous_frame.published_track_count, 0U);
  const std::vector<extension::control::RadarCommand> previous_commands =
      radar_context.SubmittedCommands();

  radar_context.SetCycleDeltaTimeSec(0.0f);
  controller.RunOnce();

  EXPECT_TRUE(controller.HasValidationError());
  const session::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(std::find_if(issues.begin(), issues.end(), [](const session::ValidationIssue& issue) {
                return issue.code == session::ValidationCode::kInvalidCycleDeltaTime;
              }) != issues.end());
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame& retained_frame = controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(retained_frame.cycle_index, previous_frame.cycle_index);
  EXPECT_EQ(retained_frame.batch_id, previous_frame.batch_id);
  EXPECT_EQ(retained_frame.published_track_count, previous_frame.published_track_count);
  EXPECT_EQ(retained_frame.confirmed_track_count, previous_frame.confirmed_track_count);
  ASSERT_EQ(radar_context.SubmittedCommands().size(), previous_commands.size());
  for (std::size_t i = 0; i < previous_commands.size(); ++i) {
    EXPECT_EQ(radar_context.SubmittedCommands()[i].type, previous_commands[i].type);
    EXPECT_EQ(radar_context.SubmittedCommands()[i].source, previous_commands[i].source);
  }
}

TEST_F(CoreControllerTest, DuplicateExternalTargetIdRetainsPreviousValidOutputFrame) {
  const model::TargetFeatureList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  const session::RadarSessionConfig session_config =
      config::presets::MakeDetectionMissionRadarSessionConfig();
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame previous_frame = controller.GetLatestTrackOutputFrame();
  ASSERT_GT(previous_frame.published_track_count, 0U);
  const std::vector<extension::control::RadarCommand> previous_commands =
      radar_context.SubmittedCommands();

  model::TargetFeature duplicate_a = input_state.front();
  duplicate_a.external_target_id = 42U;
  model::TargetFeature duplicate_b = duplicate_a;
  duplicate_b.position_x += 50.0f;
  duplicate_b.range_m += 50.0f;
  radar_context.SetTargetFeatures(model::TargetFeatureList{duplicate_a, duplicate_b});

  controller.RunOnce();

  EXPECT_TRUE(controller.HasValidationError());
  const session::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(std::find_if(issues.begin(), issues.end(), [](const session::ValidationIssue& issue) {
                return issue.code == session::ValidationCode::kDuplicateExternalTargetId;
              }) != issues.end());
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame& retained_frame = controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(retained_frame.cycle_index, previous_frame.cycle_index);
  EXPECT_EQ(retained_frame.batch_id, previous_frame.batch_id);
  EXPECT_EQ(retained_frame.published_track_count, previous_frame.published_track_count);
  EXPECT_EQ(retained_frame.confirmed_track_count, previous_frame.confirmed_track_count);
  ASSERT_EQ(radar_context.SubmittedCommands().size(), previous_commands.size());
  for (std::size_t i = 0; i < previous_commands.size(); ++i) {
    EXPECT_EQ(radar_context.SubmittedCommands()[i].type, previous_commands[i].type);
    EXPECT_EQ(radar_context.SubmittedCommands()[i].source, previous_commands[i].source);
  }
}

TEST_F(CoreControllerTest, NextCycleAppliesPendingControlProfileToSignalPipeline) {
  const model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service(BuildJammingEnvironmentConfig(12.0f));

  signal::pipeline::SignalPipeline signal_pipeline;

  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();
  EXPECT_EQ(signal_pipeline.GetControlProfile().version, 0u);
  const std::uint64_t first_pending_version = radar_context.LatestControlProfile().version;
  EXPECT_GT(first_pending_version, 0u);

  controller.RunOnce();
  EXPECT_EQ(signal_pipeline.GetControlProfile().version, first_pending_version);
}

TEST_F(CoreControllerTest, CustomReducerConfigChangesPendingControlProfile) {
  const model::TargetFeatureList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  std::vector<extension::TacticalProposal> proposals;
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(
          extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          extension::control::ControlDirectiveSource::EMISSION_CONTROL),
      60, "reduce power"});
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(
          extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
          extension::control::ControlDirectiveSource::SURVIVABILITY),
      82, "burnthrough"});
  FixedProposalDecisionEngine decision_engine(proposals);

  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);

  extension::ControlReducerConfig reducer_config;
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
