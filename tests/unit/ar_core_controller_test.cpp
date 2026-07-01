// Copyright 2026. All Rights Reserved.
//
// @file core_controller_test.cpp
// @brief 验证核心调度器使用桩组件的最小集成流程。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/session/RadarCommand.h"
#include "1q/airborne_radar/session/RadarControlProfile.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/runtime/RadarController.h"
#include "airborne_radar/session/MutableRadarContext.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace tests {

namespace {

session::RadarSceneTargetList BuildSingleTarget(float speed, float rcs, bool jamming) {
  (void)jamming;
  session::RadarSceneTarget target(speed, 0.0f, 0.0f, rcs);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  return session::RadarSceneTargetList{target};
}

config::RadarSessionConfig MakeDetectionFocusedConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(config::profiles::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

config::EnvironmentModelConfig BuildJammingEnvironmentConfig(float jammer_power_db) {
  config::EnvironmentModelConfig config;
  config::JammerEmitterState jammer;
  jammer.technique = config::JammingTechnique::kUnknown;
  jammer.power_db = jammer_power_db;
  jammer.confidence = 1.0f;
  config.jammer_sources.push_back(jammer);
  return config;
}

}  // namespace

using FakeRadarContext = session::MutableRadarContext;

class FixedDirectiveDecisionEngine : public session::ITacticalDecisionEngine {
 public:
  explicit FixedDirectiveDecisionEngine(session::ControlDirective directive)
      : directive_(std::move(directive)) {}

  session::TacticalDecisionResult Evaluate(const session::DecisionInputFrame&,
                                           session::TacticalStateStore&) override {
    session::TacticalDecisionResult result;
    result.proposals.push_back(session::TacticalProposal(directive_, 10, "test directive"));
    return result;
  }

 private:
  session::ControlDirective directive_;
};

class FixedProposalDecisionEngine : public session::ITacticalDecisionEngine {
 public:
  explicit FixedProposalDecisionEngine(std::vector<session::TacticalProposal> proposals)
      : proposals_(std::move(proposals)) {}

  session::TacticalDecisionResult Evaluate(const session::DecisionInputFrame&,
                                           session::TacticalStateStore&) override {
    session::TacticalDecisionResult result;
    result.proposals = proposals_;
    return result;
  }

 private:
  std::vector<session::TacticalProposal> proposals_;
};

class CapturingDecisionEngine : public session::ITacticalDecisionEngine {
 public:
  session::TacticalDecisionResult Evaluate(const session::DecisionInputFrame& frame,
                                           session::TacticalStateStore&) override {
    last_frame = frame;
    ++evaluate_count;
    return session::TacticalDecisionResult();
  }

  session::DecisionInputFrame last_frame{};
  std::size_t evaluate_count{0U};
};

/// @brief CoreControllerTest 覆盖核心调度与指令下发路径。
class CoreControllerTest : public ::testing::Test {};

class AbortingSignalPipeline : public signal::ISignalPipeline {
 public:
  session::SignalCycleResult RunCycle(const session::RadarSceneTargetList&,
                                      const environment::IEnvironmentService&) override {
    session::SignalCycleResult result;
    result.executed_this_cycle = should_execute_;
    result.abort_reason = should_execute_
                              ? session::SignalCycleAbortReason::kNone
                              : session::SignalCycleAbortReason::kRuntimePreparationFailed;
    return result;
  }

  void UpdatePlatformAttitude(const config::PlatformAttitudeDeg& platform_attitude_deg) override {
    platform_attitude_deg_ = platform_attitude_deg;
  }

  config::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_deg_;
  }

  void SetControlProfile(const session::RadarControlProfile& control_profile) override {
    control_profile_ = control_profile;
  }

  session::RadarControlProfile GetControlProfile() const override { return control_profile_; }

  bool UpdateConfig(const config::RadarSessionConfig& config) override {
    config_ = config;
    return true;
  }

  session::AssociationQualityMetrics GetLastAssociationQualityMetrics() const override {
    return {};
  }

  void SetShouldExecute(bool should_execute) { should_execute_ = should_execute; }

  signal::SignalPipelineRuntimeState CaptureRuntimeState() const override {
    std::shared_ptr<RuntimeState> state(new RuntimeState());
    state->platform_attitude_deg = platform_attitude_deg_;
    state->control_profile = control_profile_;
    state->config = config_;
    state->should_execute = should_execute_;
    signal::SignalPipelineRuntimeState runtime_state;
    runtime_state.owner_identity = this;
    runtime_state.schema_version = 1U;
    runtime_state.opaque = state;
    return runtime_state;
  }

  void RestoreRuntimeState(const signal::SignalPipelineRuntimeState& state) override {
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
    config::PlatformAttitudeDeg platform_attitude_deg{};
    session::RadarControlProfile control_profile{};
    config::RadarSessionConfig config{};
    bool should_execute{false};
  };

  config::PlatformAttitudeDeg platform_attitude_deg_{};
  session::RadarControlProfile control_profile_{};
  config::RadarSessionConfig config_{};
  bool should_execute_{false};
};

TEST_F(CoreControllerTest, RunOnceSubmitsCommands) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service(BuildJammingEnvironmentConfig(12.0f));

  signal::pipeline::SignalPipeline signal_pipeline;

  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  const auto& cmds = radar_context.SubmittedCommands();
  EXPECT_GT(cmds.size(), 0u);
}

TEST_F(CoreControllerTest, PushesPlatformAttitudeIntoSignalPipelineBeforeRun) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);
  config::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 18.0f;
  platform_attitude_deg.pitch_deg = -4.0f;
  platform_attitude_deg.roll_deg = 2.0f;
  radar_context.SetPlatformAttitude(platform_attitude_deg);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  const config::PlatformAttitudeDeg cached_platform_attitude =
      signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 18.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -4.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 2.0f);
}

TEST_F(CoreControllerTest, ReusesFrozenEnvironmentSnapshotAcrossSignalAndDecision) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;
  config::JammerEmitterState jammer_source;
  jammer_source.technique = config::JammingTechnique::kDeception;
  jammer_source.power_db = 10.0f;
  jammer_source.confidence = 1.0f;
  jammer_source.js_db = 7.0f;
  jammer_source.position_x = 0.0f;
  jammer_source.position_y = 10000.0f;
  jammer_source.position_z = 174.55f;
  jammer_source.angular_span_deg = 5.0f;
  environment_service.UpdateSceneState([&] {
    session::EnvironmentSceneState s;
    s.jammer_emitters.push_back(jammer_source);
    return s;
  }());

  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);

  controller.RunOnce();

  const session::EnvironmentSnapshot frozen_snapshot = environment_service.SampleEnvironment();
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
  const session::RadarSceneTargetList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);

  controller.RunOnce();
  EXPECT_FALSE(decision_engine.last_frame.environment_jamming_detected);

  config::JammerEmitterState jammer_source;
  jammer_source.technique = config::JammingTechnique::kNoiseSuppression;
  jammer_source.power_db = 9.0f;
  jammer_source.confidence = 1.0f;
  jammer_source.position_x = 3746.07f;  // range 10000m * sin(22 deg)
  jammer_source.position_y = 9271.84f;  // range 10000m * cos(22 deg)
  jammer_source.position_z = 0.0f;      // elevation 0 deg
  jammer_source.angular_span_deg = 12.0f;
  environment_service.UpdateSceneState([&] {
    session::EnvironmentSceneState s;
    s.jammer_emitters.push_back(jammer_source);
    return s;
  }());

  EXPECT_FALSE(environment_service.SampleEnvironment().jamming_detected);

  controller.RunOnce();

  EXPECT_TRUE(decision_engine.last_frame.environment_jamming_detected);
  EXPECT_TRUE(environment_service.SampleEnvironment().jamming_detected);
}

TEST_F(CoreControllerTest, MapsMultiSourceJammingFactsIntoDecisionFrame) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  config::EnvironmentModelConfig env_config;
  config::JammerEmitterState deception_emitter;
  deception_emitter.technique = config::JammingTechnique::kDeception;
  deception_emitter.power_db = 9.0f;
  deception_emitter.js_db = 7.5f;
  deception_emitter.position_x = 5000.0f;   // range 10000m * sin(30 deg)
  deception_emitter.position_y = 8660.25f;  // range 10000m * cos(30 deg)
  deception_emitter.position_z = 874.887f;  // range 10000m * tan(5 deg)
  deception_emitter.angular_span_deg = 8.0f;
  deception_emitter.confidence = 0.95f;
  env_config.jammer_sources.push_back(deception_emitter);
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);

  controller.RunOnce();

  const session::EnvironmentSnapshot frozen_snapshot = environment_service.SampleEnvironment();
  ASSERT_EQ(decision_engine.evaluate_count, 1u);
  EXPECT_TRUE(decision_engine.last_frame.environment_jamming_detected);
  ASSERT_EQ(decision_engine.last_frame.eccm_source_info.jammer_sources.size(), 1u);
  ASSERT_EQ(frozen_snapshot.jammer_sources.size(), 1u);
  const session::EccmJammerSourceInfo& mapped_source =
      decision_engine.last_frame.eccm_source_info.jammer_sources.front();
  const session::JammerSourceFact& frozen_source = frozen_snapshot.jammer_sources.front();
  EXPECT_EQ(mapped_source.technique, session::JammingTechnique::kDeception);
  EXPECT_FLOAT_EQ(mapped_source.jammer_power_db, 9.0f);
  EXPECT_FLOAT_EQ(mapped_source.jammer_to_signal_db, 7.5f);
  EXPECT_FLOAT_EQ(mapped_source.frequency_overlap_ratio, frozen_source.frequency_overlap_ratio);
  EXPECT_FLOAT_EQ(mapped_source.prf_lock_risk, frozen_source.prf_lock_risk);
  EXPECT_NEAR(mapped_source.direction_deg.azimuth_deg, 30.0f, 1e-4f);
  EXPECT_FLOAT_EQ(mapped_source.angular_span_deg, 8.0f);
  EXPECT_EQ(decision_engine.last_frame.association_quality_info.dominant_jamming_semantic,
            config::JammingSemantic::kDeception);
  EXPECT_GT(decision_engine.last_frame.association_quality_info.jamming_severity, 0.0f);
  EXPECT_GT(decision_engine.last_frame.association_quality_info.association_stress, 0.0f);
  EXPECT_EQ(decision_engine.last_frame.perception_quality_info.input_target_count, 1u);
  EXPECT_EQ(decision_engine.last_frame.perception_quality_info.detection_count, 1u);
  EXPECT_FLOAT_EQ(decision_engine.last_frame.perception_quality_info.detection_rate, 1.0f);
  EXPECT_FLOAT_EQ(decision_engine.last_frame.perception_quality_info.detection_stress, 0.0f);
}

TEST_F(CoreControllerTest, DefaultLifecyclePathBuildsTentativeDecisionFrameOnFirstCycle) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(640.0f, 1.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  CapturingDecisionEngine decision_engine;

  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);

  controller.RunOnce();

  EXPECT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame& latest_track_output_frame =
      controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(latest_track_output_frame.tracks.size(), 1U);
  EXPECT_EQ(
      session::CountTracksByStatus(latest_track_output_frame, session::TrackStatus::kConfirmed),
      0U);
  EXPECT_FALSE(
      session::CountTracksByStatus(latest_track_output_frame, session::TrackStatus::kLost) > 0U);
  ASSERT_EQ(decision_engine.last_frame.tracks.size(), 1U);
  EXPECT_EQ(decision_engine.last_frame.tracks[0].status, session::TrackStatus::kTentative);
  EXPECT_EQ(decision_engine.last_frame.tracks[0].association_key,
            latest_track_output_frame.tracks[0].association_key);
}

TEST_F(CoreControllerTest, PublicOutputReaderApiExposesLatestTrackOutputFrame) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());

  controller.RunOnce();

  EXPECT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame& latest_track_output_frame =
      controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(latest_track_output_frame.cycle_index, 1U);
  EXPECT_EQ(latest_track_output_frame.batch_id, 1U);
  EXPECT_EQ(latest_track_output_frame.tracks.size(), 1U);
  ASSERT_EQ(latest_track_output_frame.tracks.size(), 1U);
  EXPECT_EQ(latest_track_output_frame.tracks[0].status, session::TrackStatus::kTentative);
}

TEST_F(CoreControllerTest, RuntimeValidationErrorsAreExposedAndSkipCommandSubmission) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
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
  const session::RadarSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);
  environment::EnvironmentService environment_service;
  AbortingSignalPipeline signal_pipeline;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  EXPECT_FALSE(controller.HasValidationError());
  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());
}

TEST_F(CoreControllerTest, InvalidDeltaTimeRetainsPreviousValidOutputFrame) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  const config::RadarSessionConfig session_config = MakeDetectionFocusedConfig();
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame previous_frame = controller.GetLatestTrackOutputFrame();
  ASSERT_GT(previous_frame.tracks.size(), 0U);
  const std::vector<session::RadarCommand> previous_commands = radar_context.SubmittedCommands();

  radar_context.SetCycleDeltaTimeSec(0.0f);
  controller.RunOnce();

  EXPECT_TRUE(controller.HasValidationError());
  const session::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(std::find_if(issues.begin(), issues.end(), [](const session::ValidationIssue& issue) {
                return issue.code == session::ValidationCode::kInvalidCycleDeltaTime;
              }) != issues.end());
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame& retained_frame = controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(retained_frame.cycle_index, previous_frame.cycle_index);
  EXPECT_EQ(retained_frame.batch_id, previous_frame.batch_id);
  EXPECT_EQ(retained_frame.tracks.size(), previous_frame.tracks.size());
  EXPECT_EQ(session::CountTracksByStatus(retained_frame, session::TrackStatus::kConfirmed),
            session::CountTracksByStatus(previous_frame, session::TrackStatus::kConfirmed));
  ASSERT_EQ(radar_context.SubmittedCommands().size(), previous_commands.size());
  for (std::size_t i = 0; i < previous_commands.size(); ++i) {
    EXPECT_EQ(radar_context.SubmittedCommands()[i].type, previous_commands[i].type);
    EXPECT_EQ(radar_context.SubmittedCommands()[i].source, previous_commands[i].source);
  }
}

TEST_F(CoreControllerTest, DuplicateExternalTargetIdRetainsPreviousValidOutputFrame) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  const config::RadarSessionConfig session_config = MakeDetectionFocusedConfig();
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce();
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame previous_frame = controller.GetLatestTrackOutputFrame();
  ASSERT_GT(previous_frame.tracks.size(), 0U);
  const std::vector<session::RadarCommand> previous_commands = radar_context.SubmittedCommands();

  session::RadarSceneTarget duplicate_a = input_state.front();
  duplicate_a.external_target_id = 42U;
  session::RadarSceneTarget duplicate_b = duplicate_a;
  duplicate_b.position_x += 50.0f;
  duplicate_b.range_m += 50.0f;
  radar_context.SetSceneTargets(session::RadarSceneTargetList{duplicate_a, duplicate_b});

  controller.RunOnce();

  EXPECT_TRUE(controller.HasValidationError());
  const session::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(std::find_if(issues.begin(), issues.end(), [](const session::ValidationIssue& issue) {
                return issue.code == session::ValidationCode::kDuplicateExternalTargetId;
              }) != issues.end());
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame& retained_frame = controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(retained_frame.cycle_index, previous_frame.cycle_index);
  EXPECT_EQ(retained_frame.batch_id, previous_frame.batch_id);
  EXPECT_EQ(retained_frame.tracks.size(), previous_frame.tracks.size());
  EXPECT_EQ(session::CountTracksByStatus(retained_frame, session::TrackStatus::kConfirmed),
            session::CountTracksByStatus(previous_frame, session::TrackStatus::kConfirmed));
  ASSERT_EQ(radar_context.SubmittedCommands().size(), previous_commands.size());
  for (std::size_t i = 0; i < previous_commands.size(); ++i) {
    EXPECT_EQ(radar_context.SubmittedCommands()[i].type, previous_commands[i].type);
    EXPECT_EQ(radar_context.SubmittedCommands()[i].source, previous_commands[i].source);
  }
}

TEST_F(CoreControllerTest, NextCycleAppliesPendingControlProfileToSignalPipeline) {
  const session::RadarSceneTargetList input_state = BuildSingleTarget(800.0f, 2.5f, false);
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

}  // namespace tests
}  // namespace airborne_radar
