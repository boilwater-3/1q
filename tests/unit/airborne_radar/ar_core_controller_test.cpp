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

#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/runtime/ArController.h"
#include "airborne_radar/session/MutableArContext.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/pipeline/SignalCycleInput.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace tests {

namespace {

session::ArSceneTargetList BuildSingleTarget(float speed, float rcs, bool jamming) {
  (void)jamming;
  session::ArSceneTarget target(speed, 0.0f, 0.0f, rcs);

  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  return session::ArSceneTargetList{target};
}

config::ArSessionConfig MakeDetectionFocusedConfig() {
  return config::ArSessionConfigBuilder()
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

}  // namespace

using FakeRadarContext = session::MutableArContext;

/// @brief CoreControllerTest 覆盖核心调度与指令下发路径。
class CoreControllerTest : public ::testing::Test {};

class AbortingSignalPipeline : public signal::ISignalPipeline {
 public:
  session::SignalCycleResult RunCycle(
      const signal::pipeline::SignalCycleInput&,
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

  void SetControlProfile(const session::ArControlProfile& control_profile) override {
    control_profile_ = control_profile;
  }

  session::ArControlProfile GetControlProfile() const override { return control_profile_; }

  bool UpdateConfig(const config::ArSessionConfig& config) override {
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
    session::ArControlProfile control_profile{};
    config::ArSessionConfig config{};
    bool should_execute{false};
  };

  config::PlatformAttitudeDeg platform_attitude_deg_{};
  session::ArControlProfile control_profile_{};
  config::ArSessionConfig config_{};
  bool should_execute_{false};
};

TEST_F(CoreControllerTest, PushesPlatformAttitudeIntoSignalPipelineBeforeRun) {
  const session::ArSceneTargetList input_state = BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);
  config::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 18.0f;
  platform_attitude_deg.pitch_deg = -4.0f;
  platform_attitude_deg.roll_deg = 2.0f;
  radar_context.SetPlatformAttitude(platform_attitude_deg);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  const config::PlatformAttitudeDeg cached_platform_attitude =
      signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 18.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -4.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 2.0f);
}

TEST_F(CoreControllerTest, DefaultLifecyclePathBuildsTentativeDecisionFrameOnFirstCycle) {
  const session::ArSceneTargetList input_state = BuildSingleTarget(640.0f, 1.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  EXPECT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame& latest_track_output_frame =
      controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(latest_track_output_frame.tracks.size(), 1U);
  EXPECT_EQ(
      session::CountTracksByStatus(latest_track_output_frame, session::TrackStatus::kConfirmed),
      0U);
  EXPECT_FALSE(
      session::CountTracksByStatus(latest_track_output_frame, session::TrackStatus::kLost) > 0U);
  const session::DecisionInputFrame& decision_frame =
      controller.GetLatestDecisionObservation().input_frame;
  ASSERT_EQ(decision_frame.tracks.size(), 1U);
  EXPECT_EQ(decision_frame.tracks[0].status, session::TrackStatus::kTentative);
  EXPECT_EQ(decision_frame.tracks[0].association_key,
            latest_track_output_frame.tracks[0].association_key);
}

TEST_F(CoreControllerTest, PublicOutputReaderApiExposesLatestTrackOutputFrame) {
  const session::ArSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

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
  const session::ArSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);
  radar_context.SetCycleDeltaTimeSec(std::numeric_limits<float>::quiet_NaN());

  environment::EnvironmentService environment_service;

  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  EXPECT_TRUE(controller.HasValidationError());
  const session::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(std::find_if(issues.begin(), issues.end(), [](const session::ValidationIssue& issue) {
                return issue.code == session::ValidationCode::kNonFiniteCycleDeltaTime;
              }) != issues.end());
  EXPECT_TRUE(radar_context.SubmittedCommands().empty());
  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());
}

TEST_F(CoreControllerTest, FirstCycleSignalPipelineAbortDoesNotPublishSyntheticLatestFrame) {
  const session::ArSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);
  environment::EnvironmentService environment_service;
  AbortingSignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  EXPECT_FALSE(controller.HasValidationError());
  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());
}

TEST_F(CoreControllerTest, InvalidDeltaTimeRetainsPreviousValidOutputFrame) {
  const session::ArSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  const config::ArSessionConfig session_config = MakeDetectionFocusedConfig();
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame previous_frame = controller.GetLatestTrackOutputFrame();
  ASSERT_GT(previous_frame.tracks.size(), 0U);
  const std::vector<session::ArCommand> previous_commands = radar_context.SubmittedCommands();

  radar_context.SetCycleDeltaTimeSec(0.0f);
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

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
  const session::ArSceneTargetList input_state = BuildSingleTarget(510.0f, 1.0f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentService environment_service;

  const config::ArSessionConfig session_config = MakeDetectionFocusedConfig();
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame previous_frame = controller.GetLatestTrackOutputFrame();
  ASSERT_GT(previous_frame.tracks.size(), 0U);
  const std::vector<session::ArCommand> previous_commands = radar_context.SubmittedCommands();

  session::ArSceneTarget duplicate_a = input_state.front();
  duplicate_a.external_target_id = 42U;
  session::ArSceneTarget duplicate_b = duplicate_a;
  duplicate_b.position_x += 50.0f;
  duplicate_b.range_m += 50.0f;
  radar_context.SetSceneTargets(session::ArSceneTargetList{duplicate_a, duplicate_b});

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

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

TEST_F(CoreControllerTest, MatchingExternalResponseReplacesInternalBaseline) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const session::DecisionInputFrame frame = controller.GetLatestDecisionObservation().input_frame;
  session::ExternalDecisionResponse response;
  response.source_cycle_index = frame.cycle_index;
  response.source_batch_id = frame.batch_id;
  response.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.4f),
      60, "external power"});
  EXPECT_EQ(controller.SubmitExternalDecision(response),
            session::ExternalDecisionSubmitStatus::kAccepted);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);
  EXPECT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 0.4f);
}

TEST_F(CoreControllerTest, MatchingExternalResponseCanBeConsumedBeforeEmission) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const session::DecisionInputFrame frame = controller.GetLatestDecisionObservation().input_frame;
  session::ExternalDecisionResponse response;
  response.source_cycle_index = frame.cycle_index;
  response.source_batch_id = frame.batch_id;
  response.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                session::ControlDirectiveSource::SURVIVABILITY),
      60, "external agility"});
  ASSERT_EQ(controller.SubmitExternalDecision(response),
            session::ExternalDecisionSubmitStatus::kAccepted);

  ASSERT_TRUE(controller.PrepareEmissionControl());
  EXPECT_TRUE(controller.GetControlProfile().enable_agility_frequency);
  EXPECT_TRUE(signal_pipeline.GetControlProfile().enable_agility_frequency);
  EXPECT_FALSE(controller.PrepareEmissionControl());

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);
}

TEST_F(CoreControllerTest, PublicDecisionControlConfigEnablesHoldWindow) {
  const session::ArSceneTargetList empty_scene;
  FakeRadarContext radar_context(empty_scene);
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  config::DecisionControlConfig decision_control_config;
  decision_control_config.lpi_hold_cycles_after_request = 1U;
  extension::ArController controller(radar_context, signal_pipeline, environment_service,
                                     decision_control_config);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  session::DecisionInputFrame frame = controller.GetLatestDecisionObservation().input_frame;
  session::ExternalDecisionResponse request;
  request.source_cycle_index = frame.cycle_index;
  request.source_batch_id = frame.batch_id;
  request.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.5f),
      80, "public hold configuration"});
  ASSERT_EQ(controller.SubmitExternalDecision(request),
            session::ExternalDecisionSubmitStatus::kAccepted);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  ASSERT_TRUE(signal_pipeline.GetControlProfile().enable_lpi_power_control);
  ASSERT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 0.5f);

  frame = controller.GetLatestDecisionObservation().input_frame;
  session::ExternalDecisionResponse release;
  release.source_cycle_index = frame.cycle_index;
  release.source_batch_id = frame.batch_id;
  ASSERT_EQ(controller.SubmitExternalDecision(release),
            session::ExternalDecisionSubmitStatus::kAccepted);
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_TRUE(signal_pipeline.GetControlProfile().enable_lpi_power_control);
  EXPECT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 0.5f);

  frame = controller.GetLatestDecisionObservation().input_frame;
  release.source_cycle_index = frame.cycle_index;
  release.source_batch_id = frame.batch_id;
  ASSERT_EQ(controller.SubmitExternalDecision(release),
            session::ExternalDecisionSubmitStatus::kAccepted);
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_FALSE(signal_pipeline.GetControlProfile().enable_lpi_power_control);
  EXPECT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 1.0f);
}

TEST_F(CoreControllerTest, ExternalLpiParametersAlterNextPhysicalDetection) {
  config::ArSessionConfig session_config = MakeDetectionFocusedConfig();
  FakeRadarContext radar_context(BuildSingleTarget(220.0f, 10000.0f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const std::vector<signal::tracking::TrackMeasurement> baseline_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(baseline_measurements.size(), 1U);
  const session::DecisionInputFrame frame = controller.GetLatestDecisionObservation().input_frame;
  session::ExternalDecisionResponse response;
  response.source_cycle_index = frame.cycle_index;
  response.source_batch_id = frame.batch_id;
  response.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.4f),
      60, "external power"});
  response.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_DWELL,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.7f),
      55, "external dwell"});
  ASSERT_EQ(controller.SubmitExternalDecision(response),
            session::ExternalDecisionSubmitStatus::kAccepted);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const std::vector<signal::tracking::TrackMeasurement> controlled_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(controlled_measurements.size(), 1U);
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);
  EXPECT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 0.4f);
  EXPECT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_dwell_scale, 0.7f);
  EXPECT_LT(controlled_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST_F(CoreControllerTest, ExternalBurnthroughGainAltersNextPhysicalDetection) {
  config::ArSessionConfig session_config = MakeDetectionFocusedConfig();
  FakeRadarContext radar_context(BuildSingleTarget(220.0f, 10000.0f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const std::vector<signal::tracking::TrackMeasurement> baseline_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(baseline_measurements.size(), 1U);
  const session::DecisionInputFrame frame = controller.GetLatestDecisionObservation().input_frame;
  session::ExternalDecisionResponse response;
  response.source_cycle_index = frame.cycle_index;
  response.source_batch_id = frame.batch_id;
  response.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                                session::ControlDirectiveSource::SURVIVABILITY, 1.5f),
      82, "external burnthrough"});
  ASSERT_EQ(controller.SubmitExternalDecision(response),
            session::ExternalDecisionSubmitStatus::kAccepted);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const std::vector<signal::tracking::TrackMeasurement> controlled_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(controlled_measurements.size(), 1U);
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);
  EXPECT_FLOAT_EQ(signal_pipeline.GetControlProfile().eccm_burnthrough_gain, 1.5f);
  EXPECT_GT(controlled_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST_F(CoreControllerTest, EmptyExternalResponseExplicitlyDisablesInternalControl) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const session::DecisionInputFrame frame = controller.GetLatestDecisionObservation().input_frame;
  session::ExternalDecisionResponse response;
  response.source_cycle_index = frame.cycle_index;
  response.source_batch_id = frame.batch_id;
  ASSERT_EQ(controller.SubmitExternalDecision(response),
            session::ExternalDecisionSubmitStatus::kAccepted);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);
  EXPECT_EQ(signal_pipeline.GetControlProfile().version, 0U);
}

TEST_F(CoreControllerTest, RejectsMismatchedDuplicateAndInvalidExternalResponses) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);
  session::ExternalDecisionResponse before_observation;
  EXPECT_EQ(controller.SubmitExternalDecision(before_observation),
            session::ExternalDecisionSubmitStatus::kNoPendingObservation);
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const session::DecisionInputFrame frame = controller.GetLatestDecisionObservation().input_frame;

  session::ExternalDecisionResponse mismatch;
  mismatch.source_cycle_index = frame.cycle_index + 1U;
  mismatch.source_batch_id = frame.batch_id;
  EXPECT_EQ(controller.SubmitExternalDecision(mismatch),
            session::ExternalDecisionSubmitStatus::kSourceMismatch);

  session::ExternalDecisionResponse wrong_batch;
  wrong_batch.source_cycle_index = frame.cycle_index;
  wrong_batch.source_batch_id = frame.batch_id + 1U;
  EXPECT_EQ(controller.SubmitExternalDecision(wrong_batch),
            session::ExternalDecisionSubmitStatus::kSourceMismatch);

  session::ExternalDecisionResponse invalid;
  invalid.source_cycle_index = frame.cycle_index;
  invalid.source_batch_id = frame.batch_id;
  invalid.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL),
      60, "missing scalar"});
  EXPECT_EQ(controller.SubmitExternalDecision(invalid),
            session::ExternalDecisionSubmitStatus::kInvalidProposal);

  session::ExternalDecisionResponse duplicate;
  duplicate.source_cycle_index = frame.cycle_index;
  duplicate.source_batch_id = frame.batch_id;
  duplicate.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.4f),
      60, "first power"});
  duplicate.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.5f),
      55, "duplicate power"});
  EXPECT_EQ(controller.SubmitExternalDecision(duplicate),
            session::ExternalDecisionSubmitStatus::kInvalidProposal);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kInternal);
  const session::DecisionInputFrame next_frame =
      controller.GetLatestDecisionObservation().input_frame;
  session::ExternalDecisionResponse stale;
  stale.source_cycle_index = frame.cycle_index;
  stale.source_batch_id = frame.batch_id;
  EXPECT_EQ(controller.SubmitExternalDecision(stale),
            session::ExternalDecisionSubmitStatus::kSourceMismatch);

  session::ExternalDecisionResponse accepted;
  accepted.source_cycle_index = next_frame.cycle_index;
  accepted.source_batch_id = next_frame.batch_id;
  EXPECT_EQ(controller.SubmitExternalDecision(accepted),
            session::ExternalDecisionSubmitStatus::kAccepted);
  EXPECT_EQ(controller.SubmitExternalDecision(accepted),
            session::ExternalDecisionSubmitStatus::kAlreadySubmitted);
}

TEST_F(CoreControllerTest, RuntimeRestoreRetainsPendingExternalResponseForRetry) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  AbortingSignalPipeline signal_pipeline;
  signal_pipeline.SetShouldExecute(true);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const session::DecisionInputFrame frame = controller.GetLatestDecisionObservation().input_frame;
  session::ExternalDecisionResponse response;
  response.source_cycle_index = frame.cycle_index;
  response.source_batch_id = frame.batch_id;
  response.proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.4f),
      60, "external power"});
  ASSERT_EQ(controller.SubmitExternalDecision(response),
            session::ExternalDecisionSubmitStatus::kAccepted);
  const extension::ArControllerRuntimeState snapshot = controller.CaptureRuntimeState();
  const signal::SignalPipelineRuntimeState pipeline_snapshot =
      signal_pipeline.CaptureRuntimeState();

  signal_pipeline.SetShouldExecute(false);
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 0.4f);

  signal_pipeline.RestoreRuntimeState(pipeline_snapshot);
  ASSERT_TRUE(controller.RestoreRuntimeState(snapshot));
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);
  EXPECT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 0.4f);
}

}  // namespace tests
}  // namespace airborne_radar
