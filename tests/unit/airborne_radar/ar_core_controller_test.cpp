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
#include "airborne_radar/runtime/ControlCommandMapper.h"
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

  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_lpi_power_control = true;
  profile.lpi_power_scale = 0.4f;
  override_decision.profile = profile;
  EXPECT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
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

  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_agility_frequency = true;
  override_decision.profile = profile;
  ASSERT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
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

  // Cycle 1: override activates LPI.
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  {
    session::ExternalDecisionOverride override_request;
    session::ArControlProfile profile;
    profile.enable_lpi_power_control = true;
    profile.lpi_power_scale = 0.5f;
    override_request.profile = profile;
    ASSERT_EQ(controller.SubmitExternalDecision(std::move(override_request)),
              session::ExternalDecisionSubmitStatus::kAccepted);
  }

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  ASSERT_TRUE(signal_pipeline.GetControlProfile().enable_lpi_power_control);
  ASSERT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 0.5f);

  // Cycle 2: override submits default profile (equivalent to identity/no-op).
  // The override path bypasses the native reducer's hold-window counter:
  // the native reducer resets LPI (no internal LPI proposals), and the
  // default-profile override does not restore it.
  {
    session::ExternalDecisionOverride release;
    session::ArControlProfile default_profile{};
    release.profile = default_profile;
    ASSERT_EQ(controller.SubmitExternalDecision(std::move(release)),
              session::ExternalDecisionSubmitStatus::kAccepted);
  }
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_FALSE(signal_pipeline.GetControlProfile().enable_lpi_power_control);
  EXPECT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 1.0f);

  // Cycle 3: LPI remains off with no override.
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

  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_lpi_power_control = true;
  profile.lpi_power_scale = 0.4f;
  profile.lpi_dwell_scale = 0.7f;
  override_decision.profile = profile;
  ASSERT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
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

  // The override must also disable LPI: the native reducer runs internal
  // proposals first (which may include LPI directives that reduce margin),
  // then the override is applied on top.  Without explicitly clearing LPI,
  // the net margin can decrease despite the burnthrough gain.
  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_lpi_power_control = false;
  profile.lpi_power_scale = 1.0f;
  profile.lpi_dwell_scale = 1.0f;
  profile.eccm_burnthrough_gain = 1.5f;
  override_decision.profile = profile;
  ASSERT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
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

// 自适应波束形成 directive 的检测后果（C1 收敛）：
// REQUEST_ENABLE_ADAPTIVE_BEAMFORMING 经 ControlProfileEffects 提升 main_beam_gain_db
// （adaptive_beam_gain_boost_db，默认 +2.0 dB），直接抬高回波功率，应在热噪声范围内提高检测裕度。
// 此前 sidelobe/adaptive beamforming 仅被断言为 proposal 存在或 cycle 仍完成，
// 从不断言实际检测后果——这是 C1「两个消费者发散物理量」中最可测量的 detector-config 路径的基线证据。
TEST_F(CoreControllerTest, ExternalAdaptiveBeamformingRaisesNextPhysicalDetectionMargin) {
  config::ArSessionConfig session_config = MakeDetectionFocusedConfig();
  FakeRadarContext radar_context(BuildSingleTarget(220.0f, 10000.0f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline(session_config);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const std::vector<signal::tracking::TrackMeasurement> baseline_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(baseline_measurements.size(), 1U);

  // The override must also disable LPI: the native reducer runs internal
  // proposals first (which may include LPI directives that reduce margin),
  // then the override is applied on top.  Without explicitly clearing LPI,
  // the net margin can decrease despite the adaptive beamforming boost.
  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_lpi_power_control = false;
  profile.lpi_power_scale = 1.0f;
  profile.lpi_dwell_scale = 1.0f;
  profile.enable_adaptive_beamforming = true;
  override_decision.profile = profile;
  ASSERT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
            session::ExternalDecisionSubmitStatus::kAccepted);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  const std::vector<signal::tracking::TrackMeasurement> controlled_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(controlled_measurements.size(), 1U);
  EXPECT_TRUE(signal_pipeline.GetControlProfile().enable_adaptive_beamforming);
  // 自适应波束形成提升主瓣增益：检测裕度应上升。
  EXPECT_GT(controlled_measurements[0].raw_measurement.detection_margin_db,
            baseline_measurements[0].raw_measurement.detection_margin_db);
}

TEST_F(CoreControllerTest, DefaultProfileOverridePreservesInternalSource) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile default_profile{};
  override_decision.profile = default_profile;
  ASSERT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
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

  // kNoPendingObservation: submit before any observation exists.
  {
    session::ExternalDecisionOverride before_observation;
    session::ArControlProfile profile;
    profile.enable_agility_frequency = true;
    before_observation.profile = profile;
    EXPECT_EQ(controller.SubmitExternalDecision(std::move(before_observation)),
              session::ExternalDecisionSubmitStatus::kNoPendingObservation);
  }

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  // kInvalidProfile: profile with out-of-range field.
  {
    session::ExternalDecisionOverride invalid;
    session::ArControlProfile profile;
    profile.enable_lpi_power_control = true;
    profile.lpi_power_scale = 99.0f;  // exceeds [0,1] range
    invalid.profile = profile;
    EXPECT_EQ(controller.SubmitExternalDecision(std::move(invalid)),
              session::ExternalDecisionSubmitStatus::kInvalidProfile);
  }

  // kAccepted: valid override.
  {
    session::ExternalDecisionOverride accepted;
    session::ArControlProfile profile;
    profile.enable_lpi_power_control = true;
    profile.lpi_power_scale = 0.4f;
    accepted.profile = profile;
    EXPECT_EQ(controller.SubmitExternalDecision(std::move(accepted)),
              session::ExternalDecisionSubmitStatus::kAccepted);
  }

  // kAlreadySubmitted: duplicate submission in the same cycle.
  {
    session::ExternalDecisionOverride duplicate;
    session::ArControlProfile profile;
    profile.enable_eccm_rejitter = true;
    duplicate.profile = profile;
    EXPECT_EQ(controller.SubmitExternalDecision(std::move(duplicate)),
              session::ExternalDecisionSubmitStatus::kAlreadySubmitted);
  }

  // First RunOnce consumes the accepted override -> external source.
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);

  // Second RunOnce with no new override -> internal source.
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kInternal);
}

TEST_F(CoreControllerTest, RuntimeRestoreRetainsPendingExternalResponseForRetry) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  AbortingSignalPipeline signal_pipeline;
  signal_pipeline.SetShouldExecute(true);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_lpi_power_control = true;
  profile.lpi_power_scale = 0.4f;
  override_decision.profile = profile;
  ASSERT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
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

// ============================================================================
// ExternalDecisionOverride (profile value override path) tests
// ============================================================================

TEST_F(CoreControllerTest, ExternalOverrideChangesNextProfile) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_agility_frequency = true;
  override_decision.profile = profile;
  EXPECT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
            session::ExternalDecisionSubmitStatus::kAccepted);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);
  EXPECT_TRUE(signal_pipeline.GetControlProfile().enable_agility_frequency);
}

TEST_F(CoreControllerTest, ExternalOverrideBypassesCooldown) {
  config::DecisionControlConfig decision_config;
  decision_config.lpi_cooldown_cycles_after_release = 3U;
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service, decision_config);

  // Cycle 1: trigger LPI via external override
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  {
    session::ExternalDecisionOverride override_decision;
    session::ArControlProfile profile;
    profile.enable_lpi_power_control = true;
    profile.lpi_power_scale = 0.5f;
    override_decision.profile = profile;
    ASSERT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
              session::ExternalDecisionSubmitStatus::kAccepted);
  }

  // Cycle 2: LPI active, then release (no proposals)
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_TRUE(signal_pipeline.GetControlProfile().enable_lpi_power_control);

  // Cycle 3: release — empty proposals trigger cooldown
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  // Cycle 4: cooldown active — native proposal would be blocked, but override bypasses
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  {
    session::ExternalDecisionOverride override_decision;
    session::ArControlProfile profile;
    profile.enable_lpi_power_control = true;
    profile.lpi_power_scale = 0.6f;
    override_decision.profile = profile;
    EXPECT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
              session::ExternalDecisionSubmitStatus::kAccepted);
  }

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);
  EXPECT_TRUE(signal_pipeline.GetControlProfile().enable_lpi_power_control);
  EXPECT_FLOAT_EQ(signal_pipeline.GetControlProfile().lpi_power_scale, 0.6f);
}

TEST_F(CoreControllerTest, ExternalOverrideDoesNotAffectNativeLpi) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  // Override only touches ECCM, LPI should still come from native path (i.e., untouched defaults)
  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_eccm_rejitter = true;
  override_decision.profile = profile;
  EXPECT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
            session::ExternalDecisionSubmitStatus::kAccepted);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kExternal);
  EXPECT_TRUE(signal_pipeline.GetControlProfile().enable_eccm_rejitter);
}

TEST_F(CoreControllerTest, ExternalOverrideRejectsInvalidProfile) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  // Profile with NaN lpi_power_scale — rejected at submit time
  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_lpi_power_control = true;
  profile.lpi_power_scale = std::numeric_limits<float>::quiet_NaN();
  override_decision.profile = profile;
  EXPECT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
            session::ExternalDecisionSubmitStatus::kInvalidProfile);
  // Invalid override never reached pending state; next cycle uses native path.
  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});
  EXPECT_EQ(controller.GetLastAppliedDecisionSource(), session::DecisionControlSource::kInternal);
}

TEST_F(CoreControllerTest, ExternalOverrideAcceptsDefaultProfile) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  // Default-constructed profile is valid (all fields in range).
  session::ExternalDecisionOverride override_decision;
  // leave profile at default-constructed values
  EXPECT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
            session::ExternalDecisionSubmitStatus::kAccepted);
}

TEST_F(CoreControllerTest, ExternalOverrideNoPendingObservation) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  // No RunOnce yet — no pending observation
  session::ExternalDecisionOverride override_decision;
  session::ArControlProfile profile;
  profile.enable_agility_frequency = true;
  override_decision.profile = profile;
  EXPECT_EQ(controller.SubmitExternalDecision(std::move(override_decision)),
            session::ExternalDecisionSubmitStatus::kNoPendingObservation);
}

TEST_F(CoreControllerTest, ExternalOverrideAlreadySubmitted) {
  FakeRadarContext radar_context(BuildSingleTarget(800.0f, 2.5f, false));
  environment::EnvironmentService environment_service;
  signal::pipeline::SignalPipeline signal_pipeline;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  controller.RunOnce(signal::pipeline::SignalCycleInput{radar_context.GetSceneTargets()});

  session::ExternalDecisionOverride first;
  session::ArControlProfile profile1;
  profile1.enable_agility_frequency = true;
  first.profile = profile1;
  EXPECT_EQ(controller.SubmitExternalDecision(std::move(first)),
            session::ExternalDecisionSubmitStatus::kAccepted);

  session::ExternalDecisionOverride second;
  session::ArControlProfile profile2;
  profile2.enable_eccm_rejitter = true;
  second.profile = profile2;
  EXPECT_EQ(controller.SubmitExternalDecision(std::move(second)),
            session::ExternalDecisionSubmitStatus::kAlreadySubmitted);
}

TEST_F(CoreControllerTest, DiffProfilesGeneratesCorrectDirectives) {
  session::ArControlProfile baseline;
  session::ArControlProfile target;
  target.enable_agility_frequency = true;
  target.enable_sidelobe_canceller = true;
  target.eccm_burnthrough_gain = 1.5f;

  const auto diffs = extension::ControlCommandMapper::DiffProfiles(baseline, target);
  ASSERT_EQ(diffs.size(), 3U);
  EXPECT_EQ(diffs[0].type, session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY);
  EXPECT_EQ(diffs[1].type, session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER);
  EXPECT_EQ(diffs[2].type, session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN);
  EXPECT_FLOAT_EQ(diffs[2].requested_value, 1.5f);
}

}  // namespace tests
}  // namespace airborne_radar
