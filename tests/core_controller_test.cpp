// Copyright 2026. All Rights Reserved.
//
// Description: 验证核心调度器使用桩组件的最小集成流程。

#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "1q/airborne_radar/core/event/RadarEvents.h"
#include "1q/airborne_radar/decision/ControlReducer.h"
#include "1q/airborne_radar/decision/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipeline.h"
#include "1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "1q/airborne_radar/core/event/EventBus.h"
#include "1q/airborne_radar/core/event/CycleEventBus.h"
#include "1q/airborne_radar/environment/EnvironmentService.h"

namespace airborne_radar { namespace tests {

namespace {

common::TargetFeatureList BuildSingleTarget(float speed, float rcs, bool jamming) {
  (void)jamming;
  common::TargetFeature target(speed, 0.0f, 0.0f, rcs);
  target.position_x = 100.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 100.0f;
  return common::TargetFeatureList{target};
}

} // namespace

/// @brief FakeRadarContext 提供最小化的雷达上下文实现。
class FakeRadarContext : public core::context::IRadarContext {
public:
  /// @brief 构造函数，注入固定的输入状态。
  explicit FakeRadarContext(common::TargetFeatureList state)
      : state_(std::move(state)) {}

  /// @brief 获取当前雷达状态。
  common::TargetFeatureList GetTargetFeatures() const override { return state_; }

  /// @brief 获取当前搭载平台姿态角。
  common::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_deg_;
  }

  /// @brief 获取当前周期时间步长。
  float GetCycleDeltaTimeSec() const override { return cycle_dt_sec_; }

  /// @brief 收集控制指令。
  void SubmitControlCommand(common::RadarCommand cmd) override {
    submitted_commands_.push_back(cmd);
  }

  /// @brief 获取已提交的指令集合。
  const std::vector<common::RadarCommand> &SubmittedCommands() const {
    return submitted_commands_;
  }

  /// @brief 记录最新控制真值。
  void UpdateRadarControlProfile(
      const common::RadarControlProfile &profile) override {
    latest_control_profile_ = profile;
  }

  /// @brief 获取最新控制真值。
  const common::RadarControlProfile &LatestControlProfile() const {
    return latest_control_profile_;
  }

  /// @brief 设置测试上下文使用的平台姿态角。
  void SetPlatformAttitude(
      const common::PlatformAttitudeDeg &platform_attitude_deg) {
    platform_attitude_deg_ = platform_attitude_deg;
  }

  /// @brief 设置当前周期时间步长。
  void SetCycleDeltaTimeSec(float cycle_dt_sec) { cycle_dt_sec_ = cycle_dt_sec; }

private:
  common::TargetFeatureList state_;
  common::PlatformAttitudeDeg platform_attitude_deg_{};
  float cycle_dt_sec_{1.0f};
  std::vector<common::RadarCommand> submitted_commands_;
  common::RadarControlProfile latest_control_profile_{};
};

class SpyTrackLifecycleManager : public signal::tracking::ITrackLifecycleManager {
public:
  void Update(const signal::tracking::CycleContext &cycle,
              const std::vector<signal::tracking::TrackMeasurement> &measurements) override {
    last_cycle = cycle;
    last_measurements = measurements;
  }

  common::TargetFeatureList BuildFeatureSnapshot() const override {
    common::TargetFeatureList snapshot;
    snapshot.reserve(last_measurements.size());
    for (const signal::tracking::TrackMeasurement &measurement : last_measurements) {
      snapshot.push_back(common::TargetFeature(
          measurement.filtered_feature.velocity(0),
          measurement.filtered_feature.velocity(1),
          measurement.filtered_feature.velocity(2),
          measurement.filtered_feature.rcs,
          measurement.filtered_feature.acceleration(0),
          measurement.filtered_feature.acceleration(1),
          measurement.filtered_feature.acceleration(2)));
    }
    return snapshot;
  }

  common::DecisionTrackSnapshotList BuildDecisionSnapshot() const override {
    common::DecisionTrackSnapshotList snapshot;
    snapshot.reserve(last_measurements.size());
    for (const signal::tracking::TrackMeasurement &measurement :
         last_measurements) {
      common::DecisionTrackSnapshot track_snapshot(
          measurement.filtered_feature.velocity(0),
          measurement.filtered_feature.velocity(1),
          measurement.filtered_feature.velocity(2),
          measurement.filtered_feature.rcs,
          measurement.filtered_feature.acceleration(0),
          measurement.filtered_feature.acceleration(1),
          measurement.filtered_feature.acceleration(2),
          measurement.filtered_feature.jamming_detected,
          measurement.raw_measurement.external_target_id,
          measurement.raw_measurement.association_key);
      track_snapshot.state.position_x = measurement.raw_measurement.position(0);
      track_snapshot.state.position_y = measurement.raw_measurement.position(1);
      track_snapshot.state.position_z = measurement.raw_measurement.position(2);
      track_snapshot.evidence.has_measurement_evidence = true;
      track_snapshot.evidence.updated_this_cycle = true;
      snapshot.push_back(track_snapshot);
    }
    return snapshot;
  }

  common::DecisionInputFrame BuildDecisionFrame(
      std::uint32_t cycle_index, std::uint64_t batch_id,
      bool environment_jamming_detected) const override {
    common::DecisionInputFrame frame;
    frame.cycle_index = cycle_index;
    frame.batch_id = batch_id;
    frame.environment_jamming_detected = environment_jamming_detected;
    frame.tracks = BuildDecisionSnapshot();
    return frame;
  }

  std::vector<signal::tracking::AssociationTrackSeed>
  BuildAssociationSeeds() const override {
    return association_seeds;
  }

  signal::tracking::CycleContext last_cycle{};
  std::vector<signal::tracking::TrackMeasurement> last_measurements;
  std::vector<signal::tracking::AssociationTrackSeed> association_seeds;
};

class FixedSeedLifecycleManager : public signal::tracking::ITrackLifecycleManager {
public:
  explicit FixedSeedLifecycleManager(
      std::vector<signal::tracking::AssociationTrackSeed> initial_seeds)
      : seeds_(std::move(initial_seeds)) {}

  void Update(const signal::tracking::CycleContext &cycle,
              const std::vector<signal::tracking::TrackMeasurement> &measurements) override {
    last_cycle = cycle;
    last_measurements = measurements;
  }

  common::TargetFeatureList BuildFeatureSnapshot() const override {
    return common::TargetFeatureList();
  }

  common::DecisionTrackSnapshotList BuildDecisionSnapshot() const override {
    return common::DecisionTrackSnapshotList();
  }

  common::DecisionInputFrame BuildDecisionFrame(
      std::uint32_t cycle_index, std::uint64_t batch_id,
      bool environment_jamming_detected) const override {
    common::DecisionInputFrame frame;
    frame.cycle_index = cycle_index;
    frame.batch_id = batch_id;
    frame.environment_jamming_detected = environment_jamming_detected;
    return frame;
  }

  std::vector<signal::tracking::AssociationTrackSeed>
  BuildAssociationSeeds() const override {
    return seeds_;
  }

  signal::tracking::CycleContext last_cycle{};
  std::vector<signal::tracking::TrackMeasurement> last_measurements;

private:
  std::vector<signal::tracking::AssociationTrackSeed> seeds_;
};

class EmptySeedLifecycleManager : public signal::tracking::ITrackLifecycleManager {
public:
  void Update(const signal::tracking::CycleContext &cycle,
              const std::vector<signal::tracking::TrackMeasurement> &measurements) override {
    last_cycle = cycle;
    last_measurements = measurements;
  }

  common::TargetFeatureList BuildFeatureSnapshot() const override {
    return common::TargetFeatureList();
  }

  common::DecisionTrackSnapshotList BuildDecisionSnapshot() const override {
    return common::DecisionTrackSnapshotList();
  }

  common::DecisionInputFrame BuildDecisionFrame(
      std::uint32_t cycle_index, std::uint64_t batch_id,
      bool environment_jamming_detected) const override {
    common::DecisionInputFrame frame;
    frame.cycle_index = cycle_index;
    frame.batch_id = batch_id;
    frame.environment_jamming_detected = environment_jamming_detected;
    return frame;
  }

  std::vector<signal::tracking::AssociationTrackSeed>
  BuildAssociationSeeds() const override {
    return std::vector<signal::tracking::AssociationTrackSeed>();
  }

  signal::tracking::CycleContext last_cycle{};
  std::vector<signal::tracking::TrackMeasurement> last_measurements;
};

class FixedDirectiveDecisionEngine : public decision::ITacticalDecisionEngine {
public:
  explicit FixedDirectiveDecisionEngine(common::ControlDirective directive)
      : directive_(std::move(directive)) {}

  decision::TacticalDecisionResult Evaluate(
      const common::DecisionInputFrame&,
      decision::TacticalStateStore&) override {
    decision::TacticalDecisionResult result;
    result.proposals.push_back(
        decision::TacticalProposal(directive_, 10, "test directive"));
    return result;
  }

private:
  common::ControlDirective directive_;
};

class FixedProposalDecisionEngine : public decision::ITacticalDecisionEngine {
public:
  explicit FixedProposalDecisionEngine(
      std::vector<decision::TacticalProposal> proposals)
      : proposals_(std::move(proposals)) {}

  decision::TacticalDecisionResult Evaluate(
      const common::DecisionInputFrame&,
      decision::TacticalStateStore&) override {
    decision::TacticalDecisionResult result;
    result.proposals = proposals_;
    return result;
  }

private:
  std::vector<decision::TacticalProposal> proposals_;
};

class CapturingDecisionEngine : public decision::ITacticalDecisionEngine {
public:
  decision::TacticalDecisionResult Evaluate(
      const common::DecisionInputFrame& frame,
      decision::TacticalStateStore&) override {
    last_frame = frame;
    ++evaluate_count;
    return decision::TacticalDecisionResult();
  }

  common::DecisionInputFrame last_frame{};
  std::size_t evaluate_count{0U};
};

/// @brief CoreControllerTest 覆盖核心调度与指令下发路径。
class CoreControllerTest : public ::testing::Test {
};

TEST_F(CoreControllerTest, RunOnceSubmitsCommands) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 12.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  const auto &cmds = radar_context.SubmittedCommands();
  EXPECT_GT(cmds.size(), 0u);
}

TEST_F(CoreControllerTest, PushesPlatformAttitudeIntoSignalPipelineBeforeRun) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);
  common::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 18.0f;
  platform_attitude_deg.pitch_deg = -4.0f;
  platform_attitude_deg.roll_deg = 2.0f;
  radar_context.SetPlatformAttitude(platform_attitude_deg);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);

  controller.RunOnce();

  const common::PlatformAttitudeDeg cached_platform_attitude =
      signal_pipeline.GetPlatformAttitude();
  EXPECT_FLOAT_EQ(cached_platform_attitude.yaw_deg, 18.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.pitch_deg, -4.0f);
  EXPECT_FLOAT_EQ(cached_platform_attitude.roll_deg, 2.0f);
}

TEST_F(CoreControllerTest, MapsMultiSourceJammingFactsIntoDecisionFrame) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
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
  core::controller::RadarController controller(
      radar_context, signal_pipeline, decision_engine, environment_service);

  controller.RunOnce();

  ASSERT_EQ(decision_engine.evaluate_count, 1u);
  EXPECT_TRUE(decision_engine.last_frame.environment_jamming_detected);
  ASSERT_EQ(decision_engine.last_frame.eccm_source_info.jammer_sources.size(), 1u);
  const common::EccmJammerSourceInfo& mapped_source =
      decision_engine.last_frame.eccm_source_info.jammer_sources.front();
  EXPECT_EQ(mapped_source.technique, common::JammingTechnique::kDeception);
  EXPECT_FLOAT_EQ(mapped_source.jammer_power_db, 9.0f);
  EXPECT_FLOAT_EQ(mapped_source.jammer_to_signal_db, 7.5f);
  EXPECT_FLOAT_EQ(mapped_source.frequency_overlap_ratio, 0.85f);
  EXPECT_FLOAT_EQ(mapped_source.prf_lock_risk, 0.90f);
  EXPECT_FLOAT_EQ(mapped_source.azimuth_deg, 30.0f);
  EXPECT_FLOAT_EQ(mapped_source.angular_span_deg, 8.0f);
  EXPECT_EQ(decision_engine.last_frame.association_quality_info.dominant_jamming_semantic,
            common::JammingSemantic::kDeception);
  EXPECT_GT(decision_engine.last_frame.association_quality_info.jamming_severity,
            0.0f);
  EXPECT_GT(decision_engine.last_frame.association_quality_info.association_stress,
            0.0f);
  EXPECT_EQ(decision_engine.last_frame.perception_quality_info.input_target_count,
            1u);
  EXPECT_EQ(decision_engine.last_frame.perception_quality_info.detection_count,
            1u);
  EXPECT_FLOAT_EQ(decision_engine.last_frame.perception_quality_info.detection_rate,
                  1.0f);
  EXPECT_FLOAT_EQ(
      decision_engine.last_frame.perception_quality_info.detection_stress, 0.0f);
}

TEST_F(CoreControllerTest, PushesCycleDeltaTimeIntoLifecycleBeforeRun) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);
  radar_context.SetCycleDeltaTimeSec(2.5f);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  SpyTrackLifecycleManager lifecycle_manager;

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  controller.RunOnce();

  EXPECT_FLOAT_EQ(lifecycle_manager.last_cycle.dt_sec, 2.5f);
}

TEST_F(CoreControllerTest, ProtectiveControlProfileExtendsLifecycleMissTolerance) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  SpyTrackLifecycleManager lifecycle_manager;
  FixedDirectiveDecisionEngine decision_engine(common::ControlDirective(
      common::ControlDirectiveType::REQUEST_ECCM_REJITTER,
      common::ControlDirectiveSource::SURVIVABILITY));

  core::controller::RadarController controller(
      radar_context, signal_pipeline, decision_engine, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  controller.RunOnce();
  EXPECT_EQ(lifecycle_manager.last_cycle.extra_miss_tolerance, 0u);

  controller.RunOnce();
  EXPECT_EQ(lifecycle_manager.last_cycle.extra_miss_tolerance, 1u);
}

TEST_F(CoreControllerTest, RunOncePublishesCycleEvent) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 12.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  core::event::EventBus event_bus;

  std::size_t received_count = 0;
  bool received_jamming = false;
  event_bus.Subscribe<core::event::RadarCycleCompletedEvent>(
      [&received_count, &received_jamming](
          const core::event::RadarCycleCompletedEvent &event) {
        received_count = event.command_count;
        received_jamming = event.jamming_detected;
      });

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service, event_bus);

  controller.RunOnce();

  EXPECT_GT(received_count, 0u);
  EXPECT_TRUE(received_jamming);
}

TEST_F(CoreControllerTest, RunOncePublishesFineGrainedEvents) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 12.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  core::event::EventBus event_bus;

  bool tracks_event_received = false;
  bool tracks_event_has_jamming_flag = false;
  bool jamming_event_received = false;
  std::size_t submitted_commands = 0;
  std::uint64_t control_profile_version = 0;
  std::size_t applied_directive_events = 0;

  event_bus.Subscribe<core::event::TracksUpdatedEvent>(
      [&tracks_event_received, &tracks_event_has_jamming_flag](
          const core::event::TracksUpdatedEvent &) {
        tracks_event_received = true;
        tracks_event_has_jamming_flag = false;
      });

  event_bus.Subscribe<core::event::JammingAlertEvent>(
      [&jamming_event_received](const core::event::JammingAlertEvent &event) {
        jamming_event_received = event.detected;
      });

  event_bus.Subscribe<core::event::CommandsSubmittedEvent>(
      [&submitted_commands](const core::event::CommandsSubmittedEvent &event) {
        submitted_commands = event.command_count;
      });

  event_bus.Subscribe<core::event::ControlProfileUpdatedEvent>(
      [&control_profile_version](
          const core::event::ControlProfileUpdatedEvent &event) {
        control_profile_version = event.profile_version;
      });

  event_bus.Subscribe<core::event::DirectiveAppliedEvent>(
      [&applied_directive_events](
          const core::event::DirectiveAppliedEvent &) {
        ++applied_directive_events;
      });

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service, event_bus);

  controller.RunOnce();

  EXPECT_TRUE(tracks_event_received);
  EXPECT_FALSE(tracks_event_has_jamming_flag);
  EXPECT_TRUE(jamming_event_received);
  EXPECT_GT(submitted_commands, 0u);
  EXPECT_GT(control_profile_version, 0u);
  EXPECT_GT(applied_directive_events, 0u);
}

TEST_F(CoreControllerTest, NextCycleAppliesPendingControlProfileToSignalPipeline) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 12.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);

  controller.RunOnce();
  EXPECT_EQ(signal_pipeline.GetControlProfile().version, 0u);
  const std::uint64_t first_pending_version =
      radar_context.LatestControlProfile().version;
  EXPECT_GT(first_pending_version, 0u);

  controller.RunOnce();
  EXPECT_EQ(signal_pipeline.GetControlProfile().version, first_pending_version);
}

TEST_F(CoreControllerTest, CustomReducerConfigChangesPendingControlProfile) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  std::vector<decision::TacticalProposal> proposals;
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      60,
      "reduce power"});
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
          common::ControlDirectiveSource::SURVIVABILITY),
      82,
      "burnthrough"});
  FixedProposalDecisionEngine decision_engine(proposals);

  core::controller::RadarController controller(
      radar_context, signal_pipeline, decision_engine, environment_service);

  decision::ControlReducerConfig reducer_config;
  reducer_config.lpi_power_scale_on_reduction = 0.60f;
  reducer_config.eccm_burnthrough_gain = 1.8f;
  reducer_config.burnthrough_lpi_power_floor = 0.95f;
  controller.UpdateControlReducerConfig(reducer_config);

  controller.RunOnce();

  EXPECT_FLOAT_EQ(radar_context.LatestControlProfile().eccm_burnthrough_gain,
                  1.8f);
  EXPECT_FLOAT_EQ(radar_context.LatestControlProfile().lpi_power_scale, 0.95f);
}

TEST_F(CoreControllerTest, CycleEventBusDeliversEventsOnNextCycle) {
  const common::TargetFeatureList input_state =
      BuildSingleTarget(800.0f, 2.5f, false);
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 12.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  core::event::CycleEventBus event_bus;

  std::size_t completed_event_hits = 0;
  std::size_t last_command_count = 0;
  event_bus.Subscribe<core::event::RadarCycleCompletedEvent>(
      [&completed_event_hits, &last_command_count](
          const core::event::RadarCycleCompletedEvent &event) {
        ++completed_event_hits;
        last_command_count = event.command_count;
      });

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service, event_bus);

  controller.RunOnce();
  EXPECT_EQ(completed_event_hits, 0u);

  controller.RunOnce();
  EXPECT_EQ(completed_event_hits, 1u);
  EXPECT_GT(last_command_count, 0u);
}

TEST_F(CoreControllerTest, LifecycleManagerConsumesRealAssociationMeasurements) {
  common::TargetFeature first(100.0f, 0.0f, 0.0f, 2.0f, false, 1.0f, 0.0f,
                              0.0f);
  first.position_x = 10.0f;
  first.position_y = 0.0f;
  first.position_z = 0.0f;
  first.range_m = 10.0f;
  common::TargetFeature second(220.0f, 0.0f, 0.0f, 5.0f, false, 3.0f, 0.0f,
                               0.0f);
  second.position_x = 100.0f;
  second.position_y = 0.0f;
  second.position_z = 0.0f;
  second.range_m = 100.0f;
  const common::TargetFeatureList input_state = {first, second};
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  SpyTrackLifecycleManager lifecycle_manager;

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  controller.RunOnce();

  ASSERT_EQ(lifecycle_manager.last_cycle.cycle_index, 1u);
  ASSERT_EQ(lifecycle_manager.last_measurements.size(), 2u);
  EXPECT_NE(lifecycle_manager.last_measurements[0].raw_measurement.association_key, 0u);
  EXPECT_NE(lifecycle_manager.last_measurements[1].raw_measurement.association_key, 0u);
  EXPECT_NE(lifecycle_manager.last_measurements[0].raw_measurement.association_key,
            lifecycle_manager.last_measurements[1].raw_measurement.association_key);
  EXPECT_EQ(lifecycle_manager.last_measurements[0].raw_measurement.source_index, 0u);
  EXPECT_EQ(lifecycle_manager.last_measurements[1].raw_measurement.source_index, 1u);
  EXPECT_FALSE(lifecycle_manager.last_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_FALSE(lifecycle_manager.last_measurements[1].raw_measurement.matched_existing_track);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0].raw_measurement.has_cartesian_position);
  EXPECT_TRUE(lifecycle_manager.last_measurements[1].raw_measurement.has_cartesian_position);

  const signal::pipeline::AssociationQualityMetrics metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();
  EXPECT_EQ(metrics.prior_track_count, 0u);
  EXPECT_EQ(metrics.detection_count, 2u);
  EXPECT_EQ(metrics.matched_count, 0u);
  EXPECT_EQ(metrics.new_track_count, 2u);
  EXPECT_EQ(metrics.missed_track_count, 0u);
  EXPECT_FLOAT_EQ(metrics.match_rate, 0.0f);
  EXPECT_FLOAT_EQ(metrics.new_track_rate, 1.0f);
  EXPECT_FLOAT_EQ(metrics.missed_track_rate, 0.0f);
}

TEST_F(CoreControllerTest, LifecycleSeedsDrivePositionAssociationBeforeRunCycle) {
  common::TargetFeature target(10.0f, 0.0f, 0.0f, 0.2f, false, 0.1f, 0.0f,
                               0.0f);
  target.position_x = 101.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 101.0f;
  const common::TargetFeatureList input_state{target};
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;

  signal::tracking::AssociationTrackSeed seed;
  seed.association_key = 42;
  seed.has_position = true;
  seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
  seed.has_gaussian_state = true;
  seed.gaussian_state.mean(0) = 100.0f;
  seed.gaussian_state.mean(2) = 0.0f;
  seed.gaussian_state.mean(4) = 0.0f;
  seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;

  FixedSeedLifecycleManager lifecycle_manager({seed});

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  controller.RunOnce();

  ASSERT_EQ(lifecycle_manager.last_measurements.size(), 1u);
  EXPECT_EQ(lifecycle_manager.last_measurements[0].raw_measurement.association_key,
            42u);
  EXPECT_TRUE(
      lifecycle_manager.last_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(
      lifecycle_manager.last_measurements[0].raw_measurement.used_position_association);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0]
                  .raw_measurement.used_external_association_seeds);
}

TEST_F(CoreControllerTest,
  EmptyLifecycleSeedsKeepAssociationStatelessAcrossCycles) {
  common::TargetFeature target(12.0f, 0.0f, 0.0f, 0.5f, false, 0.1f, 0.0f,
                               0.0f);
  target.position_x = 55.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 55.0f;
  const common::TargetFeatureList input_state{target};
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  EmptySeedLifecycleManager lifecycle_manager;

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  controller.RunOnce();
  ASSERT_EQ(lifecycle_manager.last_measurements.size(), 1u);
  const std::uint64_t first_key =
      lifecycle_manager.last_measurements[0].raw_measurement.association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(
      lifecycle_manager.last_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0]
                  .raw_measurement.used_external_association_seeds);

  controller.RunOnce();
  ASSERT_EQ(lifecycle_manager.last_measurements.size(), 1u);
  const std::uint64_t second_key =
      lifecycle_manager.last_measurements[0].raw_measurement.association_key;
  EXPECT_NE(second_key, 0u);
  EXPECT_NE(second_key, first_key);
  EXPECT_FALSE(
      lifecycle_manager.last_measurements[0].raw_measurement.matched_existing_track);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0]
                  .raw_measurement.used_external_association_seeds);
}

TEST_F(CoreControllerTest,
       LifecycleSeedMissingGaussianStateFailsFastDuringRunCycle) {
  common::TargetFeature target(10.0f, 0.0f, 0.0f, 0.2f, false, 0.1f, 0.0f,
                               0.0f);
  target.position_x = 20.0f;
  target.position_y = 0.0f;
  target.position_z = 0.0f;
  target.range_m = 20.0f;
  const common::TargetFeatureList input_state{target};
  FakeRadarContext radar_context(input_state);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;

  signal::tracking::AssociationTrackSeed invalid_seed;
  invalid_seed.association_key = 9001u;
  invalid_seed.has_position = true;
  invalid_seed.position = Eigen::Vector3f(19.0f, 0.0f, 0.0f);
  invalid_seed.has_gaussian_state = false;

  FixedSeedLifecycleManager lifecycle_manager({invalid_seed});

  core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  EXPECT_DEATH_IF_SUPPORTED(controller.RunOnce(), "missing gaussian state");
}

  TEST_F(CoreControllerTest,
       NoLifecycleManagerScrubsSideChannelExternalSeedsBeforeRunCycle) {
    common::TargetFeature target(10.0f, 0.0f, 0.0f, 0.2f, false, 0.1f, 0.0f,
                   0.0f);
    target.position_x = 101.0f;
    target.position_y = 0.0f;
    target.position_z = 0.0f;
    target.range_m = 101.0f;
    const common::TargetFeatureList input_state{target};
    FakeRadarContext radar_context(input_state);

    environment::EnvironmentModelConfig env_config;
    env_config.jammer_power_db = 0.0f;
    environment::EnvironmentService environment_service(env_config);

    signal::pipeline::SignalPipeline signal_pipeline;

    signal::tracking::AssociationTrackSeed side_channel_seed;
    side_channel_seed.association_key = 777u;
    side_channel_seed.has_position = true;
    side_channel_seed.position = Eigen::Vector3f(100.0f, 0.0f, 0.0f);
    side_channel_seed.has_gaussian_state = true;
    side_channel_seed.gaussian_state.mean = signal::tracking::StateVector::Zero();
    side_channel_seed.gaussian_state.mean(0) = 100.0f;
    side_channel_seed.gaussian_state.covariance =
      signal::tracking::StateCovariance::Identity() * 25.0f;
    signal_pipeline.SetAssociationSeeds(
      std::vector<signal::tracking::AssociationTrackSeed>(1, side_channel_seed));

    core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);

    controller.RunOnce();

    const std::vector<signal::tracking::TrackMeasurement> measurements =
      signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(measurements.size(), 1u);
    EXPECT_NE(measurements[0].raw_measurement.association_key, 777u);
    EXPECT_FALSE(measurements[0].raw_measurement.used_external_association_seeds);
  }

  TEST_F(CoreControllerTest,
       NoLifecycleManagerUsesStatelessAssociationByDefaultAcrossCycles) {
    common::TargetFeature target(10.0f, 0.0f, 0.0f, 0.2f, false, 0.1f, 0.0f,
                   0.0f);
    target.position_x = 101.0f;
    target.position_y = 0.0f;
    target.position_z = 0.0f;
    target.range_m = 101.0f;
    const common::TargetFeatureList input_state{target};
    FakeRadarContext radar_context(input_state);

    environment::EnvironmentModelConfig env_config;
    env_config.jammer_power_db = 0.0f;
    environment::EnvironmentService environment_service(env_config);

    signal::pipeline::SignalPipeline signal_pipeline;

    core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);

    controller.RunOnce();
    const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(first_measurements.size(), 1u);
    EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);
    const std::uint64_t first_key =
        first_measurements[0].raw_measurement.association_key;
    EXPECT_NE(first_key, 0u);

    controller.RunOnce();
    const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(second_measurements.size(), 1u);
    EXPECT_FALSE(second_measurements[0].raw_measurement.matched_existing_track);
    EXPECT_NE(second_measurements[0].raw_measurement.association_key, first_key);
  }

  TEST_F(CoreControllerTest,
       AutoAssemblesLifecycleManagerFromPipelineConfigWhenEnabled) {
    common::TargetFeature target(15.0f, 0.0f, 0.0f, 0.8f, false, 0.1f, 0.0f,
                   0.0f);
    target.position_x = 80.0f;
    target.position_y = 0.0f;
    target.position_z = 0.0f;
    target.range_m = 80.0f;
    const common::TargetFeatureList input_state{target};
    FakeRadarContext radar_context(input_state);

    environment::EnvironmentModelConfig env_config;
    env_config.jammer_power_db = 0.0f;
    environment::EnvironmentService environment_service(env_config);

    signal::pipeline::SignalPipelineConfig pipeline_config;
    pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
    pipeline_config.lifecycle.lifecycle_config.confirm_hits = 1;
    signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);

    core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);

    controller.RunOnce();
    const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(first_measurements.size(), 1u);
    EXPECT_FALSE(first_measurements[0].raw_measurement.matched_existing_track);

    controller.RunOnce();
    const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(second_measurements.size(), 1u);
    EXPECT_TRUE(second_measurements[0].raw_measurement.matched_existing_track);
    EXPECT_TRUE(
        second_measurements[0].raw_measurement.used_external_association_seeds);
  }

  TEST_F(CoreControllerTest,
       AutoAssembleImmLifecycleFailsFastOnInvalidInitialWeights) {
    common::TargetFeature target(20.0f, 0.0f, 0.0f, 1.0f, false, 0.1f, 0.0f,
                   0.0f);
    target.position_x = 60.0f;
    target.position_y = 0.0f;
    target.position_z = 0.0f;
    target.range_m = 60.0f;
    const common::TargetFeatureList input_state{target};
    FakeRadarContext radar_context(input_state);

    environment::EnvironmentModelConfig env_config;
    env_config.jammer_power_db = 0.0f;
    environment::EnvironmentService environment_service(env_config);

    signal::pipeline::SignalPipelineConfig pipeline_config;
    pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
    pipeline_config.lifecycle.enable_imm_lifecycle = true;
    pipeline_config.lifecycle.imm_model_noise_diff_coeffs =
        std::vector<float>{0.5f, 15.0f};
    pipeline_config.lifecycle.imm_initial_weights = std::vector<float>{1.0f};
    signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);

    core::controller::RadarController controller(
      radar_context, signal_pipeline, environment_service);

    EXPECT_DEATH_IF_SUPPORTED(controller.RunOnce(), ".*");
  }

} } // namespace airborne_radar::tests
