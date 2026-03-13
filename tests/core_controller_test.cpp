// Copyright 2026. All Rights Reserved.
//
// Description: 验证核心调度器使用桩组件的最小集成流程。

#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
#include <vector>

#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "1q/airborne_radar/core/event/RadarEvents.h"
#include "1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "airborne_radar/decision/classifier/TargetClassifier.h"
#include "airborne_radar/decision/eccm/EccmController.h"
#include "airborne_radar/decision/lpi/LpiController.h"
#include "airborne_radar/core/event/EventBus.h"
#include "airborne_radar/core/event/CycleEventBus.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar { namespace tests {

namespace {

common::TargetFeatureList BuildSingleTarget(float speed, float rcs, bool jamming) {
  common::TargetFeature target(speed, rcs, jamming);
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
      : state_(state) {}

  /// @brief 获取当前雷达状态。
  common::TargetFeatureList GetTargetFeatures() const override { return state_; }

  /// @brief 收集控制指令。
  void SubmitControlCommand(common::RadarCommand cmd) override {
    submitted_commands_.push_back(cmd);
  }

  /// @brief 获取已提交的指令集合。
  const std::vector<common::RadarCommand> &SubmittedCommands() const {
    return submitted_commands_;
  }

private:
  common::TargetFeatureList state_;
  std::vector<common::RadarCommand> submitted_commands_;
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
      snapshot.push_back(common::TargetFeature(measurement.velocity.norm(),
                                               measurement.rcs,
                                               measurement.jamming_detected,
                                               measurement.acceleration.norm()));
    }
    return snapshot;
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
      : seeds_(initial_seeds) {}

  void Update(const signal::tracking::CycleContext &cycle,
              const std::vector<signal::tracking::TrackMeasurement> &measurements) override {
    last_cycle = cycle;
    last_measurements = measurements;
  }

  common::TargetFeatureList BuildFeatureSnapshot() const override {
    return common::TargetFeatureList();
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

  std::vector<signal::tracking::AssociationTrackSeed>
  BuildAssociationSeeds() const override {
    return std::vector<signal::tracking::AssociationTrackSeed>();
  }

  signal::tracking::CycleContext last_cycle{};
  std::vector<signal::tracking::TrackMeasurement> last_measurements;
};

/// @brief CoreControllerTest 覆盖核心调度与指令下发路径。
class CoreControllerTest : public ::testing::Test {
protected:
  /// @brief 构建最小化的决策管线与核心调度器。
  void SetUp() override {
    auto classifier =
        std::unique_ptr<decision::classifier::TargetClassifier>(new decision::classifier::TargetClassifier());
    auto lpi = std::unique_ptr<decision::lpi::LpiController>(new decision::lpi::LpiController());
    auto eccm = std::unique_ptr<decision::eccm::EccmController>(new decision::eccm::EccmController());

    classifier->SetNext(std::move(lpi))->SetNext(std::move(eccm));
    decision_pipeline_ = std::move(classifier);
  }

  std::unique_ptr<decision::pipeline::ITacticalProcessor> decision_pipeline_;
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
      radar_context, signal_pipeline, *decision_pipeline_,
      environment_service);

  controller.RunOnce();

  const auto &cmds = radar_context.SubmittedCommands();
  EXPECT_GT(cmds.size(), 0u);
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
      radar_context, signal_pipeline, *decision_pipeline_,
      environment_service, event_bus);

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
  bool jamming_event_received = false;
  std::size_t submitted_commands = 0;

  event_bus.Subscribe<core::event::TracksUpdatedEvent>(
      [&tracks_event_received](const core::event::TracksUpdatedEvent &event) {
        tracks_event_received = std::any_of(
            event.state.begin(), event.state.end(),
            [](const common::TargetFeature &feature) {
              return feature.check_jamming_detected;
            });
      });

  event_bus.Subscribe<core::event::JammingAlertEvent>(
      [&jamming_event_received](const core::event::JammingAlertEvent &event) {
        jamming_event_received = event.detected;
      });

  event_bus.Subscribe<core::event::CommandsSubmittedEvent>(
      [&submitted_commands](const core::event::CommandsSubmittedEvent &event) {
        submitted_commands = event.command_count;
      });

  core::controller::RadarController controller(
      radar_context, signal_pipeline, *decision_pipeline_,
      environment_service, event_bus);

  controller.RunOnce();

  EXPECT_TRUE(tracks_event_received);
  EXPECT_TRUE(jamming_event_received);
  EXPECT_GT(submitted_commands, 0u);
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
      radar_context, signal_pipeline, *decision_pipeline_,
      environment_service, event_bus);

  controller.RunOnce();
  EXPECT_EQ(completed_event_hits, 0u);

  controller.RunOnce();
  EXPECT_EQ(completed_event_hits, 1u);
  EXPECT_GT(last_command_count, 0u);
}

TEST_F(CoreControllerTest, LifecycleManagerConsumesRealAssociationMeasurements) {
  common::TargetFeature first(100.0f, 2.0f, false, 1.0f);
  first.position_x = 10.0f;
  first.position_y = 0.0f;
  first.position_z = 0.0f;
  first.range_m = 10.0f;
  common::TargetFeature second(220.0f, 5.0f, false, 3.0f);
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
      radar_context, signal_pipeline, *decision_pipeline_, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  controller.RunOnce();

  ASSERT_EQ(lifecycle_manager.last_cycle.cycle_index, 1u);
  ASSERT_EQ(lifecycle_manager.last_measurements.size(), 2u);
  EXPECT_NE(lifecycle_manager.last_measurements[0].association_key, 0u);
  EXPECT_NE(lifecycle_manager.last_measurements[1].association_key, 0u);
  EXPECT_NE(lifecycle_manager.last_measurements[0].association_key,
            lifecycle_manager.last_measurements[1].association_key);
  EXPECT_EQ(lifecycle_manager.last_measurements[0].source_index, 0u);
  EXPECT_EQ(lifecycle_manager.last_measurements[1].source_index, 1u);
  EXPECT_FALSE(lifecycle_manager.last_measurements[0].matched_existing_track);
  EXPECT_FALSE(lifecycle_manager.last_measurements[1].matched_existing_track);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0].has_cartesian_position);
  EXPECT_TRUE(lifecycle_manager.last_measurements[1].has_cartesian_position);

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
  common::TargetFeature target(10.0f, 0.2f, false, 0.1f);
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
      radar_context, signal_pipeline, *decision_pipeline_, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  controller.RunOnce();

  ASSERT_EQ(lifecycle_manager.last_measurements.size(), 1u);
  EXPECT_EQ(lifecycle_manager.last_measurements[0].association_key, 42u);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0].matched_existing_track);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0].used_position_association);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0].used_external_association_seeds);
}

TEST_F(CoreControllerTest,
  EmptyLifecycleSeedsKeepAssociationStatelessAcrossCycles) {
  common::TargetFeature target(12.0f, 0.5f, false, 0.1f);
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
      radar_context, signal_pipeline, *decision_pipeline_, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  controller.RunOnce();
  ASSERT_EQ(lifecycle_manager.last_measurements.size(), 1u);
  const std::uint64_t first_key = lifecycle_manager.last_measurements[0].association_key;
  EXPECT_NE(first_key, 0u);
  EXPECT_FALSE(lifecycle_manager.last_measurements[0].matched_existing_track);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0].used_external_association_seeds);

  controller.RunOnce();
  ASSERT_EQ(lifecycle_manager.last_measurements.size(), 1u);
  const std::uint64_t second_key = lifecycle_manager.last_measurements[0].association_key;
  EXPECT_NE(second_key, 0u);
  EXPECT_NE(second_key, first_key);
  EXPECT_FALSE(lifecycle_manager.last_measurements[0].matched_existing_track);
  EXPECT_TRUE(lifecycle_manager.last_measurements[0].used_external_association_seeds);
}

TEST_F(CoreControllerTest,
       LifecycleSeedMissingGaussianStateFailsFastDuringRunCycle) {
  common::TargetFeature target(10.0f, 0.2f, false, 0.1f);
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
      radar_context, signal_pipeline, *decision_pipeline_, environment_service);
  controller.SetTrackLifecycleManager(&lifecycle_manager);

  EXPECT_DEATH_IF_SUPPORTED(controller.RunOnce(), "missing gaussian state");
}

  TEST_F(CoreControllerTest,
       NoLifecycleManagerScrubsSideChannelExternalSeedsBeforeRunCycle) {
    common::TargetFeature target(10.0f, 0.2f, false, 0.1f);
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
      radar_context, signal_pipeline, *decision_pipeline_, environment_service);

    controller.RunOnce();

    const std::vector<signal::tracking::TrackMeasurement> measurements =
      signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(measurements.size(), 1u);
    EXPECT_NE(measurements[0].association_key, 777u);
    EXPECT_FALSE(measurements[0].used_external_association_seeds);
  }

  TEST_F(CoreControllerTest,
       NoLifecycleManagerUsesStatelessAssociationByDefaultAcrossCycles) {
    common::TargetFeature target(10.0f, 0.2f, false, 0.1f);
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
      radar_context, signal_pipeline, *decision_pipeline_, environment_service);

    controller.RunOnce();
    const std::vector<signal::tracking::TrackMeasurement> first_measurements =
      signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(first_measurements.size(), 1u);
    EXPECT_FALSE(first_measurements[0].matched_existing_track);
    const std::uint64_t first_key = first_measurements[0].association_key;
    EXPECT_NE(first_key, 0u);

    controller.RunOnce();
    const std::vector<signal::tracking::TrackMeasurement> second_measurements =
      signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(second_measurements.size(), 1u);
    EXPECT_FALSE(second_measurements[0].matched_existing_track);
    EXPECT_NE(second_measurements[0].association_key, first_key);
  }

} } // namespace airborne_radar::tests
