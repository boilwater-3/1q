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
#include "airborne_radar/decision/classifier/TargetClassifier.h"
#include "airborne_radar/decision/eccm/EccmController.h"
#include "airborne_radar/decision/lpi/LpiController.h"
#include "airborne_radar/core/event/EventBus.h"
#include "airborne_radar/core/event/CycleEventBus.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar::tests {

namespace {

common::TargetFeatureList BuildSingleTarget(float speed, float rcs, bool jamming) {
  return common::TargetFeatureList{common::TargetFeature(speed, rcs, jamming)};
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

/// @brief CoreControllerTest 覆盖核心调度与指令下发路径。
class CoreControllerTest : public ::testing::Test {
protected:
  /// @brief 构建最小化的决策管线与核心调度器。
  void SetUp() override {
    auto classifier =
        std::make_unique<decision::classifier::TargetClassifier>();
    auto lpi = std::make_unique<decision::lpi::LpiController>();
    auto eccm = std::make_unique<decision::eccm::EccmController>();

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

} // namespace airborne_radar::tests
