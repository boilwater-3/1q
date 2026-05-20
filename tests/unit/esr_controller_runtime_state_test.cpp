/**
 * @file esr_controller_runtime_state_test.cpp
 * @brief 验证 ESR 控制器运行态快照与失败回退契约。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "1q/electronic_surveillance_radar/extension/EsrController.h"
#include "1q/electronic_surveillance_radar/extension/IInterceptPipeline.h"
#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"

namespace electronic_surveillance_radar {
namespace extension {
namespace {

class StubEnvironmentService final : public environment::IEsrEnvironmentService {
 public:
  void BeginCycle(const environment::EsrEnvironmentCycleContext&) override {}
  environment::EsrEnvironmentSnapshot SampleEnvironment() const override {
    return environment::EsrEnvironmentSnapshot{};
  }
  void UpdateModelConfig(environment::EsrEnvironmentModelConfig) override {}
};

class MockInterceptPipeline final : public IInterceptPipeline {
 public:
  void UpdateConfig(InterceptPipelineConfig config) override { last_config_ = config; }
  void UpdateRuntimeConfig(InterceptRuntimeConfig runtime_config) override {
    last_runtime_config_ = runtime_config;
  }

  InterceptPipelineRuntimeState CaptureRuntimeState() const override {
    auto snapshot = std::make_shared<int>(execute_count_);
    InterceptPipelineRuntimeState state;
    state.owner_identity = this;
    state.schema_version = 1U;
    state.snapshot = snapshot;
    return state;
  }

  bool RestoreRuntimeState(const InterceptPipelineRuntimeState& state) override {
    if (force_restore_reject_) {
      return false;
    }
    if (state.owner_identity != this || state.schema_version != 1U || state.snapshot == nullptr) {
      return false;
    }
    execute_count_ = *static_cast<const int*>(state.snapshot.get());
    return true;
  }

  InterceptPipelineResult Execute(
      const session::EsrCycleInput& input,
      const environment::IEsrEnvironmentService& environment) override {
    (void)input;
    (void)environment;
    ++execute_count_;

    InterceptPipelineResult result;
    result.observation_output.raw_observation_count = static_cast<std::size_t>(execute_count_);
    return result;
  }

  bool force_abort_{false};
  bool force_restore_reject_{false};
  int execute_count_{0};
  InterceptPipelineConfig last_config_{};
  InterceptRuntimeConfig last_runtime_config_{};
};

session::EsrCycleInput MakeValidInput(std::uint32_t cycle_index) {
  session::EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0f;
  return input;
}

}  // namespace

TEST(EsrControllerRuntimeStateTest, CaptureAndRestoreRoundTripState) {
  MockInterceptPipeline pipeline;
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  controller.RunOnce(MakeValidInput(1U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());
  const EsrControllerRuntimeState snapshot = controller.CaptureRuntimeState();

  controller.RunOnce(MakeValidInput(2U));
  ASSERT_EQ(controller.GetLatestOutputFrame().cycle_index, 2U);

  controller.RestoreRuntimeState(snapshot);
  EXPECT_TRUE(controller.HasLatestOutputFrame());
  EXPECT_EQ(controller.GetLatestOutputFrame().cycle_index, 1U);
  EXPECT_TRUE(controller.ExecutedLatestCycle());
  EXPECT_FALSE(controller.ReusedPreviousOutputLatestCycle());
}

TEST(EsrControllerRuntimeStateTest, RestoreRejectsIncompatiblePipelineSnapshot) {
  MockInterceptPipeline pipeline_a;
  MockInterceptPipeline pipeline_b;
  StubEnvironmentService env;
  EsrController controller_a(pipeline_a, env);
  EsrController controller_b(pipeline_b, env);

  controller_a.RunOnce(MakeValidInput(20U));
  controller_b.RunOnce(MakeValidInput(30U));

  EsrControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();
  controller_a.RestoreRuntimeState(foreign_state);

  EXPECT_EQ(controller_a.GetLatestOutputFrame().cycle_index, 20U);
}

TEST(EsrControllerRuntimeStateTest, RestoreRejectsSnapshotFromOtherControllerInstance) {
  MockInterceptPipeline shared_pipeline;
  StubEnvironmentService env;
  EsrController controller_a(shared_pipeline, env);
  EsrController controller_b(shared_pipeline, env);

  controller_a.RunOnce(MakeValidInput(21U));
  controller_b.RunOnce(MakeValidInput(31U));

  const EsrControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();
  controller_a.RestoreRuntimeState(foreign_state);

  EXPECT_EQ(controller_a.GetLatestOutputFrame().cycle_index, 21U);
}

TEST(EsrControllerRuntimeStateTest, ValidationRejectSetsAbortReasonAndReusesPreviousOutput) {
  MockInterceptPipeline pipeline;
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  controller.RunOnce(MakeValidInput(40U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());

  session::EsrCycleInput invalid_input = MakeValidInput(41U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_FALSE(controller.ExecutedLatestCycle());
  EXPECT_TRUE(controller.ReusedPreviousOutputLatestCycle());
  EXPECT_EQ(controller.GetLastAbortReason(), EsrPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(controller.GetLatestOutputFrame().cycle_index, 40U);
}

TEST(EsrControllerRuntimeStateTest, FirstValidationRejectBuildsEmptyOutputFrame) {
  MockInterceptPipeline pipeline;
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  session::EsrCycleInput invalid_input = MakeValidInput(45U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_TRUE(controller.HasLatestOutputFrame());
  EXPECT_FALSE(controller.ExecutedLatestCycle());
  EXPECT_EQ(controller.GetLastAbortReason(), EsrPipelineAbortReason::kValidationRejected);
}

TEST(EsrControllerRuntimeStateTest,
     SessionCaptureRestoreSkipsRollbackOnValidationRejection) {
  MockInterceptPipeline pipeline;
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  controller.RunOnce(MakeValidInput(100U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());

  const auto pipeline_state = pipeline.CaptureRuntimeState();
  const auto controller_state = controller.CaptureRuntimeState();

  session::EsrCycleInput invalid_input = MakeValidInput(101U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_FALSE(controller.ExecutedLatestCycle());
  EXPECT_EQ(controller.GetLastAbortReason(), EsrPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(controller.GetLatestOutputFrame().cycle_index, 100U);
  EXPECT_TRUE(controller.ReusedPreviousOutputLatestCycle());
}

}  // namespace extension
}  // namespace electronic_surveillance_radar
