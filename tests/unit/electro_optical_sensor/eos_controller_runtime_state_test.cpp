/**
 * @file eos_controller_runtime_state_test.cpp
 * @brief 验证 EOS 控制器运行态快照与失败回退契约。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "electro_optical_sensor/runtime/EosController.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "electro_optical_sensor/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace extension {
namespace {

config::execution::EosInternalExecutionConfig MakePipelineConfig() {
  config::execution::EosInternalExecutionConfig config;
  config.scan.scan_start_az_deg = -60.0f;
  config.scan.scan_end_az_deg = 60.0f;
  config.scan.scan_rate_deg_per_sec = 5.0f;
  config.scan.horizontal_fov_deg = 20.0f;
  config.scan.vertical_fov_deg = 4.0f;
  config.scan.work_mode = config::EosWorkMode::kInfraredOnly;
  return config;
}

::electro_optical_sensor::session::EosCycleInput MakeValidInput(std::uint32_t cycle_index) {
  ::electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1f;
  return input;
}

TEST(EosControllerRuntimeStateTest, CaptureAndRestoreRoundTripState) {
  signal::pipeline::EosPipeline pipeline(MakePipelineConfig());
  EosController controller(pipeline);

  controller.RunOnce(MakeValidInput(1U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());
  const EosControllerRuntimeState snapshot = controller.CaptureRuntimeState();

  controller.RunOnce(MakeValidInput(2U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());

  controller.RestoreRuntimeState(snapshot);
  EXPECT_TRUE(controller.ExecutedLatestCycle());
}

TEST(EosControllerRuntimeStateTest, RestoreRejectsIncompatiblePipelineSnapshot) {
  signal::pipeline::EosPipeline pipeline_a(MakePipelineConfig());
  signal::pipeline::EosPipeline pipeline_b(MakePipelineConfig());
  EosController controller_a(pipeline_a);
  EosController controller_b(pipeline_b);

  controller_a.RunOnce(MakeValidInput(20U));
  controller_b.RunOnce(MakeValidInput(30U));

  EosControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();
  EXPECT_FALSE(controller_a.RestoreRuntimeState(foreign_state));
  // 拒绝恢复后，controller_a 状态保持不变。
  EXPECT_TRUE(controller_a.ExecutedLatestCycle());
  const auto result = controller_a.BuildCycleResult();
  EXPECT_EQ(result.output_frame.cycle_index, 20U);
}

TEST(EosControllerRuntimeStateTest, RestoreRejectsSnapshotFromOtherControllerInstance) {
  signal::pipeline::EosPipeline shared_pipeline(MakePipelineConfig());
  EosController controller_a(shared_pipeline);
  EosController controller_b(shared_pipeline);

  controller_a.RunOnce(MakeValidInput(21U));
  controller_b.RunOnce(MakeValidInput(31U));

  const EosControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();
  EXPECT_FALSE(controller_a.RestoreRuntimeState(foreign_state));
  // 拒绝恢复后，controller_a 状态保持不变。
  EXPECT_TRUE(controller_a.ExecutedLatestCycle());
  const auto result = controller_a.BuildCycleResult();
  EXPECT_EQ(result.output_frame.cycle_index, 21U);
}

TEST(EosControllerRuntimeStateTest, ValidationRejectSetsAbortReasonAndReturnsDefaultFrame) {
  signal::pipeline::EosPipeline pipeline(MakePipelineConfig());
  EosController controller(pipeline);

  controller.RunOnce(MakeValidInput(40U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());

  ::electro_optical_sensor::session::EosCycleInput invalid_input = MakeValidInput(41U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_FALSE(controller.ExecutedLatestCycle());
  const auto result = controller.BuildCycleResult();
  EXPECT_EQ(result.abort_reason, EosPipelineAbortReason::kValidationRejected);
  EXPECT_TRUE(::electro_optical_sensor::session::HasValidationError(result.issues));
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
}

TEST(EosControllerRuntimeStateTest, FirstValidationRejectReturnsDefaultFrame) {
  signal::pipeline::EosPipeline pipeline(MakePipelineConfig());
  EosController controller(pipeline);

  ::electro_optical_sensor::session::EosCycleInput invalid_input = MakeValidInput(45U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);
  const ::electro_optical_sensor::session::EosCycleResult result =
      controller.BuildCycleResult();

  EXPECT_NE(result.status, session::EosCycleStatus::kCompleted);
  EXPECT_EQ(result.abort_reason, EosPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
}

}  // namespace
}  // namespace extension
}  // namespace electro_optical_sensor
