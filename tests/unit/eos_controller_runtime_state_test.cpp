/**
 * @file eos_controller_runtime_state_test.cpp
 * @brief 验证 EOS 控制器运行态快照与失败回退契约。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"

namespace electro_optical_sensor {
namespace extension {
namespace {

class TrackingPipeline final : public IEosPipeline {
 public:
  void UpdateConfig(const EosPipelineConfig& config, bool reset_scan_phase) override {
    (void)config;
    if (reset_scan_phase) {
      scan_azimuth_deg = -5.0f;
    }
  }

  EosPipelineRuntimeState CaptureRuntimeState() const override {
    EosPipelineRuntimeState state;
    state.owner_identity = this;
    state.schema_version = 1U;
    state.current_scan_azimuth_deg = scan_azimuth_deg;
    state.scan_start_az_deg = -5.0f;
    state.scan_end_az_deg = 5.0f;
    state.scan_rate_deg_per_sec = 2.0f;
    return state;
  }

  bool RestoreRuntimeState(const EosPipelineRuntimeState& state) override {
    if (force_restore_reject) {
      return false;
    }
    if (state.owner_identity != this || state.schema_version != 1U) {
      return false;
    }
    scan_azimuth_deg = state.current_scan_azimuth_deg;
    return true;
  }

  EosPipelineExecuteResult Execute(const session::EosCycleInput& input) override {
    ++execute_count;
    scan_azimuth_deg += 2.0f;

    EosPipelineExecuteResult result;
    result.output_frame.cycle_index = emit_cycle_index_mismatch ? input.cycle_index + 100U
                                                                : input.cycle_index;
    result.output_frame.scan_azimuth_deg = scan_azimuth_deg;
    result.executed_this_cycle = !force_abort;
    result.abort_reason = emit_abort_on_executed_cycle
                              ? EosPipelineAbortReason::kOutputContractViolation
                              : (force_abort ? EosPipelineAbortReason::kOutputContractViolation
                                             : EosPipelineAbortReason::kNone);
    return result;
  }

  bool force_abort{false};
  bool force_restore_reject{false};
  bool emit_cycle_index_mismatch{false};
  bool emit_abort_on_executed_cycle{false};
  std::size_t execute_count{0U};
  float scan_azimuth_deg{-5.0f};
};

session::EosCycleInput MakeValidInput(std::uint32_t cycle_index) {
  session::EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0f;
  input.solar_irradiance_w_m2 = 800.0f;
  input.solar_altitude_deg = 45.0f;
  input.cloud_coverage_ratio = 0.2f;
  input.background_temperature_k = 289.0f;
  input.day_night_type = session::DayNightType::kDay;
  return input;
}

TEST(EosControllerRuntimeStateTest, CaptureAndRestoreRoundTripState) {
  TrackingPipeline pipeline;
  EosController controller(pipeline);

  controller.RunOnce(MakeValidInput(1U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());
  const EosControllerRuntimeState snapshot = controller.CaptureRuntimeState();

  controller.RunOnce(MakeValidInput(2U));
  ASSERT_EQ(controller.GetLatestOutputFrame().cycle_index, 2U);

  controller.RestoreRuntimeState(snapshot);
  EXPECT_TRUE(controller.HasLatestOutputFrame());
  EXPECT_EQ(controller.GetLatestOutputFrame().cycle_index, 1U);
  EXPECT_TRUE(controller.ExecutedLatestCycle());
  EXPECT_FALSE(controller.ReusedPreviousOutputLatestCycle());
}

TEST(EosControllerRuntimeStateTest, ExecuteAbortRestoresPipelineStateAndReusesPreviousOutput) {
  TrackingPipeline pipeline;
  EosController controller(pipeline);

  controller.RunOnce(MakeValidInput(10U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());
  const float baseline_scan_azimuth = pipeline.scan_azimuth_deg;
  const output::EosOutputFrame baseline_output = controller.GetLatestOutputFrame();

  pipeline.force_abort = true;
  controller.RunOnce(MakeValidInput(11U));

  EXPECT_FALSE(controller.ExecutedLatestCycle());
  EXPECT_TRUE(controller.ReusedPreviousOutputLatestCycle());
  EXPECT_EQ(controller.GetLastAbortReason(), EosPipelineAbortReason::kOutputContractViolation);
  EXPECT_EQ(controller.GetLatestOutputFrame().cycle_index, baseline_output.cycle_index);
  EXPECT_FLOAT_EQ(pipeline.scan_azimuth_deg, baseline_scan_azimuth);
}

TEST(EosControllerRuntimeStateTest, RestoreRejectsIncompatiblePipelineSnapshot) {
  TrackingPipeline pipeline_a;
  TrackingPipeline pipeline_b;
  EosController controller_a(pipeline_a);
  EosController controller_b(pipeline_b);

  controller_a.RunOnce(MakeValidInput(20U));
  controller_b.RunOnce(MakeValidInput(30U));

  EosControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();
  controller_a.RestoreRuntimeState(foreign_state);

  EXPECT_EQ(controller_a.GetLatestOutputFrame().cycle_index, 20U);
}

TEST(EosControllerRuntimeStateTest, RestoreRejectsSnapshotFromOtherControllerInstance) {
  TrackingPipeline shared_pipeline;
  EosController controller_a(shared_pipeline);
  EosController controller_b(shared_pipeline);

  controller_a.RunOnce(MakeValidInput(21U));
  controller_b.RunOnce(MakeValidInput(31U));

  const EosControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();
  controller_a.RestoreRuntimeState(foreign_state);

  EXPECT_EQ(controller_a.GetLatestOutputFrame().cycle_index, 21U);
}

TEST(EosControllerRuntimeStateTest, ValidationRejectSetsAbortReasonAndReusesPreviousOutput) {
  TrackingPipeline pipeline;
  EosController controller(pipeline);

  controller.RunOnce(MakeValidInput(40U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());

  session::EosCycleInput invalid_input = MakeValidInput(41U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_TRUE(controller.HasValidationError());
  EXPECT_FALSE(controller.ExecutedLatestCycle());
  EXPECT_TRUE(controller.ReusedPreviousOutputLatestCycle());
  EXPECT_EQ(controller.GetLastAbortReason(), EosPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(controller.GetLatestOutputFrame().cycle_index, 40U);
}

TEST(EosControllerRuntimeStateTest, FirstValidationRejectDoesNotSynthesizeLatestOutput) {
  TrackingPipeline pipeline;
  EosController controller(pipeline);

  session::EosCycleInput invalid_input = MakeValidInput(45U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);
  const model::EosCycleResult result = controller.BuildCycleResult(invalid_input);

  EXPECT_FALSE(controller.HasLatestOutputFrame());
  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_EQ(result.abort_reason, EosPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(result.output_frame.cycle_index, 45U);
}

TEST(EosControllerRuntimeStateTest,
     BuildCycleResultOnFirstAbortUsesInputCycleIndexWithoutSynthesizingReuse) {
  TrackingPipeline pipeline;
  EosController controller(pipeline);

  pipeline.force_abort = true;
  const session::EosCycleInput input = MakeValidInput(50U);
  controller.RunOnce(input);
  const model::EosCycleResult result = controller.BuildCycleResult(input);

  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_EQ(result.abort_reason, EosPipelineAbortReason::kOutputContractViolation);
  EXPECT_EQ(result.output_frame.cycle_index, input.cycle_index);
}

TEST(EosControllerRuntimeStateTest,
     ExecutedCycleWithMismatchedOutputCycleIndexFallsBackToPreviousOutput) {
  TrackingPipeline pipeline;
  EosController controller(pipeline);

  controller.RunOnce(MakeValidInput(60U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());

  pipeline.emit_cycle_index_mismatch = true;
  controller.RunOnce(MakeValidInput(61U));

  EXPECT_FALSE(controller.ExecutedLatestCycle());
  EXPECT_TRUE(controller.ReusedPreviousOutputLatestCycle());
  EXPECT_EQ(controller.GetLastAbortReason(), EosPipelineAbortReason::kOutputContractViolation);
  EXPECT_EQ(controller.GetLatestOutputFrame().cycle_index, 60U);
}

TEST(EosControllerRuntimeStateTest, RestoreRejectDuringRollbackReturnsHardFailure) {
  TrackingPipeline pipeline;
  EosController controller(pipeline);

  controller.RunOnce(MakeValidInput(70U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());

  pipeline.force_abort = true;
  pipeline.force_restore_reject = true;
  controller.RunOnce(MakeValidInput(71U));
  const model::EosCycleResult result = controller.BuildCycleResult(MakeValidInput(71U));

  EXPECT_FALSE(controller.HasLatestOutputFrame());
  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_EQ(result.abort_reason, EosPipelineAbortReason::kRuntimeStateRestoreRejected);
}

}  // namespace
}  // namespace extension
}  // namespace electro_optical_sensor
