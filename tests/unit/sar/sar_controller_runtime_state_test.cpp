/**
 * @file sar_controller_runtime_state_test.cpp
 * @brief 验证 SAR 控制器与处理流水线运行态快照契约。
 */

#include <gtest/gtest.h>

#include <cstdint>

#include "sar/pipeline/SarProcessingPipeline.h"
#include "sar/runtime/SarController.h"

namespace sar {
namespace extension {
namespace {

config::SarSessionConfig MakeSmallRdaConfig() {
  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 20.0;
  config.hardware.sample_rate_hz = 100.0e6;
  config.mission.nominal_slant_range_m = 29.9792458;
  config.mission.scene_center_latitude_deg =
      29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.policy.enable_l1_rda_imaging = true;
  return config;
}

session::SarCycleInput MakeInput(std::uint32_t cycle_index) {
  session::SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1f;
  input.platform.latitude_deg = 0.0;
  input.platform.longitude_deg = 0.0;
  input.platform.altitude_m = 0.0;

  session::SarPointTarget target;
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

TEST(SarControllerRuntimeStateTest, CaptureAndRestoreRoundTripState) {
  const config::SarSessionConfig config = MakeSmallRdaConfig();
  pipeline::SarProcessingPipeline pipeline(config);
  SarController controller(pipeline, config);

  controller.RunOnce(MakeInput(1U));
  ASSERT_TRUE(controller.BuildCycleResult(MakeInput(1U)).executed_this_cycle);
  const SarControllerRuntimeState snapshot = controller.CaptureRuntimeState();

  controller.RunOnce(MakeInput(2U));
  ASSERT_EQ(controller.BuildCycleResult(MakeInput(2U)).output_frame.cycle_index, 2U);

  EXPECT_TRUE(controller.RestoreRuntimeState(snapshot));
  const session::SarCycleResult restored = controller.BuildCycleResult(MakeInput(1U));
  EXPECT_TRUE(restored.executed_this_cycle);
  EXPECT_EQ(restored.output_frame.cycle_index, 1U);
  EXPECT_FALSE(restored.reused_previous_output);
}

TEST(SarControllerRuntimeStateTest, RestoreRejectsSnapshotFromOtherControllerInstance) {
  const config::SarSessionConfig config = MakeSmallRdaConfig();
  pipeline::SarProcessingPipeline pipeline_a(config);
  pipeline::SarProcessingPipeline pipeline_b(config);
  SarController controller_a(pipeline_a, config);
  SarController controller_b(pipeline_b, config);

  controller_a.RunOnce(MakeInput(20U));
  controller_b.RunOnce(MakeInput(30U));
  const SarControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();

  EXPECT_FALSE(controller_a.RestoreRuntimeState(foreign_state));
  EXPECT_EQ(controller_a.BuildCycleResult(MakeInput(20U)).output_frame.cycle_index, 20U);
}

TEST(SarControllerRuntimeStateTest, ValidationRejectReusesPreviousOutput) {
  const config::SarSessionConfig config = MakeSmallRdaConfig();
  pipeline::SarProcessingPipeline pipeline(config);
  SarController controller(pipeline, config);

  controller.RunOnce(MakeInput(40U));
  ASSERT_TRUE(controller.BuildCycleResult(MakeInput(40U)).executed_this_cycle);

  session::SarCycleInput invalid_input = MakeInput(41U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);
  const session::SarCycleResult result = controller.BuildCycleResult(invalid_input);

  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_TRUE(result.has_error);
  EXPECT_TRUE(result.reused_previous_output);
  EXPECT_EQ(result.abort_reason, "invalid_cycle_input");
  EXPECT_EQ(result.output_frame.cycle_index, 40U);
}

TEST(SarControllerRuntimeStateTest, PipelineAbortRestoresAllCrossCycleState) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  pipeline::SarProcessingPipeline pipeline(config);
  SarController controller(pipeline, config);
  const session::SarCycleInput successful_input = MakeInput(49U);
  controller.RunOnce(successful_input);
  ASSERT_TRUE(controller.BuildCycleResult(successful_input).executed_this_cycle);

  config::SarRuntimeConfigPatch reject_snr;
  reject_snr.has_minimum_snr_db = true;
  reject_snr.minimum_snr_db = 1000.0;
  ASSERT_TRUE(controller.TryApplyRuntimeConfig(reject_snr));
  const pipeline::SarProcessingPipelineRuntimeState before = pipeline.CaptureRuntimeState();

  const session::SarCycleInput input = MakeInput(50U);
  controller.RunOnce(input);
  const session::SarCycleResult result = controller.BuildCycleResult(input);
  const pipeline::SarProcessingPipelineRuntimeState after = pipeline.CaptureRuntimeState();

  ASSERT_FALSE(result.executed_this_cycle);
  ASSERT_EQ(result.abort_reason, "snr_below_minimum");
  EXPECT_TRUE(result.reused_previous_output);
  EXPECT_EQ(result.output_frame.cycle_index, successful_input.cycle_index);
  EXPECT_EQ(after.next_pulse_id, before.next_pulse_id);
  EXPECT_DOUBLE_EQ(after.pulse_fraction_carry, before.pulse_fraction_carry);
  EXPECT_EQ(after.raw_pulse_buffer_state.records.size(),
            before.raw_pulse_buffer_state.records.size());
  for (std::size_t index = 0U; index < before.raw_pulse_buffer_state.records.size(); ++index) {
    EXPECT_EQ(after.raw_pulse_buffer_state.records[index].pulse_id,
              before.raw_pulse_buffer_state.records[index].pulse_id);
    EXPECT_EQ(after.raw_pulse_buffer_state.records[index].samples,
              before.raw_pulse_buffer_state.records[index].samples);
  }
  EXPECT_EQ(after.raw_pulse_buffer_state.overflow_sticky,
            before.raw_pulse_buffer_state.overflow_sticky);
  EXPECT_EQ(after.ideal_trajectory_buffer.size(), before.ideal_trajectory_buffer.size());
  EXPECT_EQ(after.actual_trajectory_buffer.size(), before.actual_trajectory_buffer.size());
  ASSERT_FALSE(before.actual_trajectory_buffer.empty());
  EXPECT_EQ(after.actual_trajectory_buffer.front().pulse_id,
            before.actual_trajectory_buffer.front().pulse_id);
  EXPECT_DOUBLE_EQ(after.actual_trajectory_buffer.front().time_s,
                   before.actual_trajectory_buffer.front().time_s);
  EXPECT_EQ(after.actual_trajectory_buffer.back().pulse_id,
            before.actual_trajectory_buffer.back().pulse_id);
  EXPECT_DOUBLE_EQ(after.actual_trajectory_buffer.back().time_s,
                   before.actual_trajectory_buffer.back().time_s);
}

}  // namespace
}  // namespace extension
}  // namespace sar
