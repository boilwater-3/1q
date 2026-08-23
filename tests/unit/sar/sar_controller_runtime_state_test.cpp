/**
 * @file sar_controller_runtime_state_test.cpp
 * @brief 验证 SAR 控制器与处理流水线运行态快照契约。
 */

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/sar/session/SarInputValidation.h"
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
  ASSERT_EQ(controller.BuildCycleResult().status, session::SarCycleStatus::kCompleted);
  const SarControllerRuntimeState snapshot = controller.CaptureRuntimeState();

  controller.RunOnce(MakeInput(2U));
  ASSERT_EQ(controller.BuildCycleResult().product.output_frame.cycle_index, 2U);

  EXPECT_TRUE(controller.RestoreRuntimeState(snapshot));
  const session::SarCycleResult restored = controller.BuildCycleResult();
  EXPECT_EQ(restored.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(restored.product.output_frame.cycle_index, 1U);
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
  EXPECT_EQ(controller_a.BuildCycleResult().product.output_frame.cycle_index, 20U);
}

TEST(SarControllerRuntimeStateTest, ValidationRejectReturnsEmptyOutputNotReused) {
  const config::SarSessionConfig config = MakeSmallRdaConfig();
  pipeline::SarProcessingPipeline pipeline(config);
  SarController controller(pipeline, config);

  controller.RunOnce(MakeInput(40U));
  ASSERT_EQ(controller.BuildCycleResult().status, session::SarCycleStatus::kCompleted);

  session::SarCycleInput invalid_input = MakeInput(41U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);
  const session::SarCycleResult result = controller.BuildCycleResult();

  EXPECT_NE(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_TRUE(session::HasValidationError(result.issues));
  EXPECT_EQ(result.abort_reason, session::SarPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(result.product.output_frame.cycle_index, 0U);
}

TEST(SarControllerRuntimeStateTest, PipelineAbortRestoresAllCrossCycleState) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  pipeline::SarProcessingPipeline pipeline(config);
  SarController controller(pipeline, config);
  const session::SarCycleInput successful_input = MakeInput(49U);
  controller.RunOnce(successful_input);
  ASSERT_EQ(controller.BuildCycleResult().status, session::SarCycleStatus::kCompleted);

  config::SarRuntimeConfigPatch reject_snr;
  reject_snr.has_minimum_snr_db = true;
  reject_snr.minimum_snr_db = 1000.0;
  ASSERT_TRUE(controller.TryApplyRuntimeConfig(reject_snr));
  const pipeline::SarProcessingPipelineRuntimeState before = pipeline.CaptureRuntimeState();

  const session::SarCycleInput input = MakeInput(50U);
  controller.RunOnce(input);
  const session::SarCycleResult result = controller.BuildCycleResult();
  const pipeline::SarProcessingPipelineRuntimeState after = pipeline.CaptureRuntimeState();

  ASSERT_NE(result.status, session::SarCycleStatus::kCompleted);
  ASSERT_EQ(result.abort_reason, session::SarPipelineAbortReason::kPipelineExecutionFailed);
  // Pipeline abort 后 output_frame 保留已初始化的元数据（cycle_index = input.cycle_index），
  // 区别于校验失败的默认空帧（cycle_index = 0）。
  EXPECT_EQ(result.product.output_frame.cycle_index, input.cycle_index);
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

TEST(SarControllerRuntimeStateTest, PlatformInputOwnsGeneratedTrajectoryKinematics) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l1_rda_imaging = false;
  pipeline::SarProcessingPipeline pipeline(config);
  SarController controller(pipeline, config);

  session::SarCycleInput first = MakeInput(60U);
  first.platform.time_s = 10.0;
  first.platform.latitude_deg = config.mission.scene_center_latitude_deg;
  first.platform.velocity_north_mps = 3.0;
  first.platform.velocity_east_mps = 4.0;
  first.platform.velocity_down_mps = 1.0;
  first.platform.roll_deg = 2.0;
  first.platform.pitch_deg = 3.0;
  first.platform.yaw_deg = 4.0;
  controller.RunOnce(first);
  const session::SarCycleResult first_result = controller.BuildCycleResult();
  ASSERT_EQ(first_result.status, session::SarCycleStatus::kCompleted)
      << static_cast<int>(first_result.abort_reason);
  const pipeline::SarProcessingPipelineRuntimeState first_state = pipeline.CaptureRuntimeState();
  ASSERT_FALSE(first_state.actual_trajectory_buffer.empty());
  const geometry::PlatformPulseState& first_pulse = first_state.actual_trajectory_buffer.front();
  EXPECT_DOUBLE_EQ(first_pulse.time_s, 10.0);
  EXPECT_DOUBLE_EQ(first_pulse.position_m.x_m, 0.0);
  EXPECT_NEAR(first_pulse.position_m.y_m, 0.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(first_pulse.velocity_x_mps, 4.0);
  EXPECT_DOUBLE_EQ(first_pulse.velocity_y_mps, 3.0);
  EXPECT_DOUBLE_EQ(first_pulse.velocity_z_mps, -1.0);
  EXPECT_DOUBLE_EQ(first_pulse.roll_deg, 2.0);
  EXPECT_DOUBLE_EQ(first_pulse.pitch_deg, 3.0);
  EXPECT_DOUBLE_EQ(first_pulse.yaw_deg, 4.0);

  session::SarCycleInput second = first;
  second.cycle_index = 61U;
  second.platform.time_s = 11.0;
  second.platform.latitude_deg += 5.0 / 6378137.0 * 180.0 / 3.14159265358979323846;
  controller.RunOnce(second);
  ASSERT_EQ(controller.BuildCycleResult().status, session::SarCycleStatus::kCompleted);
  const pipeline::SarProcessingPipelineRuntimeState second_state = pipeline.CaptureRuntimeState();
  ASSERT_FALSE(second_state.actual_trajectory_buffer.empty());
  const geometry::PlatformPulseState* second_cycle_first = nullptr;
  for (const geometry::PlatformPulseState& pulse : second_state.actual_trajectory_buffer) {
    if (pulse.time_s == 11.0) {
      second_cycle_first = &pulse;
      break;
    }
  }
  ASSERT_NE(second_cycle_first, nullptr);
  EXPECT_NEAR(second_cycle_first->position_m.y_m, 5.0, 1.0e-6);
}

TEST(SarControllerRuntimeStateTest, ZeroPlatformVelocityMeansStationary) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l1_rda_imaging = false;
  pipeline::SarProcessingPipeline pipeline(config);
  SarController controller(pipeline, config);

  session::SarCycleInput input = MakeInput(62U);
  input.platform.time_s = 12.0;
  controller.RunOnce(input);
  const session::SarCycleResult result = controller.BuildCycleResult();
  ASSERT_EQ(result.status, session::SarCycleStatus::kCompleted)
      << static_cast<int>(result.abort_reason);

  const pipeline::SarProcessingPipelineRuntimeState state = pipeline.CaptureRuntimeState();
  ASSERT_GE(state.actual_trajectory_buffer.size(), 2U);
  const geometry::PlatformPulseState& first = state.actual_trajectory_buffer.front();
  const geometry::PlatformPulseState& last = state.actual_trajectory_buffer.back();
  EXPECT_DOUBLE_EQ(first.velocity_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(first.velocity_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(first.velocity_z_mps, 0.0);
  EXPECT_DOUBLE_EQ(last.position_m.x_m, first.position_m.x_m);
  EXPECT_DOUBLE_EQ(last.position_m.y_m, first.position_m.y_m);
  EXPECT_DOUBLE_EQ(last.position_m.z_m, first.position_m.z_m);
}

}  // namespace
}  // namespace extension
}  // namespace sar
