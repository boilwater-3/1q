/**
 * @file esr_controller_runtime_state_test.cpp
 * @brief 验证 ESR 控制器运行态快照与失败回退契约。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>

#include "electronic_surveillance_radar/runtime/EsrController.h"
#include "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"
#include "electronic_surveillance_radar/pipeline/PipelineRuntimeSnapshot.h"

namespace electronic_surveillance_radar {
namespace extension {
namespace {

EsrInternalExecutionConfig MakeDefaultConfig() {
  EsrInternalExecutionConfig config;
  config.mission.power_on = true;
  config.mission.scan.scan_rate_hz = 1.0f;
  config.resolved_scan.scan_start_az_deg = -60.0f;
  config.resolved_scan.scan_end_az_deg = 60.0f;
  config.resolved_scan.scan_start_el_deg = -10.0f;
  config.resolved_scan.scan_end_el_deg = 10.0f;
  config.resolved_scan.az_step_deg = 5.0f;
  config.resolved_scan.el_step_deg = 5.0f;
  config.detection.minimum_snr_db = 6.0f;
  config.detection.pfa = 1.0e-6f;
  config.detection.pulse_count = 8U;
  config.hardware.receiver_sensitivity_w = 1.0e-12f;
  config.hardware.integrated_receive_loss_db = 0.0f;
  config.hardware.antenna_mount_az_deg = 0.0f;
  config.hardware.antenna_mount_el_deg = 0.0f;
  return config;
}

class StubEnvironmentService final : public environment::IEsrEnvironmentService {
 public:
  void BeginCycle(const session::EsrEnvironmentCycleContext&) override {}
  session::EsrEnvironmentSnapshot SampleEnvironment() const override {
    return session::EsrEnvironmentSnapshot{};
  }
  void UpdateModelConfig(config::EsrEnvironmentScenarioConfig) override {}
};

session::EsrCycleInput MakeValidInput(std::uint32_t cycle_index) {
  session::EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index);
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  return input;
}

double ReadScanPhase(const pipeline::InterceptPipeline& pipeline) {
  const extension::InterceptPipelineRuntimeState state = pipeline.CaptureRuntimeState();
  const pipeline::PipelineRuntimeSnapshot* snapshot =
      pipeline::RestorePipelineSnapshot(state);
  EXPECT_NE(snapshot, nullptr);
  return snapshot == nullptr ? -1.0 : snapshot->scan_phase_cycles;
}

}  // namespace

TEST(EsrControllerRuntimeStateTest, CaptureAndRestoreRoundTripState) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  controller.RunOnce(MakeValidInput(1U));
  ASSERT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kCompleted);
  const EsrControllerRuntimeState snapshot = controller.CaptureRuntimeState();

  controller.RunOnce(MakeValidInput(2U));
  ASSERT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 2U);

  controller.RestoreRuntimeState(snapshot);
  EXPECT_TRUE(controller.HasLatestInterceptOutputFrame());
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 1U);
  EXPECT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kCompleted);
}

TEST(EsrControllerRuntimeStateTest, SuccessfulCyclesAdvanceBatchAndRejectedCycleDoesNot) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);
  const std::uint64_t initial_batch_id = controller.CaptureRuntimeState().next_batch_id;

  controller.RunOnce(MakeValidInput(10U));
  ASSERT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 10U);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().batch_id, initial_batch_id);
  EXPECT_EQ(controller.CaptureRuntimeState().next_batch_id, initial_batch_id + 1U);

  session::EsrCycleInput invalid_input = MakeValidInput(11U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);
  EXPECT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().batch_id, initial_batch_id);
  EXPECT_EQ(controller.CaptureRuntimeState().next_batch_id, initial_batch_id + 1U);

  controller.RunOnce(MakeValidInput(12U));
  ASSERT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kCompleted);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 12U);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().batch_id, initial_batch_id + 1U);
  EXPECT_EQ(controller.CaptureRuntimeState().next_batch_id, initial_batch_id + 2U);
}

TEST(EsrControllerRuntimeStateTest, RestoreRejectsIncompatiblePipelineSnapshot) {
  pipeline::InterceptPipeline pipeline_a(MakeDefaultConfig());
  pipeline::InterceptPipeline pipeline_b(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller_a(pipeline_a, env);
  EsrController controller_b(pipeline_b, env);

  controller_a.RunOnce(MakeValidInput(20U));
  controller_b.RunOnce(MakeValidInput(30U));

  EsrControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();
  controller_a.RestoreRuntimeState(foreign_state);

  EXPECT_EQ(controller_a.GetLatestInterceptOutputFrame().cycle_index, 20U);
}

TEST(EsrControllerRuntimeStateTest, RestoreRejectsSnapshotFromOtherControllerInstance) {
  pipeline::InterceptPipeline shared_pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller_a(shared_pipeline, env);
  EsrController controller_b(shared_pipeline, env);

  controller_a.RunOnce(MakeValidInput(21U));
  controller_b.RunOnce(MakeValidInput(31U));

  const EsrControllerRuntimeState foreign_state = controller_b.CaptureRuntimeState();
  controller_a.RestoreRuntimeState(foreign_state);

  EXPECT_EQ(controller_a.GetLatestInterceptOutputFrame().cycle_index, 21U);
}

TEST(EsrControllerRuntimeStateTest, ValidationRejectSetsAbortReasonAndReusesPreviousOutput) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  controller.RunOnce(MakeValidInput(40U));
  ASSERT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kCompleted);

  session::EsrCycleInput invalid_input = MakeValidInput(41U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(controller.GetLastInterceptCycleAbortReason(), session::EsrPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 40U);
}

TEST(EsrControllerRuntimeStateTest, FirstValidationRejectDoesNotCreateOutputFrame) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  session::EsrCycleInput invalid_input = MakeValidInput(45U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_FALSE(controller.HasLatestInterceptOutputFrame());
  EXPECT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(controller.GetLastInterceptCycleAbortReason(), session::EsrPipelineAbortReason::kValidationRejected);
}

TEST(EsrControllerRuntimeStateTest,
     SessionCaptureRestoreSkipsRollbackOnValidationRejection) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  controller.RunOnce(MakeValidInput(100U));
  ASSERT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kCompleted);

  const auto pipeline_state = pipeline.CaptureRuntimeState();
  const auto controller_state = controller.CaptureRuntimeState();

  session::EsrCycleInput invalid_input = MakeValidInput(101U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_EQ(controller.GetLatestCycleStatus(), session::EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(controller.GetLastInterceptCycleAbortReason(), session::EsrPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 100U);
}

TEST(EsrControllerRuntimeStateTest, ScanPhaseUsesFullPatternCycleRateAndVariableStep) {
  EsrInternalExecutionConfig config = MakeDefaultConfig();
  config.mission.scan.scan_rate_hz = 0.5f;
  pipeline::InterceptPipeline pipeline(config);
  StubEnvironmentService env;

  session::EsrCycleInput input = MakeValidInput(1U);
  input.dt_sec = 0.2f;
  input.rf_emissions.window_duration_s = input.dt_sec;
  pipeline.RunCycle(input, env);
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.1, 5.0e-8);

  config.mission.scan.scan_rate_hz = 2.0f;
  pipeline.UpdateConfig(config);
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.1, 5.0e-8);
  input.dt_sec = 0.15f;
  input.rf_emissions.window_duration_s = input.dt_sec;
  pipeline.RunCycle(input, env);
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.4, 5.0e-8);

  config.mission.scan.scan_rate_hz = 5.0f;
  pipeline.UpdateConfig(config);
  input.dt_sec = 0.25f;
  input.rf_emissions.window_duration_s = input.dt_sec;
  pipeline.RunCycle(input, env);
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.65, 5.0e-8);
}

TEST(EsrControllerRuntimeStateTest, ScanGeometryResetPowerFreezeAndSnapshotRestore) {
  EsrInternalExecutionConfig config = MakeDefaultConfig();
  config.mission.scan.scan_rate_hz = 0.5f;
  pipeline::InterceptPipeline pipeline(config);
  StubEnvironmentService env;
  session::EsrCycleInput input = MakeValidInput(1U);
  input.dt_sec = 0.2f;
  input.rf_emissions.window_duration_s = input.dt_sec;
  pipeline.RunCycle(input, env);
  const extension::InterceptPipelineRuntimeState saved = pipeline.CaptureRuntimeState();
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.1, 5.0e-8);

  config.resolved_scan.scan_sequence = 1;
  pipeline.UpdateConfig(config);
  EXPECT_DOUBLE_EQ(ReadScanPhase(pipeline), 0.0);

  pipeline.RunCycle(input, env);
  config.mission.power_on = false;
  pipeline.UpdateConfig(config);
  const double frozen_phase = ReadScanPhase(pipeline);
  pipeline.RunCycle(input, env);
  EXPECT_DOUBLE_EQ(ReadScanPhase(pipeline), frozen_phase);

  EXPECT_TRUE(pipeline.RestoreRuntimeState(saved));
  EXPECT_NEAR(ReadScanPhase(pipeline), 0.1, 5.0e-8);
}

TEST(EsrControllerRuntimeStateTest, RestoreRejectsNonFiniteScanPhase) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  extension::InterceptPipelineRuntimeState state = pipeline.CaptureRuntimeState();
  const pipeline::PipelineRuntimeSnapshot* original =
      pipeline::RestorePipelineSnapshot(state);
  ASSERT_NE(original, nullptr);
  std::shared_ptr<pipeline::PipelineRuntimeSnapshot> corrupted(
      new pipeline::PipelineRuntimeSnapshot(*original));
  corrupted->scan_phase_cycles = std::numeric_limits<double>::quiet_NaN();
  pipeline::CapturePipelineSnapshot(state, corrupted);
  EXPECT_FALSE(pipeline.RestoreRuntimeState(state));
}

}  // namespace extension
}  // namespace electronic_surveillance_radar
