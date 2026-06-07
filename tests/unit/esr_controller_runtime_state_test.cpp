/**
 * @file esr_controller_runtime_state_test.cpp
 * @brief 验证 ESR 控制器运行态快照与失败回退契约。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "1q/electronic_surveillance_radar/extension/EsrController.h"
#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

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
  config.detection.min_detect_snr_db = 6.0f;
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
  void BeginCycle(const environment::EsrEnvironmentCycleContext&) override {}
  environment::EsrEnvironmentSnapshot SampleEnvironment() const override {
    return environment::EsrEnvironmentSnapshot{};
  }
  void UpdateModelConfig(environment::EsrEnvironmentScenarioConfig) override {}
};

session::EsrCycleInput MakeValidInput(std::uint32_t cycle_index) {
  session::EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0f;
  return input;
}

}  // namespace

TEST(EsrControllerRuntimeStateTest, CaptureAndRestoreRoundTripState) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  controller.RunOnce(MakeValidInput(1U));
  ASSERT_TRUE(controller.ExecutedLatestCycle());
  const EsrControllerRuntimeState snapshot = controller.CaptureRuntimeState();

  controller.RunOnce(MakeValidInput(2U));
  ASSERT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 2U);

  controller.RestoreRuntimeState(snapshot);
  EXPECT_TRUE(controller.HasLatestInterceptOutputFrame());
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 1U);
  EXPECT_TRUE(controller.ExecutedLatestCycle());
  EXPECT_FALSE(controller.ReusedPreviousInterceptOutputLatestCycle());
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
  ASSERT_TRUE(controller.ExecutedLatestCycle());

  session::EsrCycleInput invalid_input = MakeValidInput(41U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_FALSE(controller.ExecutedLatestCycle());
  EXPECT_TRUE(controller.ReusedPreviousInterceptOutputLatestCycle());
  EXPECT_EQ(controller.GetLastInterceptCycleAbortReason(), EsrPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 40U);
}

TEST(EsrControllerRuntimeStateTest, FirstValidationRejectBuildsEmptyOutputFrame) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
  StubEnvironmentService env;
  EsrController controller(pipeline, env);

  session::EsrCycleInput invalid_input = MakeValidInput(45U);
  invalid_input.dt_sec = 0.0f;
  controller.RunOnce(invalid_input);

  EXPECT_TRUE(controller.HasLatestInterceptOutputFrame());
  EXPECT_FALSE(controller.ExecutedLatestCycle());
  EXPECT_EQ(controller.GetLastInterceptCycleAbortReason(), EsrPipelineAbortReason::kValidationRejected);
}

TEST(EsrControllerRuntimeStateTest,
     SessionCaptureRestoreSkipsRollbackOnValidationRejection) {
  pipeline::InterceptPipeline pipeline(MakeDefaultConfig());
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
  EXPECT_EQ(controller.GetLastInterceptCycleAbortReason(), EsrPipelineAbortReason::kValidationRejected);
  EXPECT_EQ(controller.GetLatestInterceptOutputFrame().cycle_index, 100U);
  EXPECT_TRUE(controller.ReusedPreviousInterceptOutputLatestCycle());
}

}  // namespace extension
}  // namespace electronic_surveillance_radar
