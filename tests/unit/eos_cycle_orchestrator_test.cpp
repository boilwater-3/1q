/**
 * @file eos_cycle_orchestrator_test.cpp
 * @brief 验证 EOS 运行期周期编排器的运行时配置提交与周期执行契约。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/extension/EosController.h"
#include "electro_optical_sensor/runtime/EosCycleOrchestrator.h"
#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {
namespace {

namespace eos_config = ::electro_optical_sensor::config;

EosSessionConfig MakeSessionConfig() {
  EosSessionConfig config;
  config.work_mode = EosWorkMode::kInfraredOnly;
  config.minimum_snr_db = -120.0f;
  config.scan_start_az_deg = -10.0f;
  config.scan_end_az_deg = 10.0f;
  config.scan_rate_deg_per_sec = 5.0f;
  config.horizontal_fov_deg = 20.0f;
  config.vertical_fov_deg = 4.0f;
  return config;
}

EosCycleInput MakeCycleInput(std::uint32_t cycle_index, float dt_sec) {
  EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = dt_sec;
  input.solar_irradiance_w_m2 = 850.0f;
  input.solar_altitude_deg = 45.0f;
  input.atmospheric_transmittance = 0.8f;
  input.cloud_coverage_ratio = 0.2f;
  input.background_temperature_k = 289.0f;
  input.day_night_type = DayNightType::kDay;
  return input;
}

TEST(EosCycleOrchestratorTest, StepProducesOutputAndPreservesCycleIndex) {
  const EosSessionConfig config = MakeSessionConfig();
  core::pipeline::EosPipeline pipeline(::electro_optical_sensor::extension::EosPipelineConfig{});
  extension::EosController controller(pipeline);
  EosCycleOrchestrator orchestrator(config, pipeline, controller);

  const EosCycleResult result = orchestrator.Step(MakeCycleInput(7U, 1.0f));

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 7U);
}

TEST(EosCycleOrchestratorTest, ValidRuntimePatchTakesEffectOnNextStep) {
  const EosSessionConfig config = MakeSessionConfig();
  core::pipeline::EosPipeline pipeline(::electro_optical_sensor::extension::EosPipelineConfig{});
  extension::EosController controller(pipeline);
  EosCycleOrchestrator orchestrator(config, pipeline, controller);

  const EosCycleResult baseline = orchestrator.Step(MakeCycleInput(1U, 1.0f));
  const float baseline_scan_azimuth = baseline.output_frame.scan_azimuth_deg;

  orchestrator.ApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(9.0f).Build());

  const EosCycleResult patched = orchestrator.Step(MakeCycleInput(2U, 1.0f));
  EXPECT_FALSE(patched.has_validation_error);
  EXPECT_TRUE(patched.executed_this_cycle);
  EXPECT_NEAR(patched.output_frame.scan_azimuth_deg, -1.0f, 1.0e-6f);
  EXPECT_NE(baseline_scan_azimuth, patched.output_frame.scan_azimuth_deg);
}

TEST(EosCycleOrchestratorTest, InvalidRuntimePatchDoesNotChangeUpdateBehavior) {
  const EosSessionConfig config = MakeSessionConfig();
  core::pipeline::EosPipeline pipeline(::electro_optical_sensor::extension::EosPipelineConfig{});
  extension::EosController controller(pipeline);
  EosCycleOrchestrator orchestrator(config, pipeline, controller);

  orchestrator.ApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithFrameRateHz(0.0f).Build());

  const EosCycleResult result = orchestrator.Step(MakeCycleInput(3U, 1.0f));
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_NEAR(result.output_frame.scan_azimuth_deg, -5.0f, 1.0e-6f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
