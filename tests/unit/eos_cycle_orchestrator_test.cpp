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
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"
#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace internal {
namespace {

namespace eos_config = ::electro_optical_sensor::config;
namespace eos_session = ::electro_optical_sensor::session;

eos_session::EosSessionConfig MakeSessionConfig() {
  eos_session::EosSessionConfig config;
  config.scan.work_mode = eos_session::EosWorkMode::kInfraredOnly;
  config.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.pointing.scan_start_az_deg = -10.0f;
  config.pointing.scan_end_az_deg = 10.0f;
  config.scan.scan_rate_deg_per_sec = 5.0f;
  config.scan.horizontal_fov_deg = 20.0f;
  config.scan.vertical_fov_deg = 4.0f;
  return config;
}

eos_session::EosCycleInput MakeCycleInput(std::uint32_t cycle_index, float dt_sec) {
  eos_session::EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = dt_sec;
  input.solar_irradiance_w_m2 = 850.0f;
  input.solar_altitude_deg = 45.0f;
  input.cloud_coverage_ratio = 0.2f;
  input.background_temperature_k = 289.0f;
  input.day_night_type = eos_session::DayNightType::kDay;
  return input;
}

TEST(EosCycleOrchestratorTest, StepProducesOutputAndPreservesCycleIndex) {
  const eos_session::EosSessionConfig config = MakeSessionConfig();
  const ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config =
      BuildEosPipelineConfig(config);
  signal::pipeline::EosPipeline pipeline(pipeline_config);
  extension::EosController controller(pipeline);
  EosCycleOrchestrator orchestrator(config, pipeline_config, true, pipeline, controller);

  const ::electro_optical_sensor::model::EosCycleResult result = orchestrator.Step(MakeCycleInput(7U, 1.0f));

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 7U);
}

TEST(EosCycleOrchestratorTest, ValidRuntimePatchTakesEffectOnNextStep) {
  const eos_session::EosSessionConfig config = MakeSessionConfig();
  const ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config =
      BuildEosPipelineConfig(config);
  signal::pipeline::EosPipeline pipeline(pipeline_config);
  extension::EosController controller(pipeline);
  EosCycleOrchestrator orchestrator(config, pipeline_config, true, pipeline, controller);

  const ::electro_optical_sensor::model::EosCycleResult baseline = orchestrator.Step(MakeCycleInput(1U, 1.0f));
  const float baseline_scan_azimuth = baseline.output_frame.scan_azimuth_deg;

  orchestrator.ApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(9.0f).Build());

  const ::electro_optical_sensor::model::EosCycleResult patched = orchestrator.Step(MakeCycleInput(2U, 1.0f));
  EXPECT_FALSE(patched.has_validation_error);
  EXPECT_TRUE(patched.executed_this_cycle);
  EXPECT_NEAR(patched.output_frame.scan_azimuth_deg, -1.0f, 1.0e-6f);
  EXPECT_NE(baseline_scan_azimuth, patched.output_frame.scan_azimuth_deg);
}

TEST(EosCycleOrchestratorTest, InvalidRuntimePatchDoesNotChangeUpdateBehavior) {
  const eos_session::EosSessionConfig config = MakeSessionConfig();
  const ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config =
      BuildEosPipelineConfig(config);
  signal::pipeline::EosPipeline pipeline(pipeline_config);
  extension::EosController controller(pipeline);
  EosCycleOrchestrator orchestrator(config, pipeline_config, true, pipeline, controller);

  orchestrator.ApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithFrameRateHz(0.0f).Build());

  const ::electro_optical_sensor::model::EosCycleResult result = orchestrator.Step(MakeCycleInput(3U, 1.0f));
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_NEAR(result.output_frame.scan_azimuth_deg, -5.0f, 1.0e-6f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
