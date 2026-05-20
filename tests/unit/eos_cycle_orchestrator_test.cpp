/**
 * @file eos_cycle_orchestrator_test.cpp
 * @brief 验证 EOS 会话的周期执行、配置提交与运行期补丁契约。
 *
 * 原测试通过 EosCycleOrchestrator 直接测试 RunCycle + ApplyRuntimeConfig。
 * Orchestrator 内联到 EosSession::Impl 后，通过 EosSessionFactory::Create 测试等效行为。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace {

namespace eos_config = ::electro_optical_sensor::config;
namespace eos_session = ::electro_optical_sensor::session;

eos_session::EosSessionConfig MakeSessionConfig() {
  eos_session::EosSessionConfig config;
  config.mission.work_mode = eos_config::EosWorkMode::kInfraredOnly;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  config.mission.horizontal_fov_deg = 20.0f;
  config.mission.vertical_fov_deg = 4.0f;
  return config;
}

eos_session::EosCycleInput MakeCycleInput(std::uint32_t cycle_index, float dt_sec) {
  eos_session::EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = dt_sec;
  input.environment.solar_irradiance_w_m2 = 850.0f;
  input.environment.solar_altitude_deg = 45.0f;
  input.environment.cloud_coverage_ratio = 0.2f;
  input.environment.background_temperature_k = 289.0f;
  input.environment.day_night_type = eos_session::DayNightType::kDay;
  return input;
}

TEST(EosSessionTest, RunCycleProducesOutputAndPreservesCycleIndex) {
  const eos_session::EosSessionConfig config = MakeSessionConfig();
  eos_session::EosSession session = eos_session::EosSessionFactory::Create(config);

  const eos_session::EosCycleResult result = session.StepWithResult(MakeCycleInput(7U, 1.0f));

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 7U);
}

TEST(EosSessionTest, ValidRuntimePatchTakesEffectOnNextStep) {
  const eos_session::EosSessionConfig config = MakeSessionConfig();
  eos_session::EosSession session = eos_session::EosSessionFactory::Create(config);

  const eos_session::EosCycleResult baseline =
      session.StepWithResult(MakeCycleInput(1U, 1.0f));
  const float baseline_scan_azimuth = baseline.output_frame.scan_azimuth_deg;

  session.ApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(9.0f).Build());

  const eos_session::EosCycleResult patched =
      session.StepWithResult(MakeCycleInput(2U, 1.0f));
  EXPECT_FALSE(patched.has_validation_error);
  EXPECT_TRUE(patched.executed_this_cycle);
  EXPECT_NEAR(patched.output_frame.scan_azimuth_deg, -1.0f, 1.0e-6f);
  EXPECT_NE(baseline_scan_azimuth, patched.output_frame.scan_azimuth_deg);
}

TEST(EosSessionTest, InvalidRuntimePatchDoesNotChangeUpdateBehavior) {
  const eos_session::EosSessionConfig config = MakeSessionConfig();
  eos_session::EosSession session = eos_session::EosSessionFactory::Create(config);

  session.ApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithFrameRateHz(0.0f).Build());

  const eos_session::EosCycleResult result =
      session.StepWithResult(MakeCycleInput(3U, 1.0f));
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_NEAR(result.output_frame.scan_azimuth_deg, -5.0f, 1.0e-6f);
}

}  // namespace
}  // namespace electro_optical_sensor
