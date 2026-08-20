/**
 * @file eos_cycle_orchestrator_test.cpp
 * @brief 验证 EOS 会话的周期执行、配置提交与运行期补丁契约。
 *
 * 原测试通过 EosCycleOrchestrator 直接测试 RunCycle + TryApplyRuntimeConfig。
 * Orchestrator 内联到 EosSession::Impl 后，通过 EosSession::Create 测试等效行为。
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

eos_config::EosSessionConfig MakeSessionConfig() {
  eos_config::EosSessionConfig config;
  config.mission.work_mode = eos_config::EosWorkMode::kInfraredOnly;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
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
  return input;
}

TEST(EosSessionTest, RunCycleProducesOutputAndPreservesCycleIndex) {
  const eos_config::EosSessionConfig config = MakeSessionConfig();
  eos_session::EosSession session = eos_session::EosSession::Create(config);

  const eos_session::EosCycleResult result = session.StepWithResult(MakeCycleInput(7U, 0.1f));

  EXPECT_FALSE(eos_session::HasValidationError(result.issues));
  EXPECT_EQ(result.status, eos_session::EosCycleStatus::kCompleted);
  EXPECT_EQ(result.output_frame.cycle_index, 7U);
}

TEST(EosSessionTest, ValidRuntimePatchTakesEffectOnNextStep) {
  const eos_config::EosSessionConfig config = MakeSessionConfig();
  eos_session::EosSession session = eos_session::EosSession::Create(config);

  // Cycle 1: rate=5.0, dt=0.1 → advance=0.5, scan_az = -10 + 0.5 = -9.5
  const eos_session::EosCycleResult baseline =
      session.StepWithResult(MakeCycleInput(1U, 0.1f));
  const float baseline_scan_azimuth = baseline.output_frame.scan_azimuth_deg;
  EXPECT_NEAR(baseline_scan_azimuth, -9.5f, 1.0e-6f);

  ASSERT_TRUE(session.TryApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(9.0f).Build()));

  // Cycle 2: rate patched to 9.0 → scan phase resets to start, advance=9*0.1=0.9,
  // scan_az = -10 + 0.9 = -9.1
  const eos_session::EosCycleResult patched =
      session.StepWithResult(MakeCycleInput(2U, 0.1f));
  EXPECT_FALSE(eos_session::HasValidationError(patched.issues));
  EXPECT_EQ(patched.status, eos_session::EosCycleStatus::kCompleted);
  EXPECT_NEAR(patched.output_frame.scan_azimuth_deg, -9.1f, 1.0e-6f);
  EXPECT_NE(baseline_scan_azimuth, patched.output_frame.scan_azimuth_deg);
}

TEST(EosSessionTest, InvalidRuntimePatchDoesNotChangeUpdateBehavior) {
  const eos_config::EosSessionConfig config = MakeSessionConfig();
  eos_session::EosSession session = eos_session::EosSession::Create(config);

  EXPECT_FALSE(session.TryApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithFrameRateHz(0.0f).Build()));

  // Invalid patch rejected; rate remains 5.0, dt=0.1 → advance=0.5, scan_az = -10 + 0.5 = -9.5
  const eos_session::EosCycleResult result =
      session.StepWithResult(MakeCycleInput(3U, 0.1f));
  EXPECT_FALSE(eos_session::HasValidationError(result.issues));
  EXPECT_EQ(result.status, eos_session::EosCycleStatus::kCompleted);
  EXPECT_NEAR(result.output_frame.scan_azimuth_deg, -9.5f, 1.0e-6f);
}

}  // namespace
}  // namespace electro_optical_sensor
