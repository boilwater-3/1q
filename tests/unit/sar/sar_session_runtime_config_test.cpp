/**
 * @file sar_session_runtime_config_test.cpp
 * @brief SarSession 运行期配置补丁的 session 层边缘路径测试。
 *
 * 立即提交契约（docs/common/contract.md）：TryApplyRuntimeConfig 调用即生效、
 * 单向落定、无 session 层回滚。有效补丁在下个 Step 立即反映；空补丁与依赖
 * 违规补丁在入口拒绝且不污染 runtime_config。核心路径已由 SarSessionPipelineTest
 * 覆盖（立即生效 / 拒绝不污染 / L1-RDA 依赖），本文件补充其未覆盖的边缘路径。
 */

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/sar/config/SarRuntimeConfigBuilder.h"
#include "1q/sar/session/SarSession.h"

namespace sar {
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
  config.policy.max_allowed_squint_angle_deg = 89.0;
  return config;
}

session::SarCycleInput MakeInput(std::uint32_t cycle_index = 1U) {
  session::SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1f;
  input.platform.latitude_deg = 0.0;
  input.platform.longitude_deg = 0.0;
  input.platform.altitude_m = 0.0;
  input.platform.velocity_east_mps = 2.0;

  session::SarPointTarget target;
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

TEST(SarSessionRuntimeConfigTest, EmptyPatchRejectedAndSessionStaysHealthy) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());

  // 空补丁（无 has_* 标志）：resolver 判定 has_requested_update=false，
  // 控制器入口拒绝（SarController::TryApplyRuntimeConfig 返回 false）。
  EXPECT_FALSE(session.TryApplyRuntimeConfig(config::SarRuntimeConfigPatch{}));

  // 会话不受影响：仍可正常执行 RDA 成像。
  const session::SarCycleResult result = session.StepWithResult(MakeInput());
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_TRUE(result.output_frame.has_l1_image);
}

TEST(SarSessionRuntimeConfigTest, RetainRawPhaseHistoryWithoutRawEchoRejected) {
  // 无 L1 依赖的 RDA 配置：raw_echo 默认开启，可正常步进（completed_stage=kNone）。
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l1_rda_imaging = false;
  session::SarSession session = session::SarSession::Create(config);

  // 第一步：关闭 raw echo（有效补丁，立即提交），下个 Step 立即反映。
  EXPECT_TRUE(session.TryApplyRuntimeConfig(
      config::SarRuntimeConfigBuilder().WithEnableRawEchoGeneration(false).Build()));
  const session::SarCycleResult after_close = session.StepWithResult(MakeInput());
  EXPECT_TRUE(after_close.executed_this_cycle);
  EXPECT_FALSE(after_close.output_frame.has_raw_echo);

  // 第二步：retain_raw_phase_history 依赖 raw echo，应被 resolver 拒绝。
  EXPECT_FALSE(session.TryApplyRuntimeConfig(
      config::SarRuntimeConfigBuilder().WithRetainRawPhaseHistory(true).Build()));

  // 拒绝后 runtime_config 未被污染：步进行为与"从未应用被拒补丁"一致。
  const session::SarCycleResult after_reject = session.StepWithResult(MakeInput(2U));
  EXPECT_TRUE(after_reject.executed_this_cycle);
  EXPECT_FALSE(after_reject.output_frame.has_raw_echo);
  EXPECT_EQ(after_reject.output_frame.completed_stage, session::SarProcessingStage::kNone);
}

}  // namespace
}  // namespace sar
