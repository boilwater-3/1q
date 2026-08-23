// Copyright 2026. All Rights Reserved.
//
// @file sar_public_api_convenience_test.cpp
// @brief SAR 对外易用性 API 契约测试。
//
// 补齐 SAR 与 EOS/ESR/AR 对称的 public api convenience test 缺口。
// SAR 会话配置用直接字段赋值，故聚焦：
//   - SarSessionConfig 字段可达与默认值
//   - SarSession::Create + StepWithResult 主路径
//   - SarCycleInput/Result 结构化执行结果字段
//   - 三层输出（debug view / lifecycle recorder）类型可达

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/sar/session/SarProductDebugView.h"
#include "1q/sar/session/SarProductLifecycleRecorder.h"
#include "1q/sar/session/SarSession.h"

namespace sar {
namespace tests {
namespace {

config::SarSessionConfig MakeMinimalConfig() {
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

session::SarCycleInput MakeMinimalInput() {
  session::SarCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.1f;
  session::SarPointTarget target;
  target.target_id = 1U;
  target.target_name = "convenience-target";
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

}  // namespace

TEST(SarPublicApiConvenienceTest, SessionConfigFieldsAreAssignable) {
  // 验证配置字段可读可写（API 可达性），不假设具体业务默认值。
  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.policy.enable_l1_rda_imaging = true;
  config.policy.enable_l3_bp_imaging = false;
  EXPECT_DOUBLE_EQ(config.hardware.carrier_frequency_hz, 1.0e9);
  EXPECT_TRUE(config.policy.enable_l1_rda_imaging);
  EXPECT_FALSE(config.policy.enable_l3_bp_imaging);
}

TEST(SarPublicApiConvenienceTest, SessionCreatesFromConfig) {
  session::SarSession session = session::SarSession::Create(MakeMinimalConfig());
  const session::SarCycleResult result = session.StepWithResult(MakeMinimalInput());

  // 结构化执行结果字段可达。
  EXPECT_EQ(result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(result.input_cycle_index, 1U);
}

TEST(SarPublicApiConvenienceTest, StepWithResultProducesL1RdaImageProduct) {
  session::SarSession session = session::SarSession::Create(MakeMinimalConfig());
  const session::SarCycleResult result = session.StepWithResult(MakeMinimalInput());

  EXPECT_TRUE(result.product.output_frame.has_raw_echo);
  EXPECT_TRUE(result.product.output_frame.has_range_compressed_echo);
  EXPECT_TRUE(result.product.output_frame.has_l1_image);
  EXPECT_EQ(result.product.output_frame.completed_stage, session::SarProcessingStage::kL1RdaImage);
}

// 规则 15e：聚焦图像是产品，Step() 必须能拿到图像（成功路径 guard；
// 失败路径空载荷由下方 StepReturnsEmptyFrameOnValidationFailureAfterSuccess 锁定）。
TEST(SarPublicApiConvenienceTest, StepReturnsFocusedImageProductOnCompletedCycle) {
  session::SarSession session = session::SarSession::Create(MakeMinimalConfig());
  const session::SarCycleProduct product = session.Step(MakeMinimalInput());

  EXPECT_EQ(product.output_frame.cycle_index, 1U);
  EXPECT_TRUE(product.output_frame.has_l1_image);
  EXPECT_EQ(product.output_frame.completed_stage, session::SarProcessingStage::kL1RdaImage);
  EXPECT_EQ(product.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_EQ(product.focused_image.row_count, 9U);
  EXPECT_EQ(product.focused_image.column_count, 64U);
  EXPECT_FALSE(product.focused_image.is_placeholder);
  EXPECT_EQ(product.focused_image.real_values.size(), 9U * 64U);
}

TEST(SarPublicApiConvenienceTest, ProductDebugViewAndLifecycleRecorderAreReachable) {
  // 三层输出类型在 public API 可达（阶段 9 三层模型）。
  session::SarSession session = session::SarSession::Create(MakeMinimalConfig());
  session::SarProductLifecycleRecorder recorder;
  session.AttachProductLifecycleRecorder(&recorder);
  const session::SarCycleInput input = MakeMinimalInput();
  const session::SarCycleResult result = session.StepWithResult(input);

  const session::SarProductDebugView view = session::SarProductDebugViewBuilder::Build(input, result);
  EXPECT_TRUE(view.has_l1_image);
  ASSERT_EQ(view.point_targets.size(), 1U);
  // 点目标 name 只在 debug view 的 point_targets 中，不进入产品输出帧。
  EXPECT_EQ(view.point_targets[0].target_name, "convenience-target");

  // recorder 由 Session 自动驱动（Attach 契约），事件经 GetLastEvents 获取。
  EXPECT_FALSE(recorder.GetLastEvents().empty());
}

TEST(SarPublicApiConvenienceTest, StepResultExposesStructuredExecutionState) {
  // 结构化执行结果字段可达：status/abort_reason/阶段。
  session::SarSession session = session::SarSession::Create(MakeMinimalConfig());
  const session::SarCycleResult result = session.StepWithResult(MakeMinimalInput());
  (void)result.status;
  (void)result.abort_reason;
  SUCCEED();
}

// SAR 非执行周期返回默认空帧（不复用上一有效输出）。guard 锁定该边界。
TEST(SarPublicApiConvenienceTest, StepReturnsEmptyFrameOnValidationFailureAfterSuccess) {
  session::SarSession session = session::SarSession::Create(MakeMinimalConfig());
  ASSERT_EQ(session.StepWithResult(MakeMinimalInput()).status, session::SarCycleStatus::kCompleted);

  // 校验失败：dt_sec 非正。cycle_index 推进到 8，与上一帧的 1 形成可观测差异。
  session::SarCycleInput invalid_input;
  invalid_input.cycle_index = 8U;
  invalid_input.dt_sec = 0.0f;

  const session::SarCycleResult failed_result = session.StepWithResult(invalid_input);
  EXPECT_NE(failed_result.status, session::SarCycleStatus::kCompleted);
  EXPECT_EQ(failed_result.product.output_frame.cycle_index, 0U);

  // Step() 与 StepWithResult().product 一致，均为空载荷（规则 15c：同一记录取出）。
  const session::SarCycleProduct step_product = session.Step(invalid_input);
  EXPECT_EQ(step_product.output_frame.cycle_index, 0U);
  EXPECT_EQ(step_product.output_frame.cycle_index,
            failed_result.product.output_frame.cycle_index);
  EXPECT_EQ(step_product.focused_image.source, failed_result.product.focused_image.source);
}

}  // namespace tests
}  // namespace sar
