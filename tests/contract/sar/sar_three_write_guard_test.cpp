// Copyright 2026. All Rights Reserved.
//
// @file sar_three_write_guard_test.cpp
// @brief SAR 三写约束机制性守卫（session_contract.md 规则 9）。
//
// 任何 abort 路径（abort_reason 非 kNone）必须同时写：
//   1) abort_reason（结构化信号）
//   2) issues（结构化诊断：severity + phase + code + message，code 带模块前缀）
//   3) PROJECT_LOG（人读日志，由统一 RecordAbort 写入；测试不解析日志文本，规则 3）
// 并保证 status 与 abort 类别一致（validation → kRejectedInvalidInput，
// execution → kRejectedExecution）。
//
// 守卫对所有模块统一形态；新增 abort 路径时必须通过本测试。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "1q/sar/config/SarRuntimeConfigBuilder.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/sar/session/SarInputValidation.h"
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
  target.target_name = "guard-target";
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

// 三写守卫断言：abort 周期必须三写齐备且 status 与 abort 类别一致。
void ExpectThreeWriteAbort(const session::SarCycleResult& result,
                           session::SarPipelineAbortReason expected_reason,
                           session::SarCycleStatus expected_status) {
  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_EQ(result.abort_reason, expected_reason);
  EXPECT_EQ(result.status, expected_status);

  // 写二：结构化诊断中至少一条 error 级、code 带模块前缀
  // （info/warning 级伴随诊断合法，三写要求的是 error 级主诊断）。
  EXPECT_FALSE(result.issues.empty());
  bool saw_error_diagnostic = false;
  for (const auto& issue : result.issues) {
    EXPECT_FALSE(issue.code.empty());
    EXPECT_EQ(issue.code.compare(0, 4, "sar."), 0);
    if (issue.severity == session::SarIssueSeverity::kError) {
      saw_error_diagnostic = true;
    }
  }
  EXPECT_TRUE(saw_error_diagnostic);
  // 写三：人读日志由统一 RecordAbort（PROJECT_LOG_ERROR）保证，测试不解析日志文本（规则 3）。
}

// 规则 14 断言：校验拒绝时错误条目 phase=kInputValidation；执行中止时 phase=kExecution。
bool ContainsPhase(const session::SarIssueList& issues, session::SarIssuePhase phase) {
  for (const auto& issue : issues) {
    if (issue.phase == phase) {
      return true;
    }
  }
  return false;
}

TEST(SarThreeWriteGuardTest, ValidationAbortWritesAllThree) {
  session::SarSession session = session::SarSession::Create(MakeMinimalConfig());
  session::SarCycleInput invalid_input = MakeMinimalInput();
  invalid_input.point_targets[0].latitude_deg = std::numeric_limits<double>::quiet_NaN();

  const session::SarCycleResult result = session.StepWithResult(invalid_input);
  ExpectThreeWriteAbort(result, session::SarPipelineAbortReason::kValidationRejected,
                        session::SarCycleStatus::kRejectedInvalidInput);
  // 规则 14：校验拒绝的问题条目 phase=kInputValidation，code 为 "sar.validation.<snake>"
  // （校验问题本身就是 error 级诊断，不再有聚合的 "sar.invalid_cycle_input" 条目）。
  EXPECT_TRUE(ContainsPhase(result.issues, session::SarIssuePhase::kInputValidation));
  EXPECT_TRUE(session::HasValidationError(result.issues));
  bool saw_validation_code = false;
  for (const auto& issue : result.issues) {
    if (issue.code.compare(0, 15, "sar.validation.") == 0) {
      saw_validation_code = true;
    }
  }
  EXPECT_TRUE(saw_validation_code);
}

TEST(SarThreeWriteGuardTest, ExecutionAbortWritesAllThree) {
  config::SarSessionConfig config = MakeMinimalConfig();
  config.policy.minimum_snr_db = 1.0e6;  // 任何实际 SNR 都低于该门限 → snr_below_minimum
  session::SarSession session = session::SarSession::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeMinimalInput());
  ExpectThreeWriteAbort(result, session::SarPipelineAbortReason::kPipelineExecutionFailed,
                        session::SarCycleStatus::kRejectedExecution);
  // 规则 14：执行中止的问题条目 phase=kExecution。
  EXPECT_TRUE(ContainsPhase(result.issues, session::SarIssuePhase::kExecution));
}

TEST(SarThreeWriteGuardTest, PoweredOffAbortWritesAllThree) {
  // 电源关机（COMMON-OQ-4 字段提升）：关机是合法非执行状态（status=kPoweredOff），
  // 但 abort 路径同样必须三写（abort_reason + error 级诊断 + 日志）。
  session::SarSession session = session::SarSession::Create(MakeMinimalConfig());
  const session::SarCycleResult active = session.StepWithResult(MakeMinimalInput());
  ASSERT_EQ(active.status, session::SarCycleStatus::kCompleted);

  (void)session.TryApplyRuntimeConfig(
      config::SarRuntimeConfigBuilder().WithSensorEnabled(false).Build());

  const session::SarCycleResult powered_off = session.StepWithResult(MakeMinimalInput());
  ExpectThreeWriteAbort(powered_off, session::SarPipelineAbortReason::kSensorPoweredOff,
                        session::SarCycleStatus::kPoweredOff);
  // 规则 14：关机属运行态条件，中止条目 phase=kExecution。
  EXPECT_TRUE(ContainsPhase(powered_off.issues, session::SarIssuePhase::kExecution));
}

}  // namespace
}  // namespace tests
}  // namespace sar
