// Copyright 2026. All Rights Reserved.
//
// @file eos_three_write_guard_test.cpp
// @brief EOS 三写约束机制性守卫（session_contract.md 规则 9 + 统一问题列表模型规则 14）。
//
// 任何 abort 路径（abort_reason 非 kNone）必须同时写：
//   1) abort_reason（结构化信号）
//   2) issues（结构化诊断：severity + phase + code + message，code 带模块前缀）
//   3) PROJECT_LOG（人读日志，由统一 RecordAbort 写入；测试不解析日志文本，规则 3）
// 并保证 status 与 abort 类别一致（validation → kRejectedInvalidInput，
// powered-off → kPoweredOff，execution → kRejectedExecution）。
// 校验拒绝时校验问题本身就是 error 级诊断（phase=kInputValidation），不再附加粗粒度条目。
//
// 守卫对所有模块统一形态；新增 abort 路径时必须通过本测试。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace tests {
namespace {

session::EosCycleInput MakeValidInput(std::uint32_t cycle_index = 1U) {
  session::EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1f;
  session::EosSceneTarget target;
  target.target_id = 601U;
  target.range_m = 1000.0f;
  target.azimuth_deg = 0.0f;
  target.elevation_deg = 0.0f;
  target.appearance.apparent_temperature_k = 310.0f;
  target.appearance.emissivity = 0.9f;
  target.appearance.reflectance = 0.1f;
  target.appearance.projected_area_m2 = 1.5f;
  input.scene.push_back(target);
  return input;
}

// 三写守卫断言：abort 周期必须三写齐备且 status 与 abort 类别一致。
void ExpectThreeWriteAbort(const session::EosCycleResult& result,
                           session::EosPipelineAbortReason expected_reason,
                           session::EosCycleStatus expected_status) {
  EXPECT_EQ(result.abort_reason, expected_reason);
  EXPECT_EQ(result.status, expected_status);

  // 写二：结构化诊断中至少一条 error 级、code 带模块前缀
  // （info/warning 级伴随诊断合法，三写要求的是 error 级主诊断）。
  EXPECT_FALSE(result.issues.empty());
  bool saw_error_diagnostic = false;
  for (const auto& issue : result.issues) {
    EXPECT_FALSE(issue.code.empty());
    EXPECT_EQ(issue.code.compare(0, 4, "eos."), 0);
    if (issue.severity == session::EosIssueSeverity::kError) {
      saw_error_diagnostic = true;
    }
  }
  EXPECT_TRUE(saw_error_diagnostic);
  // 写三：人读日志由统一 RecordAbort（PROJECT_LOG_ERROR）保证，测试不解析日志文本（规则 3）。
}

// 规则 14 断言：校验拒绝时错误条目 phase=kInputValidation；执行中止时 phase=kExecution。
bool ContainsPhase(const session::EosIssueList& issues, session::EosIssuePhase phase) {
  for (const auto& issue : issues) {
    if (issue.phase == phase) {
      return true;
    }
  }
  return false;
}

TEST(EosThreeWriteGuardTest, ValidationAbortWritesAllThree) {
  session::EosSession session = session::EosSession::Create();
  session::EosCycleInput invalid_input = MakeValidInput();
  invalid_input.dt_sec = std::numeric_limits<float>::quiet_NaN();

  const session::EosCycleResult result = session.StepWithResult(invalid_input);
  ExpectThreeWriteAbort(result, session::EosPipelineAbortReason::kValidationRejected,
                        session::EosCycleStatus::kRejectedInvalidInput);
  // 规则 14：校验拒绝的问题条目 phase=kInputValidation（校验问题本身就是 error 级诊断）。
  EXPECT_TRUE(ContainsPhase(result.issues, session::EosIssuePhase::kInputValidation));
}

TEST(EosThreeWriteGuardTest, PoweredOffAbortWritesAllThree) {
  session::EosSession session = session::EosSession::Create();
  const session::EosCycleResult active = session.StepWithResult(MakeValidInput());
  ASSERT_EQ(active.status, session::EosCycleStatus::kCompleted);

  (void)session.TryApplyRuntimeConfig(
      config::EosRuntimeConfigBuilder().WithSensorEnabled(false).Build());

  const session::EosCycleResult powered_off = session.StepWithResult(MakeValidInput(2U));
  ExpectThreeWriteAbort(powered_off, session::EosPipelineAbortReason::kSensorPoweredOff,
                        session::EosCycleStatus::kPoweredOff);
  // 规则 14：关机属运行态条件，中止条目 phase=kExecution。
  EXPECT_TRUE(ContainsPhase(powered_off.issues, session::EosIssuePhase::kExecution));
}

}  // namespace
}  // namespace tests
}  // namespace electro_optical_sensor
