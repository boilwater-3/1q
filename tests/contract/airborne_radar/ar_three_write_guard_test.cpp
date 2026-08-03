// Copyright 2026. All Rights Reserved.
//
// @file ar_three_write_guard_test.cpp
// @brief AR 三写约束机制性守卫（session_contract.md 规则 9）。
//
// 任何 abort 路径（abort_reason 非 kNone）必须同时写：
//   1) abort_reason（结构化信号）
//   2) diagnostics（结构化诊断：severity + code + message，code 带模块前缀）
//   3) PROJECT_LOG（人读日志，由统一 RecordAbort 写入；测试不解析日志文本，规则 3）
// 并保证 status 与 abort 类别一致（validation → kRejectedInvalidInput，
// powered-off → kPoweredOff，execution → kRejectedExecution）。
//
// 守卫对所有模块统一形态；新增 abort 路径时必须通过本测试。

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"

namespace airborne_radar {
namespace tests {
namespace {

config::ArSessionConfig MakeSessionConfig() {
  config::ArSessionConfig cfg;
  cfg.policy.detection = config::profiles::kDetectionPriorityDetection;
  cfg.policy.tracking = config::profiles::kFastAssociationTracking;
  cfg.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  return cfg;
}

session::ArCycleInput MakeInput(std::uint32_t cycle_index = 1U, double cycle_start_time_s = 0.0) {
  session::ArCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = cycle_start_time_s;
  input.dt_sec = 0.5;
  input.platform.platform_entity_id = 42U;

  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(
      oneq::coordinate::TryLlaToEcef(platform_lla, &input.platform.platform_position_ecef_m));

  session::ArTargetInput target;
  target.target_id = 7U;
  target.target_name = "guard-target";
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  target.kinematics.position_ecef_m = input.platform.platform_position_ecef_m;
  target.kinematics.position_ecef_m.x_m += 5000.0;
  target.rcs = 2.0f;
  input.targets.push_back(target);
  return input;
}

// 三写守卫断言：abort 周期必须三写齐备且 status 与 abort 类别一致。
void ExpectThreeWriteAbort(const session::ArCycleResult& result,
                           session::SignalCycleAbortReason expected_reason,
                           session::ArCycleStatus expected_status) {
  EXPECT_NE(result.status, session::ArCycleStatus::kCompleted);
  EXPECT_EQ(result.abort_reason, expected_reason);
  EXPECT_EQ(result.status, expected_status);

  // 写二：结构化诊断中至少一条 error 级、code 带模块前缀
  // （info/warning 级伴随诊断合法，三写要求的是 error 级主诊断）。
  EXPECT_FALSE(result.diagnostics.empty());
  bool saw_error_diagnostic = false;
  for (const auto& issue : result.diagnostics) {
    EXPECT_FALSE(issue.code.empty());
    EXPECT_EQ(issue.code.compare(0, 3, "ar."), 0);
    if (issue.severity == session::ArDiagnosticSeverity::kError) {
      saw_error_diagnostic = true;
    }
  }
  EXPECT_TRUE(saw_error_diagnostic);
  // 写三：人读日志由统一 RecordAbort（PROJECT_LOG_ERROR）保证，测试不解析日志文本（规则 3）。
}

TEST(ArThreeWriteGuardTest, ValidationAbortWritesAllThree) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());

  session::ArCycleInput invalid = MakeInput();
  invalid.dt_sec = 0.0;
  const session::ArCycleResult result = radar.StepWithResult(invalid);
  ExpectThreeWriteAbort(result, session::SignalCycleAbortReason::kValidationRejected,
                        session::ArCycleStatus::kRejectedInvalidInput);
}

TEST(ArThreeWriteGuardTest, PoweredOffAbortWritesAllThree) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  const config::ArRuntimeConfigPatch power_off =
      config::ArRuntimeConfigBuilder().WithSensorEnabled(false).Build();
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(power_off));

  const session::ArCycleResult result = radar.StepWithResult(MakeInput());
  ExpectThreeWriteAbort(result, session::SignalCycleAbortReason::kSensorPoweredOff,
                        session::ArCycleStatus::kPoweredOff);
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
