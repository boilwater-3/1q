// Copyright 2026. All Rights Reserved.
//
// @file rir_three_write_guard_test.cpp
// @brief RIR 三写约束机制性守卫（session_contract.md 规则 9）。
//
// 任何 abort 路径（abort_reason 非 kNone）必须同时写：
//   1) abort_reason（结构化信号）
//   2) issues（结构化诊断：severity + phase + code + message，code 带模块前缀）
//   3) PROJECT_LOG（人读日志；测试不解析日志文本，规则 3）
// 并保证 status 与 abort 类别一致（validation → kRejectedInvalidInput，
// powered-off → kPoweredOff）。
//
// 守卫对所有模块统一形态；新增 abort 路径时必须通过本测试。

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirInputValidation.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "1q/remote_identification_radar/session/RirSession.h"

namespace remote_identification_radar {
namespace tests {
namespace {

config::RirSessionConfig MakeIdentifyConfig() {
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  return config;
}

session::RirCycleInput MakeValidInput() {
  session::RirCycleInput input;
  input.input_cycle_index = 1U;
  input.dt_sec = 0.5;
  input.sim_time_sec = 0.0f;
  oneq::coordinate::LlaPositionDegM lla{};
  lla.latitude_deg = 30.0;
  lla.longitude_deg = 120.0;
  lla.altitude_m = 1000.0;
  oneq::coordinate::TryLlaToEcef(lla, &input.platform_position);
  return input;
}

void ExpectThreeWriteAbort(const session::RirCycleResult& result,
                           session::RirCycleAbortReason expected_reason,
                           session::RirCycleStatus expected_status) {
  EXPECT_NE(result.status, session::RirCycleStatus::kCompleted);
  EXPECT_EQ(result.abort_reason, expected_reason);
  EXPECT_EQ(result.status, expected_status);

  EXPECT_FALSE(result.issues.empty());
  bool saw_error_diagnostic = false;
  for (const auto& issue : result.issues) {
    EXPECT_FALSE(issue.code.empty());
    EXPECT_EQ(issue.code.compare(0, 4, "rir."), 0);
    if (issue.severity == session::RirIssueSeverity::kError) {
      saw_error_diagnostic = true;
    }
  }
  EXPECT_TRUE(saw_error_diagnostic);
}

bool ContainsPhase(const session::RirIssueList& issues, session::RirIssuePhase phase) {
  for (const auto& issue : issues) {
    if (issue.phase == phase) {
      return true;
    }
  }
  return false;
}

TEST(RirThreeWriteGuardTest, ValidationAbortWritesAllThree) {
  session::RirSession session = session::RirSession::Create(MakeIdentifyConfig());
  session::RirCycleInput invalid = MakeValidInput();
  invalid.dt_sec = 0.0;

  const session::RirCycleResult result = session.StepWithResult(invalid);
  ExpectThreeWriteAbort(result, session::RirCycleAbortReason::kValidationRejected,
                        session::RirCycleStatus::kRejectedInvalidInput);
  EXPECT_TRUE(ContainsPhase(result.issues, session::RirIssuePhase::kInputValidation));
  EXPECT_TRUE(session::HasValidationError(result.issues));
}

TEST(RirThreeWriteGuardTest, PoweredOffAbortWritesAllThree) {
  config::RirSessionConfig config = MakeIdentifyConfig();
  config.sensor_enabled = false;
  session::RirSession session = session::RirSession::Create(config);

  const session::RirCycleResult result = session.StepWithResult(MakeValidInput());
  ExpectThreeWriteAbort(result, session::RirCycleAbortReason::kPoweredOff,
                        session::RirCycleStatus::kPoweredOff);
  EXPECT_TRUE(ContainsPhase(result.issues, session::RirIssuePhase::kExecution));
  bool saw_powered_off = false;
  for (const auto& issue : result.issues) {
    if (issue.code == session::codes::kSensorPoweredOff) {
      saw_powered_off = true;
    }
  }
  EXPECT_TRUE(saw_powered_off);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
