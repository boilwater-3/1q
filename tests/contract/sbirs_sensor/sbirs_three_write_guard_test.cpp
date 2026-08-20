// Copyright 2026. All Rights Reserved.
//
// @file sbirs_three_write_guard_test.cpp
// @brief SBIRS 三写约束机制性守卫（session_contract.md 规则 9）。
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

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace sbirs_sensor {
namespace tests {
namespace {

namespace sbirs_config = ::sbirs_sensor::config;

session::SbirsVector3M Vector(double x, double y, double z) {
  session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

session::SbirsSceneTarget MakeTarget(std::uint64_t id) {
  session::SbirsSceneTarget target;
  target.target_id = id;
  target.target_name = "guard-target";
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e8;
  return target;
}

config::SbirsSessionConfig MakeExecutableConfig() {
  config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = 359.0f;  // ECI 方位 [0,360)：-1° 等价折入 359°
  config.mission.scan_span_deg = 11.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  return config;
}

session::SbirsCycleInput MakeInput(std::uint32_t cycle_index = 1U, float dt_sec = 1.0f) {
  return session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(dt_sec)
      .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
      .AddTarget(MakeTarget(1U))
      .Build();
}

// 三写守卫断言：abort 周期必须三写齐备且 status 与 abort 类别一致。
void ExpectThreeWriteAbort(const session::SbirsCycleResult& result,
                           session::SbirsPipelineAbortReason expected_reason,
                           session::SbirsCycleStatus expected_status) {
  EXPECT_EQ(result.abort_reason, expected_reason);
  EXPECT_EQ(result.status, expected_status);

  // 写二：结构化诊断中至少一条 error 级、code 带模块前缀
  // （info/warning 级伴随诊断合法，三写要求的是 error 级主诊断）。
  EXPECT_FALSE(result.issues.empty());
  bool saw_error_diagnostic = false;
  for (const auto& issue : result.issues) {
    EXPECT_FALSE(issue.code.empty());
    EXPECT_EQ(issue.code.compare(0, 6, "sbirs."), 0);
    if (issue.severity == session::SbirsIssueSeverity::kError) {
      saw_error_diagnostic = true;
    }
  }
  EXPECT_TRUE(saw_error_diagnostic);
  // 写三：人读日志由统一 RecordAbort（PROJECT_LOG_ERROR）保证，测试不解析日志文本（规则 3）。
}

TEST(SbirsThreeWriteGuardTest, ValidationAbortWritesAllThree) {
  session::SbirsSession session = session::SbirsSession::Create(MakeExecutableConfig());

  const session::SbirsCycleResult result = session.StepWithResult(MakeInput(1U, 0.0f));
  ExpectThreeWriteAbort(result, session::SbirsPipelineAbortReason::kValidationRejected,
                        session::SbirsCycleStatus::kRejectedInvalidInput);
}

TEST(SbirsThreeWriteGuardTest, PoweredOffAbortWritesAllThree) {
  session::SbirsSession session = session::SbirsSession::Create(MakeExecutableConfig());
  const session::SbirsCycleResult active = session.StepWithResult(MakeInput());
  ASSERT_EQ(active.status, session::SbirsCycleStatus::kCompleted);

  (void)session.TryApplyRuntimeConfig(
      config::SbirsRuntimeConfigBuilder().WithSensorEnabled(false).Build());

  const session::SbirsCycleResult powered_off = session.StepWithResult(MakeInput(2U));
  ExpectThreeWriteAbort(powered_off, session::SbirsPipelineAbortReason::kSensorPoweredOff,
                        session::SbirsCycleStatus::kPoweredOff);
}

}  // namespace
}  // namespace tests
}  // namespace sbirs_sensor
