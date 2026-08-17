// Copyright 2026. All Rights Reserved.
//
// @file sbirs_public_api_convenience_test.cpp
// @brief SBIRS 对外易用性 API 契约测试。
//
// 补齐 SBIRS 与 EOS/ESR/SAR/AR 对称的 public api convenience test 缺口。
// 聚焦：
//   - SbirsSessionConfigBuilder 默认值与字段可达
//   - SbirsRuntimeConfigBuilder 默认 unset / 全 set
//   - ValidateSbirsCycleInput 边界校验码
//   - SbirsSession::Create + StepWithResult 结构化执行结果字段
//   - 三层输出（debug view / lifecycle recorder）类型可达

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigBuilder.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleOutputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"
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
  target.target_name = "convenience-target";
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e8;
  return target;
}

session::SbirsCycleInput MakeMinimalInput(std::uint32_t cycle_index = 1U) {
  return session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
      .AddTarget(MakeTarget(1U))
      .Build();
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

TEST(SbirsPublicApiConvenienceTest, SessionConfigBuilderDefaultsMatchDirectConstruction) {
  // builder 无覆盖时与默认构造的 config 行为一致。
  const config::SbirsSessionConfig built = config::SbirsSessionConfigBuilder().Build();
  const config::SbirsSessionConfig direct;
  // 同配置驱动相同输出：用一个低阈值场景跑单周期比较。
  session::SbirsSession built_session = session::SbirsSession::Create(built);
  session::SbirsSession direct_session = session::SbirsSession::Create(direct);
  const session::SbirsCycleResult built_result = built_session.StepWithResult(MakeMinimalInput());
  const session::SbirsCycleResult direct_result = direct_session.StepWithResult(MakeMinimalInput());
  EXPECT_EQ(built_result.status, direct_result.status);
}

TEST(SbirsPublicApiConvenienceTest, SessionConfigFieldsAreAssignable) {
  // 硬件 / 任务 / 安装指向 / 策略 / 环境五域字段可读可写（API 可达性）。
  config::SbirsSessionConfig config;
  config.hardware.optical_aperture_m = 0.8f;
  config.mission.scan_span_deg = 90.0f;
  config.mission.scan_direction = config::SbirsScanDirection::kDecreasingAzimuth;
  config.mission.scan_el_start_deg = -5.0f;  // 阶段 4 俯仰栅格字段可赋值
  config.mission.scan_el_span_deg = 30.0f;
  config.mission.scan_el_step_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 12.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 45.0f;
  config.mission.narrow_pointing_settle_tolerance_deg = 0.02f;
  config.policy.detection.wide_min_snr_linear = 5.0f;
  config.environment.weather_type = config::SbirsWeatherType::kRain;
  config.orientation.mount_angles_deg.yaw_deg = 15.0;
  config.orientation.sensor_scan_limits_deg.az_min_deg = -30.0f;
  config.orientation.sensor_scan_limits_deg.az_max_deg = 30.0f;
  config.orientation.stabilization_mode =
      config::SbirsStabilizationMode::kInertialStabilized;
  config.orientation.misalignment.bias_deg.pitch_deg = 2.0;
  config.orientation.misalignment.random_sigma_deg = 0.25f;
  config.orientation.misalignment.random_seed = 9U;
  EXPECT_FLOAT_EQ(config.hardware.optical_aperture_m, 0.8f);
  EXPECT_FLOAT_EQ(config.mission.scan_span_deg, 90.0f);
  EXPECT_EQ(config.mission.scan_direction, config::SbirsScanDirection::kDecreasingAzimuth);
  EXPECT_FLOAT_EQ(config.mission.scan_el_start_deg, -5.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_el_span_deg, 30.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_el_step_deg, 10.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_rate_deg_per_sec, 12.0f);
  EXPECT_FLOAT_EQ(config.mission.narrow_pointing_max_slew_rate_deg_per_sec, 45.0f);
  EXPECT_FLOAT_EQ(config.mission.narrow_pointing_settle_tolerance_deg, 0.02f);
  EXPECT_FLOAT_EQ(config.policy.detection.wide_min_snr_linear, 5.0f);
  EXPECT_EQ(config.environment.weather_type, config::SbirsWeatherType::kRain);
  EXPECT_DOUBLE_EQ(config.orientation.mount_angles_deg.yaw_deg, 15.0);
  EXPECT_FLOAT_EQ(config.orientation.sensor_scan_limits_deg.az_min_deg, -30.0f);
  EXPECT_FLOAT_EQ(config.orientation.sensor_scan_limits_deg.az_max_deg, 30.0f);
  EXPECT_EQ(config.orientation.stabilization_mode,
            config::SbirsStabilizationMode::kInertialStabilized);
  EXPECT_DOUBLE_EQ(config.orientation.misalignment.bias_deg.pitch_deg, 2.0);
  EXPECT_FLOAT_EQ(config.orientation.misalignment.random_sigma_deg, 0.25f);
  EXPECT_EQ(config.orientation.misalignment.random_seed, 9U);
}

TEST(SbirsPublicApiConvenienceTest, MinimalInputSetsSatelliteAttitudeFlag) {
  // 存在性标志必须与数据一致（docs/common/contract.md 规则 2）：便捷工厂置位
  // has_satellite_attitude，否则输入会被 kInvalidSatelliteAttitude 拒绝。
  const session::SbirsCycleInput input = MakeMinimalInput();
  EXPECT_TRUE(input.has_satellite_attitude);
  EXPECT_TRUE(input.has_satellite_velocity_ecef_m_per_s);
  EXPECT_TRUE(input.has_satellite_position);
  session::SbirsSession session = session::SbirsSession::Create(MakeExecutableConfig());
  EXPECT_EQ(session.StepWithResult(input).status, session::SbirsCycleStatus::kCompleted);
}

TEST(SbirsPublicApiConvenienceTest, RuntimeConfigBuilderDefaultsAreAllUnset) {
  // 默认 Build() 后所有 has_* flag 应为 false（无变更请求）。
  const config::SbirsRuntimeConfigPatch patch = sbirs_config::SbirsRuntimeConfigBuilder().Build();
  EXPECT_FALSE(patch.has_mission);
  EXPECT_FALSE(patch.has_policy);
  EXPECT_FALSE(patch.has_environment);
  EXPECT_FALSE(patch.has_work_mode);
  EXPECT_FALSE(patch.has_scan_rate_deg_per_sec);
  EXPECT_FALSE(patch.has_sensor_enabled);
}

TEST(SbirsPublicApiConvenienceTest, RuntimeConfigBuilderAllFieldsPopulateFlags) {
  const config::SbirsRuntimeConfigPatch patch =
      sbirs_config::SbirsRuntimeConfigBuilder()
          .WithWorkMode(config::SbirsWorkMode::kWideSearch)
          .WithScanRateDegPerSec(8.0f)
          .WithSensorEnabled(true)
          .Build();
  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, config::SbirsWorkMode::kWideSearch);
  EXPECT_TRUE(patch.has_scan_rate_deg_per_sec);
  EXPECT_FLOAT_EQ(patch.scan_rate_deg_per_sec, 8.0f);
  EXPECT_TRUE(patch.has_sensor_enabled);
  EXPECT_TRUE(patch.sensor_enabled);
}

TEST(SbirsPublicApiConvenienceTest, ValidateCycleInputFlagsInvalidDeltaTime) {
  session::SbirsCycleInput input = MakeMinimalInput();
  input.dt_sec = -1.0f;
  const session::SbirsIssueList issues = session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(SbirsPublicApiConvenienceTest, ValidateCycleInputAcceptsValidInput) {
  const session::SbirsIssueList issues = session::ValidateSbirsCycleInput(MakeMinimalInput(), 10.0f);
  EXPECT_FALSE(session::HasValidationError(issues));
}

TEST(SbirsPublicApiConvenienceTest, SessionCreatesAndExecutesOneCycle) {
  session::SbirsSession session = session::SbirsSession::Create(MakeExecutableConfig());
  const session::SbirsCycleResult result = session.StepWithResult(MakeMinimalInput());
  // 结构化执行结果字段可达。
  EXPECT_EQ(result.status, session::SbirsCycleStatus::kCompleted);
  EXPECT_FALSE(session::HasValidationError(result.issues));
  EXPECT_EQ(result.input_cycle_index, 1U);
  EXPECT_EQ(result.abort_reason, session::SbirsPipelineAbortReason::kNone);
}

TEST(SbirsPublicApiConvenienceTest, CreateWithDiagnosticsReportsIssues) {
  // CreateWithDiagnostics 即使配置有问题也构造 session，并把 issues 回填。
  config::SbirsSessionConfig invalid;
  invalid.hardware.wavelength_lower_um = 0.0f;  // 触发波段校验错误
  session::SbirsIssueList issues;
  session::SbirsSession session = session::SbirsSession::CreateWithDiagnostics(invalid, &issues);
  EXPECT_FALSE(issues.empty());
  // session 仍可调用（controller 不会因配置校验 issue 拒绝构造）。
  const session::SbirsCycleResult result = session.StepWithResult(MakeMinimalInput());
  (void)result.status;

  // nullptr issues 参数也接受。
  session::SbirsSession session_null =
      session::SbirsSession::CreateWithDiagnostics(invalid, nullptr);
  (void)session_null;
  SUCCEED();
}

TEST(SbirsPublicApiConvenienceTest, DebugViewAndLifecycleRecorderAreReachable) {
  // 三层输出类型在 public API 可达（design 2.11 三层分离）。
  session::SbirsSession session = session::SbirsSession::Create(MakeExecutableConfig());
  session::SbirsDetectionLifecycleRecorder recorder;
  session.AttachDetectionLifecycleRecorder(&recorder);
  const session::SbirsCycleInput input = MakeMinimalInput();
  const session::SbirsCycleResult result = session.StepWithResult(input);

  const session::SbirsOutputDebugView view =
      session::SbirsOutputDebugViewBuilder::Build(input, result);
  EXPECT_EQ(view.input_cycle_index, input.cycle_index);
  // debug view 回填 target id/name/range/truth-assist，不改变 raw detection。
  ASSERT_EQ(view.targets.size(), 1U);
  EXPECT_EQ(view.targets[0].target_name, "convenience-target");

  // recorder 由 Session 自动驱动（Attach 契约），事件经 GetLastEvents 获取。
  EXPECT_FALSE(recorder.GetLastEvents().empty());
}

TEST(SbirsPublicApiConvenienceTest, RawOutputFrameContainsOnlyNativeFields) {
  // raw output 契约：SbirsOutputFrame 只含原生 SBIRS 观测字段，
  // 不含 range / visible SNR / fused SNR / target id / name / 状态机枚举（design 2.11）。
  session::SbirsSession session = session::SbirsSession::Create(MakeExecutableConfig());
  const session::SbirsCycleResult result = session.StepWithResult(MakeMinimalInput());
  EXPECT_TRUE(session::SbirsOutputFrameContainsOnlyNativeFields(result.output_frame));
}

// SBIRS 遵守统一不复用语义：成功周期后再遇校验失败时，Step() 与 StepWithResult() 均返回
// 默认空帧，不复用上一有效输出。这条 guard 锁定该契约（见 contract.md §实现安全与失败语义规则 3）。
TEST(SbirsPublicApiConvenienceTest, StepReturnsEmptyFrameOnValidationFailureAfterSuccess) {
  session::SbirsSession session = session::SbirsSession::Create(MakeExecutableConfig());
  ASSERT_EQ(session.StepWithResult(MakeMinimalInput(1U)).status, session::SbirsCycleStatus::kCompleted);

  // 校验失败：dt_sec 非正。cycle_index 推进到 8，与上一帧的 1 形成可观测差异。
  session::SbirsCycleInput invalid_input = MakeMinimalInput(8U);
  invalid_input.dt_sec = 0.0f;

  const session::SbirsCycleResult result = session.StepWithResult(invalid_input);
  EXPECT_NE(result.status, session::SbirsCycleStatus::kCompleted);
  EXPECT_TRUE(session::HasValidationError(result.issues));

  // 失败周期返回默认空帧：cycle_index==0（非本次输入 8，也非上一帧的 1），detections 为空。
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
  EXPECT_TRUE(result.output_frame.detections.empty());

  // Step() 与 StepWithResult().output_frame 一致：均为默认空帧。
  const session::SbirsOutputFrame step_frame = session.Step(invalid_input);
  EXPECT_EQ(step_frame.cycle_index, 0U);
  EXPECT_TRUE(step_frame.detections.empty());
}

}  // namespace
}  // namespace tests
}  // namespace sbirs_sensor
