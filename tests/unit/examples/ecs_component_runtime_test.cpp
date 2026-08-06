/**
 * @file ecs_component_runtime_test.cpp
 * @brief component_attachment 组件运行时修改接口单元测试。
 *
 * 覆盖四个组件的运行时修改入口（未来外部调用入口，薄包装库 API）：
 * - ArSensorComponent::TryApplyRuntimeConfig —— AR 事务性提交（补丁先暂存、
 *   下个周期边界生效；与现有配置冲突的非法补丁入口即原子拒绝）。
 * - EsrSensorComponent::TryApplyRuntimeConfig / ApplyRuntimeConfigWithResult
 *   —— ESR 立即提交（调用即生效）；结构化结果携带拒绝原因枚举。
 * - EosSensorComponent::TryApplyRuntimeConfig —— EOS 立即提交（原子校验）。
 * - FlightComponent::PushManeuver / ClearManeuvers / Abort —— FD 命令式
 *   入口；FD 未启用/初始化失败（运动学回退）时返回 false。
 *
 * 只调用组件接口本身（不驱动 Step），验证转发语义与库一致；合法/非法
 * patch 取值沿用各模块 resolver 单测的已知用例。Fusion 无运行时修改设计
 * （库内无 RuntimeConfigPatch），不在覆盖范围（组件注释已注明）。
 */

#include <limits>

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/flight_dynamic/FlightManager.h"
#include "components/ar_sensor_component.h"
#include "components/eos_sensor_component.h"
#include "components/esr_sensor_component.h"
#include "components/flight_component.h"

namespace ca = component_attachment;

namespace {

/// AR 最小合法会话配置（与 ar_rf_session_test 同源：policy 三 profile）。
airborne_radar::config::ArSessionConfig MakeArConfig() {
  airborne_radar::config::ArSessionConfig cfg;
  cfg.policy.detection = airborne_radar::config::profiles::kDetectionPriorityDetection;
  cfg.policy.tracking = airborne_radar::config::profiles::kFastAssociationTracking;
  cfg.policy.lifecycle = airborne_radar::config::profiles::kFastConfirmLifecycle;
  return cfg;
}

/// 机场位置（度制 LLA，FD 模式下视为地面）。
oneq::coordinate::LlaPositionDegM MakeAirfield() {
  oneq::coordinate::LlaPositionDegM airfield;
  airfield.latitude_deg = 30.0;
  airfield.longitude_deg = 120.0;
  airfield.altitude_m = 0.0;
  return airfield;
}

}  // namespace

// =============================================================================
// ArSensorComponent：AR 事务性提交（暂存语义，入口原子拒绝）
// =============================================================================

TEST(ArSensorComponentRuntimeTest, ValidWorkModePatchAccepted) {
  ca::ArSensorComponent component(airborne_radar::session::ArSession::Create(MakeArConfig()));

  airborne_radar::config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = airborne_radar::config::ArWorkMode::kStt;

  EXPECT_TRUE(component.TryApplyRuntimeConfig(patch));
}

TEST(ArSensorComponentRuntimeTest, NonPositiveBeamwidthRejected) {
  ca::ArSensorComponent component(airborne_radar::session::ArSession::Create(MakeArConfig()));

  // 非正指令波束宽度（沿用 ar_runtime_patch_mapper_test 已知非法用例）。
  airborne_radar::config::ArRuntimeConfigPatch patch;
  patch.has_commanded_beamwidth_enabled = true;
  patch.commanded_beamwidth_enabled = true;
  patch.has_commanded_beamwidth_deg = true;
  patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 0.0f;

  EXPECT_FALSE(component.TryApplyRuntimeConfig(patch));
}

// =============================================================================
// EsrSensorComponent：ESR 立即提交 + 结构化结果
// =============================================================================

TEST(EsrSensorComponentRuntimeTest, ValidScanRatePatchAppliesImmediately) {
  ca::EsrSensorComponent component(electronic_surveillance_radar::session::EsrSession::Create());

  electronic_surveillance_radar::config::EsrRuntimeConfigPatch patch;
  patch.has_scan_rate_hz = true;
  patch.scan_rate_hz = 4.0f;  // 合法扫描率（沿用 resolver 单测取值）

  EXPECT_TRUE(component.TryApplyRuntimeConfig(patch));

  // 结构化结果入口：同值语义下仍为已应用（resolver 校验值域而非变化）。
  const auto result = component.ApplyRuntimeConfigWithResult(patch);
  EXPECT_EQ(result.status,
            electronic_surveillance_radar::session::EsrRuntimeConfigApplyStatus::kApplied);
  EXPECT_TRUE(result.applied);
}

TEST(EsrSensorComponentRuntimeTest, InfiniteScanRateRejectedWithStructuredStatus) {
  ca::EsrSensorComponent component(electronic_surveillance_radar::session::EsrSession::Create());

  // 非有限扫描率（沿用 resolver 单测 kRejectedInvalidScanRate 用例）。
  electronic_surveillance_radar::config::EsrRuntimeConfigPatch patch;
  patch.has_scan_rate_hz = true;
  patch.scan_rate_hz = std::numeric_limits<float>::infinity();

  EXPECT_FALSE(component.TryApplyRuntimeConfig(patch));

  const auto result = component.ApplyRuntimeConfigWithResult(patch);
  EXPECT_EQ(
      result.status,
      electronic_surveillance_radar::session::EsrRuntimeConfigApplyStatus::kRejectedInvalidScanRate);
  EXPECT_FALSE(result.applied);
}

// =============================================================================
// EosSensorComponent：EOS 立即提交（原子校验）
// =============================================================================

TEST(EosSensorComponentRuntimeTest, ValidScanRatePatchAppliesImmediately) {
  ca::EosSensorComponent component(electro_optical_sensor::session::EosSession::Create());

  electro_optical_sensor::config::EosRuntimeConfigPatch patch;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 60.0f;  // 合法扫描角速度（沿用 resolver 单测取值）

  EXPECT_TRUE(component.TryApplyRuntimeConfig(patch));
}

TEST(EosSensorComponentRuntimeTest, ZeroFrameRateRejectsWholePatch) {
  ca::EosSensorComponent component(electro_optical_sensor::session::EosSession::Create());

  // 非正帧率（沿用 resolver 单测 InvalidFieldRejectsWholePatch 用例）。
  electro_optical_sensor::config::EosRuntimeConfigPatch patch;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 60.0f;
  patch.has_frame_rate_hz = true;
  patch.frame_rate_hz = 0.0f;

  EXPECT_FALSE(component.TryApplyRuntimeConfig(patch));
}

// =============================================================================
// FlightComponent：FD 命令式入口（FD 不可用返回 false）
// =============================================================================

TEST(FlightComponentRuntimeTest, ManeuverInterfacesReportFlightDynamicsAvailability) {
  ca::FlightComponent component(MakeAirfield(), /*initial_heading_deg=*/90.0,
                                /*initial_speed_mps=*/50.0, /*cruise_altitude_m=*/400.0,
                                {});

  oneq::flight_dynamic::ManeuverCommand cmd;
  cmd.type = oneq::flight_dynamic::guidance::ManeuverType::kFlyToWaypoint;
  cmd.target.latitude_rad = 0.52;  // ≈ 30°（正东 3 航点首点量级）
  cmd.target.longitude_rad = 2.10;
  cmd.target.altitude_m = 400.0;
  cmd.target.radius_m = 500.0;
  cmd.target.speed_mps = 50.0;

  const bool pushed = component.PushManeuver(cmd);
  const bool cleared = component.ClearManeuvers();
  const bool aborted = component.Abort();

#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
  // FD 启用且 JSBSim c172x 数据可用（third_party/jsbsim，与 fd 测试分区同
  // 前提）：三个入口均成功转发到 FlightManager。
  EXPECT_TRUE(pushed);
  EXPECT_TRUE(cleared);
  EXPECT_TRUE(aborted);
#else
  // FD 未启用：运动学回退路径，指令接口返回 false（不崩溃）。
  EXPECT_FALSE(pushed);
  EXPECT_FALSE(cleared);
  EXPECT_FALSE(aborted);
#endif
}
