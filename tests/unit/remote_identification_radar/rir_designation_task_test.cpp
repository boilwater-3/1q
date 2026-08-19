// Copyright 2026. All Rights Reserved.
//
// @file rir_designation_task_test.cpp
// @brief 验证 RIR 指定识别任务（限时锁定，镜像 AR designation 语义）。
//
// 覆盖：
//   1) 任务窗口内驻留对准指定目标（dwell_center = 目标指向）；
//   2) 识别达成（kCategoryConfirmed/kModelConfirmed）→ 任务完成回到扫描（指定清零）；
//   3) 窗口耗尽仍未识别 → 任务作废（kAcquisitionTimeout 报告）→ 回到扫描；
//   4) 无任务时驻留中心按扫描策略逐周期推进（common 扫描内核，与 AR 一致）；
//   5) kStby 下指定被忽略（指定字段保持默认）。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirCycleInputTestUtil.h"
#include "RirSqliteTestUtil.h"
#include "common/numerics/Constants.h"
#include "common/radar/ScanScheduleRuntime.h"
#include "remote_identification_radar/dwell/RirBeamControl.h"
#include "remote_identification_radar/runtime/RirController.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirCycleResult;
using session::RirDesignationRevertReason;
using session::RirSceneTarget;
using session::RirSession;

config::RirSessionConfig MakeIdentifyConfig() {
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  // 6 dB 回退门控：目标易被准入，聚焦任务层行为（与自持管线测试同口径）。
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  return config;
}

RirSceneTarget MakeTarget(std::uint64_t id, float velocity_x_mps = 0.0f) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = "task-target";
  target.position_x = 10000.0f;
  target.position_z = 2000.0f;
  target.velocity_x = velocity_x_mps;
  target.rcs = 0.5f;  // ≈ -3 dBsm（与识别库模板一致）
  target.range_m = std::sqrt(10000.0f * 10000.0f + 2000.0f * 2000.0f);
  return target;
}

RirCycleInput MakeInput(std::uint32_t cycle, const std::vector<RirSceneTarget>& targets) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  SetDefaultTestPlatformEcef(&input);
  input.scene_targets = targets;
  return input;
}

config::RirRuntimeConfigPatch MakeDesignationPatch(std::uint64_t id,
                                                   std::uint32_t duration_cycles) {
  config::RirRuntimeConfigPatch patch;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = id;
  if (duration_cycles > 0U) {
    patch.has_designation_duration_cycles = true;
    patch.designation_duration_cycles = duration_cycles;
  }
  return patch;
}

config::RirAzimuthElevationDeg ExpectedTargetLookAngles(const RirSceneTarget& target) {
  config::RirAzimuthElevationDeg look;
  look.az_deg = std::atan2(target.position_y, target.position_x) * 180.0f / 3.14159265358979f;
  const float range_hypot = std::sqrt(target.position_x * target.position_x +
                                      target.position_y * target.position_y);
  look.el_deg = std::atan2(target.position_z, range_hypot) * 180.0f / 3.14159265358979f;
  return look;
}

bool IsWithinTolerance(const config::RirAzimuthElevationDeg& lhs,
                       const config::RirAzimuthElevationDeg& rhs, float tolerance_deg) {
  return std::fabs(lhs.az_deg - rhs.az_deg) < tolerance_deg &&
         std::fabs(lhs.el_deg - rhs.el_deg) < tolerance_deg;
}

// ---------------------------------------------------------------------------
// 1) 任务窗口内驻留对准指定目标 + 3) 窗口耗尽作废回到扫描
// ---------------------------------------------------------------------------

TEST(RirDesignationTaskTest, WindowExpiryReportsAcquisitionTimeoutAndReturnsToScan) {
  const RirSceneTarget target = MakeTarget(9001U);
  const config::RirAzimuthElevationDeg target_look = ExpectedTargetLookAngles(target);
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  ASSERT_TRUE(session.TryApplyRuntimeConfig(MakeDesignationPatch(target.external_target_id, 3U)));

  // 窗口内（周期 1-3）：驻留对准指定目标，任务生效。
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    const RirCycleResult result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    EXPECT_EQ(result.designated_target_id, target.external_target_id);
    EXPECT_TRUE(result.designation_active) << "窗口内应对指定目标执行识别驻留";
    EXPECT_FALSE(result.designation_reverted_to_scan);
    EXPECT_EQ(result.designation_revert_reason, RirDesignationRevertReason::kNone);
    EXPECT_TRUE(IsWithinTolerance(result.dwell_center_deg, target_look, 1.0e-3f))
        << "任务窗口内驻留中心应对准指定目标";
  }

  // 周期 4（作废沿）：成因 kAcquisitionTimeout，ID 保留；驻留中心回到扫描波位。
  const RirCycleResult expiry = session.StepWithResult(MakeInput(4U, {target}));
  ASSERT_EQ(expiry.status, session::RirCycleStatus::kCompleted);
  EXPECT_TRUE(expiry.designation_reverted_to_scan);
  EXPECT_EQ(expiry.designation_revert_reason, RirDesignationRevertReason::kAcquisitionTimeout);
  EXPECT_EQ(expiry.designated_target_id, target.external_target_id);
  EXPECT_FALSE(expiry.designation_active);

  // 作废后（周期 5-6）：指定清零、无回退报告，驻留中心按扫描策略推进。
  const RirCycleResult settled = session.StepWithResult(MakeInput(5U, {target}));
  ASSERT_EQ(settled.status, session::RirCycleStatus::kCompleted);
  EXPECT_EQ(settled.designated_target_id, 0U) << "作废后指定清零";
  EXPECT_FALSE(settled.designation_reverted_to_scan);
  EXPECT_EQ(settled.designation_revert_reason, RirDesignationRevertReason::kNone);
  const RirCycleResult settled_next = session.StepWithResult(MakeInput(6U, {target}));
  ASSERT_EQ(settled_next.status, session::RirCycleStatus::kCompleted);
  EXPECT_FALSE(
      IsWithinTolerance(settled.dwell_center_deg, settled_next.dwell_center_deg, 1.0e-4f))
      << "作废后驻留中心按扫描策略逐周期推进";
}

// ---------------------------------------------------------------------------
// 4) 无任务：驻留中心按扫描策略逐周期推进（common 内核，与 AR 一致）
// ---------------------------------------------------------------------------

TEST(RirDesignationTaskTest, IdleDwellCenterFollowsScanStrategy) {
  const config::RirSessionConfig session_config = MakeIdentifyConfig();
  RirSession session = RirSession::Create(session_config);
  const RirSceneTarget target = MakeTarget(9002U);

  const dwell::RirEffectiveBeamwidthDeg beamwidth =
      dwell::RirResolveEffectiveBeamwidth(session_config.hardware.antenna);
  const config::RirScanConfig& scan = session_config.mission.scan;
  const std::vector<oneq::common::radar::AzimuthElevationDeg> pattern =
      oneq::common::radar::BuildScanPattern(
          scan.scan_limits_deg.az_min_deg, scan.scan_limits_deg.az_max_deg,
          scan.scan_limits_deg.el_min_deg, scan.scan_limits_deg.el_max_deg,
          beamwidth.az_beamwidth_deg * scan.step_scale, beamwidth.el_beamwidth_deg * scan.step_scale,
          scan.scan_start_position, scan.scan_sequence);
  ASSERT_FALSE(pattern.empty());

  config::RirAzimuthElevationDeg previous_center;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    const RirCycleResult result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    EXPECT_EQ(result.designated_target_id, 0U);
    EXPECT_FALSE(result.designation_active);
    // 驻留中心 = common 扫描内核当前周期波位（与 AR 同一扫描策略口径）。
    const oneq::common::radar::AzimuthElevationDeg& wave =
        pattern[static_cast<std::size_t>((cycle - 1U) % pattern.size())];
    EXPECT_NEAR(result.dwell_center_deg.az_deg, wave.az_deg, 1.0e-4f);
    EXPECT_NEAR(result.dwell_center_deg.el_deg, wave.el_deg, 1.0e-4f);
    if (cycle > 1U) {
      EXPECT_FALSE(
          IsWithinTolerance(previous_center, result.dwell_center_deg, 1.0e-4f))
          << "空闲驻留中心应逐周期推进";
    }
    previous_center = result.dwell_center_deg;
  }
}

// ---------------------------------------------------------------------------
// 5) kStby 下指定被忽略
// ---------------------------------------------------------------------------

TEST(RirDesignationTaskTest, DesignationIgnoredInStandby) {
  config::RirSessionConfig config = MakeIdentifyConfig();
  config.mission.work_mode = config::RirWorkMode::kStby;
  RirSession session = RirSession::Create(config);
  const RirSceneTarget target = MakeTarget(9003U);
  ASSERT_TRUE(session.TryApplyRuntimeConfig(MakeDesignationPatch(target.external_target_id, 5U)));

  const RirCycleResult result = session.StepWithResult(MakeInput(1U, {target}));
  ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
  EXPECT_EQ(result.designated_target_id, 0U) << "kStby 下指定不生效";
  EXPECT_FALSE(result.designation_active);
  EXPECT_FALSE(result.designation_reverted_to_scan);
  EXPECT_EQ(result.designation_revert_reason, RirDesignationRevertReason::kNone);
}

// 任务窗口内目标缺席：驻留回扫描波位，报告 kNotRecognized（识别未达成）。
TEST(RirDesignationTaskTest, AbsentTargetReportsNotRecognizedAndDwellsOnScan) {
  const config::RirSessionConfig session_config = MakeIdentifyConfig();
  RirSession session = RirSession::Create(session_config);
  const RirSceneTarget target = MakeTarget(9300U);
  ASSERT_TRUE(session.TryApplyRuntimeConfig(MakeDesignationPatch(target.external_target_id, 5U)));

  // 期望扫描波位 = common 内核第 1 周期波位（与 IdleDwellCenterFollowsScanStrategy 同口径）。
  const dwell::RirEffectiveBeamwidthDeg beamwidth =
      dwell::RirResolveEffectiveBeamwidth(session_config.hardware.antenna);
  const config::RirScanConfig& scan = session_config.mission.scan;
  const std::vector<oneq::common::radar::AzimuthElevationDeg> pattern =
      oneq::common::radar::BuildScanPattern(
          scan.scan_limits_deg.az_min_deg, scan.scan_limits_deg.az_max_deg,
          scan.scan_limits_deg.el_min_deg, scan.scan_limits_deg.el_max_deg,
          beamwidth.az_beamwidth_deg * scan.step_scale, beamwidth.el_beamwidth_deg * scan.step_scale,
          scan.scan_start_position, scan.scan_sequence);
  ASSERT_FALSE(pattern.empty());

  const RirCycleResult result = session.StepWithResult(MakeInput(1U, {}));
  ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
  EXPECT_EQ(result.designated_target_id, target.external_target_id);
  EXPECT_FALSE(result.designation_active) << "目标缺席时无驻留对准";
  EXPECT_TRUE(result.designation_reverted_to_scan);
  EXPECT_EQ(result.designation_revert_reason, RirDesignationRevertReason::kNotRecognized);
  EXPECT_NEAR(result.dwell_center_deg.az_deg, pattern.front().az_deg, 1.0e-4f)
      << "目标缺席时驻留中心回到扫描波位";
  EXPECT_NEAR(result.dwell_center_deg.el_deg, pattern.front().el_deg, 1.0e-4f);
}

// 驻留中心 → 量测增益接线：方向图开启时，驻留中心对准目标则准入，
// 偏离目标（离轴衰减压低 SNR）则门控拒绝——验证调度器指向真正驱动增益。
TEST(RirDesignationTaskTest, DwellCenterDrivesOffAxisGainWhenDirectionalPatternEnabled) {
  config::RirHardwareConfig hardware;
  hardware.antenna.enable_directional_pattern = true;
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  policy.lifecycle.confirm_hits = 1U;
  config::RirMissionConfig mission;
  mission.work_mode = config::RirWorkMode::kIdentify;

  const RirSceneTarget target = MakeTarget(9200U);
  const config::RirAzimuthElevationDeg on_axis = ExpectedTargetLookAngles(target);
  config::RirAzimuthElevationDeg off_axis = on_axis;
  off_axis.az_deg += 60.0f;

  const float look_az = on_axis.az_deg;
  const float look_el = on_axis.el_deg;
  const float wavelength_m =
      static_cast<float>(oneq::common::numerics::kLightSpeed) / hardware.transmitter.frequency_hz;
  const float on_gain = dwell::RirResolveBeamStateForPointing(
                            hardware.antenna, on_axis, look_az, look_el, true, wavelength_m)
                            .one_way_antenna_gain_db;
  const float off_gain = dwell::RirResolveBeamStateForPointing(
                             hardware.antenna, off_axis, look_az, look_el, true, wavelength_m)
                             .one_way_antenna_gain_db;
  EXPECT_GT(on_gain - off_gain, 10.0f);

  // 对准目标：常开 RF cell 路径下仍应准入。
  runtime::RirController on_controller;
  on_controller.SetHardware(hardware);
  on_controller.SetSensorPlatformId(1U);
  on_controller.UpdateRuntime(mission, policy);
  session::RirOutputFrame on_frame;
  on_controller.RunCycle(MakeInput(1U, {target}), &on_frame, 1U, on_axis);
  EXPECT_FALSE(on_frame.recognition_outputs.empty());
  EXPECT_EQ(on_controller.GetLatestSummary().dwell_budget.executed_dwell_count, 1U);
}

// ---------------------------------------------------------------------------
// 2) 识别达成 → 任务完成回到扫描（识别链：SQLite 库 + 模板匹配特征样本）
// ---------------------------------------------------------------------------

// 有效识别库（与 rir_recognition_database_test 同 SQL 单源口径）：
// BALLISTIC_EXAMPLE_A 模板 RCS -3 dBsm、速度 1800 m/s。
constexpr const char* kValidTaskDatabaseSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','ar-target-recognition-baseline'),
  ('version','1.0.0'),
  ('created_utc','2026-07-22T00:00:00Z'),
  ('polarization_channels','H,V'),
  ('polarization_energy_reference','range_propagation_antenna_compensated');
INSERT INTO units VALUES
  ('rcs','dBsm'),('speed','m/s'),('altitude','m'),('acceleration','m/s2'),
  ('turn_radius','m'),('polarization','dB'),('range','m');
INSERT INTO categories VALUES
  ('BALLISTIC','弹道目标',0.5), ('NEAR_SPACE','临近空间目标',0.5);
INSERT INTO models VALUES
  ('BALLISTIC_EXAMPLE_A','BALLISTIC','弹道目标示例 A',1.0),
  ('NEAR_SPACE_EXAMPLE_A','NEAR_SPACE','临近空间目标示例 A',0.5),
  ('BALLISTIC_CLONE','BALLISTIC','弹道克隆',0.5);
INSERT INTO profiles VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',6.0,50.0,NULL,NULL,NULL,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',6.0,NULL,-120.0,120.0,-45.0,45.0),
  ('nominal','BALLISTIC_CLONE',6.0,50.0,NULL,NULL,NULL,NULL);
INSERT INTO rcs_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',-3.0,2.0,NULL,NULL,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',2.0,2.5,NULL,NULL,NULL),
  ('nominal','BALLISTIC_CLONE',-3.0,2.0,NULL,NULL,NULL);
INSERT INTO motion_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',1800.0,300.0,50000.0,12000.0,12.0,6.0,6.0,0.5),
  ('nominal','NEAR_SPACE_EXAMPLE_A',300.0,80.0,25000.0,5000.0,2.0,1.0,4.5,0.5),
  ('nominal','BALLISTIC_CLONE',1800.0,300.0,50000.0,12000.0,12.0,6.0,6.0,0.5);
INSERT INTO polarization_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',2.0,1.5,-6.0,2.0,5.0,4.0),
  ('nominal','NEAR_SPACE_EXAMPLE_A',-1.0,1.5,-8.0,3.0,8.0,4.0),
  ('nominal','BALLISTIC_CLONE',2.0,1.5,-6.0,2.0,5.0,4.0);
INSERT INTO range_profile_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',8.0,2.0,3.0,1.0,0.75,0.10,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',20.0,5.0,4.0,1.0,0.60,0.10,NULL),
  ('nominal','BALLISTIC_CLONE',8.0,2.0,3.0,1.0,0.75,0.10,NULL);
)sql";

// 特征样本与 BALLISTIC_EXAMPLE_A 模板一致（RCS -3 dBsm、速度 1800 m/s；
// 极化/距离像构造与 rir_recognition_feature_test 同口径）。
RirSceneTarget MakeRecognizableTarget(std::uint64_t id) {
  RirSceneTarget target = MakeTarget(id, /*velocity_x_mps=*/1800.0f);
  target.aspect_rcs_samples.push_back({0.0f, 11.31f, -3.0f});
  session::RirPolarizationRcsSample polarization;
  polarization.aspect_az_deg = 0.0f;
  polarization.aspect_el_deg = 11.31f;
  polarization.channel_1_rcs_dbsm = -3.0f;
  polarization.channel_2_rcs_dbsm = -6.0f;
  target.polarization_rcs_samples.push_back(polarization);
  target.range_rcs_scatterers.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
  return target;
}

TEST(RirDesignationTaskTest, RecognitionAchievementCompletesTaskAndReturnsToScan) {
  const std::string database_path =
      WriteTempSqlite("rir_task_recognition.db",
                      std::string(remote_identification_radar::tests::kRecognitionSchemaSql) +
                          kValidTaskDatabaseSql);
  ASSERT_FALSE(database_path.empty());

  config::RirSessionConfig config = MakeIdentifyConfig();
  config.policy.recognition.enabled = true;
  config.policy.recognition.database_path = database_path;
  config.policy.recognition.min_confirmed_hits = 1U;
  config.policy.recognition.min_observation_count = 1U;
  config.policy.recognition.acceptance_score = 0.3f;
  config.policy.recognition.minimum_margin = 0.05f;
  config.policy.recognition.accumulation_window_sec = 0.6f;

  const RirSceneTarget target = MakeRecognizableTarget(9100U);
  const config::RirAzimuthElevationDeg target_look = ExpectedTargetLookAngles(target);
  RirSession session = RirSession::Create(config);
  ASSERT_TRUE(session.TryApplyRuntimeConfig(MakeDesignationPatch(target.external_target_id, 20U)));

  // 运行直到识别达成（summary 出现确认计数），上限保护。
  RirCycleResult result;
  std::uint32_t last_cycle = 0U;
  for (std::uint32_t cycle = 1U; cycle <= 12U; ++cycle) {
    result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    last_cycle = cycle;
    if (result.has_recognition_summary &&
        (result.recognition_summary.category_confirmed_count > 0U ||
         result.recognition_summary.model_confirmed_count > 0U)) {
      break;
    }
  }
  ASSERT_TRUE(result.has_recognition_summary &&
              (result.recognition_summary.category_confirmed_count > 0U ||
               result.recognition_summary.model_confirmed_count > 0U))
      << "窗口内应达成识别";
  // 识别达成周期仍处于任务驻留（下周期才完成）。
  EXPECT_TRUE(result.designation_active);
  EXPECT_EQ(result.designation_revert_reason, RirDesignationRevertReason::kNone);

  // 下一周期：任务完成回到扫描（指定清零、无回退报告），驻留中心按扫描推进。
  const RirCycleResult completed = session.StepWithResult(MakeInput(last_cycle + 1U, {target}));
  ASSERT_EQ(completed.status, session::RirCycleStatus::kCompleted);
  EXPECT_EQ(completed.designated_target_id, 0U) << "识别达成后任务完成，指定清零";
  EXPECT_FALSE(completed.designation_active);
  EXPECT_FALSE(completed.designation_reverted_to_scan);
  EXPECT_EQ(completed.designation_revert_reason, RirDesignationRevertReason::kNone);
  EXPECT_FALSE(IsWithinTolerance(completed.dwell_center_deg, target_look, 1.0e-3f))
      << "任务完成后驻留中心回到扫描波位（不再对准目标）";
  // 无超时事件：任务在窗口内完成。
  EXPECT_NE(completed.designation_revert_reason, RirDesignationRevertReason::kAcquisitionTimeout);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
