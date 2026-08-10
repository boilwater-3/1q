// Copyright 2026. All Rights Reserved.
//
// @file ar_recognition_integration_test.cpp
// @brief 验证远程识别链路集成（kLrr 调度、积累、判定、保持、回滚与 replay 溯源）。

#include <gtest/gtest.h>

#include <string>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigBuilder.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "RecognitionSqliteTestUtil.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"
#include "1q/coordinate/position_transform.h"

namespace airborne_radar {
namespace session {
namespace {

using session::ArCycleInput;
using session::ArCycleResult;
using session::ArRecognitionCategory;
using session::ArRecognitionState;
using session::ArSession;
using session::ArTargetInput;
using session::TrackStatus;
using tests::kRecognitionSchemaSql;
using tests::WriteTempSqlite;

constexpr const char* kDatabaseSql = R"sql(
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
INSERT INTO categories VALUES ('BALLISTIC','弹道目标',0.5);
INSERT INTO models VALUES ('BALLISTIC_EXAMPLE_A','BALLISTIC','弹道目标示例 A',1.0);
INSERT INTO profiles VALUES ('nominal','BALLISTIC_EXAMPLE_A',-30.0,NULL,NULL,NULL,NULL,NULL);
INSERT INTO rcs_templates VALUES ('nominal','BALLISTIC_EXAMPLE_A',-3.0,2.0,NULL,NULL,NULL);
INSERT INTO motion_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',100.0,30.0,3000.0,500.0,0.0,6.0,6.0,0.5);
)sql";

class ArRecognitionIntegrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    database_path_ =
        WriteTempSqlite("ar_recognition_integration.db", std::string(kRecognitionSchemaSql) + kDatabaseSql);
    ASSERT_FALSE(database_path_.empty());
  }

  config::ArSessionConfig MakeRecognitionConfig() const {
    config::ArSessionConfig cfg;
    cfg.policy.detection = config::profiles::kDetectionPriorityDetection;
    cfg.policy.tracking = config::profiles::kFastAssociationTracking;
    cfg.policy.tracking.enable_kalman_filter = true;  // 关联种子需滤波状态预测
    cfg.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
    cfg.policy.recognition.enabled = true;
    cfg.policy.recognition.database_path = database_path_;
    cfg.policy.recognition.min_confirmed_hits = 1U;
    cfg.policy.recognition.min_observation_count = 1U;
    cfg.policy.recognition.acceptance_score = 0.6f;
    cfg.policy.recognition.minimum_margin = 0.05f;
    cfg.policy.recognition.result_hold_sec = 1.0f;
    return cfg;
  }

  /** @brief 弹道目标：5km 前向、2km 上方，速度 100 m/s，RCS 5 m²。 */
  ArCycleInput MakeBallisticInput(std::uint32_t cycle, double start_time_s) {
    ArCycleInput input;
    input.cycle_index = cycle;
    input.cycle_start_time_s = start_time_s;
    input.dt_sec = 0.5;
    input.platform.platform_entity_id = 10U;
    oneq::coordinate::LlaPositionDegM platform_lla;
    platform_lla.latitude_deg = 31.0;
    platform_lla.longitude_deg = 121.0;
    platform_lla.altitude_m = 1000.0;
    EXPECT_TRUE(
        oneq::coordinate::TryLlaToEcef(platform_lla, &input.platform.platform_position_ecef_m));
    ArTargetInput target;
    target.target_id = 77U;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = input.platform.platform_position_ecef_m;
    // 目标按速度实际移动：位置 = 平台 + (5000 + v·t, 0, 2000)，与速度输入自洽。
    target.kinematics.position_ecef_m.x_m +=
        5000.0 + 100.0 * static_cast<double>(cycle - 1U) * input.dt_sec;
    target.kinematics.position_ecef_m.z_m += 2000.0;
    target.kinematics.velocity_mps.x_mps = 100.0;
    target.rcs = 5.0f;
    // 识别特征真值：覆盖视线角附近的 3×3 视角网格，RCS -3 dBsm（匹配数据库模板）。
    for (float az = -5.0f; az <= 5.0f; az += 5.0f) {
      for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
        session::AspectRcsSample aspect;
        aspect.aspect_az_deg = az;
        aspect.aspect_el_deg = el;
        aspect.rcs_dbsm = -3.0f;
        target.aspect_rcs_samples.push_back(aspect);
      }
    }
    input.targets.push_back(target);
    return input;
  }

  /** @brief 运行 n 个弹道周期（kLrr 已 patch 时积累）。 */
  std::vector<ArCycleResult> RunBallisticCycles(ArSession* radar, std::uint32_t count) {
    std::vector<ArCycleResult> results;
    for (std::uint32_t cycle = 1U; cycle <= count; ++cycle) {
      results.push_back(radar->StepWithResult(
          MakeBallisticInput(cycle, static_cast<double>(cycle - 1U) * 0.5)));
    }
    return results;
  }

  std::string database_path_{};
};


TEST_F(ArRecognitionIntegrationTest, LrrAccumulatesAndConfirmsModelOnConfirmedTrack) {
  ArSession radar = ArSession::Create(MakeRecognitionConfig());
  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kLrr;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(patch));

  const std::vector<ArCycleResult> results = RunBallisticCycles(&radar, 3U);
  const ArCycleResult& last = results.back();
  ASSERT_EQ(last.status, ArCycleStatus::kCompleted);
  ASSERT_FALSE(last.output_frame.tracks.empty());
  // 已确认航迹参与识别并达到型号确认（特征与模板匹配）。
  const auto& track = last.output_frame.tracks.front();
  EXPECT_EQ(track.status, TrackStatus::kConfirmed);
  EXPECT_EQ(track.recognition.state, ArRecognitionState::kModelConfirmed);
  EXPECT_EQ(track.recognition.target_model, "BALLISTIC_EXAMPLE_A");
  EXPECT_EQ(track.recognition.target_category, ArRecognitionCategory::kBallistic);
  EXPECT_GT(track.recognition.confidence, 0.5f);
}

TEST_F(ArRecognitionIntegrationTest, NonLrrModeKeepsRecognitionDisabled) {
  ArSession radar = ArSession::Create(MakeRecognitionConfig());
  // 默认 kTws：识别启用但不执行。
  const ArCycleResult result = radar.StepWithResult(MakeBallisticInput(1U, 0.0));
  ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
  ASSERT_FALSE(result.output_frame.tracks.empty());
  EXPECT_EQ(result.output_frame.tracks.front().recognition.state,
            ArRecognitionState::kDisabled);
}

TEST_F(ArRecognitionIntegrationTest, SummaryCountsMatchOutputFrameStates) {
  ArSession radar = ArSession::Create(MakeRecognitionConfig());
  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kLrr;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(patch));

  const std::vector<ArCycleResult> results = RunBallisticCycles(&radar, 3U);
  const ArCycleResult& last = results.back();
  ASSERT_EQ(last.status, ArCycleStatus::kCompleted);
  ASSERT_TRUE(last.has_recognition_summary);
  std::uint32_t model_confirmed = 0U;
  std::uint32_t participating = 0U;
  for (const auto& track : last.output_frame.tracks) {
    if (track.recognition.state != ArRecognitionState::kDisabled) {
      ++participating;
    }
    if (track.recognition.state == ArRecognitionState::kModelConfirmed) {
      ++model_confirmed;
    }
  }
  EXPECT_EQ(last.recognition_summary.participating_track_count, participating);
  EXPECT_EQ(last.recognition_summary.model_confirmed_count, model_confirmed);
  EXPECT_EQ(last.recognition_summary.category_confirmed_count, 0U);
  EXPECT_EQ(last.recognition_summary.unknown_count, 0U);
}

TEST_F(ArRecognitionIntegrationTest, ExitingLrrHoldsConclusionThenStale) {
  ArSession radar = ArSession::Create(MakeRecognitionConfig());
  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kLrr;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(patch));
  const std::vector<ArCycleResult> lrr_results = RunBallisticCycles(&radar, 3U);
  ASSERT_EQ(lrr_results.back().output_frame.tracks.front().recognition.state,
            ArRecognitionState::kModelConfirmed);

  // 切出 kLrr：结论保持（hold=1.0s），随后过期为 kStale。
  config::ArRuntimeConfigPatch tws_patch;
  tws_patch.has_work_mode = true;
  tws_patch.work_mode = config::ArWorkMode::kTws;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(tws_patch));
  const ArCycleResult held = radar.StepWithResult(MakeBallisticInput(4U, 1.5));
  ASSERT_EQ(held.status, ArCycleStatus::kCompleted);
  EXPECT_EQ(held.output_frame.tracks.front().recognition.state,
            ArRecognitionState::kModelConfirmed);

  // 结论保持 1.0s（sim 时钟累计执行周期 dt）；第 6 周期（sim=3.0）超出 → kStale。
  const ArCycleResult _cycle5 = radar.StepWithResult(MakeBallisticInput(5U, 2.0));
  ASSERT_EQ(_cycle5.status, ArCycleStatus::kCompleted);
  const ArCycleResult stale = radar.StepWithResult(MakeBallisticInput(6U, 3.0));
  ASSERT_EQ(stale.status, ArCycleStatus::kCompleted);
  EXPECT_EQ(stale.output_frame.tracks.front().recognition.state,
            ArRecognitionState::kStale);
}

TEST_F(ArRecognitionIntegrationTest, RejectedPatchKeepsRecognitionOutput) {
  ArSession radar = ArSession::Create(MakeRecognitionConfig());
  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kLrr;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(patch));
  const std::vector<ArCycleResult> lrr_results = RunBallisticCycles(&radar, 3U);
  ASSERT_EQ(lrr_results.back().output_frame.tracks.front().recognition.state,
            ArRecognitionState::kModelConfirmed);

  // 非法 patch（非有限 scan_center）被原子拒绝：识别输出不变。
  config::ArRuntimeConfigPatch invalid;
  invalid.has_scan_center_deg = true;
  invalid.scan_center_deg.az_deg = std::numeric_limits<float>::quiet_NaN();
  invalid.scan_center_deg.el_deg = 0.0f;
  EXPECT_FALSE(radar.TryApplyRuntimeConfig(invalid));

  const ArCycleResult after = radar.StepWithResult(MakeBallisticInput(4U, 1.5));
  ASSERT_EQ(after.status, ArCycleStatus::kCompleted);
  EXPECT_EQ(after.output_frame.tracks.front().recognition.state,
            ArRecognitionState::kModelConfirmed);
}

TEST_F(ArRecognitionIntegrationTest, PoweredOffHoldsConclusionUntilStaleOnNextSuccess) {
  config::ArSessionConfig cfg = MakeRecognitionConfig();
  cfg.policy.recognition.result_hold_sec = 0.4f;  // 缩短保持期：关机后首个成功周期即过期
  ArSession radar = ArSession::Create(cfg);
  config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kLrr;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(patch));
  const std::vector<ArCycleResult> lrr_results = RunBallisticCycles(&radar, 3U);
  ASSERT_EQ(lrr_results.back().output_frame.tracks.front().recognition.state,
            ArRecognitionState::kModelConfirmed);

  // 关机周期不执行：结论保持。
  config::ArRuntimeConfigPatch power_off;
  power_off.has_sensor_enabled = true;
  power_off.sensor_enabled = false;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(power_off));
  const ArCycleResult off = radar.StepWithResult(MakeBallisticInput(4U, 1.5));
  ASSERT_EQ(off.status, ArCycleStatus::kPoweredOff);

  // 恢复供电（仍为 kLrr）但目标消失（无观测）：结论超过 hold 期限（0.4s）→ kStale。
  config::ArRuntimeConfigPatch power_on;
  power_on.has_sensor_enabled = true;
  power_on.sensor_enabled = true;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(power_on));
  for (std::uint32_t cycle = 5U; cycle <= 6U; ++cycle) {
    ArCycleInput no_target = MakeBallisticInput(cycle, static_cast<double>(cycle - 1U) * 0.5);
    no_target.targets.clear();
    const ArCycleResult step = radar.StepWithResult(no_target);
    ASSERT_EQ(step.status, ArCycleStatus::kCompleted);
    if (cycle == 6U) {
      ASSERT_FALSE(step.output_frame.tracks.empty());
      EXPECT_EQ(step.output_frame.tracks.front().recognition.state,
                ArRecognitionState::kStale);
    }
  }
}

TEST_F(ArRecognitionIntegrationTest, ReplayStateCarriesActiveDatabaseVersion) {
  ArSession radar = ArSession::Create(MakeRecognitionConfig());
  const ArSessionReplayState state = ArSessionReplayAccess::CaptureSessionState(radar);
  EXPECT_EQ(state.active_database_version, "1.0.0");
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
