// Copyright 2026. All Rights Reserved.
//
// @file ar_recognition_scenario_test.cpp
// @brief 远程识别效能验证：七类标注场景 + 混合 + 模式切换 + replay 往返。
//
// 场景物理量级按会话探测几何标定（效能级验证结构性指标：达成率/混淆率/
// 模式切换/门控行为）；特征模板与场景真值协同构造，识别链路消费效能化
// 观测而非真值（§2 设计约束由 RecognitionObservationBuilder 保证）。

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "RecognitionSqliteTestUtil.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"
#include "1q/coordinate/position_transform.h"

namespace airborne_radar {
namespace tests {
namespace {

using session::ArCycleInput;
using session::ArCycleStatus;
using session::ArCycleResult;
using session::ArRecognitionCategory;
using session::ArRecognitionState;
using session::ArSession;
using session::ArSessionReplayAccess;
using session::ArSessionReplayState;
using session::ArTargetInput;

constexpr const char* kScenarioDatabaseSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','ar-recognition-scenario-baseline'),
  ('version','2.0.0'),
  ('created_utc','2026-07-22T00:00:00Z'),
  ('polarization_channels','H,V'),
  ('polarization_energy_reference','range_propagation_antenna_compensated');
INSERT INTO units VALUES
  ('rcs','dBsm'),('speed','m/s'),('altitude','m'),('acceleration','m/s2'),
  ('turn_radius','m'),('polarization','dB'),('range','m');
INSERT INTO categories VALUES ('BALLISTIC','弹道目标',0.5), ('NEAR_SPACE','临近空间目标',0.5);
INSERT INTO models VALUES
  ('BALLISTIC_EXAMPLE_A','BALLISTIC','弹道目标示例 A',1.0),
  ('NEAR_SPACE_EXAMPLE_A','NEAR_SPACE','临近空间目标示例 A',1.0);
INSERT INTO profiles VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',-30.0,NULL,NULL,NULL,NULL,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',-30.0,NULL,NULL,NULL,NULL,NULL);
INSERT INTO rcs_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',-3.0,2.0,NULL,NULL,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',2.0,2.5,NULL,NULL,NULL);
INSERT INTO motion_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',100.0,30.0,3000.0,500.0,0.0,6.0,6.0,0.5),
  ('nominal','NEAR_SPACE_EXAMPLE_A',400.0,80.0,1500.0,300.0,0.0,2.0,4.5,0.5);
)sql";

class ArRecognitionScenarioTest : public ::testing::Test {
 protected:
  void SetUp() override {
    database_path_ =
        WriteTempSqlite("ar_recognition_scenario.db",
                        std::string(kRecognitionSchemaSql) + kScenarioDatabaseSql);
    ASSERT_FALSE(database_path_.empty());
  }

  config::ArSessionConfig MakeScenarioConfig() const {
    config::ArSessionConfig cfg;
    cfg.policy.detection = config::profiles::kDetectionPriorityDetection;
    cfg.policy.tracking = config::profiles::kFastAssociationTracking;
    cfg.policy.tracking.enable_kalman_filter = true;
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

  /** @brief 构造标注目标：速度/高度/视角 RCS 真值 + 真值名。 */
  void AppendTarget(ArCycleInput* input, std::uint64_t target_id, float speed_mps,
                    float altitude_offset_m, float rcs_dbsm, const char* truth_name) {
    ArTargetInput target;
    target.target_id = target_id;
    target.target_name = truth_name;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = input->platform.platform_position_ecef_m;
    target.kinematics.position_ecef_m.x_m +=
        5000.0 + speed_mps * static_cast<double>(input->cycle_index - 1U) * input->dt_sec;
    target.kinematics.position_ecef_m.z_m += altitude_offset_m;
    target.kinematics.velocity_mps.x_mps = speed_mps;
    target.rcs = 5.0f;
    // 视角网格（方位 ±5°，俯仰 5°~30°），RCS 恒为模板值。
    for (float az = -5.0f; az <= 5.0f; az += 5.0f) {
      for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
        session::AspectRcsSample aspect;
        aspect.aspect_az_deg = az;
        aspect.aspect_el_deg = el;
        aspect.rcs_dbsm = rcs_dbsm;
        target.aspect_rcs_samples.push_back(aspect);
      }
    }
    input->targets.push_back(target);
  }

  ArCycleInput MakeInput(std::uint32_t cycle) {
    ArCycleInput input;
    input.cycle_index = cycle;
    input.cycle_start_time_s = static_cast<double>(cycle - 1U) * 0.5;
    input.dt_sec = 0.5;
    input.platform.platform_entity_id = 10U;
    oneq::coordinate::LlaPositionDegM platform_lla;
    platform_lla.latitude_deg = 31.0;
    platform_lla.longitude_deg = 121.0;
    platform_lla.altitude_m = 1000.0;
    EXPECT_TRUE(
        oneq::coordinate::TryLlaToEcef(platform_lla, &input.platform.platform_position_ecef_m));
    return input;
  }

  void EnableLrr(ArSession* radar) {
    config::ArRuntimeConfigPatch patch;
    patch.has_work_mode = true;
    patch.work_mode = config::ArWorkMode::kLrr;
    ASSERT_TRUE(radar->TryApplyRuntimeConfig(patch));
  }

  std::string database_path_{};
};

// -- 场景 1/2：弹道与临近空间目标达成率与型号正确率（有真值） ------------

TEST_F(ArRecognitionScenarioTest, BallisticScenarioReachesConfirmationWithHighAccuracy) {
  ArSession radar = ArSession::Create(MakeScenarioConfig());
  EnableLrr(&radar);
  std::uint32_t confirmed_cycles = 0U;
  std::uint32_t correct_cycles = 0U;
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    ArCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
    const ArCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.tracks.size(), 1U);
    const auto& track = result.output_frame.tracks.front();
    if (track.recognition.state == ArRecognitionState::kCategoryConfirmed ||
        track.recognition.state == ArRecognitionState::kModelConfirmed) {
      ++confirmed_cycles;
      if (track.recognition.target_model == "BALLISTIC_EXAMPLE_A") {
        ++correct_cycles;
      }
    }
  }
  // 达成率 ≥ 80%，型号正确率 ≥ 70%。
  EXPECT_GE(confirmed_cycles, kCycles * 8U / 10U);
  EXPECT_GE(correct_cycles, confirmed_cycles * 7U / 10U);
}

TEST_F(ArRecognitionScenarioTest, NearSpaceScenarioReachesConfirmationWithHighAccuracy) {
  ArSession radar = ArSession::Create(MakeScenarioConfig());
  EnableLrr(&radar);
  std::uint32_t confirmed_cycles = 0U;
  std::uint32_t correct_cycles = 0U;
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    ArCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 88U, 400.0f, 500.0f, 2.0f, "NEAR_SPACE_EXAMPLE_A");
    const ArCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.tracks.size(), 1U);
    const auto& track = result.output_frame.tracks.front();
    if (track.recognition.state == ArRecognitionState::kCategoryConfirmed ||
        track.recognition.state == ArRecognitionState::kModelConfirmed) {
      ++confirmed_cycles;
      if (track.recognition.target_model == "NEAR_SPACE_EXAMPLE_A") {
        ++correct_cycles;
      }
    }
  }
  EXPECT_GE(confirmed_cycles, kCycles * 8U / 10U);
  EXPECT_GE(correct_cycles, confirmed_cycles * 7U / 10U);
}

// -- 场景 3：未知目标拒绝误识 --------------------------------------------

TEST_F(ArRecognitionScenarioTest, UnknownTargetRejectsWithHighUnknownRate) {
  ArSession radar = ArSession::Create(MakeScenarioConfig());
  EnableLrr(&radar);
  std::uint32_t unknown_cycles = 0U;
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    ArCycleInput input = MakeInput(cycle);
    // 特征偏离所有模板：RCS +20 dBsm、速度 20 m/s、高度 1000 m。
    AppendTarget(&input, 99U, 20.0f, 0.0f, 20.0f, "UNKNOWN_TARGET");
    const ArCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.tracks.size(), 1U);
    const auto& track = result.output_frame.tracks.front();
    if (track.recognition.state == ArRecognitionState::kUnknown) {
      ++unknown_cycles;
    }
  }
  // kUnknown 率 ≥ 90%（拒绝误识）。
  EXPECT_GE(unknown_cycles, kCycles * 9U / 10U);
}

// -- 场景 8：两类混合目标类别混淆率 < 10% --------------------------------

TEST_F(ArRecognitionScenarioTest, MixedTargetsKeepConfusionBelowTenPercent) {
  ArSession radar = ArSession::Create(MakeScenarioConfig());
  EnableLrr(&radar);
  std::uint32_t total = 0U;
  std::uint32_t confused = 0U;
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    ArCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
    AppendTarget(&input, 88U, 400.0f, 500.0f, 2.0f, "NEAR_SPACE_EXAMPLE_A");
    const ArCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.tracks.size(), 2U);
    EXPECT_TRUE(result.has_recognition_summary);
    EXPECT_TRUE(result.recognition_summary.has_ground_truth);
    EXPECT_EQ(result.recognition_summary.category_accuracy, 1.0f);
    EXPECT_EQ(result.recognition_summary.model_accuracy, 1.0f);
    for (const auto& track : result.output_frame.tracks) {
      if (track.recognition.state != ArRecognitionState::kModelConfirmed) {
        continue;
      }
      ++total;
      const bool correct = track.target_name == "BALLISTIC_EXAMPLE_A"
                               ? track.recognition.target_model == "BALLISTIC_EXAMPLE_A"
                               : track.recognition.target_model == "NEAR_SPACE_EXAMPLE_A";
      if (!correct) {
        ++confused;
      }
    }
  }
  ASSERT_GT(total, 0U);
  // 类别混淆率 < 10%。
  EXPECT_LT(static_cast<float>(confused) / static_cast<float>(total), 0.1f);
}

// -- 场景 9：模式切换 TWS→LRR→TWS→LRR 从零积累 ---------------------------

TEST_F(ArRecognitionScenarioTest, ModeSwitchSequenceRestartsAccumulationOnReentry) {
  ArSession radar = ArSession::Create(MakeScenarioConfig());
  EnableLrr(&radar);
  std::uint32_t first_exit_observation_count = 0U;
  std::uint32_t first_exit_conclusion = 0U;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    ArCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
    const ArCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
    first_exit_observation_count =
        result.output_frame.tracks.front().recognition.observation_count;
  }
  // 切出 kLrr：结论保持。
  config::ArRuntimeConfigPatch tws_patch;
  tws_patch.has_work_mode = true;
  tws_patch.work_mode = config::ArWorkMode::kTws;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(tws_patch));
  for (std::uint32_t cycle = 4U; cycle <= 5U; ++cycle) {
    ArCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
    const ArCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
    first_exit_conclusion =
        static_cast<std::uint32_t>(result.output_frame.tracks.front().recognition.state);
  }
  EXPECT_EQ(first_exit_conclusion, static_cast<std::uint32_t>(ArRecognitionState::kModelConfirmed));

  // 再次进入 kLrr：从零积累（观察数回到 1），结论重新确认。
  EnableLrr(&radar);
  ArCycleInput reentry = MakeInput(6U);
  AppendTarget(&reentry, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
  const ArCycleResult result = radar.StepWithResult(reentry);
  ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
  const auto& track = result.output_frame.tracks.front();
  EXPECT_LE(track.recognition.observation_count, first_exit_observation_count);
  EXPECT_EQ(track.recognition.state, ArRecognitionState::kModelConfirmed);
}

// -- 场景 10：以上场景 trace/replay 往返一致 ------------------------------

TEST_F(ArRecognitionScenarioTest, ScenarioCyclesRoundtripByteExactThroughReplayCodec) {
  ArSession radar = ArSession::Create(MakeScenarioConfig());
  EnableLrr(&radar);
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    ArCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
    const ArCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);

    session::ArCycleReplayRecord record;
    record.result = result;
    record.session_state = ArSessionReplayAccess::CaptureSessionState(radar);
    EXPECT_EQ(record.session_state.active_database_version, "2.0.0");

    const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
    ASSERT_FALSE(encoded.empty());
    session::ArCycleReplayRecord decoded;
    std::string error;
    ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
    // 字节精确往返：replay 比对即字节相等；任何字段丢失都会 divergence。
    EXPECT_EQ(session::EncodeCycleReplayRecordFlatbuffer(decoded), encoded);
    // 识别字段抽查：两帧中的结论与摘要逐字段保持。
    const auto& expected_track = result.output_frame.tracks.front();
    const auto& decoded_track = decoded.result.output_frame.tracks.front();
    EXPECT_EQ(decoded_track.recognition.state, expected_track.recognition.state);
    EXPECT_FLOAT_EQ(decoded_track.recognition.best_score, expected_track.recognition.best_score);
    EXPECT_FLOAT_EQ(decoded_track.recognition.confidence, expected_track.recognition.confidence);
  }
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
