// Copyright 2026. All Rights Reserved.
//
// @file rir_recognition_scenario_test.cpp
// @brief 远程识别效能验证：标注场景 + 混合 + 模式切换（RIR 独立模块版）。
//
// 与 AR 版（ar_recognition_scenario_test.cpp）同源改写：场景物理量级与 SQL 库
// 一致；驱动由 AR 会话改为 RIR 会话（航迹供给直连），识别链路消费效能化
// 观测而非真值。replay 往返用例随 RIR replay codec 落地（阶段 1 步骤 5）。

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "1q/remote_identification_radar/config/RirProfileConstants.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirSqliteTestUtil.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirRecognitionCategory;
using session::RirRecognitionState;
using session::RirSceneTarget;
using session::RirSession;
using session::RirTrackFeedEntry;
using session::RirTrackFeedStatus;

constexpr const char* kScenarioDatabaseSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','rir-recognition-scenario-baseline'),
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

class RirRecognitionScenarioTest : public ::testing::Test {
 protected:
  void SetUp() override {
    database_path_ =
        WriteTempSqlite("rir_recognition_scenario.db",
                        std::string(kRecognitionSchemaSql) + kScenarioDatabaseSql);
    ASSERT_FALSE(database_path_.empty());
  }

  config::RirSessionConfig MakeScenarioConfig() const {
    config::RirSessionConfig cfg;
    cfg.mission.work_mode = config::RirWorkMode::kIdentify;
    cfg.policy.recognition.enabled = true;
    cfg.policy.recognition.database_path = database_path_;
    cfg.policy.recognition.min_confirmed_hits = 1U;
    cfg.policy.recognition.min_observation_count = 1U;
    cfg.policy.recognition.acceptance_score = 0.6f;
    cfg.policy.recognition.minimum_margin = 0.05f;
    cfg.policy.recognition.result_hold_sec = 1.0f;
    return cfg;
  }

  /** @brief 构造标注目标：速度/高度/视角 RCS 真值 + 真值名 + 已确认航迹供给。 */
  void AppendTarget(RirCycleInput* input, std::uint64_t target_id, float speed_mps,
                    float altitude_offset_m, float rcs_dbsm, const char* truth_name) {
    RirSceneTarget target;
    target.external_target_id = target_id;
    target.position_x = 5000.0f + speed_mps * static_cast<float>(input->input_cycle_index - 1U) *
                                     static_cast<float>(input->dt_sec);
    target.position_z = altitude_offset_m;
    target.range_m = 5000.0f + speed_mps * static_cast<float>(input->input_cycle_index - 1U) *
                                   static_cast<float>(input->dt_sec);
    target.rcs = 5.0f;
    // 视角网格（方位 ±5°，俯仰 5°~30°），RCS 恒为模板值。
    for (float az = -5.0f; az <= 5.0f; az += 5.0f) {
      for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
        session::RirAspectRcsSample aspect;
        aspect.aspect_az_deg = az;
        aspect.aspect_el_deg = el;
        aspect.rcs_dbsm = rcs_dbsm;
        target.aspect_rcs_samples.push_back(aspect);
      }
    }
    input->scene_targets.push_back(target);

    // 已确认航迹供给（运动特征直接来源于航迹）。
    RirTrackFeedEntry track;
    track.association_key = target_id;
    track.external_target_id = target_id;
    track.target_name = truth_name;
    track.status = RirTrackFeedStatus::kConfirmed;
    track.hit_count = 1U;
    track.speed = speed_mps;
    track.velocity_x = speed_mps;
    track.acceleration = 0.0f;
    track.position_z = altitude_offset_m;
    track.estimation_uncertainty_trace = 1000.0f;
    input->track_feed.push_back(track);
  }

  RirCycleInput MakeInput(std::uint32_t cycle) {
    RirCycleInput input;
    input.input_cycle_index = cycle;
    input.batch_id = 1U;
    input.dt_sec = 0.5;
    input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
    input.platform_altitude_m = 1000.0f;
    return input;
  }

  std::string database_path_{};
};

// -- 场景 1/2：弹道与临近空间目标达成率与型号正确率（有真值） ------------

TEST_F(RirRecognitionScenarioTest, BallisticScenarioReachesConfirmationWithHighAccuracy) {
  RirSession radar = RirSession::Create(MakeScenarioConfig());
  std::uint32_t confirmed_cycles = 0U;
  std::uint32_t correct_cycles = 0U;
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    RirCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
    const RirCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.recognition_outputs.size(), 1U);
    const auto& output = result.output_frame.recognition_outputs.front();
    if (output.result.state == RirRecognitionState::kCategoryConfirmed ||
        output.result.state == RirRecognitionState::kModelConfirmed) {
      ++confirmed_cycles;
      if (output.result.target_model == "BALLISTIC_EXAMPLE_A") {
        ++correct_cycles;
      }
    }
  }
  // 达成率 ≥ 80%，型号正确率 ≥ 70%。
  EXPECT_GE(confirmed_cycles, kCycles * 8U / 10U);
  EXPECT_GE(correct_cycles, confirmed_cycles * 7U / 10U);
}

TEST_F(RirRecognitionScenarioTest, NearSpaceScenarioReachesConfirmationWithHighAccuracy) {
  RirSession radar = RirSession::Create(MakeScenarioConfig());
  std::uint32_t confirmed_cycles = 0U;
  std::uint32_t correct_cycles = 0U;
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    RirCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 88U, 400.0f, 500.0f, 2.0f, "NEAR_SPACE_EXAMPLE_A");
    const RirCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.recognition_outputs.size(), 1U);
    const auto& output = result.output_frame.recognition_outputs.front();
    if (output.result.state == RirRecognitionState::kCategoryConfirmed ||
        output.result.state == RirRecognitionState::kModelConfirmed) {
      ++confirmed_cycles;
      if (output.result.target_model == "NEAR_SPACE_EXAMPLE_A") {
        ++correct_cycles;
      }
    }
  }
  EXPECT_GE(confirmed_cycles, kCycles * 8U / 10U);
  EXPECT_GE(correct_cycles, confirmed_cycles * 7U / 10U);
}

// -- 场景 3：未知目标拒绝误识 --------------------------------------------

TEST_F(RirRecognitionScenarioTest, UnknownTargetRejectsWithHighUnknownRate) {
  RirSession radar = RirSession::Create(MakeScenarioConfig());
  std::uint32_t unknown_cycles = 0U;
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    RirCycleInput input = MakeInput(cycle);
    // 特征偏离所有模板：RCS +20 dBsm、速度 20 m/s、高度 1000 m。
    AppendTarget(&input, 99U, 20.0f, 0.0f, 20.0f, "UNKNOWN_TARGET");
    const RirCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.recognition_outputs.size(), 1U);
    const auto& output = result.output_frame.recognition_outputs.front();
    if (output.result.state == RirRecognitionState::kUnknown) {
      ++unknown_cycles;
    }
  }
  // kUnknown 率 ≥ 90%（拒绝误识）。
  EXPECT_GE(unknown_cycles, kCycles * 9U / 10U);
}

// -- 场景 8：两类混合目标类别混淆率 < 10% --------------------------------

TEST_F(RirRecognitionScenarioTest, MixedTargetsKeepConfusionBelowTenPercent) {
  RirSession radar = RirSession::Create(MakeScenarioConfig());
  std::uint32_t total = 0U;
  std::uint32_t confused = 0U;
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    RirCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
    AppendTarget(&input, 88U, 400.0f, 500.0f, 2.0f, "NEAR_SPACE_EXAMPLE_A");
    const RirCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.recognition_outputs.size(), 2U);
    EXPECT_TRUE(result.has_recognition_summary);
    EXPECT_TRUE(result.recognition_summary.has_ground_truth);
    EXPECT_EQ(result.recognition_summary.category_accuracy, 1.0f);
    EXPECT_EQ(result.recognition_summary.model_accuracy, 1.0f);
    for (const auto& output : result.output_frame.recognition_outputs) {
      if (output.result.state != RirRecognitionState::kModelConfirmed) {
        continue;
      }
      ++total;
      const bool correct = output.result.target_model == "BALLISTIC_EXAMPLE_A" ||
                           output.result.target_model == "NEAR_SPACE_EXAMPLE_A";
      if (!correct) {
        ++confused;
      }
    }
  }
  ASSERT_GT(total, 0U);
  // 类别混淆率 < 10%。
  EXPECT_LT(static_cast<float>(confused) / static_cast<float>(total), 0.1f);
}

// -- 场景 9：模式切换 Identify→Stby→Identify 从零积累 --------------------

TEST_F(RirRecognitionScenarioTest, ModeSwitchSequenceRestartsAccumulationOnReentry) {
  RirSession radar = RirSession::Create(MakeScenarioConfig());
  std::uint32_t first_exit_observation_count = 0U;
  std::uint32_t first_exit_conclusion = 0U;
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    RirCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
    const RirCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
    first_exit_observation_count =
        result.output_frame.recognition_outputs.front().result.observation_count;
  }
  // 切出 kIdentify：结论保持。
  config::RirRuntimeConfigPatch stby_patch;
  stby_patch.has_work_mode = true;
  stby_patch.work_mode = config::RirWorkMode::kStby;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(stby_patch));
  for (std::uint32_t cycle = 4U; cycle <= 5U; ++cycle) {
    RirCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
    const RirCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
    first_exit_conclusion =
        static_cast<std::uint32_t>(result.output_frame.recognition_outputs.front().result.state);
  }
  EXPECT_EQ(first_exit_conclusion,
            static_cast<std::uint32_t>(RirRecognitionState::kModelConfirmed));

  // 再次进入 kIdentify：从零积累（观察数回落），结论重新确认。
  config::RirRuntimeConfigPatch identify_patch;
  identify_patch.has_work_mode = true;
  identify_patch.work_mode = config::RirWorkMode::kIdentify;
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(identify_patch));
  RirCycleInput reentry = MakeInput(6U);
  AppendTarget(&reentry, 77U, 100.0f, 2000.0f, -3.0f, "BALLISTIC_EXAMPLE_A");
  const RirCycleResult result = radar.StepWithResult(reentry);
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  const auto& output = result.output_frame.recognition_outputs.front();
  EXPECT_LE(output.result.observation_count, first_exit_observation_count);
  EXPECT_EQ(output.result.state, RirRecognitionState::kModelConfirmed);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
