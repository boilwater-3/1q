// Copyright 2026. All Rights Reserved.
//
// @file rir_recognition_us_military_scenario_test.cpp
// @brief 美方公开型号识别效能验证（RIR 独立模块版，交付库直载）。
//
// 与 AR 版（ar_recognition_us_military_scenario_test.cpp）同源改写：模型参数与
// 断言阈值一致；驱动改为 RIR 会话。帧约定差异：AR 版经 ECEF→ENU 投影（含
// sin(lat) 补偿），RIR 版航迹供给直接携带雷达局部 ENU 上向分量（调用方职责），
// 故 position_z = 模板高度 − 平台海拔，无需三角函数补偿。

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirSqliteTestUtil.h"
#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using recognition::RirFeatureDatabase;
using session::RirCycleInput;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirRecognitionCategory;
using session::RirRecognitionState;
using session::RirSceneTarget;
using session::RirSession;

#ifndef ONEQ_RIR_EXAMPLE_DATABASE_PATH
#error "ONEQ_RIR_EXAMPLE_DATABASE_PATH 未定义（Integration.cmake 注入）"
#endif

struct ScenarioOutcome {
  std::uint32_t confirmed_cycles = 0U;
  std::uint32_t correct_cycles = 0U;
  std::uint32_t category_correct_cycles = 0U;
  float last_confidence = 0.0f;
  float last_best_score = 0.0f;
  float last_runner_up_score = 0.0f;
};

class RirUsMilitaryRecognitionScenarioTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 交付库可加载是全部场景的前提；失败即快速暴露（错误含表/字段上下文）。
    RirFeatureDatabase database;
    std::string error;
    ASSERT_TRUE(RirFeatureDatabase::Load(ONEQ_RIR_EXAMPLE_DATABASE_PATH, &database, &error))
        << error;
  }

  config::RirSessionConfig MakeScenarioConfig() const {
    config::RirSessionConfig cfg;
    cfg.mission.work_mode = config::RirWorkMode::kIdentify;
    cfg.policy.lifecycle.confirm_hits = 1U;
    cfg.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
    cfg.policy.recognition.enabled = true;
    cfg.policy.recognition.database_path = ONEQ_RIR_EXAMPLE_DATABASE_PATH;
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
    target.target_name = truth_name;
    target.velocity_x = speed_mps;
    target.target_swerling_type = session::RirSwerlingType::kSwerling0;
    input->scene_targets.push_back(target);
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

  /** @brief 模板绝对高度 → 雷达局部 ENU 上向分量（平台海拔 1000 m）。 */
  float AltitudeOffsetFor(float template_altitude_m) const { return template_altitude_m - 1000.0f; }

  /** @brief 跑单目标 5 周期场景并汇总确认率/正确率/末周期分数。 */
  ScenarioOutcome RunSingleTargetScenario(float speed_mps, float altitude_offset_m, float rcs_dbsm,
                                          const char* truth_name,
                                          RirRecognitionCategory expected_category) {
    RirSession radar = RirSession::Create(MakeScenarioConfig());
    ScenarioOutcome outcome;
    const std::uint32_t kCycles = 5U;
    for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
      RirCycleInput input = MakeInput(cycle);
      AppendTarget(&input, 77U, speed_mps, altitude_offset_m, rcs_dbsm, truth_name);
      const RirCycleResult result = radar.StepWithResult(input);
      EXPECT_EQ(result.status, RirCycleStatus::kCompleted);
      EXPECT_EQ(result.output_frame.recognition_outputs.size(), 1U);
      const auto& output = result.output_frame.recognition_outputs.front();
      if (output.result.state == RirRecognitionState::kCategoryConfirmed ||
          output.result.state == RirRecognitionState::kModelConfirmed) {
        ++outcome.confirmed_cycles;
        if (output.result.target_model == truth_name) {
          ++outcome.correct_cycles;
        }
        if (output.result.target_category == expected_category) {
          ++outcome.category_correct_cycles;
        }
      }
      outcome.last_confidence = output.result.confidence;
      outcome.last_best_score = output.result.best_score;
      outcome.last_runner_up_score = output.result.runner_up_score;
    }
    return outcome;
  }
};

// S1 战斗机识别：F-16C 剖面（速度 250 m/s、绝对高度 10500 m、RCS 0.8 dBsm）。
TEST_F(RirUsMilitaryRecognitionScenarioTest, FighterScenarioRecognizesF16C) {
  const ScenarioOutcome outcome = RunSingleTargetScenario(
      250.0f, AltitudeOffsetFor(10500.0f), 0.8f, "F-16C", RirRecognitionCategory::kFighter);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.15f);
}

// S2 轰炸机识别：B-52H 剖面（240 m/s、10000 m、20 dBsm）。
TEST_F(RirUsMilitaryRecognitionScenarioTest, BomberScenarioRecognizesB52H) {
  const ScenarioOutcome outcome = RunSingleTargetScenario(
      240.0f, AltitudeOffsetFor(10000.0f), 20.0f, "B-52H", RirRecognitionCategory::kBomber);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.15f);
}

// S3 巡航导弹识别：BGM-109 剖面（255 m/s、40 m 掠海、-10 dBsm）。
TEST_F(RirUsMilitaryRecognitionScenarioTest, CruiseMissileScenarioRecognizesBgm109) {
  const ScenarioOutcome outcome = RunSingleTargetScenario(
      255.0f, AltitudeOffsetFor(40.0f), -10.0f, "BGM-109", RirRecognitionCategory::kMissile);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.15f);
}

// S4 无人机识别：MQ-9A 剖面（78 m/s、7600 m、-12 dBsm）。
// 实测（2026-08-04 校准）：best≈0.82、margin≈0.46、conf≈0.33。
TEST_F(RirUsMilitaryRecognitionScenarioTest, UavScenarioRecognizesMq9A) {
  const ScenarioOutcome outcome = RunSingleTargetScenario(78.0f, AltitudeOffsetFor(7600.0f), -12.0f,
                                                          "MQ-9A", RirRecognitionCategory::kUav);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.25f);
}

// S5 同类歧义边界：AGM-86C 剖面（246 m/s、40 m、-5 dBsm）——与 BGM-109 特征接近。
// 期望：型号确认、类别始终正确（歧义不跨类别）；置信度低于清晰分离场景（S7）。
TEST_F(RirUsMilitaryRecognitionScenarioTest, MissileFamilyAmbiguityStaysWithinCategory) {
  const ScenarioOutcome outcome = RunSingleTargetScenario(
      246.0f, AltitudeOffsetFor(40.0f), -5.0f, "AGM-86C", RirRecognitionCategory::kMissile);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.15f);
  EXPECT_GE(outcome.last_best_score - outcome.last_runner_up_score, 0.05f);
}

// S6 跨类隔离：同帧双目标（战斗机 + 巡航导弹），类别/型号正确率 100%。
TEST_F(RirUsMilitaryRecognitionScenarioTest, MixedFighterAndMissileKeepFullAccuracy) {
  RirSession radar = RirSession::Create(MakeScenarioConfig());
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    RirCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 250.0f, AltitudeOffsetFor(10500.0f), 0.8f, "F-16C");
    AppendTarget(&input, 88U, 255.0f, AltitudeOffsetFor(40.0f), -10.0f, "BGM-109");
    const RirCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.recognition_outputs.size(), 2U);
    EXPECT_TRUE(result.has_recognition_summary);
    EXPECT_TRUE(result.recognition_summary.has_ground_truth);
    EXPECT_EQ(result.recognition_summary.category_accuracy, 1.0f);
    EXPECT_EQ(result.recognition_summary.model_accuracy, 1.0f);
    for (const auto& output : result.output_frame.recognition_outputs) {
      EXPECT_EQ(output.result.state, RirRecognitionState::kModelConfirmed);
      const bool correct =
          output.result.target_model == "F-16C" || output.result.target_model == "BGM-109";
      EXPECT_TRUE(correct);
    }
  }
}

// S7 置信度排序：清晰分离（MQ-9A）> 同类歧义（AGM-86C），相对断言不依赖绝对值。
// 实测（2026-08-04 校准）：conf(MQ-9A)≈0.33、conf(AGM-86C)≈0.20。
TEST_F(RirUsMilitaryRecognitionScenarioTest, ConfidenceClearSeparationExceedsAmbiguity) {
  const ScenarioOutcome clear = RunSingleTargetScenario(78.0f, AltitudeOffsetFor(7600.0f), -12.0f,
                                                        "MQ-9A", RirRecognitionCategory::kUav);
  const ScenarioOutcome ambiguous = RunSingleTargetScenario(
      246.0f, AltitudeOffsetFor(40.0f), -5.0f, "AGM-86C", RirRecognitionCategory::kMissile);
  EXPECT_GT(clear.last_confidence, ambiguous.last_confidence);
  EXPECT_GE(clear.last_confidence - ambiguous.last_confidence, 0.08f);
}

// -- S8 全型号覆盖：交付库 15 个美方型号逐个端到端识别 ----------------------

struct RecognitionTargetSpec {
  const char* model_id;
  float speed_mps;
  float altitude_m;
  float rcs_dbsm;
  RirRecognitionCategory category;
};

class RirUsMilitaryRecognitionParamTest
    : public RirUsMilitaryRecognitionScenarioTest,
      public ::testing::WithParamInterface<RecognitionTargetSpec> {};

TEST_P(RirUsMilitaryRecognitionParamTest, RecognizesDeliverableModel) {
  const RecognitionTargetSpec& spec = GetParam();
  const ScenarioOutcome outcome =
      RunSingleTargetScenario(spec.speed_mps, AltitudeOffsetFor(spec.altitude_m), spec.rcs_dbsm,
                              spec.model_id, spec.category);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_best_score - outcome.last_runner_up_score, 0.05f);
  EXPECT_GE(outcome.last_confidence, 0.12f);
}

INSTANTIATE_TEST_SUITE_P(
    DeliverableModels, RirUsMilitaryRecognitionParamTest,
    ::testing::Values(
        RecognitionTargetSpec{"F-16C", 250.0f, 10500.0f, 0.8f, RirRecognitionCategory::kFighter},
        RecognitionTargetSpec{"F-15E", 265.0f, 12000.0f, 11.8f, RirRecognitionCategory::kFighter},
        RecognitionTargetSpec{"F/A-18E", 250.0f, 10500.0f, -10.0f,
                              RirRecognitionCategory::kFighter},
        RecognitionTargetSpec{"F-22A", 520.0f, 16000.0f, -37.0f, RirRecognitionCategory::kFighter},
        RecognitionTargetSpec{"F-35A", 255.0f, 12000.0f, -27.0f, RirRecognitionCategory::kFighter},
        RecognitionTargetSpec{"B-52H", 240.0f, 10000.0f, 20.0f, RirRecognitionCategory::kBomber},
        RecognitionTargetSpec{"B-1B", 270.0f, 100.0f, 3.8f, RirRecognitionCategory::kBomber},
        RecognitionTargetSpec{"B-2A", 250.0f, 13000.0f, -10.0f, RirRecognitionCategory::kBomber},
        RecognitionTargetSpec{"BGM-109", 255.0f, 40.0f, -10.0f, RirRecognitionCategory::kMissile},
        RecognitionTargetSpec{"AGM-158A", 240.0f, 80.0f, -25.0f, RirRecognitionCategory::kMissile},
        RecognitionTargetSpec{"AGM-86C", 246.0f, 40.0f, -5.0f, RirRecognitionCategory::kMissile},
        RecognitionTargetSpec{"MQ-9A", 78.0f, 7600.0f, -12.0f, RirRecognitionCategory::kUav},
        RecognitionTargetSpec{"RQ-4B", 159.0f, 18000.0f, -5.0f, RirRecognitionCategory::kUav},
        RecognitionTargetSpec{"MQ-4C", 160.0f, 16500.0f, -6.0f, RirRecognitionCategory::kUav},
        RecognitionTargetSpec{"MQ-1C", 60.0f, 4800.0f, -15.0f, RirRecognitionCategory::kUav}));

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
