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
#include "RirCycleInputTestUtil.h"
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
    // 主瓣覆盖门放宽：本文件聚焦识别效能场景，不测波束覆盖门。波束取 200°
    // （半宽 100°）：体积放宽到 el ±85° 后波位表横跨全扇区，而场景目标仰角
    // 跨 −10°~+62°，160° 波束（半宽 80°）盖不住最远角差，目标永不被照到。
    cfg.hardware.antenna.nominal_az_beamwidth_deg = 200.0f;
    cfg.hardware.antenna.nominal_el_beamwidth_deg = 200.0f;
    // 增益同步 +10 dB：宽波束下离轴角差仍带来 ~4 dB 方向图衰减，掠海导弹
    // 剖面（低 RCS + 负仰角）的 SNR 余量会被压破 6 dB 回退门。
    cfg.hardware.antenna.main_beam_gain_db = 45.0f;
    // 波位步长与波束宽度解耦：步长=波束宽度×step_scale，不压小 step_scale
    // 的话 200° 波束会让波位表退化成 4 个角点（扫描覆盖名存实亡）。
    cfg.mission.step_scale = 0.02f;
    cfg.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
    cfg.policy.recognition.enabled = true;
    cfg.policy.recognition.database_path = ONEQ_RIR_EXAMPLE_DATABASE_PATH;
    cfg.policy.recognition.min_confirmed_hits = 1U;
    cfg.policy.recognition.min_observation_count = 1U;
    cfg.policy.recognition.acceptance_score = 0.6f;
    cfg.policy.recognition.minimum_margin = 0.05f;
    cfg.policy.recognition.result_hold_sec = 1.0f;
    // 场景几何为近距高仰角（如战斗机 5 km 斜距 62° 仰角），超出默认可扫描体积
    // （el ±30°）会被体积裁剪正当排除（2026-08-22 起语义）。本测试聚焦识别链路，
    // 显式放宽体积；体积裁剪语义由库单测与场景级测试覆盖。
    cfg.orientation.el_min_deg = -85.0f;
    cfg.orientation.el_max_deg = 85.0f;
    // 方位体积同步收窄：波位表随体积生成，全向体积会让波位 0 远离目标
    // （方位差 180° 超出放宽后的主瓣覆盖门），目标永不被照到。
    cfg.orientation.az_min_deg = -60.0f;
    cfg.orientation.az_max_deg = 60.0f;
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
    input.dt_sec = 0.5;
    input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
    SetDefaultTestPlatformEcef(&input);
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
      // 航迹缺失时提前返回：继续读 front() 是空引用崩溃（本函数非 void，不能用
      // ASSERT_*）；计数保持 0 让外层期望如实失败。
      if (result.output_frame.recognition_outputs.size() != 1U) {
        ADD_FAILURE() << "cycle " << cycle << ": recognition_outputs.size()="
                      << result.output_frame.recognition_outputs.size();
        return outcome;
      }
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

// S4 无人机剖面：MQ-9A（78 m/s、7600 m、-12 dBsm）。2026-08-22 甲方裁定移除
// 无人机识别——大类按 kUnknown 断言；型号匹配与置信度不受影响。
// 实测（2026-08-04 校准）：best≈0.82、margin≈0.46、conf≈0.33。
TEST_F(RirUsMilitaryRecognitionScenarioTest, UavScenarioMapsToUnknown) {
  const ScenarioOutcome outcome = RunSingleTargetScenario(78.0f, AltitudeOffsetFor(7600.0f), -12.0f,
                                                          "MQ-9A", RirRecognitionCategory::kUnknown);
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
                                                        "MQ-9A", RirRecognitionCategory::kUnknown);
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
        RecognitionTargetSpec{"MQ-9A", 78.0f, 7600.0f, -12.0f, RirRecognitionCategory::kUnknown},
        RecognitionTargetSpec{"RQ-4B", 159.0f, 18000.0f, -5.0f, RirRecognitionCategory::kUnknown},
        RecognitionTargetSpec{"MQ-4C", 160.0f, 16500.0f, -6.0f, RirRecognitionCategory::kUnknown},
        RecognitionTargetSpec{"MQ-1C", 60.0f, 4800.0f, -15.0f, RirRecognitionCategory::kUnknown}));

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
