// Copyright 2026. All Rights Reserved.
//
// @file ar_recognition_us_military_scenario_test.cpp
// @brief 美方常见型号识别效能验证：端到端加载交付库（target_feature_database_v1.1.db）。
//
// 与 ar_recognition_scenario_test.cpp（内联库）不同，本文件直接加载提交入库的
// 示例识别基线（ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH），验证
// 工具 → 权威 DDL → 加载器 → 匹配器 → 会话 全链路对交付物的识别正确性：
// 命中型号/类别、置信度、同类歧义与跨类隔离边界。
// 场景签名取模板均值（速度/绝对高度/视角 RCS），阈值按实测校准（规划文档 Stage C）。

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"
#include "airborne_radar/recognition/RecognitionFeatureDatabase.h"

namespace airborne_radar {
namespace tests {
namespace {

using recognition::RecognitionFeatureDatabase;
using session::ArCycleInput;
using session::ArCycleStatus;
using session::ArCycleResult;
using session::ArRecognitionCategory;
using session::ArRecognitionState;
using session::ArSession;
using session::ArTargetInput;

#ifndef ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH
#error "ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH 未定义（Integration.cmake 注入）"
#endif

struct ScenarioOutcome {
  std::uint32_t confirmed_cycles = 0U;
  std::uint32_t correct_cycles = 0U;
  std::uint32_t category_correct_cycles = 0U;
  float last_confidence = 0.0f;
  float last_best_score = 0.0f;
  float last_runner_up_score = 0.0f;
};

class ArUsMilitaryRecognitionScenarioTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 交付库可加载是全部场景的前提；失败即快速暴露（错误含表/字段上下文）。
    RecognitionFeatureDatabase database;
    std::string error;
    ASSERT_TRUE(RecognitionFeatureDatabase::Load(ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH,
                                                 &database, &error))
        << error;
  }

  config::ArSessionConfig MakeScenarioConfig() const {
    config::ArSessionConfig cfg;
    cfg.policy.detection = config::profiles::kDetectionPriorityDetection;
    cfg.policy.tracking = config::profiles::kFastAssociationTracking;
    cfg.policy.tracking.enable_kalman_filter = true;
    cfg.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
    cfg.policy.recognition.enabled = true;
    cfg.policy.recognition.database_path = ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH;
    cfg.policy.recognition.min_confirmed_hits = 1U;
    cfg.policy.recognition.min_observation_count = 1U;
    cfg.policy.recognition.acceptance_score = 0.6f;
    cfg.policy.recognition.minimum_margin = 0.05f;
    cfg.policy.recognition.result_hold_sec = 1.0f;
    return cfg;
  }

  /** @brief 构造标注目标：速度/高度/视角 RCS 真值 + 真值名（同现有场景测试）。 */
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
    // 经度取 90°：识别管线在平台 ENU 局部系观测，高度 = 平台海拔 +
    // sin(lat)·Δz_ecef + cos(lat)cos(lon)·Δx。cos(90°)=0 使目标沿 x 的
    // 运动不污染高度观测（否则每周期引入 ~55 m 漂移），sin 补偿即精确。
    platform_lla.longitude_deg = 90.0;
    platform_lla.altitude_m = 1000.0;
    EXPECT_TRUE(
        oneq::coordinate::TryLlaToEcef(platform_lla, &input.platform.platform_position_ecef_m));
    return input;
  }

  /** @brief 模板高度 → 场景 z 偏移（目标相对平台 ECEF z 差）。
   *
   * 识别管线 altitude 观测 = 平台海拔 + 目标 ENU 上向分量；平台纬度 31° 时
   * ECEF z 差在 ENU 上向投影 sin(31°)。为使观测高度等于模板高度，偏移需
   * 除以 sin(31°) 补偿（管线帧约定，经度 90° 下无 x 项耦合）。
   */
  float AltitudeOffsetFor(float template_altitude_m) const {
    constexpr float kPlatformLatitudeSin = 0.515038f;  // sin(31°)
    return (template_altitude_m - 1000.0f) / kPlatformLatitudeSin;
  }

  void EnableLrr(ArSession* radar) {
    config::ArRuntimeConfigPatch patch;
    patch.has_work_mode = true;
    patch.work_mode = config::ArWorkMode::kLrr;
    ASSERT_TRUE(radar->TryApplyRuntimeConfig(patch));
  }

  /** @brief 跑单目标 5 周期场景并汇总确认率/正确率/末周期分数。 */
  ScenarioOutcome RunSingleTargetScenario(float speed_mps, float altitude_offset_m,
                                          float rcs_dbsm, const char* truth_name,
                                          ArRecognitionCategory expected_category) {
    ArSession radar = ArSession::Create(MakeScenarioConfig());
    EnableLrr(&radar);
    ScenarioOutcome outcome;
    const std::uint32_t kCycles = 5U;
    for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
      ArCycleInput input = MakeInput(cycle);
      AppendTarget(&input, 77U, speed_mps, altitude_offset_m, rcs_dbsm, truth_name);
      const ArCycleResult result = radar.StepWithResult(input);
      EXPECT_EQ(result.status, ArCycleStatus::kCompleted);
      EXPECT_EQ(result.output_frame.tracks.size(), 1U);
      const auto& track = result.output_frame.tracks.front();
      if (track.recognition.state == ArRecognitionState::kCategoryConfirmed ||
          track.recognition.state == ArRecognitionState::kModelConfirmed) {
        ++outcome.confirmed_cycles;
        if (track.recognition.target_model == truth_name) {
          ++outcome.correct_cycles;
        }
        if (track.recognition.target_category == expected_category) {
          ++outcome.category_correct_cycles;
        }
      }
      outcome.last_confidence = track.recognition.confidence;
      outcome.last_best_score = track.recognition.best_score;
      outcome.last_runner_up_score = track.recognition.runner_up_score;
    }
    return outcome;
  }
};

// S1 战斗机识别：F-16C 剖面（速度 250 m/s、绝对高度 10500 m、RCS 0.8 dBsm）。
TEST_F(ArUsMilitaryRecognitionScenarioTest, FighterScenarioRecognizesF16C) {
  const ScenarioOutcome outcome =
      RunSingleTargetScenario(250.0f, AltitudeOffsetFor(10500.0f), 0.8f, "F-16C", ArRecognitionCategory::kFighter);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.15f);
}

// S2 轰炸机识别：B-52H 剖面（240 m/s、10000 m、20 dBsm）。
TEST_F(ArUsMilitaryRecognitionScenarioTest, BomberScenarioRecognizesB52H) {
  const ScenarioOutcome outcome =
      RunSingleTargetScenario(240.0f, AltitudeOffsetFor(10000.0f), 20.0f, "B-52H", ArRecognitionCategory::kBomber);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.15f);
}

// S3 巡航导弹识别：BGM-109 剖面（255 m/s、40 m 掠海、-10 dBsm）。
TEST_F(ArUsMilitaryRecognitionScenarioTest, CruiseMissileScenarioRecognizesBgm109) {
  const ScenarioOutcome outcome = RunSingleTargetScenario(255.0f, AltitudeOffsetFor(40.0f), -10.0f, "BGM-109",
                                                          ArRecognitionCategory::kMissile);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.15f);
}

// S4 无人机识别：MQ-9A 剖面（78 m/s、7600 m、-12 dBsm）。
// S4 无人机识别：MQ-9A 剖面（78 m/s、7600 m、-12 dBsm）。
// 实测（2026-08-04 校准）：best≈0.82、margin≈0.46、conf≈0.33。
TEST_F(ArUsMilitaryRecognitionScenarioTest, UavScenarioRecognizesMq9A) {
  const ScenarioOutcome outcome =
      RunSingleTargetScenario(78.0f, AltitudeOffsetFor(7600.0f), -12.0f, "MQ-9A", ArRecognitionCategory::kUav);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.25f);
}

// S5 同类歧义边界：AGM-86C 剖面（246 m/s、40 m、-5 dBsm）——与 BGM-109 特征接近。
// 期望：型号确认、类别始终正确（歧义不跨类别）；置信度低于清晰分离场景（S7）。
TEST_F(ArUsMilitaryRecognitionScenarioTest, MissileFamilyAmbiguityStaysWithinCategory) {
  const ScenarioOutcome outcome = RunSingleTargetScenario(246.0f, AltitudeOffsetFor(40.0f), -5.0f, "AGM-86C",
                                                          ArRecognitionCategory::kMissile);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_confidence, 0.15f);
  EXPECT_GE(outcome.last_best_score - outcome.last_runner_up_score, 0.05f);
}

// S6 跨类隔离：同帧双目标（战斗机 + 巡航导弹），类别/型号正确率 100%。
TEST_F(ArUsMilitaryRecognitionScenarioTest, MixedFighterAndMissileKeepFullAccuracy) {
  ArSession radar = ArSession::Create(MakeScenarioConfig());
  EnableLrr(&radar);
  const std::uint32_t kCycles = 5U;
  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    ArCycleInput input = MakeInput(cycle);
    AppendTarget(&input, 77U, 250.0f, AltitudeOffsetFor(10500.0f), 0.8f, "F-16C");
    AppendTarget(&input, 88U, 255.0f, AltitudeOffsetFor(40.0f), -10.0f, "BGM-109");
    const ArCycleResult result = radar.StepWithResult(input);
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
    ASSERT_EQ(result.output_frame.tracks.size(), 2U);
    EXPECT_TRUE(result.has_recognition_summary);
    EXPECT_TRUE(result.recognition_summary.has_ground_truth);
    EXPECT_EQ(result.recognition_summary.category_accuracy, 1.0f);
    EXPECT_EQ(result.recognition_summary.model_accuracy, 1.0f);
    for (const auto& track : result.output_frame.tracks) {
      EXPECT_EQ(track.recognition.state, ArRecognitionState::kModelConfirmed);
      const bool correct = track.target_name == "F-16C"
                               ? track.recognition.target_model == "F-16C"
                               : track.recognition.target_model == "BGM-109";
      EXPECT_TRUE(correct);
    }
  }
}

// S7 置信度排序：清晰分离（MQ-9A）> 同类歧义（AGM-86C），相对断言不依赖绝对值。
// 实测（2026-08-04 校准）：conf(MQ-9A)≈0.33、conf(AGM-86C)≈0.20。
TEST_F(ArUsMilitaryRecognitionScenarioTest, ConfidenceClearSeparationExceedsAmbiguity) {
  const ScenarioOutcome clear =
      RunSingleTargetScenario(78.0f, AltitudeOffsetFor(7600.0f), -12.0f, "MQ-9A", ArRecognitionCategory::kUav);
  const ScenarioOutcome ambiguous =
      RunSingleTargetScenario(246.0f, AltitudeOffsetFor(40.0f), -5.0f, "AGM-86C", ArRecognitionCategory::kMissile);
  EXPECT_GT(clear.last_confidence, ambiguous.last_confidence);
  EXPECT_GE(clear.last_confidence - ambiguous.last_confidence, 0.08f);
}

// -- S8 全型号覆盖：交付库 15 个美方型号逐个端到端识别 ----------------------

struct RecognitionTargetSpec {
  const char* model_id;
  float speed_mps;
  float altitude_m;
  float rcs_dbsm;
  ArRecognitionCategory category;
};

class ArUsMilitaryRecognitionParamTest
    : public ArUsMilitaryRecognitionScenarioTest,
      public ::testing::WithParamInterface<RecognitionTargetSpec> {};

TEST_P(ArUsMilitaryRecognitionParamTest, RecognizesDeliverableModel) {
  const RecognitionTargetSpec& spec = GetParam();
  const ScenarioOutcome outcome = RunSingleTargetScenario(
      spec.speed_mps, AltitudeOffsetFor(spec.altitude_m), spec.rcs_dbsm, spec.model_id,
      spec.category);
  EXPECT_GE(outcome.confirmed_cycles, 4U);
  EXPECT_GE(outcome.correct_cycles, outcome.confirmed_cycles * 7U / 10U);
  EXPECT_EQ(outcome.category_correct_cycles, outcome.confirmed_cycles);
  EXPECT_GE(outcome.last_best_score - outcome.last_runner_up_score, 0.05f);
  EXPECT_GE(outcome.last_confidence, 0.12f);
}

INSTANTIATE_TEST_SUITE_P(
    DeliverableModels, ArUsMilitaryRecognitionParamTest,
    ::testing::Values(
        RecognitionTargetSpec{"F-16C", 250.0f, 10500.0f, 0.8f, ArRecognitionCategory::kFighter},
        RecognitionTargetSpec{"F-15E", 265.0f, 12000.0f, 11.8f, ArRecognitionCategory::kFighter},
        RecognitionTargetSpec{"F/A-18E", 250.0f, 10500.0f, -10.0f, ArRecognitionCategory::kFighter},
        RecognitionTargetSpec{"F-22A", 520.0f, 16000.0f, -37.0f, ArRecognitionCategory::kFighter},
        RecognitionTargetSpec{"F-35A", 255.0f, 12000.0f, -27.0f, ArRecognitionCategory::kFighter},
        RecognitionTargetSpec{"B-52H", 240.0f, 10000.0f, 20.0f, ArRecognitionCategory::kBomber},
        RecognitionTargetSpec{"B-1B", 270.0f, 100.0f, 3.8f, ArRecognitionCategory::kBomber},
        RecognitionTargetSpec{"B-2A", 250.0f, 13000.0f, -10.0f, ArRecognitionCategory::kBomber},
        RecognitionTargetSpec{"BGM-109", 255.0f, 40.0f, -10.0f, ArRecognitionCategory::kMissile},
        RecognitionTargetSpec{"AGM-158A", 240.0f, 80.0f, -25.0f, ArRecognitionCategory::kMissile},
        RecognitionTargetSpec{"AGM-86C", 246.0f, 40.0f, -5.0f, ArRecognitionCategory::kMissile},
        RecognitionTargetSpec{"MQ-9A", 78.0f, 7600.0f, -12.0f, ArRecognitionCategory::kUav},
        RecognitionTargetSpec{"RQ-4B", 159.0f, 18000.0f, -5.0f, ArRecognitionCategory::kUav},
        RecognitionTargetSpec{"MQ-4C", 160.0f, 16500.0f, -6.0f, ArRecognitionCategory::kUav},
        RecognitionTargetSpec{"MQ-1C", 60.0f, 4800.0f, -15.0f, ArRecognitionCategory::kUav}));

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
