// Copyright 2026. All Rights Reserved.
//
// @file threat_evaluator_test.cpp
// @brief 验证归一化加权和威胁评估器：归一化边界、加权和、等级映射、钳制与确定性。
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "1q/threat_assessment/threat_assessment.hpp"

namespace threat_assessment {
namespace {

constexpr float kEps = 1e-6f;

ThreatEvaluationInput MakeInput(std::uint64_t key) {
  ThreatEvaluationInput input;
  input.key = key;
  return input;
}

// 全部属性取"归一化中点 0.5"的输入（默认配置下总分应为 0.5）：
// range=110km → (200-110)/180=0.5；speed=275 → (275-50)/450=0.5；
// accel=25 → 25/50=0.5；rcs=5.25 → (5.25-0.5)/9.5=0.5；prob/conf 直通 0.5。
ThreatEvaluationInput MakeMidpointInput(std::uint64_t key) {
  ThreatEvaluationInput input;
  input.key = key;
  input.range_m = 110000.0f;
  input.speed = 275.0f;
  input.acceleration = 25.0f;
  input.rcs = 5.25f;
  input.target_probability = 0.5f;
  input.fusion_confidence = 0.5f;
  return input;
}

TEST(ThreatEvaluatorTest, EmptyInputYieldsEmptyOutput) {
  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  EXPECT_TRUE(evaluator.Evaluate({}).empty());
}

TEST(ThreatEvaluatorTest, KeyPassthroughAndInputOrder) {
  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  const auto results = evaluator.Evaluate({MakeInput(7U), MakeInput(3U), MakeInput(9U)});

  ASSERT_EQ(results.size(), 3U);
  EXPECT_EQ(results[0].key, 7U);
  EXPECT_EQ(results[1].key, 3U);
  EXPECT_EQ(results[2].key, 9U);
}

TEST(ThreatEvaluatorTest, AllLowestAttributesScoreZeroLowLevel) {
  // 注意 range_m=0 是"零距离 = 满分"的合法语义，最低属性须显式给远距离。
  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = 200000.0f;  // ≥ far 断点 → 距离归一化 0
  input.speed = 0.0f;
  input.acceleration = 0.0f;
  input.rcs = 0.0f;
  input.target_probability = 0.0f;
  input.fusion_confidence = 0.0f;

  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  const auto results = evaluator.Evaluate({input});

  ASSERT_EQ(results.size(), 1U);
  EXPECT_NEAR(results[0].threat_score, 0.0f, kEps);
  EXPECT_EQ(results[0].level, ThreatLevel::kLow);
}

TEST(ThreatEvaluatorTest, AllHighestAttributesScoreOneHighLevel) {
  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = 0.0f;
  input.speed = 500.0f;
  input.acceleration = 50.0f;
  input.rcs = 10.0f;
  input.target_probability = 1.0f;
  input.fusion_confidence = 2.0f;

  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  const auto results = evaluator.Evaluate({input});

  ASSERT_EQ(results.size(), 1U);
  EXPECT_NEAR(results[0].threat_score, 1.0f, kEps);
  EXPECT_EQ(results[0].level, ThreatLevel::kHigh);
}

TEST(ThreatEvaluatorTest, MidpointAttributesScoreHalfMediumLevel) {
  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  const auto results = evaluator.Evaluate({MakeMidpointInput(1U)});

  ASSERT_EQ(results.size(), 1U);
  // 六属性均取 0.5，默认权重和为 1 → 总分 = 0.5。
  EXPECT_NEAR(results[0].threat_score, 0.5f, kEps);
  EXPECT_EQ(results[0].level, ThreatLevel::kMedium);
}

TEST(ThreatEvaluatorTest, ContributionSumEqualsThreatScore) {
  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  const auto results = evaluator.Evaluate(
      {MakeMidpointInput(1U), MakeInput(2U)});

  for (const ThreatResult& result : results) {
    const float sum = result.contributions.range + result.contributions.speed +
                      result.contributions.acceleration +
                      result.contributions.rcs +
                      result.contributions.target_probability +
                      result.contributions.fusion_confidence;
    EXPECT_NEAR(sum, result.threat_score, kEps);
  }
}

TEST(ThreatEvaluatorTest, PerAttributeNormalizationBoundaries) {
  // 单属性贡献验证：只点亮一个属性，其余为 0，贡献应等于 权重 × 归一化值。
  const ThreatEvaluatorConfig config;
  const ThreatEvaluator evaluator(config);

  // 距离：≤ near → 1；≥ far → 0；中点 → 0.5。
  {
    ThreatEvaluationInput input = MakeInput(1U);
    input.range_m = config.range_near_m;
    auto result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.range, config.weight_range, kEps);
    EXPECT_NEAR(result.threat_score, config.weight_range, kEps);

    input.range_m = config.range_far_m;
    result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.range, 0.0f, kEps);

    input.range_m = 110000.0f;  // 距 20km~200km 的中点
    result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.range, 0.5f * config.weight_range, kEps);
  }

  // 速度：≤ min → 0；≥ max → 1；中点 → 0.5。
  {
    ThreatEvaluationInput input = MakeInput(1U);
    input.speed = config.speed_min_mps;
    auto result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.speed, 0.0f, kEps);

    input.speed = config.speed_max_mps;
    result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.speed, config.weight_speed, kEps);

    input.speed = 275.0f;  // 50~500 的中点
    result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.speed, 0.5f * config.weight_speed, kEps);
  }

  // 加速度：0 → 0；≥ max → 1；中点 → 0.5。
  {
    ThreatEvaluationInput input = MakeInput(1U);
    input.acceleration = config.acceleration_max_mps2;
    auto result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.acceleration,
                config.weight_acceleration, kEps);

    input.acceleration = 25.0f;
    result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.acceleration,
                0.5f * config.weight_acceleration, kEps);
  }

  // RCS：≤ min → 0；≥ max → 1；中点 → 0.5。
  {
    ThreatEvaluationInput input = MakeInput(1U);
    input.rcs = config.rcs_min_sqm;
    auto result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.rcs, 0.0f, kEps);

    input.rcs = config.rcs_max_sqm;
    result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.rcs, config.weight_rcs, kEps);

    input.rcs = 5.25f;  // 0.5~10 的中点
    result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.rcs, 0.5f * config.weight_rcs, kEps);
  }

  // 类型概率：直通。
  {
    ThreatEvaluationInput input = MakeInput(1U);
    input.target_probability = 0.4f;
    auto result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.target_probability,
                0.4f * config.weight_target_probability, kEps);
  }

  // 融合置信度：钳制到 [0,1]（可 >1 的冻结公式输入）。
  {
    ThreatEvaluationInput input = MakeInput(1U);
    input.fusion_confidence = 5.0f;
    auto result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.fusion_confidence,
                config.weight_fusion_confidence, kEps);

    input.fusion_confidence = 0.25f;
    result = evaluator.Evaluate({input})[0];
    EXPECT_NEAR(result.contributions.fusion_confidence,
                0.25f * config.weight_fusion_confidence, kEps);
  }
}

TEST(ThreatEvaluatorTest, ConfidenceClampedAboveOne) {
  // 融合置信度 >1（fusion 冻结公式允许）必须钳制为 1 而非破坏威胁分上界。
  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = 200000.0f;  // 距离贡献归零，孤立置信度属性
  input.fusion_confidence = 100.0f;

  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  const auto result = evaluator.Evaluate({input})[0];
  EXPECT_NEAR(result.contributions.fusion_confidence,
              ThreatEvaluatorConfig{}.weight_fusion_confidence, kEps);
  EXPECT_NEAR(result.threat_score, 0.10f, kEps);
  EXPECT_EQ(result.level, ThreatLevel::kLow);
}

TEST(ThreatEvaluatorTest, NonUnitWeightsAreNormalized) {
  // 权重按相对值解释：全部权重相等（和 ≠ 1）→ 均分后每属性权重 = 1/6。
  ThreatEvaluatorConfig config;
  config.weight_range = 6.0f;
  config.weight_speed = 6.0f;
  config.weight_acceleration = 6.0f;
  config.weight_rcs = 6.0f;
  config.weight_target_probability = 6.0f;
  config.weight_fusion_confidence = 6.0f;

  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = 200000.0f;  // 距离贡献归零，孤立 RCS 属性
  input.rcs = 5.25f;  // 归一化 0.5

  const ThreatEvaluator evaluator(config);
  const auto result = evaluator.Evaluate({input})[0];
  EXPECT_NEAR(result.contributions.rcs, 0.5f / 6.0f, kEps);
  EXPECT_NEAR(result.threat_score, 0.5f / 6.0f, kEps);
}

TEST(ThreatEvaluatorTest, ZeroWeightsDegradeToUniform) {
  // 全零权重配置属配置错误：防呆退化为均分，避免威胁分恒零或除零。
  ThreatEvaluatorConfig config;
  config.weight_range = 0.0f;
  config.weight_speed = 0.0f;
  config.weight_acceleration = 0.0f;
  config.weight_rcs = 0.0f;
  config.weight_target_probability = 0.0f;
  config.weight_fusion_confidence = 0.0f;

  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = 200000.0f;  // 距离贡献归零，孤立速度属性
  input.speed = config.speed_max_mps;

  const ThreatEvaluator evaluator(config);
  const auto result = evaluator.Evaluate({input})[0];
  EXPECT_NEAR(result.contributions.speed, 1.0f / 6.0f, kEps);
  EXPECT_NEAR(result.threat_score, 1.0f / 6.0f, kEps);
}

TEST(ThreatEvaluatorTest, NegativeAndNaNValuesTreatAsMissing) {
  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = std::nanf("");
  input.speed = -100.0f;
  input.acceleration = std::numeric_limits<float>::infinity();  // 超界 → 归一化 1
  input.rcs = -1.0f;
  input.target_probability = 1.5f;  // 超界 → 钳制 1
  input.fusion_confidence = std::nanf("");

  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  const auto result = evaluator.Evaluate({input})[0];
  EXPECT_NEAR(result.contributions.range, 0.0f, kEps);
  EXPECT_NEAR(result.contributions.speed, 0.0f, kEps);
  EXPECT_NEAR(result.contributions.acceleration,
              ThreatEvaluatorConfig{}.weight_acceleration, kEps);
  EXPECT_NEAR(result.contributions.rcs, 0.0f, kEps);
  EXPECT_NEAR(result.contributions.target_probability,
              ThreatEvaluatorConfig{}.weight_target_probability, kEps);
  EXPECT_NEAR(result.contributions.fusion_confidence, 0.0f, kEps);
}

TEST(ThreatEvaluatorTest, LevelThresholdMapping) {
  // 等级映射使用远离阈值的分数构造（权重归一化引入 ≤1e-6 级浮点误差，
  // 边界精确值断言过脆）；阈值比较语义见 algorithms.md 反直觉点。
  const ThreatEvaluatorConfig config;
  const ThreatEvaluator evaluator(config);

  // 高分 → HIGH：range(f=1) + speed(f=1) + prob(f=1) + conf(f=1)
  //               = 0.30+0.25+0.15+0.10 = 0.80 > 0.70。
  {
    ThreatEvaluationInput input = MakeInput(1U);
    input.range_m = 0.0f;
    input.speed = config.speed_max_mps;
    input.target_probability = 1.0f;
    input.fusion_confidence = 5.0f;
    const auto result = evaluator.Evaluate({input})[0];
    EXPECT_GT(result.threat_score, 0.70f);
    EXPECT_EQ(result.level, ThreatLevel::kHigh);
  }

  // 中分 → MEDIUM：range(f=5/6) + speed(f=0.5) + prob(f=1) + conf(f=1)
  //                = 0.25+0.125+0.15+0.10 = 0.625 ∈ [0.40, 0.70)。
  {
    ThreatEvaluationInput input = MakeInput(1U);
    input.range_m = 50000.0f;  // (200-50)/180 = 5/6
    input.speed = 275.0f;      // (275-50)/450 = 0.5
    input.target_probability = 1.0f;
    input.fusion_confidence = 5.0f;
    const auto result = evaluator.Evaluate({input})[0];
    EXPECT_GE(result.threat_score, 0.40f);
    EXPECT_LT(result.threat_score, 0.70f);
    EXPECT_EQ(result.level, ThreatLevel::kMedium);
  }

  // 低分 → LOW：range(f=2/3) + speed(f=0.2) + prob(f=0.9)
  //            = 0.20+0.05+0.135 = 0.385 < 0.40。
  {
    ThreatEvaluationInput input = MakeInput(1U);
    input.range_m = 80000.0f;  // (200-80)/180 = 2/3
    input.speed = 140.0f;      // (140-50)/450 = 0.2
    input.target_probability = 0.9f;
    const auto result = evaluator.Evaluate({input})[0];
    EXPECT_LT(result.threat_score, 0.40f);
    EXPECT_EQ(result.level, ThreatLevel::kLow);
  }
}

TEST(ThreatEvaluatorTest, DegenerateBreakpointsNeverProduceNaN) {
  // 断点退化（far ≤ near / max ≤ min）时归一化取阶梯语义；断点 NaN 时取
  // 退化端常量——任何配置下威胁分必须有限（algorithms.md 边界承诺）。
  ThreatEvaluatorConfig config;
  config.range_near_m = 200000.0f;  // 倒置：far(20km) ≤ near(200km)
  config.range_far_m = 20000.0f;
  config.speed_min_mps = 500.0f;    // 倒置：max(50) ≤ min(500)
  config.speed_max_mps = 50.0f;
  config.rcs_min_sqm = 10.0f;       // 倒置
  config.rcs_max_sqm = 0.5f;
  config.acceleration_max_mps2 = -1.0f;  // 退化：max ≤ 0

  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = 100000.0f;  // 阶梯区间内
  input.speed = 300.0f;
  input.acceleration = 5.0f;
  input.rcs = 5.0f;
  input.target_probability = 0.5f;
  input.fusion_confidence = 2.0f;

  const ThreatEvaluator evaluator(config);
  const auto result = evaluator.Evaluate({input})[0];
  EXPECT_TRUE(std::isfinite(result.threat_score));
  EXPECT_GE(result.threat_score, 0.0f);
  EXPECT_LE(result.threat_score, 1.0f);
  for (const float contribution :
       {result.contributions.range, result.contributions.speed,
        result.contributions.acceleration, result.contributions.rcs,
        result.contributions.target_probability,
        result.contributions.fusion_confidence}) {
    EXPECT_TRUE(std::isfinite(contribution));
  }
}

TEST(ThreatEvaluatorTest, InvertedThresholdsNeverTriggerMedium) {
  // 阈值倒置（high ≤ medium）时先判 HIGH，MEDIUM 永不触发（文档化配置语义）。
  ThreatEvaluatorConfig config;
  config.high_threshold = 0.30f;
  config.medium_threshold = 0.60f;

  // score = 0.5：≥ high(0.3) → HIGH（MEDIUM 分支不可达）。
  const ThreatEvaluator evaluator(config);
  const auto high = evaluator.Evaluate({MakeMidpointInput(1U)})[0];
  EXPECT_NEAR(high.threat_score, 0.5f, kEps);
  EXPECT_EQ(high.level, ThreatLevel::kHigh);

  // 低分仍 LOW。
  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = 200000.0f;
  const auto low = evaluator.Evaluate({input})[0];
  EXPECT_EQ(low.level, ThreatLevel::kLow);
}

TEST(ThreatEvaluatorTest, InfWeightDegradesToUniform) {
  // 任一权重为 +inf → 权重和非有限 → 均分退化，威胁分不被 NaN 污染。
  ThreatEvaluatorConfig config;
  config.weight_range = std::numeric_limits<float>::infinity();

  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = 0.0f;  // 零距离 → 距离归一化 1

  const ThreatEvaluator evaluator(config);
  const auto result = evaluator.Evaluate({input})[0];
  EXPECT_TRUE(std::isfinite(result.threat_score));
  EXPECT_NEAR(result.contributions.range, 1.0f / 6.0f, kEps);
  EXPECT_NEAR(result.threat_score, 1.0f / 6.0f, kEps);
}

TEST(ThreatEvaluatorTest, PartialZeroWeightNullifiesAttribute) {
  // 单个属性权重为 0：该属性贡献恒 0，其余权重按相对值归一化。
  ThreatEvaluatorConfig config;
  config.weight_speed = 0.0f;  // 其余权重和 = 0.75

  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = 200000.0f;  // 距离贡献归零，孤立速度属性
  input.speed = config.speed_max_mps;

  const ThreatEvaluator evaluator(config);
  const auto result = evaluator.Evaluate({input})[0];
  EXPECT_NEAR(result.contributions.speed, 0.0f, kEps);
  EXPECT_NEAR(result.threat_score, 0.0f, kEps);
}

TEST(ThreatEvaluatorTest, InfinityRangeAndNegativeProbConfTreatAsMissing) {
  // range=+inf（isfinite 分支）、负类型概率/负融合置信度 → 按缺失处理。
  ThreatEvaluationInput input = MakeInput(1U);
  input.range_m = std::numeric_limits<float>::infinity();
  input.target_probability = -0.5f;
  input.fusion_confidence = -1.0f;
  input.speed = 0.0f;
  input.rcs = 0.0f;

  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  const auto result = evaluator.Evaluate({input})[0];
  EXPECT_NEAR(result.contributions.range, 0.0f, kEps);
  EXPECT_NEAR(result.contributions.target_probability, 0.0f, kEps);
  EXPECT_NEAR(result.contributions.fusion_confidence, 0.0f, kEps);
  EXPECT_NEAR(result.threat_score, 0.0f, kEps);
}

TEST(ThreatEvaluatorTest, DeterministicSameInputSameOutput) {
  const ThreatEvaluator evaluator(ThreatEvaluatorConfig{});
  const auto inputs = {MakeMidpointInput(1U), MakeInput(2U)};
  const auto first = evaluator.Evaluate(inputs);
  const auto second = evaluator.Evaluate(inputs);

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(first[i].key, second[i].key);
    EXPECT_FLOAT_EQ(first[i].threat_score, second[i].threat_score);
    EXPECT_EQ(first[i].level, second[i].level);
    EXPECT_FLOAT_EQ(first[i].contributions.range,
                    second[i].contributions.range);
    EXPECT_FLOAT_EQ(first[i].contributions.speed,
                    second[i].contributions.speed);
    EXPECT_FLOAT_EQ(first[i].contributions.acceleration,
                    second[i].contributions.acceleration);
    EXPECT_FLOAT_EQ(first[i].contributions.rcs, second[i].contributions.rcs);
    EXPECT_FLOAT_EQ(first[i].contributions.target_probability,
                    second[i].contributions.target_probability);
    EXPECT_FLOAT_EQ(first[i].contributions.fusion_confidence,
                    second[i].contributions.fusion_confidence);
  }
}

}  // namespace
}  // namespace threat_assessment
