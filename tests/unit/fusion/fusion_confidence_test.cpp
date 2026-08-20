// Copyright 2026. All Rights Reserved.
//
// @file fusion_confidence_test.cpp
// @brief 验证融合置信度公式：Σ 判决值 × 质量归一化 × 权重（滑窗内精确求和）。
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "1q/fusion/fusion.hpp"

namespace fusion {
namespace {

DetectionRecord MakeDetection(std::uint64_t key, std::uint32_t source_id, double verdict,
                              double quality) {
  DetectionRecord detection;
  detection.key = key;
  detection.source_id = source_id;
  detection.verdict = verdict;
  detection.quality = quality;
  return detection;
}

const FusedTarget* FindTarget(const std::vector<FusedTarget>& targets, std::uint64_t key) {
  for (const FusedTarget& target : targets) {
    if (target.key == key) {
      return &target;
    }
  }
  return nullptr;
}

TEST(FusionConfidenceTest, SingleDetectionConfidenceEqualsProduct) {
  FusionEngine engine(FusionConfig{});
  const auto targets = engine.Update({MakeDetection(7U, 0U, 1.0, 0.5)}, 1U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 0.5);
}

TEST(FusionConfidenceTest, MultipleSourcesSumConfidence) {
  FusionEngine engine(FusionConfig{});
  const auto targets =
      engine.Update({MakeDetection(7U, 0U, 1.0, 0.5), MakeDetection(7U, 1U, 1.0, 1.0)}, 1U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 1.5);
  // 各源通道状态独立记录。
  ASSERT_EQ(targets[0].channels.size(), 2U);
  EXPECT_EQ(targets[0].channels[0].source_id, 0U);
  EXPECT_EQ(targets[0].channels[0].sample_count, 1U);
  EXPECT_EQ(targets[0].channels[1].source_id, 1U);
  EXPECT_EQ(targets[0].channels[1].sample_count, 1U);
}

TEST(FusionConfidenceTest, ZeroVerdictContributesNothing) {
  FusionEngine engine(FusionConfig{});
  const auto targets = engine.Update({MakeDetection(7U, 0U, 0.0, 0.8)}, 1U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 0.0);
}

TEST(FusionConfidenceTest, SourceWeightsApply) {
  FusionConfig config;
  config.source_weights = {2.0};
  FusionEngine engine(config);
  const auto targets = engine.Update({MakeDetection(7U, 0U, 1.0, 0.5)}, 1U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 1.0);
}

TEST(FusionConfidenceTest, MissingWeightDefaultsToOne) {
  FusionConfig config;
  config.source_weights = {2.0};  // 仅覆盖 source 0，source 1 缺省 1.0。
  FusionEngine engine(config);
  const auto targets =
      engine.Update({MakeDetection(7U, 0U, 1.0, 0.5), MakeDetection(7U, 1U, 1.0, 0.5)}, 1U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 1.0 + 0.5);
}

TEST(FusionConfidenceTest, ConfidenceAccumulatesAcrossCyclesWithinWindow) {
  FusionConfig config;
  config.window_size = 10U;
  FusionEngine engine(config);
  engine.Update({MakeDetection(7U, 0U, 1.0, 0.5)}, 1U);
  const auto targets = engine.Update({MakeDetection(7U, 0U, 1.0, 1.0)}, 2U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 1.5);
  EXPECT_EQ(targets[0].channels[0].sample_count, 2U);
  EXPECT_DOUBLE_EQ(targets[0].channels[0].latest_quality, 1.0);
}

TEST(FusionConfidenceTest, WindowEvictsOldSamples) {
  FusionConfig config;
  config.window_size = 2U;
  FusionEngine engine(config);
  engine.Update({MakeDetection(7U, 0U, 1.0, 1.0)}, 1U);
  engine.Update({MakeDetection(7U, 0U, 1.0, 1.0)}, 2U);
  const auto targets = engine.Update({MakeDetection(7U, 0U, 1.0, 1.0)}, 3U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 2.0);
  EXPECT_EQ(targets[0].channels[0].sample_count, 2U);
}

TEST(FusionConfidenceTest, ConfidenceIsUnboundedByEvidence) {
  // 冻结公式为精确求和（不归一化）：置信度随窗口内证据单调累积。
  FusionConfig config;
  config.window_size = 10U;
  FusionEngine engine(config);
  std::vector<DetectionRecord> detections;
  for (std::uint32_t i = 0; i < 5U; ++i) {
    detections.push_back(MakeDetection(7U, i, 1.0, 1.0));
  }
  const auto targets = engine.Update(detections, 1U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_DOUBLE_EQ(targets[0].confidence, 5.0);
}

TEST(FusionConfidenceTest, LastUpdateCycleTracksLatestSample) {
  FusionEngine engine(FusionConfig{});
  engine.Update({MakeDetection(7U, 0U, 1.0, 0.5)}, 1U);
  const auto targets = engine.Update({MakeDetection(7U, 0U, 1.0, 1.0)}, 9U);

  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets[0].last_update_cycle, 9U);
  const FusedTarget* target = FindTarget(targets, 7U);
  ASSERT_NE(target, nullptr);
  EXPECT_EQ(target->last_update_cycle, 9U);
}

}  // namespace
}  // namespace fusion
