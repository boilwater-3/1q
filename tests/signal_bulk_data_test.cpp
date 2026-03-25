// Copyright 2026. All Rights Reserved.
//
// @file signal_bulk_data_test.cpp
// @brief 验证信号层在批量与大规模输入下的稳定性与关联一致性。

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace tests {
namespace {

/// @brief 生成一批具备有效位置的目标输入，确保可进入位置关联路径。
common::TargetFeatureList BuildBatchTargets(std::size_t count, float x_bias, float y_bias,
                                            float z_bias) {
  common::TargetFeatureList targets;
  targets.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    common::TargetFeature target(120.0f + static_cast<float>(i % 7), 0.0f, 0.0f, 6.0f, 0.2f, 0.0f,
                                 0.0f, 1000.0f + static_cast<float>(i) * 2.0f, 0);
    target.position_x = x_bias + static_cast<float>(i) * 15.0f;
    target.position_y = y_bias + static_cast<float>(i % 5) * 3.0f;
    target.position_z = z_bias + static_cast<float>(i % 3) * 2.0f;
    target.external_target_id = 100000u + static_cast<std::uint64_t>(i);
    targets.push_back(target);
  }

  return targets;
}

/// @brief 统计量测列表中命中已有轨迹的数量。
std::size_t CountMatchedTracks(
    const std::vector<signal::tracking::TrackMeasurement>& measurements) {
  std::size_t matched = 0;
  for (const signal::tracking::TrackMeasurement& measurement : measurements) {
    if (measurement.raw_measurement.matched_existing_track) {
      ++matched;
    }
  }
  return matched;
}

/// @brief 计算毫秒样本在给定分位数处的值（最近邻法）。
double ComputePercentileMs(std::vector<double> samples, double percentile) {
  if (samples.empty()) {
    return 0.0;
  }

  std::sort(samples.begin(), samples.end());
  const double clamped = std::max(0.0, std::min(percentile, 1.0));
  const std::size_t index =
      static_cast<std::size_t>(clamped * static_cast<double>(samples.size() - 1));
  return samples[index];
}

/// @brief 从目标列表抽取外部原始 ID 集合。
std::unordered_set<std::uint64_t> BuildExternalIdSet(const common::TargetFeatureList& features) {
  std::unordered_set<std::uint64_t> ids;
  ids.reserve(features.size());
  for (const common::TargetFeature& feature : features) {
    if (feature.external_target_id != 0U) {
      ids.insert(feature.external_target_id);
    }
  }
  return ids;
}

/// @brief 运行双周期 IMM 生命周期场景并返回总耗时。
double RunImmLifecycleScenarioMs(std::size_t target_count,
                                 signal::pipeline::ImmActivationPolicy activation_policy) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -100.0f;
  pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
  pipeline_config.lifecycle.enable_imm_lifecycle = true;
  pipeline_config.lifecycle.lifecycle_config.confirm_hits = 1;
  pipeline_config.lifecycle.lifecycle_config.imm_activation_policy = activation_policy;
  pipeline_config.lifecycle.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 8.0f};
  signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal_pipeline.CreateAutoLifecycleManager();
  EXPECT_TRUE(lifecycle_manager != nullptr);
  if (lifecycle_manager == nullptr) {
    return 0.0;
  }

  const std::array<common::TargetFeatureList, 2> cycle_inputs{{
      BuildBatchTargets(target_count, 400.0f, 12.0f, 1.0f),
      BuildBatchTargets(target_count, 401.0f, 12.4f, 1.2f),
  }};

  const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

  for (std::size_t cycle_index = 0; cycle_index < cycle_inputs.size(); ++cycle_index) {
    signal_pipeline.SetAssociationSeeds(lifecycle_manager->BuildAssociationSeeds());
    signal_pipeline.RunCycle(cycle_inputs[cycle_index], environment_service);
    const std::vector<signal::tracking::TrackMeasurement> measurements =
        signal_pipeline.GetLastTrackMeasurements();
    EXPECT_EQ(measurements.size(), target_count);

    signal::tracking::CycleContext cycle;
    cycle.cycle_index = static_cast<std::uint32_t>(cycle_index + 1U);
    cycle.batch_id = static_cast<std::uint32_t>(cycle_index + 1U);
    lifecycle_manager->Update(cycle, measurements);
  }

  const std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

/// @brief 大批量单周期输入应稳定产出等量量测和合理质量指标。
TEST(SignalBulkDataTest, LargeBatchSingleCycleProducesConsistentMeasurements) {
  const std::size_t kTargetCount = 5000u;

  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -100.0f;
  signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  const common::TargetFeatureList input_state =
      BuildBatchTargets(kTargetCount, 1000.0f, 10.0f, 5.0f);

  const signal::pipeline::SignalCycleResult output_state =
      signal_pipeline.RunCycle(input_state, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> measurements =
      signal_pipeline.GetLastTrackMeasurements();
  const signal::pipeline::AssociationQualityMetrics metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();

  ASSERT_EQ(output_state.updated_features.size(), kTargetCount);
  ASSERT_EQ(measurements.size(), kTargetCount);
  EXPECT_EQ(metrics.detection_count, kTargetCount);
  EXPECT_EQ(metrics.new_track_count, kTargetCount);
  EXPECT_EQ(metrics.matched_count, 0u);
  EXPECT_EQ(metrics.prior_track_count, 0u);
  EXPECT_FLOAT_EQ(metrics.match_rate, 0.0f);
  EXPECT_FLOAT_EQ(metrics.new_track_rate, 1.0f);

  for (const signal::tracking::TrackMeasurement& measurement : measurements) {
    EXPECT_NE(measurement.raw_measurement.association_key, 0u);
    EXPECT_TRUE(measurement.raw_measurement.has_cartesian_position);
  }
}

/// @brief 批量多周期下，自动装配 IMM 生命周期应在第二周期形成高比例稳定匹配。
TEST(SignalBulkDataTest, LargeBatchImmAutoLifecycleMaintainsHighMatchRateOnNextCycle) {
  const std::size_t kTargetCount = 2000u;

  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -100.0f;
  pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
  pipeline_config.lifecycle.enable_imm_lifecycle = true;
  pipeline_config.lifecycle.lifecycle_config.confirm_hits = 1;
  pipeline_config.lifecycle.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 8.0f};
  signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(lifecycle_manager != nullptr);

  const common::TargetFeatureList cycle_1_input =
      BuildBatchTargets(kTargetCount, 500.0f, 20.0f, 3.0f);

  signal_pipeline.SetAssociationSeeds(lifecycle_manager->BuildAssociationSeeds());
  signal_pipeline.RunCycle(cycle_1_input, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> cycle_1_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_1_measurements.size(), kTargetCount);

  signal::tracking::CycleContext cycle_1_context;
  cycle_1_context.cycle_index = 1u;
  cycle_1_context.batch_id = 1u;
  lifecycle_manager->Update(cycle_1_context, cycle_1_measurements);

  const common::TargetFeatureList cycle_2_input =
      BuildBatchTargets(kTargetCount, 501.0f, 20.5f, 3.1f);

  signal_pipeline.SetAssociationSeeds(lifecycle_manager->BuildAssociationSeeds());
  signal_pipeline.RunCycle(cycle_2_input, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> cycle_2_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_2_measurements.size(), kTargetCount);

  const std::size_t matched_count = CountMatchedTracks(cycle_2_measurements);
  const float match_ratio = static_cast<float>(matched_count) / static_cast<float>(kTargetCount);

  EXPECT_GT(match_ratio, 0.95f);
  for (const signal::tracking::TrackMeasurement& measurement : cycle_2_measurements) {
    EXPECT_TRUE(measurement.raw_measurement.used_external_association_seeds);
  }
}

/// @brief 分级批量数据基准：统计 1k/5k/10k 单周期处理耗时的 P50/P95。
TEST(SignalBulkDataTest, TieredBatchSingleCycleReportsP50AndP95Latency) {
  struct BatchTier {
    std::size_t target_count;
    const char* label;
  };

  const std::array<BatchTier, 3> tiers{{
      {1000u, "1k"},
      {5000u, "5k"},
      {10000u, "10k"},
  }};
  const std::size_t kRepeatsPerTier = 5u;

  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -100.0f;
  signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  double previous_p95_ms = 0.0;
  for (std::size_t tier_index = 0; tier_index < tiers.size(); ++tier_index) {
    const BatchTier& tier = tiers[tier_index];
    std::vector<double> elapsed_ms_samples;
    elapsed_ms_samples.reserve(kRepeatsPerTier);

    for (std::size_t repeat = 0; repeat < kRepeatsPerTier; ++repeat) {
      const common::TargetFeatureList input_state = BuildBatchTargets(
          tier.target_count, 1000.0f + static_cast<float>(repeat),
          8.0f + static_cast<float>(repeat) * 0.1f, 2.0f + static_cast<float>(repeat) * 0.1f);

      const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
      const signal::pipeline::SignalCycleResult output_state =
          signal_pipeline.RunCycle(input_state, environment_service);
      const std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      const std::vector<signal::tracking::TrackMeasurement> measurements =
          signal_pipeline.GetLastTrackMeasurements();
      const signal::pipeline::AssociationQualityMetrics metrics =
          signal_pipeline.GetLastAssociationQualityMetrics();

      ASSERT_EQ(output_state.updated_features.size(), tier.target_count);
      ASSERT_EQ(measurements.size(), tier.target_count);
      EXPECT_EQ(metrics.detection_count, tier.target_count);
      EXPECT_EQ(metrics.new_track_count, tier.target_count);
      EXPECT_EQ(metrics.matched_count, 0u);

      const std::chrono::duration<double, std::milli> elapsed = end - start;
      elapsed_ms_samples.push_back(elapsed.count());
    }

    const double p50_ms = ComputePercentileMs(elapsed_ms_samples, 0.50);
    const double p95_ms = ComputePercentileMs(elapsed_ms_samples, 0.95);

    EXPECT_GT(p50_ms, 0.0);
    EXPECT_GE(p95_ms, p50_ms);
    if (tier_index > 0u) {
      const double allowed_regression_ms = std::max(0.05, previous_p95_ms * 0.15);
      EXPECT_GE(p95_ms + allowed_regression_ms, previous_p95_ms);
    }
    previous_p95_ms = p95_ms;

    std::cout << "[SignalBulkDataTest] tier=" << tier.label << " targets=" << tier.target_count
              << " p50_ms=" << p50_ms << " p95_ms=" << p95_ms << std::endl;
  }
}

/// @brief 验证 external_target_id 在跨周期 IMM 路径中可保持并回传到快照。
TEST(SignalBulkDataTest, ExternalTargetIdStaysConsistentAcrossImmLifecycleCycles) {
  const std::size_t kTargetCount = 512u;

  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -100.0f;
  pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
  pipeline_config.lifecycle.enable_imm_lifecycle = true;
  pipeline_config.lifecycle.lifecycle_config.confirm_hits = 1;
  pipeline_config.lifecycle.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 8.0f};
  signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);

  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  std::unique_ptr<signal::tracking::ITrackLifecycleManager> lifecycle_manager =
      signal_pipeline.CreateAutoLifecycleManager();
  ASSERT_TRUE(lifecycle_manager != nullptr);

  const common::TargetFeatureList cycle_1_input =
      BuildBatchTargets(kTargetCount, 600.0f, 15.0f, 2.0f);
  const std::unordered_set<std::uint64_t> expected_ids = BuildExternalIdSet(cycle_1_input);
  ASSERT_EQ(expected_ids.size(), kTargetCount);

  signal_pipeline.SetAssociationSeeds(lifecycle_manager->BuildAssociationSeeds());
  signal_pipeline.RunCycle(cycle_1_input, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> cycle_1_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_1_measurements.size(), kTargetCount);

  for (const signal::tracking::TrackMeasurement& measurement : cycle_1_measurements) {
    ASSERT_LT(measurement.raw_measurement.source_index, cycle_1_input.size());
    EXPECT_EQ(measurement.raw_measurement.external_target_id,
              cycle_1_input[measurement.raw_measurement.source_index].external_target_id);
    EXPECT_NE(measurement.raw_measurement.external_target_id, 0U);
  }

  signal::tracking::CycleContext cycle_1_context;
  cycle_1_context.cycle_index = 1u;
  cycle_1_context.batch_id = 1u;
  lifecycle_manager->Update(cycle_1_context, cycle_1_measurements);

  const common::TargetFeatureList snapshot_after_cycle_1 =
      lifecycle_manager->BuildFeatureSnapshot();
  EXPECT_EQ(BuildExternalIdSet(snapshot_after_cycle_1), expected_ids);

  const common::TargetFeatureList cycle_2_input =
      BuildBatchTargets(kTargetCount, 601.0f, 15.2f, 2.1f);
  signal_pipeline.SetAssociationSeeds(lifecycle_manager->BuildAssociationSeeds());
  signal_pipeline.RunCycle(cycle_2_input, environment_service);
  const std::vector<signal::tracking::TrackMeasurement> cycle_2_measurements =
      signal_pipeline.GetLastTrackMeasurements();
  ASSERT_EQ(cycle_2_measurements.size(), kTargetCount);

  for (const signal::tracking::TrackMeasurement& measurement : cycle_2_measurements) {
    ASSERT_LT(measurement.raw_measurement.source_index, cycle_2_input.size());
    EXPECT_EQ(measurement.raw_measurement.external_target_id,
              cycle_2_input[measurement.raw_measurement.source_index].external_target_id);
    EXPECT_NE(measurement.raw_measurement.external_target_id, 0U);
  }

  signal::tracking::CycleContext cycle_2_context;
  cycle_2_context.cycle_index = 2u;
  cycle_2_context.batch_id = 2u;
  lifecycle_manager->Update(cycle_2_context, cycle_2_measurements);

  const common::TargetFeatureList snapshot_after_cycle_2 =
      lifecycle_manager->BuildFeatureSnapshot();
  EXPECT_EQ(BuildExternalIdSet(snapshot_after_cycle_2), expected_ids);
}

/// @brief 比较全轨迹 IMM 与仅确认轨迹 IMM 的双周期批量耗时分布。
TEST(SignalBulkDataTest, ImmLifecyclePoliciesReportLatencyComparison) {
  struct BatchTier {
    std::size_t target_count;
    const char* label;
  };

  const std::array<BatchTier, 1> tiers{{
      {2000u, "2k"},
  }};
  const std::size_t kRepeatsPerTier = 1u;

  for (const BatchTier& tier : tiers) {
    std::vector<double> all_tracks_samples;
    std::vector<double> confirmed_only_samples;
    all_tracks_samples.reserve(kRepeatsPerTier);
    confirmed_only_samples.reserve(kRepeatsPerTier);

    for (std::size_t repeat = 0; repeat < kRepeatsPerTier; ++repeat) {
      all_tracks_samples.push_back(RunImmLifecycleScenarioMs(
          tier.target_count, signal::pipeline::ImmActivationPolicy::kAllTracks));
      confirmed_only_samples.push_back(RunImmLifecycleScenarioMs(
          tier.target_count, signal::pipeline::ImmActivationPolicy::kConfirmedTracksOnly));
    }

    const double all_tracks_p50_ms = ComputePercentileMs(all_tracks_samples, 0.50);
    const double all_tracks_p95_ms = ComputePercentileMs(all_tracks_samples, 0.95);
    const double confirmed_only_p50_ms = ComputePercentileMs(confirmed_only_samples, 0.50);
    const double confirmed_only_p95_ms = ComputePercentileMs(confirmed_only_samples, 0.95);

    EXPECT_GT(all_tracks_p50_ms, 0.0);
    EXPECT_GT(all_tracks_p95_ms, 0.0);
    EXPECT_GT(confirmed_only_p50_ms, 0.0);
    EXPECT_GT(confirmed_only_p95_ms, 0.0);

    std::cout << "[SignalBulkDataTest] imm_policy_compare tier=" << tier.label
              << " targets=" << tier.target_count << " all_tracks_p50_ms=" << all_tracks_p50_ms
              << " all_tracks_p95_ms=" << all_tracks_p95_ms
              << " confirmed_only_p50_ms=" << confirmed_only_p50_ms
              << " confirmed_only_p95_ms=" << confirmed_only_p95_ms << std::endl;
  }
}

}  // namespace tests
}  // namespace airborne_radar
