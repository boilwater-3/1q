/**
 * @file esr_intercept_post_processing_test.cpp
 * @brief 验证 InterceptPostProcessingExecutor::Execute 的全链路分支覆盖。
 *
 * 现有测试只覆盖 ObservationPreprocessor / KdTreeClusterer / HypothesisAssociator
 * 的隔离行为，从不调用 Execute。本测试直接驱动 Execute，覆盖簇摘要、
 * 频谱分析与缺失关联等分支。
 */

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/InterceptPostProcessingExecutor.h"
#include "electronic_surveillance_radar/pipeline/KdTreeClusterer.h"
#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/ObservationPreprocessor.h"
#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

using session::EsrCycleInput;

// 构造一个最小可用的辐射源记录（复制自 esr_kdtree_clusterer_test 的 MakeRecord 惯例）
RawObservationRecord MakeRecord(std::uint64_t observation_id, double timestamp_s,
                                double rf_hz, float az_deg, float el_deg, float snr_db) {
  RawObservationRecord record;
  record.observation.observation_id = observation_id;
  record.observation.timestamp_s = timestamp_s;
  record.observation.rf_hz = static_cast<float>(rf_hz);
  record.observation.pulse_width_s = 1.0e-6f;
  record.observation.aoa_az_deg = az_deg;
  record.observation.aoa_el_deg = el_deg;
  record.observation.snr_db = snr_db;
  return record;
}

// 构造一个已初始化的上下文（BeginCycle 后）
MutableEsrContext MakeContext(bool spectral_enabled) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.cycle_index = 1U;

  session::EsrEnvironmentSnapshot snapshot;
  extension::InterceptPipelineConfig config;
  config.spectral_analysis.enable = spectral_enabled;
  config.spectral_analysis.min_sequence_length = 4U;

  MutableEsrContext ctx;
  ctx.BeginCycle(input, snapshot, config, extension::InterceptRuntimeConfig{});
  return ctx;
}

TEST(InterceptPostProcessingExecutorTest, EmptyRecordsProducesZeroOutput) {
  MutableEsrContext ctx = MakeContext(true);
  ObservationPreprocessor preprocessor;
  KdTreeClusterer clusterer;
  HypothesisAssociator associator;
  ObservationFeatureScales feature_scales;
  std::uint64_t next_id = 1U;

  InterceptPostProcessingExecutor executor;
  const extension::InterceptPipelineResult result =
      executor.Execute({}, ctx, preprocessor, clusterer, associator,
                       feature_scales, next_id);

  EXPECT_EQ(result.observation_output.raw_observation_count, 0u);
}

TEST(InterceptPostProcessingExecutorTest, ClusterWithSpectralAnalysisLabelsEmitters) {
  // 4+ 样本的密集簇，rf_hz 微小变化 → 触发频谱分析 + 三路标签
  std::vector<RawObservationRecord> records;
  for (int i = 0; i < 5; ++i) {
    records.push_back(MakeRecord(100U + i, 1.0 + i * 0.1,
                                 10.0e9 + i * 0.5e6, 10.0f, 1.0f, 15.0f));
  }

  MutableEsrContext ctx = MakeContext(true);
  ObservationPreprocessor preprocessor;
  KdTreeClusterer clusterer;
  HypothesisAssociator associator;
  ObservationFeatureScales feature_scales;
  std::uint64_t next_id = 1U;

  InterceptPostProcessingExecutor executor;
  const extension::InterceptPipelineResult result =
      executor.Execute(records, ctx, preprocessor, clusterer, associator,
                       feature_scales, next_id);

  EXPECT_GT(result.observation_output.raw_observation_count, 0u);
}

TEST(InterceptPostProcessingExecutorTest, SingleDeclassifiedRecordProducesOutput) {
  std::vector<RawObservationRecord> records;
  RawObservationRecord rec = MakeRecord(300U, 1.0, 8.0e9, 20.0f, 2.0f, 12.0f);
  records.push_back(rec);

  MutableEsrContext ctx = MakeContext(false);
  ObservationPreprocessor preprocessor;
  KdTreeClusterer clusterer;
  HypothesisAssociator associator;
  ObservationFeatureScales feature_scales;
  std::uint64_t next_id = 1U;

  InterceptPostProcessingExecutor executor;
  const extension::InterceptPipelineResult result =
      executor.Execute(records, ctx, preprocessor, clusterer, associator,
                       feature_scales, next_id);

  EXPECT_GT(result.observation_output.raw_observation_count, 0u);
}

TEST(InterceptPostProcessingExecutorTest, SmallClusterGetsInsufficientSpectralLabel) {
  // 只有 2 个样本 < min_sequence_length(4) → SPECTRAL_INSUFFICIENT
  std::vector<RawObservationRecord> records;
  records.push_back(MakeRecord(400U, 1.0, 11.0e9, 15.0f, 1.0f, 14.0f));
  records.push_back(MakeRecord(401U, 1.1, 11.0e9, 15.0f, 1.0f, 14.0f));

  MutableEsrContext ctx = MakeContext(true);
  ObservationPreprocessor preprocessor;
  KdTreeClusterer clusterer;
  HypothesisAssociator associator;
  ObservationFeatureScales feature_scales;
  std::uint64_t next_id = 1U;

  InterceptPostProcessingExecutor executor;
  const extension::InterceptPipelineResult result =
      executor.Execute(records, ctx, preprocessor, clusterer, associator,
                       feature_scales, next_id);

  EXPECT_GT(result.observation_output.raw_observation_count, 0u);
}

TEST(InterceptPostProcessingExecutorTest, SpectralDisabledSkipsAnalysis) {
  std::vector<RawObservationRecord> records;
  for (int i = 0; i < 5; ++i) {
    records.push_back(MakeRecord(500U + i, 1.0 + i * 0.1,
                                 10.0e9, 10.0f, 1.0f, 15.0f));
  }

  MutableEsrContext ctx = MakeContext(false);  // spectral disabled
  ObservationPreprocessor preprocessor;
  KdTreeClusterer clusterer;
  HypothesisAssociator associator;
  ObservationFeatureScales feature_scales;
  std::uint64_t next_id = 1U;

  InterceptPostProcessingExecutor executor;
  const extension::InterceptPipelineResult result =
      executor.Execute(records, ctx, preprocessor, clusterer, associator,
                       feature_scales, next_id);

  EXPECT_GT(result.observation_output.raw_observation_count, 0u);
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
