/**
 * @file esr_kdtree_clusterer_test.cpp
 * @brief 验证 ESR 预处理与 KD-tree 聚类组件行为。
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/KdTreeClusterer.h"
#include "electronic_surveillance_radar/pipeline/ObservationFeatureEncoder.h"
#include "electronic_surveillance_radar/pipeline/ObservationPreprocessor.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {
namespace {

/**
 * @brief 构造基础观测记录。
 * @param[in] observation_id 观测 ID。
 * @param[in] timestamp_s 时间戳。
 * @param[in] rf_hz 载频。
 * @param[in] pulse_width_s 脉宽。
 * @param[in] az_deg 方位角。
 * @param[in] el_deg 俯仰角。
 * @param[in] snr_db 信噪比。
 * @return 观测记录。
 */
RawObservationRecord MakeRecord(std::uint64_t observation_id, double timestamp_s, double rf_hz,
                                double pulse_width_s, float az_deg, float el_deg, float snr_db) {
  RawObservationRecord record;
  record.observation.observation_id = observation_id;
  record.observation.timestamp_s = timestamp_s;
  record.observation.rf_hz = rf_hz;
  record.observation.pulse_width_s = pulse_width_s;
  record.observation.aoa_az_deg = az_deg;
  record.observation.aoa_el_deg = el_deg;
  record.observation.snr_db = snr_db;
  record.truth_emitter_id = "E";
  record.matched_truth = true;
  return record;
}

TEST(EsrKdTreeClustererTest, PreprocessorSortsFiltersAndDeduplicates) {
  std::vector<RawObservationRecord> records;
  records.push_back(MakeRecord(10U, 1.0, 10.0e9, 1.0e-6, 10.0f, 1.0f, 8.0f));
  records.push_back(MakeRecord(9U, 0.9, 10.0e9, 1.0e-6, 10.0f, 1.0f, 12.0f));
  records.push_back(MakeRecord(11U, 1.0 + 1.0e-6, 10.0e9 + 0.5e6, 1.1e-6, 10.5f, 1.2f, 15.0f));
  records.push_back(MakeRecord(12U, 1.2, 0.0, 1.0e-6, 0.0f, 0.0f, 5.0f));
  RawObservationRecord invalid_nan = MakeRecord(13U, 1.3, 12.0e9, 1.0e-6, 0.0f, 0.0f, 5.0f);
  invalid_nan.observation.snr_db = std::numeric_limits<float>::quiet_NaN();
  records.push_back(invalid_nan);

  ObservationPreprocessor preprocessor;
  InterceptPreprocessConfig config;
  config.dedup_time_window_sec = 5.0e-6f;
  config.dedup_rf_window_hz = 1.0e6;
  config.dedup_pw_window_sec = 2.0e-7;
  config.dedup_az_window_deg = 1.0f;
  config.dedup_el_window_deg = 1.0f;
  config.normalize_quality = true;

  const std::vector<RawObservationRecord> output = preprocessor.Run(records, config);

  ASSERT_EQ(output.size(), 2U);
  EXPECT_LT(output[0].observation.timestamp_s, output[1].observation.timestamp_s);
  EXPECT_EQ(output[1].observation.observation_id, 11U);
  EXPECT_EQ(output[1].observation.quality, common::ObservationQuality::kMedium);
}

TEST(EsrKdTreeClustererTest, ClustererGroupsNearbyFeatures) {
  std::vector<ObservationFeatureVector> features(3U);
  features[0].values =
      std::array<float, kObservationFeatureDimension>{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
  features[1].values =
      std::array<float, kObservationFeatureDimension>{{0.2f, 0.1f, 0.0f, 0.0f, 0.1f}};
  features[2].values =
      std::array<float, kObservationFeatureDimension>{{4.0f, 4.0f, 0.0f, 0.0f, 0.0f}};

  KdTreeClusterer clusterer;
  InterceptClusterConfig config;
  config.radius = 0.6f;
  config.min_points = 1U;

  const KdTreeClusterResult result = clusterer.Cluster(features, config);

  ASSERT_EQ(result.clusters.size(), 2U);
  EXPECT_TRUE(result.noise_indices.empty());
  EXPECT_EQ(result.clusters[0].size(), 2U);
  EXPECT_EQ(result.clusters[1].size(), 1U);
}

TEST(EsrKdTreeClustererTest, ClustererMarksNoiseWhenMinPointsNotMet) {
  std::vector<ObservationFeatureVector> features(3U);
  features[0].values =
      std::array<float, kObservationFeatureDimension>{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
  features[1].values =
      std::array<float, kObservationFeatureDimension>{{3.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
  features[2].values =
      std::array<float, kObservationFeatureDimension>{{6.0f, 0.0f, 0.0f, 0.0f, 0.0f}};

  KdTreeClusterer clusterer;
  InterceptClusterConfig config;
  config.radius = 0.5f;
  config.min_points = 2U;

  const KdTreeClusterResult result = clusterer.Cluster(features, config);

  EXPECT_TRUE(result.clusters.empty());
  ASSERT_EQ(result.noise_indices.size(), 3U);
  EXPECT_EQ(result.noise_indices[0], 0U);
  EXPECT_EQ(result.noise_indices[1], 1U);
  EXPECT_EQ(result.noise_indices[2], 2U);
}

}  // namespace
}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
