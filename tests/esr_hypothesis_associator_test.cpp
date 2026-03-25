/**
 * @file esr_hypothesis_associator_test.cpp
 * @brief 验证 ESR 假设关联器的关联与回收行为。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {
namespace {

/**
 * @brief 构造簇摘要输入。
 * @param[in] x 特征维度 0。
 * @param[in] y 特征维度 1。
 * @param[in] az_deg 均值方位。
 * @param[in] el_deg 均值俯仰。
 * @param[in] rf_hz 均值载频。
 * @param[in] snr_db 均值信噪比。
 * @param[in] support_count 样本数。
 * @return 簇摘要。
 */
ClusterSummary MakeCluster(float x, float y, float az_deg, float el_deg, double rf_hz, float snr_db,
                           std::size_t support_count, float deception_support_ratio = 0.0f) {
  ClusterSummary summary;
  summary.centroid_feature.values =
      std::array<float, kObservationFeatureDimension>{{x, y, 0.0f, 0.0f, 0.0f}};
  summary.representative_index = 0U;
  summary.support_count = support_count;
  summary.mean_snr_db = snr_db;
  summary.mean_az_deg = az_deg;
  summary.mean_el_deg = el_deg;
  summary.mean_rf_hz = rf_hz;
  summary.mean_pulse_width_s = 1.0e-6;
  summary.confidence_score = 0.8f;
  summary.deception_support_ratio = deception_support_ratio;
  return summary;
}

TEST(EsrHypothesisAssociatorTest, MaintainsStableIdsAcrossMatchedCycles) {
  InterceptAssociationConfig config;
  config.gate_distance = 1.0f;
  config.confirm_hits = 2U;
  config.max_missed_cycles = 4U;
  config.confidence_alpha = 0.3f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters_cycle_1;
  clusters_cycle_1.push_back(MakeCluster(0.0f, 0.0f, 10.0f, 1.0f, 10.0e9, 15.0f, 3U));
  clusters_cycle_1.push_back(MakeCluster(3.0f, 3.0f, -20.0f, 0.0f, 5.0e9, 12.0f, 2U));
  const common::EmitterHypothesisList cycle_1 =
      associator.Update(10U, clusters_cycle_1, &next_hypothesis_id);
  ASSERT_EQ(cycle_1.size(), 2U);

  std::vector<ClusterSummary> clusters_cycle_2;
  clusters_cycle_2.push_back(MakeCluster(0.1f, 0.1f, 11.0f, 1.2f, 10.1e9, 16.0f, 4U));
  clusters_cycle_2.push_back(MakeCluster(3.1f, 2.9f, -19.0f, 0.1f, 5.1e9, 13.0f, 3U));
  const common::EmitterHypothesisList cycle_2 =
      associator.Update(11U, clusters_cycle_2, &next_hypothesis_id);
  ASSERT_EQ(cycle_2.size(), 2U);

  EXPECT_EQ(cycle_1[0].hypothesis_id, cycle_2[0].hypothesis_id);
  EXPECT_EQ(cycle_1[1].hypothesis_id, cycle_2[1].hypothesis_id);
  EXPECT_EQ(cycle_2[0].last_seen_cycle, 11U);
  EXPECT_EQ(cycle_2[1].last_seen_cycle, 11U);
}

TEST(EsrHypothesisAssociatorTest, CreatesNewTrackWhenTwoClustersCompeteOneTrack) {
  InterceptAssociationConfig config;
  config.gate_distance = 0.8f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 4U;
  config.confidence_alpha = 0.3f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters_cycle_1;
  clusters_cycle_1.push_back(MakeCluster(0.0f, 0.0f, 10.0f, 1.0f, 10.0e9, 15.0f, 3U));
  const common::EmitterHypothesisList cycle_1 =
      associator.Update(20U, clusters_cycle_1, &next_hypothesis_id);
  ASSERT_EQ(cycle_1.size(), 1U);
  const std::uint64_t stable_id = cycle_1[0].hypothesis_id;

  std::vector<ClusterSummary> clusters_cycle_2;
  clusters_cycle_2.push_back(MakeCluster(0.1f, 0.1f, 10.5f, 1.1f, 10.0e9, 14.5f, 2U));
  clusters_cycle_2.push_back(MakeCluster(0.6f, 0.6f, 15.0f, 2.0f, 9.0e9, 8.0f, 2U));
  const common::EmitterHypothesisList cycle_2 =
      associator.Update(21U, clusters_cycle_2, &next_hypothesis_id);
  ASSERT_EQ(cycle_2.size(), 2U);

  EXPECT_EQ(cycle_2[0].hypothesis_id, stable_id);
  EXPECT_NE(cycle_2[1].hypothesis_id, stable_id);
}

TEST(EsrHypothesisAssociatorTest, RecyclesTrackAfterConfiguredMissedCycles) {
  InterceptAssociationConfig config;
  config.gate_distance = 1.0f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 2U;
  config.confidence_alpha = 0.3f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters_cycle_1;
  clusters_cycle_1.push_back(MakeCluster(0.0f, 0.0f, 0.0f, 0.0f, 10.0e9, 10.0f, 1U));
  const common::EmitterHypothesisList cycle_1 =
      associator.Update(30U, clusters_cycle_1, &next_hypothesis_id);
  ASSERT_EQ(cycle_1.size(), 1U);

  const std::vector<ClusterSummary> empty_clusters;
  const common::EmitterHypothesisList cycle_2 =
      associator.Update(31U, empty_clusters, &next_hypothesis_id);
  EXPECT_EQ(cycle_2.size(), 1U);
  const common::EmitterHypothesisList cycle_3 =
      associator.Update(32U, empty_clusters, &next_hypothesis_id);
  EXPECT_TRUE(cycle_3.empty());
}

TEST(EsrHypothesisAssociatorTest, HighDeceptionSupportLowersConfidenceAndAddsAmbiguousClass) {
  InterceptAssociationConfig config;
  config.gate_distance = 1.0f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 2U;
  config.confidence_alpha = 0.3f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters;
  clusters.push_back(MakeCluster(0.0f, 0.0f, 1.0f, 0.1f, 10.0e9, 14.0f, 3U, 0.0f));
  clusters.push_back(MakeCluster(5.0f, 5.0f, 2.0f, 0.2f, 12.0e9, 14.0f, 3U, 0.8f));

  const common::EmitterHypothesisList hypotheses =
      associator.Update(40U, clusters, &next_hypothesis_id);
  ASSERT_EQ(hypotheses.size(), 2U);

  const common::EmitterHypothesis* clean_hypothesis = nullptr;
  const common::EmitterHypothesis* deceptive_hypothesis = nullptr;
  for (std::size_t i = 0; i < hypotheses.size(); ++i) {
    if (hypotheses[i].bearing_az_deg < 1.5f) {
      clean_hypothesis = &hypotheses[i];
    } else {
      deceptive_hypothesis = &hypotheses[i];
    }
  }
  ASSERT_NE(clean_hypothesis, static_cast<const common::EmitterHypothesis*>(nullptr));
  ASSERT_NE(deceptive_hypothesis, static_cast<const common::EmitterHypothesis*>(nullptr));
  EXPECT_LT(deceptive_hypothesis->confidence, clean_hypothesis->confidence);
  EXPECT_NE(std::find(deceptive_hypothesis->candidate_classes.begin(),
                      deceptive_hypothesis->candidate_classes.end(), "AMBIGUOUS_CLASS"),
            deceptive_hypothesis->candidate_classes.end());
}

}  // namespace
}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
