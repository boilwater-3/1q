/**
 * @file esr_hypothesis_associator_test.cpp
 * @brief 验证 ESR 假设关联器的关联与回收行为。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
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
                           std::size_t support_count,
                           const std::string& spectral_label = std::string(),
                           session::EsrWaveformClass waveform_class =
                               session::EsrWaveformClass::kPulse) {
  ClusterSummary summary;
  summary.centroid_feature.values =
      std::array<float, kObservationFeatureDimension>{{x, y, 0.0f, 0.0f, 0.0f}};
  summary.representative_index = 0U;
  summary.support_count = support_count;
  summary.mean_snr_db = snr_db;
  summary.mean_az_deg = az_deg;
  summary.mean_el_deg = el_deg;
  summary.mean_rf_hz = rf_hz;
  summary.mean_bandwidth_hz = 2.0e6;
  summary.mean_pulse_width_s = 1.0e-6;
  summary.mean_pri_s = 1.0e-3;
  summary.rf_std_hz = 1000.0;
  summary.bandwidth_std_hz = 2000.0;
  summary.pri_std_s = 1.0e-6;
  summary.pulse_width_std_s = 1.0e-8;
  summary.confidence_score = 0.8f;
  summary.spectral_class_label = spectral_label;
  summary.waveform_class = waveform_class;
  return summary;
}

TEST(EsrHypothesisAssociatorTest, DoesNotAssociateDifferentWaveformClasses) {
  extension::InterceptAssociationConfig config;
  config.gate_distance = 1.0f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 3U;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> first;
  first.push_back(MakeCluster(0.0f, 0.0f, 10.0f, 0.0f, 10.0e9, 15.0f, 2U));
  ASSERT_EQ(associator.Update(1U, first, &next_hypothesis_id).size(), 1U);

  std::vector<ClusterSummary> second;
  second.push_back(MakeCluster(0.0f, 0.0f, 10.0f, 0.0f, 10.0e9, 15.0f, 2U, "",
                               session::EsrWaveformClass::kContinuous));
  ASSERT_EQ(associator.Update(2U, second, &next_hypothesis_id).size(), 2U);

  const std::vector<HypothesisAssociator::TrackState> tracks = associator.CaptureTracks();
  ASSERT_EQ(tracks.size(), 2U);
  EXPECT_EQ(tracks[0].waveform_class, session::EsrWaveformClass::kPulse);
  EXPECT_EQ(tracks[0].missed_cycles, 1U);
  EXPECT_EQ(tracks[1].waveform_class, session::EsrWaveformClass::kContinuous);
  EXPECT_EQ(tracks[1].missed_cycles, 0U);
}

TEST(EsrHypothesisAssociatorTest, SnapshotContinuationPreservesWaveformClassGate) {
  extension::InterceptAssociationConfig config;
  config.gate_distance = 1.0f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 3U;
  config.output_tentative = true;
  HypothesisAssociator original(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> pulse;
  pulse.push_back(MakeCluster(0.0f, 0.0f, 10.0f, 0.0f, 10.0e9, 15.0f, 2U));
  ASSERT_EQ(original.Update(1U, pulse, &next_hypothesis_id).size(), 1U);

  HypothesisAssociator restored(config);
  restored.RestoreTracks(original.CaptureTracks());
  std::vector<ClusterSummary> continuous;
  continuous.push_back(MakeCluster(
      0.0f, 0.0f, 10.0f, 0.0f, 10.0e9, 15.0f, 2U, "",
      session::EsrWaveformClass::kContinuous));
  ASSERT_EQ(restored.Update(2U, continuous, &next_hypothesis_id).size(), 2U);

  const std::vector<HypothesisAssociator::TrackState> tracks =
      restored.CaptureTracks();
  ASSERT_EQ(tracks.size(), 2U);
  EXPECT_EQ(tracks[0].waveform_class, session::EsrWaveformClass::kPulse);
  EXPECT_EQ(tracks[0].missed_cycles, 1U);
  EXPECT_EQ(tracks[1].waveform_class,
            session::EsrWaveformClass::kContinuous);
}

TEST(EsrHypothesisAssociatorTest, MaintainsStableIdsAcrossMatchedCycles) {
  extension::InterceptAssociationConfig config;
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
  const session::EmitterHypothesisList cycle_1 =
      associator.Update(10U, clusters_cycle_1, &next_hypothesis_id);
  ASSERT_EQ(cycle_1.size(), 2U);

  std::vector<ClusterSummary> clusters_cycle_2;
  clusters_cycle_2.push_back(MakeCluster(0.1f, 0.1f, 11.0f, 1.2f, 10.1e9, 16.0f, 4U));
  clusters_cycle_2.push_back(MakeCluster(3.1f, 2.9f, -19.0f, 0.1f, 5.1e9, 13.0f, 3U));
  const session::EmitterHypothesisList cycle_2 =
      associator.Update(11U, clusters_cycle_2, &next_hypothesis_id);
  ASSERT_EQ(cycle_2.size(), 2U);

  EXPECT_EQ(cycle_1[0].hypothesis_id, cycle_2[0].hypothesis_id);
  EXPECT_EQ(cycle_1[1].hypothesis_id, cycle_2[1].hypothesis_id);
  EXPECT_EQ(cycle_2[0].last_seen_cycle, 11U);
  EXPECT_EQ(cycle_2[1].last_seen_cycle, 11U);
  EXPECT_GT(cycle_2[0].estimated_center_frequency_hz, 10.0e9);
  EXPECT_DOUBLE_EQ(cycle_2[0].estimated_bandwidth_hz, 2.0e6);
  EXPECT_DOUBLE_EQ(cycle_2[0].estimated_pri_s, 1.0e-3);
  EXPECT_DOUBLE_EQ(cycle_2[0].estimated_pulse_width_s, 1.0e-6);
  EXPECT_GT(cycle_2[0].center_frequency_std_hz, 0.0);
}

TEST(EsrHypothesisAssociatorTest, CreatesNewTrackWhenTwoClustersCompeteOneTrack) {
  extension::InterceptAssociationConfig config;
  config.gate_distance = 0.8f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 4U;
  config.confidence_alpha = 0.3f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters_cycle_1;
  clusters_cycle_1.push_back(MakeCluster(0.0f, 0.0f, 10.0f, 1.0f, 10.0e9, 15.0f, 3U));
  const session::EmitterHypothesisList cycle_1 =
      associator.Update(20U, clusters_cycle_1, &next_hypothesis_id);
  ASSERT_EQ(cycle_1.size(), 1U);
  const std::uint64_t stable_id = cycle_1[0].hypothesis_id;

  std::vector<ClusterSummary> clusters_cycle_2;
  clusters_cycle_2.push_back(MakeCluster(0.1f, 0.1f, 10.5f, 1.1f, 10.0e9, 14.5f, 2U));
  clusters_cycle_2.push_back(MakeCluster(0.6f, 0.6f, 15.0f, 2.0f, 9.0e9, 8.0f, 2U));
  const session::EmitterHypothesisList cycle_2 =
      associator.Update(21U, clusters_cycle_2, &next_hypothesis_id);
  ASSERT_EQ(cycle_2.size(), 2U);

  EXPECT_EQ(cycle_2[0].hypothesis_id, stable_id);
  EXPECT_NE(cycle_2[1].hypothesis_id, stable_id);
}

TEST(EsrHypothesisAssociatorTest, UsesMaximumCardinalityAssignmentBeforeMinimumDistance) {
  extension::InterceptAssociationConfig config;
  config.gate_distance = 0.75f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 4U;
  config.confidence_alpha = 1.0f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters_cycle_1;
  clusters_cycle_1.push_back(
      MakeCluster(0.0f, 0.0f, 0.0f, 0.0f, 10.0e9, 12.0f, 3U, "TRACK_ZERO"));
  clusters_cycle_1.push_back(
      MakeCluster(1.0f, 0.0f, 1.0f, 0.0f, 10.0e9, 12.0f, 3U, "TRACK_ONE"));
  const session::EmitterHypothesisList cycle_1 =
      associator.Update(25U, clusters_cycle_1, &next_hypothesis_id);
  ASSERT_EQ(cycle_1.size(), 2U);
  ASSERT_EQ(next_hypothesis_id, 3U);

  std::vector<ClusterSummary> clusters_cycle_2;
  clusters_cycle_2.push_back(
      MakeCluster(0.4f, 0.0f, 4.0f, 0.0f, 10.0e9, 12.0f, 3U, "FLEXIBLE"));
  clusters_cycle_2.push_back(
      MakeCluster(-0.5f, 0.0f, -5.0f, 0.0f, 10.0e9, 12.0f, 3U, "CONSTRAINED"));
  const session::EmitterHypothesisList cycle_2 =
      associator.Update(26U, clusters_cycle_2, &next_hypothesis_id);

  ASSERT_EQ(cycle_2.size(), 2U);
  EXPECT_EQ(next_hypothesis_id, 3U);
  const std::vector<HypothesisAssociator::TrackState> tracks = associator.CaptureTracks();
  ASSERT_EQ(tracks.size(), 2U);
  ASSERT_EQ(tracks[0].hypothesis_id, 1U);
  ASSERT_EQ(tracks[1].hypothesis_id, 2U);
  EXPECT_FLOAT_EQ(tracks[0].feature.values[0], -0.5f);
  EXPECT_FLOAT_EQ(tracks[1].feature.values[0], 0.4f);
  EXPECT_EQ(tracks[0].missed_cycles, 0U);
  EXPECT_EQ(tracks[1].missed_cycles, 0U);
  EXPECT_NE(std::find(tracks[0].candidate_classes.begin(), tracks[0].candidate_classes.end(),
                      "CONSTRAINED"),
            tracks[0].candidate_classes.end());
  EXPECT_NE(
      std::find(tracks[1].candidate_classes.begin(), tracks[1].candidate_classes.end(), "FLEXIBLE"),
      tracks[1].candidate_classes.end());
}

TEST(EsrHypothesisAssociatorTest, EqualCostPerfectMatchingUsesClusterThenHypothesisIdOrder) {
  extension::InterceptAssociationConfig config;
  config.gate_distance = 2.0f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 4U;
  config.confidence_alpha = 1.0f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> initial_clusters;
  initial_clusters.push_back(MakeCluster(1.0f, 0.0f, 1.0f, 0.0f, 10.0e9, 12.0f, 3U));
  initial_clusters.push_back(MakeCluster(0.0f, 0.0f, 0.0f, 0.0f, 10.0e9, 12.0f, 3U));
  ASSERT_EQ(associator.Update(27U, initial_clusters, &next_hypothesis_id).size(), 2U);

  std::vector<ClusterSummary> tied_clusters;
  tied_clusters.push_back(MakeCluster(0.0f, 0.0f, 2.0f, 0.0f, 10.0e9, 12.0f, 3U, "FIRST"));
  tied_clusters.push_back(MakeCluster(-1.0f, 0.0f, 3.0f, 0.0f, 10.0e9, 12.0f, 3U, "SECOND"));
  ASSERT_EQ(associator.Update(28U, tied_clusters, &next_hypothesis_id).size(), 2U);

  const std::vector<HypothesisAssociator::TrackState> tracks = associator.CaptureTracks();
  ASSERT_EQ(tracks.size(), 2U);
  ASSERT_EQ(tracks[0].hypothesis_id, 1U);
  ASSERT_EQ(tracks[1].hypothesis_id, 2U);
  EXPECT_NE(
      std::find(tracks[0].candidate_classes.begin(), tracks[0].candidate_classes.end(), "FIRST"),
      tracks[0].candidate_classes.end());
  EXPECT_NE(
      std::find(tracks[1].candidate_classes.begin(), tracks[1].candidate_classes.end(), "SECOND"),
      tracks[1].candidate_classes.end());
}

TEST(EsrHypothesisAssociatorTest, RecyclesTrackAfterConfiguredMissedCycles) {
  extension::InterceptAssociationConfig config;
  config.gate_distance = 1.0f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 2U;
  config.confidence_alpha = 0.3f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters_cycle_1;
  clusters_cycle_1.push_back(MakeCluster(0.0f, 0.0f, 0.0f, 0.0f, 10.0e9, 10.0f, 1U));
  const session::EmitterHypothesisList cycle_1 =
      associator.Update(30U, clusters_cycle_1, &next_hypothesis_id);
  ASSERT_EQ(cycle_1.size(), 1U);

  const std::vector<ClusterSummary> empty_clusters;
  const session::EmitterHypothesisList cycle_2 =
      associator.Update(31U, empty_clusters, &next_hypothesis_id);
  EXPECT_EQ(cycle_2.size(), 1U);
  const session::EmitterHypothesisList cycle_3 =
      associator.Update(32U, empty_clusters, &next_hypothesis_id);
  EXPECT_TRUE(cycle_3.empty());
}

TEST(EsrHypothesisAssociatorTest, EquivalentClustersHaveEquivalentConfidence) {
  extension::InterceptAssociationConfig config;
  config.gate_distance = 1.0f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 2U;
  config.confidence_alpha = 0.3f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters;
  clusters.push_back(MakeCluster(0.0f, 0.0f, 1.0f, 0.1f, 10.0e9, 14.0f, 3U));
  clusters.push_back(MakeCluster(5.0f, 5.0f, 2.0f, 0.2f, 12.0e9, 14.0f, 3U));

  const session::EmitterHypothesisList hypotheses =
      associator.Update(40U, clusters, &next_hypothesis_id);
  ASSERT_EQ(hypotheses.size(), 2U);

  const session::EmitterHypothesis* clean_hypothesis = nullptr;
  const session::EmitterHypothesis* second_hypothesis = nullptr;
  for (std::size_t i = 0; i < hypotheses.size(); ++i) {
    if (hypotheses[i].bearing_az_deg < 1.5f) {
      clean_hypothesis = &hypotheses[i];
    } else {
      second_hypothesis = &hypotheses[i];
    }
  }
  ASSERT_NE(clean_hypothesis, static_cast<const session::EmitterHypothesis*>(nullptr));
  ASSERT_NE(second_hypothesis, static_cast<const session::EmitterHypothesis*>(nullptr));
  EXPECT_FLOAT_EQ(second_hypothesis->confidence, clean_hypothesis->confidence);
}

TEST(EsrHypothesisAssociatorTest, AppendsSpectralClassWithoutReplacingBandClass) {
  extension::InterceptAssociationConfig config;
  config.confirm_hits = 1U;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters;
  clusters.push_back(
      MakeCluster(0.0f, 0.0f, 0.0f, 0.0f, 10.0e9, 14.0f, 5U, "SPECTRAL_BROADBAND"));

  const session::EmitterHypothesisList hypotheses =
      associator.Update(50U, clusters, &next_hypothesis_id);
  ASSERT_EQ(hypotheses.size(), 1U);
  EXPECT_NE(std::find(hypotheses[0].candidate_classes.begin(), hypotheses[0].candidate_classes.end(),
                      "RADAR_EMITTER"),
            hypotheses[0].candidate_classes.end());
  EXPECT_NE(std::find(hypotheses[0].candidate_classes.begin(), hypotheses[0].candidate_classes.end(),
                      "SPECTRAL_BROADBAND"),
            hypotheses[0].candidate_classes.end());
}

TEST(EsrHypothesisAssociatorTest, ZeroSupportCountDoesNotProduceInfiniteBearingStd) {
  extension::InterceptAssociationConfig config;
  config.confirm_hits = 1U;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> clusters;
  clusters.push_back(MakeCluster(0.0f, 0.0f, 0.0f, 0.0f, 10.0e9, 14.0f, 0U));

  const session::EmitterHypothesisList hypotheses =
      associator.Update(60U, clusters, &next_hypothesis_id);
  ASSERT_EQ(hypotheses.size(), 1U);
  EXPECT_TRUE(std::isfinite(hypotheses[0].bearing_std_deg));
  EXPECT_GT(hypotheses[0].bearing_std_deg, 0.0f);
}

TEST(EsrHypothesisAssociatorTest, TieDistanceAssociationUsesStableClusterOrder) {
  extension::InterceptAssociationConfig config;
  config.gate_distance = 1.0f;
  config.confirm_hits = 1U;
  config.max_missed_cycles = 3U;
  config.confidence_alpha = 1.0f;
  config.output_tentative = true;
  HypothesisAssociator associator(config);

  std::uint64_t next_hypothesis_id = 1U;
  std::vector<ClusterSummary> cycle_1_clusters;
  cycle_1_clusters.push_back(
      MakeCluster(0.0f, 0.0f, 0.0f, 0.0f, 10.0e9, 12.0f, 3U, "SPECTRAL_A"));
  const session::EmitterHypothesisList cycle_1 =
      associator.Update(70U, cycle_1_clusters, &next_hypothesis_id);
  ASSERT_EQ(cycle_1.size(), 1U);
  const std::uint64_t stable_id = cycle_1[0].hypothesis_id;

  std::vector<ClusterSummary> cycle_2_clusters;
  cycle_2_clusters.push_back(
      MakeCluster(0.5f, 0.0f, 1.0f, 0.0f, 10.0e9, 12.0f, 3U, "SPECTRAL_FIRST"));
  cycle_2_clusters.push_back(
      MakeCluster(-0.5f, 0.0f, -1.0f, 0.0f, 10.0e9, 12.0f, 3U, "SPECTRAL_SECOND"));
  const session::EmitterHypothesisList cycle_2 =
      associator.Update(71U, cycle_2_clusters, &next_hypothesis_id);
  ASSERT_EQ(cycle_2.size(), 2U);

  const session::EmitterHypothesis* matched_track = nullptr;
  for (std::size_t i = 0; i < cycle_2.size(); ++i) {
    if (cycle_2[i].hypothesis_id == stable_id) {
      matched_track = &cycle_2[i];
      break;
    }
  }
  ASSERT_NE(matched_track, static_cast<const session::EmitterHypothesis*>(nullptr));
  EXPECT_NE(std::find(matched_track->candidate_classes.begin(), matched_track->candidate_classes.end(),
                      "SPECTRAL_FIRST"),
            matched_track->candidate_classes.end());
  EXPECT_EQ(std::find(matched_track->candidate_classes.begin(), matched_track->candidate_classes.end(),
                      "SPECTRAL_SECOND"),
            matched_track->candidate_classes.end());
}

}  // namespace
}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
