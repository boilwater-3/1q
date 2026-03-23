#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "electronic_surveillance_radar/intercept/BandClassifier.h"
#include "electronic_surveillance_radar/pipeline/ObservationFeatureEncoder.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

namespace {

/**
 * @brief 将输入裁剪到 [0, 1]。
 * @param[in] value 输入值。
 * @return 裁剪后结果。
 */
float Clamp01(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

/**
 * @brief 根据簇摘要推断工作模式。
 * @param[in] summary 簇摘要。
 * @return 工作模式。
 */
common::EmitterMode InferModeFromCluster(const ClusterSummary& summary) {
  if (summary.mean_pulse_width_s < 1.5e-6) {
    return common::EmitterMode::kSearch;
  }
  if (summary.mean_pulse_width_s < 3.0e-6) {
    return common::EmitterMode::kTracking;
  }
  return common::EmitterMode::kGuidance;
}

/**
 * @brief 根据模式和簇信噪比推断威胁等级。
 * @param[in] mode 工作模式。
 * @param[in] mean_snr_db 簇均值信噪比。
 * @return 威胁等级。
 */
common::ThreatLevel InferThreatFromCluster(common::EmitterMode mode,
                                           float mean_snr_db) {
  if (mode == common::EmitterMode::kGuidance || mean_snr_db >= 20.0f) {
    return common::ThreatLevel::kHigh;
  }
  if (mode == common::EmitterMode::kTracking || mean_snr_db >= 10.0f) {
    return common::ThreatLevel::kMedium;
  }
  return common::ThreatLevel::kLow;
}

/**
 * @brief 构造候选类别列表。
 * @param[in] rf_hz 均值载频（单位：Hz）。
 * @return 候选类别字符串列表。
 */
std::vector<std::string> BuildCandidateClasses(double rf_hz) {
  const intercept::RadarBand band = intercept::BandClassifier::Classify(rf_hz);
  std::vector<std::string> classes;
  classes.push_back(
      std::string("RADAR_BAND_") + intercept::BandClassifier::ToString(band));
  classes.push_back("RADAR_EMITTER");
  return classes;
}

/**
 * @brief 线性插值更新标量。
 * @param[in] previous 上一状态值。
 * @param[in] current 当前观测值。
 * @param[in] alpha 更新系数。
 * @return 更新结果。
 */
float Blend(float previous, float current, float alpha) {
  const float a = Clamp01(alpha);
  return (1.0f - a) * previous + a * current;
}

/**
 * @brief 计算簇到假设的距离。
 * @param[in] feature_a 特征向量 A。
 * @param[in] feature_b 特征向量 B。
 * @return 特征空间距离。
 */
float ComputeDistance(const ObservationFeatureVector& feature_a,
                      const ObservationFeatureVector& feature_b) {
  return ObservationFeatureEncoder::Distance(feature_a, feature_b);
}

}  // namespace

HypothesisAssociator::HypothesisAssociator(InterceptAssociationConfig config)
    : config_(config) {}

void HypothesisAssociator::UpdateConfig(InterceptAssociationConfig config) {
  config_ = config;
}

common::EmitterHypothesisList HypothesisAssociator::Update(
    std::uint32_t cycle_index, const std::vector<ClusterSummary>& clusters,
    std::uint64_t* next_hypothesis_id) {
  const std::size_t original_track_count = tracks_.size();
  std::vector<std::uint8_t> track_matched(original_track_count, 0U);
  std::vector<std::uint8_t> cluster_matched(clusters.size(), 0U);

  struct CandidatePair {
    std::size_t cluster_index;
    std::size_t track_index;
    float distance;
  };
  std::vector<CandidatePair> pairs;
  for (std::size_t cluster_index = 0; cluster_index < clusters.size();
       ++cluster_index) {
    for (std::size_t track_index = 0; track_index < original_track_count;
         ++track_index) {
      const float distance =
          ComputeDistance(clusters[cluster_index].centroid_feature,
                          tracks_[track_index].feature);
      if (distance <= config_.gate_distance) {
        CandidatePair pair;
        pair.cluster_index = cluster_index;
        pair.track_index = track_index;
        pair.distance = distance;
        pairs.push_back(pair);
      }
    }
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const CandidatePair& lhs, const CandidatePair& rhs) {
              if (lhs.distance != rhs.distance) {
                return lhs.distance < rhs.distance;
              }
              if (lhs.cluster_index != rhs.cluster_index) {
                return lhs.cluster_index < rhs.cluster_index;
              }
              return lhs.track_index < rhs.track_index;
            });

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const CandidatePair pair = pairs[i];
    if (cluster_matched[pair.cluster_index] != 0U ||
        track_matched[pair.track_index] != 0U) {
      continue;
    }
    TrackState& track = tracks_[pair.track_index];
    const ClusterSummary& summary = clusters[pair.cluster_index];

    for (std::size_t dim = 0; dim < kObservationFeatureDimension; ++dim) {
      track.feature.values[dim] =
          Blend(track.feature.values[dim], summary.centroid_feature.values[dim],
                config_.confidence_alpha);
    }
    track.mode = InferModeFromCluster(summary);
    track.threat_level = InferThreatFromCluster(track.mode, summary.mean_snr_db);
    track.candidate_classes = BuildCandidateClasses(summary.mean_rf_hz);
    track.bearing_az_deg =
        Blend(track.bearing_az_deg, summary.mean_az_deg, config_.confidence_alpha);
    track.bearing_el_deg =
        Blend(track.bearing_el_deg, summary.mean_el_deg, config_.confidence_alpha);
    track.bearing_std_deg =
        std::max(0.1f, 3.0f / std::sqrt(static_cast<float>(summary.support_count)));
    track.confidence =
        Blend(track.confidence, summary.confidence_score, config_.confidence_alpha);
    track.last_seen_cycle = cycle_index;
    ++track.hit_streak;
    track.missed_cycles = 0U;
    ++track.age_cycles;
    track.support_count = summary.support_count;
    if (track.hit_streak >= config_.confirm_hits) {
      track.confirmed = true;
    }

    cluster_matched[pair.cluster_index] = 1U;
    track_matched[pair.track_index] = 1U;
  }

  for (std::size_t i = 0; i < clusters.size(); ++i) {
    if (cluster_matched[i] != 0U) {
      continue;
    }
    TrackState track;
    if (next_hypothesis_id != nullptr) {
      track.hypothesis_id = (*next_hypothesis_id)++;
    }
    track.feature = clusters[i].centroid_feature;
    track.mode = InferModeFromCluster(clusters[i]);
    track.threat_level = InferThreatFromCluster(track.mode, clusters[i].mean_snr_db);
    track.candidate_classes = BuildCandidateClasses(clusters[i].mean_rf_hz);
    track.bearing_az_deg = clusters[i].mean_az_deg;
    track.bearing_el_deg = clusters[i].mean_el_deg;
    track.bearing_std_deg =
        std::max(0.1f, 3.0f / std::sqrt(static_cast<float>(clusters[i].support_count)));
    track.confidence = Clamp01(clusters[i].confidence_score);
    track.last_seen_cycle = cycle_index;
    track.hit_streak = 1U;
    track.missed_cycles = 0U;
    track.age_cycles = 1U;
    track.support_count = clusters[i].support_count;
    track.confirmed = (config_.confirm_hits <= 1U);
    tracks_.push_back(track);
  }

  for (std::size_t i = 0; i < original_track_count; ++i) {
    if (track_matched[i] != 0U) {
      continue;
    }
    ++tracks_[i].missed_cycles;
    ++tracks_[i].age_cycles;
    tracks_[i].hit_streak = 0U;
    tracks_[i].confidence = Clamp01(tracks_[i].confidence * 0.92f);
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const TrackState& track) {
                                 return track.missed_cycles >=
                                        config_.max_missed_cycles;
                               }),
               tracks_.end());

  std::sort(tracks_.begin(), tracks_.end(),
            [](const TrackState& lhs, const TrackState& rhs) {
              return lhs.hypothesis_id < rhs.hypothesis_id;
            });

  common::EmitterHypothesisList hypotheses;
  hypotheses.reserve(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (!tracks_[i].confirmed && !config_.output_tentative) {
      continue;
    }
    common::EmitterHypothesis hypothesis;
    hypothesis.hypothesis_id = tracks_[i].hypothesis_id;
    hypothesis.candidate_classes = tracks_[i].candidate_classes;
    hypothesis.mode = tracks_[i].mode;
    hypothesis.threat_level = tracks_[i].threat_level;
    hypothesis.bearing_az_deg = tracks_[i].bearing_az_deg;
    hypothesis.bearing_el_deg = tracks_[i].bearing_el_deg;
    hypothesis.bearing_std_deg = tracks_[i].bearing_std_deg;
    hypothesis.confidence = tracks_[i].confidence;
    hypothesis.last_seen_cycle = tracks_[i].last_seen_cycle;
    hypotheses.push_back(hypothesis);
  }
  return hypotheses;
}

void HypothesisAssociator::Reset() {
  tracks_.clear();
}

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
