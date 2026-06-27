#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "electronic_surveillance_radar/environment/EsrSharedUtils.h"
#include "electronic_surveillance_radar/pipeline/BandClassifier.h"
#include "electronic_surveillance_radar/pipeline/ObservationFeatureEncoder.h"

namespace electronic_surveillance_radar {
namespace pipeline {

namespace {

/**
 * @brief 根据簇摘要推断工作模式。
 * @param[in] summary 簇摘要。
 * @return 工作模式。
 */
session::EsrEmitterMode InferModeFromCluster(const ClusterSummary& summary) {
  const bool has_valid_pri = std::isfinite(summary.mean_pri_s) && summary.mean_pri_s > 0.0;
  if (summary.mean_pulse_width_s < 1.5e-6 && (!has_valid_pri || summary.mean_pri_s >= 1.5e-4)) {
    return session::EsrEmitterMode::kSearch;
  }
  if (has_valid_pri && summary.mean_pri_s <= 8.0e-5) {
    return session::EsrEmitterMode::kGuidance;
  }
  if (summary.mean_pulse_width_s < 3.0e-6 || (has_valid_pri && summary.mean_pri_s <= 3.0e-4)) {
    return session::EsrEmitterMode::kTracking;
  }
  return session::EsrEmitterMode::kGuidance;
}

/**
 * @brief 根据模式和簇信噪比推断威胁等级。
 * @param[in] mode 工作模式。
 * @param[in] mean_snr_db 簇均值信噪比。
 * @return 威胁等级。
 */
session::EsrThreatLevel InferThreatFromCluster(session::EsrEmitterMode mode, float mean_snr_db) {
  if (mode == session::EsrEmitterMode::kGuidance || mean_snr_db >= 20.0f) {
    return session::EsrThreatLevel::kHigh;
  }
  if (mode == session::EsrEmitterMode::kTracking || mean_snr_db >= 10.0f) {
    return session::EsrThreatLevel::kMedium;
  }
  return session::EsrThreatLevel::kLow;
}

/**
 * @brief 构造候选类别列表。
 * @param[in] rf_hz 均值载频（单位：Hz）。
 * @param[in] deception_support_ratio 欺骗支持度，范围 [0, 1]。
 * @return 候选类别字符串列表。
 */
std::vector<std::string> BuildCandidateClasses(double rf_hz, float deception_support_ratio,
                                               const std::string& spectral_class_label) {
  const intercept::RadarBand band = intercept::BandClassifier::Classify(rf_hz);
  std::vector<std::string> classes;
  classes.push_back(std::string("RADAR_BAND_") + intercept::BandClassifier::ToString(band));
  classes.push_back("RADAR_EMITTER");
  const float deception_ratio = utils::Clamp01(deception_support_ratio);
  if (deception_ratio >= 0.6f) {
    classes.push_back("POSSIBLE_DECEPTION");
  }
  if (deception_ratio >= 0.3f) {
    classes.push_back("AMBIGUOUS_CLASS");
  }
  if (!spectral_class_label.empty()) {
    classes.push_back(spectral_class_label);
  }
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
  const float a = utils::Clamp01(alpha);
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

float ComputeBaseBearingStdDeg(std::size_t support_count) {
  const std::size_t safe_support_count = std::max<std::size_t>(2U, support_count);
  return std::max(0.1f, 3.0f / std::sqrt(static_cast<float>(safe_support_count)));
}

struct CandidatePair {
  std::size_t cluster_index;
  std::size_t track_index;
  double distance;
};

/**
 * @brief 候选对排序规则：按距离升序，其次按簇索引、轨迹索引升序。
 * @note 显式固定并列距离的顺序，避免关联输出在边界场景抖动。
 */
bool LessCandidatePair(const CandidatePair& lhs, const CandidatePair& rhs) {
  return std::tie(lhs.distance, lhs.cluster_index, lhs.track_index) <
         std::tie(rhs.distance, rhs.cluster_index, rhs.track_index);
}

}  // namespace

HypothesisAssociator::HypothesisAssociator(extension::InterceptAssociationConfig config)
    : config_(std::move(config)) {}

void HypothesisAssociator::UpdateConfig(extension::InterceptAssociationConfig config) { config_ = config; }

session::EmitterHypothesisList HypothesisAssociator::Update(
    std::uint32_t cycle_index, const std::vector<ClusterSummary>& clusters,
    std::uint64_t* next_hypothesis_id) {
  const std::size_t original_track_count = tracks_.size();
  std::vector<std::uint8_t> track_matched(original_track_count, 0U);
  std::vector<std::uint8_t> cluster_matched(clusters.size(), 0U);

  std::vector<CandidatePair> pairs;
  const double gate_distance = static_cast<double>(config_.gate_distance);
  for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
    for (std::size_t track_index = 0; track_index < original_track_count; ++track_index) {
      const double distance = static_cast<double>(ComputeDistance(
          clusters[cluster_index].centroid_feature, tracks_[track_index].feature));
      if (distance <= gate_distance) {
        CandidatePair pair;
        pair.cluster_index = cluster_index;
        pair.track_index = track_index;
        pair.distance = distance;
        pairs.push_back(pair);
      }
    }
  }
  std::sort(pairs.begin(), pairs.end(), LessCandidatePair);

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const CandidatePair pair = pairs[i];
    if (cluster_matched[pair.cluster_index] != 0U || track_matched[pair.track_index] != 0U) {
      continue;
    }
    TrackState& track = tracks_[pair.track_index];
    const ClusterSummary& summary = clusters[pair.cluster_index];
    const float deception_ratio = utils::Clamp01(summary.deception_support_ratio);

    for (std::size_t dim = 0; dim < kObservationFeatureDimension; ++dim) {
      track.feature.values[dim] =
          Blend(track.feature.values[dim], summary.centroid_feature.values[dim],
                config_.confidence_alpha);
    }
    track.mode = InferModeFromCluster(summary);
    track.threat_level = InferThreatFromCluster(track.mode, summary.mean_snr_db);
    track.candidate_classes = BuildCandidateClasses(summary.mean_rf_hz, deception_ratio,
                                                    summary.spectral_class_label);
    track.bearing_az_deg =
        Blend(track.bearing_az_deg, summary.mean_az_deg, config_.confidence_alpha);
    track.bearing_el_deg =
        Blend(track.bearing_el_deg, summary.mean_el_deg, config_.confidence_alpha);
    const float base_bearing_std_deg = ComputeBaseBearingStdDeg(summary.support_count);
    track.bearing_std_deg = base_bearing_std_deg * (1.0f + 0.8f * deception_ratio);
    const float confidence_measurement =
        utils::Clamp01(summary.confidence_score * (1.0f - 0.45f * deception_ratio));
    track.confidence = Blend(track.confidence, confidence_measurement, config_.confidence_alpha);
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
    const float deception_ratio = utils::Clamp01(clusters[i].deception_support_ratio);
    track.feature = clusters[i].centroid_feature;
    track.mode = InferModeFromCluster(clusters[i]);
    track.threat_level = InferThreatFromCluster(track.mode, clusters[i].mean_snr_db);
    track.candidate_classes = BuildCandidateClasses(clusters[i].mean_rf_hz, deception_ratio,
                                                    clusters[i].spectral_class_label);
    track.bearing_az_deg = clusters[i].mean_az_deg;
    track.bearing_el_deg = clusters[i].mean_el_deg;
    const float base_bearing_std_deg = ComputeBaseBearingStdDeg(clusters[i].support_count);
    track.bearing_std_deg = base_bearing_std_deg * (1.0f + 0.8f * deception_ratio);
    track.confidence = utils::Clamp01(clusters[i].confidence_score * (1.0f - 0.45f * deception_ratio));
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
    tracks_[i].confirmed = false;
    tracks_[i].confidence = utils::Clamp01(tracks_[i].confidence * 0.92f);
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const TrackState& track) {
                                 return track.missed_cycles >= config_.max_missed_cycles;
                               }),
                tracks_.end());

  std::sort(tracks_.begin(), tracks_.end(), [](const TrackState& lhs, const TrackState& rhs) {
    return lhs.hypothesis_id < rhs.hypothesis_id;
  });

  session::EmitterHypothesisList hypotheses;
  hypotheses.reserve(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (!tracks_[i].confirmed && !config_.output_tentative) {
      continue;
    }
    session::EmitterHypothesis hypothesis;
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

void HypothesisAssociator::Reset() { tracks_.clear(); }

std::vector<HypothesisAssociator::TrackState> HypothesisAssociator::CaptureTracks() const {
  return tracks_;
}

void HypothesisAssociator::RestoreTracks(const std::vector<TrackState>& tracks) {
  tracks_ = tracks;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
