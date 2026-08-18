#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"

#include <algorithm>
#include <boost/multiprecision/cpp_int.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
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
  // energy 类不经过 pri/pw 阈值：连续波照射 / 扫频搜索 / 噪声非可跟踪。
  // 仅 pulse 类保留 pri/pw 阈值推断 search/tracking/guidance。
  switch (summary.waveform_class) {
    case session::EsrWaveformClass::kContinuous:
      return session::EsrEmitterMode::kContinuousIllumination;
    case session::EsrWaveformClass::kSweep:
      return session::EsrEmitterMode::kSearch;
    case session::EsrWaveformClass::kNoise:
      return session::EsrEmitterMode::kUnknown;
    case session::EsrWaveformClass::kPulse:
      break;
  }
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
 * @brief 构造候选类别列表。
 * @param[in] rf_hz 均值载频（单位：Hz）。
 * @return 候选类别字符串列表。
 */
std::vector<std::string> BuildCandidateClasses(double rf_hz,
                                               const std::string& spectral_class_label) {
  const intercept::RadarBand band = intercept::BandClassifier::Classify(rf_hz);
  std::vector<std::string> classes;
  classes.push_back(std::string("RADAR_BAND_") + intercept::BandClassifier::ToString(band));
  classes.push_back("RADAR_EMITTER");
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

double BlendDouble(double previous, double current, float alpha) {
  const double a = static_cast<double>(utils::Clamp01(alpha));
  return (1.0 - a) * previous + a * current;
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

struct ResidualEdge {
  std::size_t to;
  std::size_t reverse_index;
  int capacity;
  double cost;
  boost::multiprecision::cpp_int tie_rank;
};

using ResidualGraph = std::vector<std::vector<ResidualEdge>>;

void AddResidualEdge(std::size_t from, std::size_t to, double cost,
                     const boost::multiprecision::cpp_int& tie_rank, ResidualGraph* graph) {
  ResidualEdge forward{to, (*graph)[to].size(), 1, cost, tie_rank};
  ResidualEdge reverse{from, (*graph)[from].size(), 0, -cost, -tie_rank};
  (*graph)[from].push_back(forward);
  (*graph)[to].push_back(reverse);
}

bool IsBetterPath(double candidate_distance,
                  const boost::multiprecision::cpp_int& candidate_tie_rank, double current_distance,
                  const boost::multiprecision::cpp_int& current_tie_rank) {
  return candidate_distance < current_distance ||
         (candidate_distance == current_distance && candidate_tie_rank < current_tie_rank);
}

std::vector<std::size_t> ComputeGlobalAssignment(
    const std::vector<ClusterSummary>& clusters,
    const std::vector<HypothesisAssociator::TrackState>& tracks, double gate_distance) {
  const std::size_t unmatched_track = tracks.size();
  std::vector<std::size_t> cluster_to_track(clusters.size(), unmatched_track);
  if (clusters.empty() || tracks.empty()) {
    return cluster_to_track;
  }

  const std::size_t source = 0U;
  const std::size_t cluster_offset = 1U;
  const std::size_t track_offset = cluster_offset + clusters.size();
  const std::size_t sink = track_offset + tracks.size();
  ResidualGraph graph(sink + 1U);

  // 把每个 cluster 的最终 assignment 看作 base=(track_count+1) 的一位：track index
  // 为 0..track_count-1，未匹配为 track_count。最大基数固定后，减去未匹配基线
  // 只改变符号 tie rank，不改变词典序；任意精度整数避免溢出或 epsilon 扰动主距离。
  std::vector<boost::multiprecision::cpp_int> cluster_weights(clusters.size());
  boost::multiprecision::cpp_int weight = 1;
  const boost::multiprecision::cpp_int base = tracks.size() + 1U;
  for (std::size_t cluster_index = clusters.size(); cluster_index > 0U; --cluster_index) {
    cluster_weights[cluster_index - 1U] = weight;
    weight *= base;
  }

  for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
    AddResidualEdge(source, cluster_offset + cluster_index, 0.0, 0, &graph);
    for (std::size_t track_index = 0; track_index < tracks.size(); ++track_index) {
      // Waveform class is an observable physical discriminator.  A class
      // change must start a distinct hypothesis rather than silently blend a
      // pulse train into a continuous, sweep, or noise emitter track.
      if (clusters[cluster_index].waveform_class != tracks[track_index].waveform_class) {
        continue;
      }
      const double distance = static_cast<double>(
          ComputeDistance(clusters[cluster_index].centroid_feature, tracks[track_index].feature));
      if (std::isfinite(distance) && distance <= gate_distance) {
        boost::multiprecision::cpp_int tie_rank = track_index;
        tie_rank -= tracks.size();
        tie_rank *= cluster_weights[cluster_index];
        AddResidualEdge(cluster_offset + cluster_index, track_offset + track_index, distance,
                        tie_rank, &graph);
      }
    }
  }
  for (std::size_t track_index = 0; track_index < tracks.size(); ++track_index) {
    AddResidualEdge(track_offset + track_index, sink, 0.0, 0, &graph);
  }

  const double infinity = std::numeric_limits<double>::infinity();
  while (true) {
    std::vector<double> distance(graph.size(), infinity);
    std::vector<boost::multiprecision::cpp_int> tie_rank(graph.size());
    std::vector<std::size_t> previous_node(graph.size(), graph.size());
    std::vector<std::size_t> previous_edge(graph.size(), 0U);
    distance[source] = 0.0;

    for (std::size_t iteration = 1U; iteration < graph.size(); ++iteration) {
      bool changed = false;
      for (std::size_t from = 0; from < graph.size(); ++from) {
        if (!std::isfinite(distance[from])) {
          continue;
        }
        for (std::size_t edge_index = 0; edge_index < graph[from].size(); ++edge_index) {
          const ResidualEdge& edge = graph[from][edge_index];
          if (edge.capacity == 0) {
            continue;
          }
          const double candidate_distance = distance[from] + edge.cost;
          const boost::multiprecision::cpp_int candidate_tie_rank = tie_rank[from] + edge.tie_rank;
          if (IsBetterPath(candidate_distance, candidate_tie_rank, distance[edge.to],
                           tie_rank[edge.to])) {
            distance[edge.to] = candidate_distance;
            tie_rank[edge.to] = candidate_tie_rank;
            previous_node[edge.to] = from;
            previous_edge[edge.to] = edge_index;
            changed = true;
          }
        }
      }
      if (!changed) {
        break;
      }
    }

    if (previous_node[sink] == graph.size()) {
      break;
    }
    for (std::size_t node = sink; node != source; node = previous_node[node]) {
      ResidualEdge& edge = graph[previous_node[node]][previous_edge[node]];
      --edge.capacity;
      ++graph[node][edge.reverse_index].capacity;
    }
  }

  for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
    const std::vector<ResidualEdge>& edges = graph[cluster_offset + cluster_index];
    for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
      const ResidualEdge& edge = edges[edge_index];
      if (edge.to >= track_offset && edge.to < sink && edge.capacity == 0) {
        cluster_to_track[cluster_index] = edge.to - track_offset;
        break;
      }
    }
  }
  return cluster_to_track;
}

}  // namespace

HypothesisAssociator::HypothesisAssociator(extension::InterceptAssociationConfig config)
    : config_(std::move(config)) {}

void HypothesisAssociator::UpdateConfig(extension::InterceptAssociationConfig config) { config_ = config; }

session::EmitterHypothesisList HypothesisAssociator::Update(
    std::uint32_t cycle_index, const std::vector<ClusterSummary>& clusters,
    std::uint64_t* next_hypothesis_id) {
  std::stable_sort(tracks_.begin(), tracks_.end(),
                   [](const TrackState& lhs, const TrackState& rhs) {
                     return lhs.hypothesis_id < rhs.hypothesis_id;
                   });
  const std::size_t original_track_count = tracks_.size();
  std::vector<std::uint8_t> track_matched(original_track_count, 0U);
  std::vector<std::uint8_t> cluster_matched(clusters.size(), 0U);

  const double gate_distance = static_cast<double>(config_.gate_distance);
  const std::vector<std::size_t> cluster_to_track =
      ComputeGlobalAssignment(clusters, tracks_, gate_distance);
  for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
    const std::size_t track_index = cluster_to_track[cluster_index];
    if (track_index == original_track_count) {
      continue;
    }
    TrackState& track = tracks_[track_index];
    const ClusterSummary& summary = clusters[cluster_index];

    for (std::size_t dim = 0; dim < kObservationFeatureDimension; ++dim) {
      track.feature.values[dim] =
          Blend(track.feature.values[dim], summary.centroid_feature.values[dim],
                config_.confidence_alpha);
    }
    track.mode = InferModeFromCluster(summary);
    track.waveform_class = summary.waveform_class;
    track.candidate_classes = BuildCandidateClasses(summary.mean_rf_hz, summary.spectral_class_label);
    track.bearing_az_deg =
        Blend(track.bearing_az_deg, summary.mean_az_deg, config_.confidence_alpha);
    track.bearing_el_deg =
        Blend(track.bearing_el_deg, summary.mean_el_deg, config_.confidence_alpha);
    track.estimated_center_frequency_hz = BlendDouble(
        track.estimated_center_frequency_hz, summary.mean_rf_hz, config_.confidence_alpha);
    track.estimated_bandwidth_hz = BlendDouble(
        track.estimated_bandwidth_hz, summary.mean_bandwidth_hz, config_.confidence_alpha);
    track.estimated_pri_s =
        BlendDouble(track.estimated_pri_s, summary.mean_pri_s, config_.confidence_alpha);
    track.estimated_pulse_width_s = BlendDouble(
        track.estimated_pulse_width_s, summary.mean_pulse_width_s, config_.confidence_alpha);
    track.center_frequency_std_hz = BlendDouble(
        track.center_frequency_std_hz, summary.rf_std_hz, config_.confidence_alpha);
    track.bandwidth_std_hz = BlendDouble(
        track.bandwidth_std_hz, summary.bandwidth_std_hz, config_.confidence_alpha);
    track.pri_std_s = BlendDouble(track.pri_std_s, summary.pri_std_s, config_.confidence_alpha);
    track.pulse_width_std_s = BlendDouble(
        track.pulse_width_std_s, summary.pulse_width_std_s, config_.confidence_alpha);
    const float base_bearing_std_deg = ComputeBaseBearingStdDeg(summary.support_count);
    track.bearing_std_deg = base_bearing_std_deg;
    const float confidence_measurement = utils::Clamp01(summary.confidence_score);
    track.confidence = Blend(track.confidence, confidence_measurement, config_.confidence_alpha);
    track.last_seen_cycle = cycle_index;
    ++track.hit_streak;
    track.missed_cycles = 0U;
    ++track.age_cycles;
    track.support_count = summary.support_count;
    if (track.hit_streak >= config_.confirm_hits) {
      track.confirmed = true;
    }

    cluster_matched[cluster_index] = 1U;
    track_matched[track_index] = 1U;
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
    track.waveform_class = clusters[i].waveform_class;
    track.candidate_classes =
        BuildCandidateClasses(clusters[i].mean_rf_hz, clusters[i].spectral_class_label);
    track.bearing_az_deg = clusters[i].mean_az_deg;
    track.bearing_el_deg = clusters[i].mean_el_deg;
    track.estimated_center_frequency_hz = clusters[i].mean_rf_hz;
    track.estimated_bandwidth_hz = clusters[i].mean_bandwidth_hz;
    track.estimated_pri_s = clusters[i].mean_pri_s;
    track.estimated_pulse_width_s = clusters[i].mean_pulse_width_s;
    track.center_frequency_std_hz = clusters[i].rf_std_hz;
    track.bandwidth_std_hz = clusters[i].bandwidth_std_hz;
    track.pri_std_s = clusters[i].pri_std_s;
    track.pulse_width_std_s = clusters[i].pulse_width_std_s;
    const float base_bearing_std_deg = ComputeBaseBearingStdDeg(clusters[i].support_count);
    track.bearing_std_deg = base_bearing_std_deg;
    track.confidence = utils::Clamp01(clusters[i].confidence_score);
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
    hypothesis.bearing_az_deg = tracks_[i].bearing_az_deg;
    hypothesis.bearing_el_deg = tracks_[i].bearing_el_deg;
    hypothesis.bearing_std_deg = tracks_[i].bearing_std_deg;
    hypothesis.estimated_center_frequency_hz = tracks_[i].estimated_center_frequency_hz;
    hypothesis.estimated_bandwidth_hz = tracks_[i].estimated_bandwidth_hz;
    hypothesis.estimated_pri_s = tracks_[i].estimated_pri_s;
    hypothesis.estimated_pulse_width_s = tracks_[i].estimated_pulse_width_s;
    hypothesis.center_frequency_std_hz = tracks_[i].center_frequency_std_hz;
    hypothesis.bandwidth_std_hz = tracks_[i].bandwidth_std_hz;
    hypothesis.pri_std_s = tracks_[i].pri_std_s;
    hypothesis.pulse_width_std_s = tracks_[i].pulse_width_std_s;
    hypothesis.confidence = tracks_[i].confidence;
    hypothesis.last_seen_cycle = tracks_[i].last_seen_cycle;
    hypothesis.waveform_class = tracks_[i].waveform_class;
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
