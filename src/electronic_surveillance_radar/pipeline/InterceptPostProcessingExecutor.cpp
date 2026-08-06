#include "electronic_surveillance_radar/pipeline/InterceptPostProcessingExecutor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "common/logging/ProjectLog.h"
#include "common/numerics/SpectralNumerics.h"
#include "electronic_surveillance_radar/environment/EsrSharedUtils.h"
#include "electronic_surveillance_radar/pipeline/ObservationFeatureEncoder.h"

namespace electronic_surveillance_radar {
namespace pipeline {

namespace {

/**
 * @brief 构造观测级置信度。
 * @param[in] snr_db 观测信噪比（单位：dB）。
 * @return 置信度，范围 [0, 1]。
 */
double ComputeObservationConfidence(double snr_db) {
  const float snr_score = utils::Clamp01(static_cast<float>((snr_db + 5.0) / 30.0));
  return static_cast<double>(snr_score);
}

/**
 * @brief 计算序列标准差。
 * @param[in] values 输入序列。
 * @return 标准差。
 */
double ComputeStandardDeviation(const std::vector<double>& values) {
  if (values.size() < 2U) {
    return 0.0;
  }
  double mean = 0.0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    mean += values[i];
  }
  mean /= static_cast<double>(values.size());
  double variance = 0.0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double diff = values[i] - mean;
    variance += diff * diff;
  }
  variance /= static_cast<double>(values.size() - 1U);
  return std::sqrt(std::max(variance, 0.0));
}

/**
 * @brief 计算并写入簇级频谱特征与标签。
 * @param[in] cluster_indices 簇样本索引。
 * @param[in] records 观测记录。
 * @param[in] spectral_config 频谱配置。
 * @param[out] summary 簇摘要。
 */
void PopulateClusterSpectralSummary(
    const std::vector<std::size_t>& cluster_indices,
    const std::vector<RawObservationRecord>& records,
    const extension::InterceptSpectralAnalysisConfig& spectral_config, ClusterSummary* summary) {
  if (summary == nullptr || !spectral_config.enable) {
    return;
  }

  if (cluster_indices.size() <
      static_cast<std::size_t>(std::max<std::uint32_t>(spectral_config.min_sequence_length, 1U))) {
    summary->spectral_class_label = "SPECTRAL_INSUFFICIENT";
    return;
  }

  std::vector<std::pair<double, double>> ordered_samples;
  ordered_samples.reserve(cluster_indices.size());
  for (std::size_t i = 0; i < cluster_indices.size(); ++i) {
    const std::size_t idx = cluster_indices[i];
    ordered_samples.push_back(
        std::make_pair(records[idx].observation.timestamp_s, records[idx].observation.rf_hz));
  }
  std::sort(ordered_samples.begin(), ordered_samples.end(),
            [](const std::pair<double, double>& lhs, const std::pair<double, double>& rhs) {
              return lhs.first < rhs.first;
            });

  std::vector<double> rf_series;
  rf_series.reserve(ordered_samples.size());
  for (std::size_t i = 0; i < ordered_samples.size(); ++i) {
    rf_series.push_back(ordered_samples[i].second);
  }

  const std::size_t fft_length =
      static_cast<std::size_t>(std::max<std::uint32_t>(spectral_config.fft_length, 4U));
  std::vector<double> power_spectrum;
  if (!oneq::common::numerics::ComputePeriodogram(rf_series, fft_length, &power_spectrum) ||
      power_spectrum.empty()) {
    summary->spectral_class_label = "SPECTRAL_INSUFFICIENT";
    return;
  }

  double total_power = 0.0;
  double max_power = 0.0;
  for (std::size_t i = 0; i < power_spectrum.size(); ++i) {
    total_power += power_spectrum[i];
    max_power = std::max(max_power, power_spectrum[i]);
  }
  total_power = std::max(total_power, 1.0e-12);
  max_power = std::max(max_power, 1.0e-12);

  const double occupancy_floor =
      static_cast<double>(utils::Clamp01(spectral_config.occupancy_peak_floor_ratio)) * max_power;
  std::size_t occupied_bins = 0U;
  for (std::size_t i = 0; i < power_spectrum.size(); ++i) {
    if (power_spectrum[i] >= occupancy_floor) {
      ++occupied_bins;
    }
  }

  summary->spectral_main_frequency_stability_hz =
      static_cast<float>(ComputeStandardDeviation(rf_series));
  summary->spectral_peak_sparsity = static_cast<float>(max_power / total_power);
  summary->spectral_bandwidth_occupancy = static_cast<float>(
      static_cast<double>(occupied_bins) / static_cast<double>(power_spectrum.size()));

  const bool is_broadband =
      summary->spectral_bandwidth_occupancy >= spectral_config.broadband_occupancy_threshold;
  const bool is_agile =
      summary->spectral_main_frequency_stability_hz >=
          spectral_config.agile_stability_threshold_hz ||
      summary->spectral_peak_sparsity <= spectral_config.agile_peak_sparsity_threshold;

  if (is_broadband) {
    summary->spectral_class_label = "SPECTRAL_BROADBAND";
  } else if (is_agile) {
    summary->spectral_class_label = "SPECTRAL_AGILE";
  } else {
    summary->spectral_class_label = "SPECTRAL_STABLE";
  }
}

/**
 * @brief 构造簇级摘要。
 * @param[in] cluster_indices 簇内观测索引。
 * @param[in] records 预处理后的观测记录。
 * @param[in] features 观测特征向量。
 * @param[in] spectral_config 频谱分析配置。
 * @return 簇摘要。
 */
ClusterSummary BuildClusterSummary(
    const std::vector<std::size_t>& cluster_indices,
    const std::vector<RawObservationRecord>& records,
    const std::vector<ObservationFeatureVector>& features,
    const extension::InterceptSpectralAnalysisConfig& spectral_config) {
  ClusterSummary summary;
  if (cluster_indices.empty()) {
    return summary;
  }

  summary.support_count = cluster_indices.size();
  std::size_t representative = cluster_indices[0];
  double representative_snr = records[representative].observation.snr_db;
  double confidence_acc = 0.0;
  double mean_snr_db = 0.0;
  double mean_az_deg = 0.0;
  double mean_el_deg = 0.0;
  double mean_rf_hz = 0.0;
  double mean_bandwidth_hz = 0.0;
  double mean_pulse_width_s = 0.0;
  double mean_pri_s = 0.0;
  double mean_rf_std_hz = 0.0;
  double mean_bandwidth_std_hz = 0.0;
  double mean_pri_std_s = 0.0;
  double mean_pulse_width_std_s = 0.0;

  for (std::size_t i = 0; i < cluster_indices.size(); ++i) {
    const std::size_t index = cluster_indices[i];
    for (std::size_t dim = 0; dim < kObservationFeatureDimension; ++dim) {
      summary.centroid_feature.values[dim] += features[index].values[dim];
    }
    mean_snr_db += records[index].observation.snr_db;
    mean_az_deg += records[index].observation.aoa_az_deg;
    mean_el_deg += records[index].observation.aoa_el_deg;
    mean_rf_hz += records[index].observation.rf_hz;
    mean_bandwidth_hz += records[index].observation.bandwidth_hz;
    mean_pulse_width_s += records[index].observation.pulse_width_s;
    mean_pri_s += records[index].observation.pri_s;
    mean_rf_std_hz += records[index].observation.rf_std_hz;
    mean_bandwidth_std_hz += records[index].observation.bandwidth_std_hz;
    mean_pri_std_s += records[index].observation.pri_std_s;
    mean_pulse_width_std_s += records[index].observation.pulse_width_std_s;
    confidence_acc += ComputeObservationConfidence(records[index].observation.snr_db);
    if (records[index].observation.snr_db > representative_snr) {
      representative = index;
      representative_snr = records[index].observation.snr_db;
    }
  }

  const float inv_count = 1.0f / static_cast<float>(cluster_indices.size());
  for (std::size_t dim = 0; dim < kObservationFeatureDimension; ++dim) {
    summary.centroid_feature.values[dim] *= inv_count;
  }
  summary.waveform_class = records[representative].observation.waveform_class;
  const double inv_count_d = static_cast<double>(inv_count);
  summary.mean_snr_db = static_cast<float>(mean_snr_db * inv_count_d);
  summary.mean_az_deg = static_cast<float>(mean_az_deg * inv_count_d);
  summary.mean_el_deg = static_cast<float>(mean_el_deg * inv_count_d);
  summary.mean_rf_hz = mean_rf_hz * inv_count_d;
  summary.mean_bandwidth_hz = mean_bandwidth_hz * inv_count_d;
  summary.mean_pulse_width_s = mean_pulse_width_s * inv_count_d;
  summary.mean_pri_s = mean_pri_s * inv_count_d;
  summary.rf_std_hz = mean_rf_std_hz * inv_count_d;
  summary.bandwidth_std_hz = mean_bandwidth_std_hz * inv_count_d;
  summary.pri_std_s = mean_pri_std_s * inv_count_d;
  summary.pulse_width_std_s = mean_pulse_width_std_s * inv_count_d;
  summary.confidence_score =
      utils::Clamp01(static_cast<float>(confidence_acc * inv_count_d));
  summary.representative_index = representative;
  PopulateClusterSpectralSummary(cluster_indices, records, spectral_config, &summary);
  return summary;
}

}  // namespace

extension::InterceptPipelineResult InterceptPostProcessingExecutor::Execute(
    const std::vector<RawObservationRecord>& raw_records, const MutableEsrContext& ctx,
    ObservationPreprocessor& preprocessor, KdTreeClusterer& clusterer,
    HypothesisAssociator& associator, const ObservationFeatureScales& feature_scales,
    std::uint64_t& next_hypothesis_id) {
  extension::InterceptPipelineResult result;
  result.observation_output.raw_observation_count = raw_records.size();

  const auto& config = ctx.GetPipelineConfig();

  // Preprocess
  const std::vector<RawObservationRecord> records =
      preprocessor.Run(raw_records, config.preprocess);
  // 中译：预处理前后记录数（原始 / 预处理后）。
  // 标识：预处理效果核对——原始观测经去重/合并等预处理后的数量变化。
  PROJECT_LOG_DEBUG("[InterceptPostProcess] raw={} preprocessed={}", raw_records.size(),
                    records.size());

  // Extract only declassified observations; RF v2 does not carry truth association.
  result.observation_output.observations.reserve(records.size());
  for (std::size_t i = 0; i < records.size(); ++i) {
    result.observation_output.observations.push_back(records[i].observation);
  }

  // Feature encoding
  std::vector<ObservationFeatureVector> features;
  features.reserve(records.size());
  for (std::size_t i = 0; i < records.size(); ++i) {
    features.push_back(ObservationFeatureEncoder::Encode(records[i].observation, feature_scales));
  }

  // Clustering — partition features by waveform class so that observations of
  // disjoint physical classes (pulse vs continuous/sweep/noise) never land in
  // the same cluster. KdTreeClusterer stays class-agnostic; the partition lives
  // here. Each bucket clusters independently, then local bucket indices are
  // remapped back to global record indices before merging into a single vector.
  std::vector<std::vector<std::size_t>> global_clusters;
  std::vector<std::size_t> class_order;
  class_order.push_back(static_cast<std::size_t>(session::EsrWaveformClass::kPulse));
  class_order.push_back(static_cast<std::size_t>(session::EsrWaveformClass::kContinuous));
  class_order.push_back(static_cast<std::size_t>(session::EsrWaveformClass::kSweep));
  class_order.push_back(static_cast<std::size_t>(session::EsrWaveformClass::kNoise));
  for (std::size_t class_value : class_order) {
    const auto current_class = static_cast<session::EsrWaveformClass>(class_value);
    std::vector<ObservationFeatureVector> bucket_features;
    std::vector<std::size_t> local_to_global;
    bucket_features.reserve(records.size());
    local_to_global.reserve(records.size());
    for (std::size_t i = 0U; i < records.size(); ++i) {
      if (records[i].observation.waveform_class == current_class) {
        bucket_features.push_back(features[i]);
        local_to_global.push_back(i);
      }
    }
    if (bucket_features.empty()) {
      continue;
    }
    KdTreeClusterResult bucket_result = clusterer.Cluster(bucket_features, config.cluster);
    for (std::size_t i = 0U; i < bucket_result.clusters.size(); ++i) {
      std::vector<std::size_t> global_cluster;
      global_cluster.reserve(bucket_result.clusters[i].size());
      for (std::size_t j = 0U; j < bucket_result.clusters[i].size(); ++j) {
        global_cluster.push_back(local_to_global[bucket_result.clusters[i][j]]);
      }
      global_clusters.push_back(std::move(global_cluster));
    }
    for (std::size_t i = 0U; i < bucket_result.noise_indices.size(); ++i) {
      std::vector<std::size_t> singleton(1U, local_to_global[bucket_result.noise_indices[i]]);
      global_clusters.push_back(std::move(singleton));
    }
  }
  std::sort(global_clusters.begin(), global_clusters.end(),
            [](const std::vector<std::size_t>& lhs, const std::vector<std::size_t>& rhs) {
              if (lhs.empty() || rhs.empty()) {
                return lhs.size() < rhs.size();
              }
              return lhs.front() < rhs.front();
            });
  KdTreeClusterResult cluster_result;
  cluster_result.clusters = std::move(global_clusters);
  result.observation_output.cluster_count = cluster_result.clusters.size();

  // Cluster summaries
  std::vector<ClusterSummary> cluster_summaries;
  cluster_summaries.reserve(cluster_result.clusters.size());
  for (std::size_t i = 0; i < cluster_result.clusters.size(); ++i) {
    cluster_summaries.push_back(
        BuildClusterSummary(cluster_result.clusters[i], records, features, config.spectral_analysis));
  }

  // Hypothesis association
  associator.UpdateConfig(config.association);
  result.emitter_output.hypotheses =
      associator.Update(ctx.GetCycleIndex(), cluster_summaries, &next_hypothesis_id);
  // 中译：周期后处理摘要（周期号、聚类数、假设数）。
  // 标识：截获处理概况——聚类与辐射源假设的数量，供核对关联效果。
  PROJECT_LOG_INFO("[InterceptPostProcess] cycle_index={} clusters={} hypotheses={}",
                   ctx.GetCycleIndex(), cluster_result.clusters.size(),
                   result.emitter_output.hypotheses.size());

  return result;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
