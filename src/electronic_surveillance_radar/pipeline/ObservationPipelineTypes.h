/**
 * @file ObservationPipelineTypes.h
 * @brief 定义 ESR 观测处理链内部共享类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_PIPELINE_TYPES_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_PIPELINE_TYPES_H_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "1q/electronic_surveillance_radar/model/EmitterObservation.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

/** @brief 特征向量维度常量。 */
static const std::size_t kObservationFeatureDimension = 5U;

/**
 * @brief ObservationFeatureVector 表示观测特征向量。
 */
struct ObservationFeatureVector {
  std::array<float, kObservationFeatureDimension> values{}; /**< 特征值数组 */
};

/**
 * @brief ObservationFeatureScales 描述特征归一化尺度。
 */
struct ObservationFeatureScales {
  float rf_scale_hz{5.0e6f};   /**< 载频尺度（单位：Hz） */
  float pw_scale_sec{1.0e-6f}; /**< 脉宽尺度（单位：s） */
  float az_scale_deg{2.0f};    /**< 方位尺度（单位：deg） */
  float el_scale_deg{2.0f};    /**< 俯仰尺度（单位：deg） */
  float snr_scale_db{8.0f};    /**< 信噪比尺度（单位：dB） */
};

/**
 * @brief RawObservationRecord 表示带真值评估上下文的观测记录。
 */
struct RawObservationRecord {
  model::EmitterObservation observation{}; /**< 观测记录 */
  std::uint64_t truth_emitter_id{0U};      /**< 真值辐射源标识 */
  double truth_pri_s{0.0};                  /**< 真值 PRI（单位：s），伪观测为 0 */
  bool matched_truth{true};                 /**< 是否来自真实辐射源链路 */
  bool deception_affected{false};           /**< 是否受到欺骗分量影响 */
  bool synthetic_false_alarm{false};        /**< 是否为欺骗注入的伪观测 */
};

/**
 * @brief ClusterSummary 表示聚类簇摘要信息。
 */
struct ClusterSummary {
  ObservationFeatureVector centroid_feature{}; /**< 簇中心特征 */
  std::size_t representative_index{0U};        /**< 代表观测索引 */
  std::size_t support_count{0U};               /**< 簇内样本数 */
  float mean_snr_db{0.0f};                     /**< 簇均值信噪比（单位：dB） */
  float mean_az_deg{0.0f};                     /**< 簇均值方位（单位：deg） */
  float mean_el_deg{0.0f};                     /**< 簇均值俯仰（单位：deg） */
  double mean_rf_hz{0.0};                      /**< 簇均值载频（单位：Hz） */
  double mean_pulse_width_s{0.0};              /**< 簇均值脉宽（单位：s） */
  double mean_pri_s{0.0};                      /**< 簇均值 PRI（单位：s） */
  float confidence_score{0.0f};                /**< 簇级置信度 */
  float deception_support_ratio{0.0f};         /**< 簇内欺骗受影响样本占比，范围 [0, 1] */
  float false_alarm_ratio{0.0f};               /**< 簇内伪观测占比，范围 [0, 1] */
  bool any_jammed{false};                      /**< 簇内是否存在受扰观测 */
  float spectral_main_frequency_stability_hz{0.0f}; /**< 主频稳定度（单位：Hz） */
  float spectral_peak_sparsity{0.0f};               /**< 谱峰稀疏度，范围 [0, 1] */
  float spectral_bandwidth_occupancy{0.0f};         /**< 带宽占用度，范围 [0, 1] */
  std::string spectral_class_label{};               /**< 频谱分类标签 */
};

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_PIPELINE_TYPES_H_
