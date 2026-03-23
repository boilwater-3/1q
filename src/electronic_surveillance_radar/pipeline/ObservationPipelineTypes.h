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

#include "1q/electronic_surveillance_radar/common/EmitterObservation.h"

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
  float rf_scale_hz{5.0e6f}; /**< 载频尺度（单位：Hz） */
  float pw_scale_sec{1.0e-6f}; /**< 脉宽尺度（单位：s） */
  float az_scale_deg{2.0f}; /**< 方位尺度（单位：deg） */
  float el_scale_deg{2.0f}; /**< 俯仰尺度（单位：deg） */
  float snr_scale_db{8.0f}; /**< 信噪比尺度（单位：dB） */
};

/**
 * @brief RawObservationRecord 表示带真值评估上下文的观测记录。
 */
struct RawObservationRecord {
  common::EmitterObservation observation{}; /**< 观测记录 */
  std::string truth_emitter_id{}; /**< 真值辐射源标识 */
  bool matched_truth{true}; /**< 是否来自真实辐射源链路 */
};

/**
 * @brief ClusterSummary 表示聚类簇摘要信息。
 */
struct ClusterSummary {
  ObservationFeatureVector centroid_feature{}; /**< 簇中心特征 */
  std::size_t representative_index{0U}; /**< 代表观测索引 */
  std::size_t support_count{0U}; /**< 簇内样本数 */
  float mean_snr_db{0.0f}; /**< 簇均值信噪比（单位：dB） */
  float mean_az_deg{0.0f}; /**< 簇均值方位（单位：deg） */
  float mean_el_deg{0.0f}; /**< 簇均值俯仰（单位：deg） */
  double mean_rf_hz{0.0}; /**< 簇均值载频（单位：Hz） */
  double mean_pulse_width_s{0.0}; /**< 簇均值脉宽（单位：s） */
  float confidence_score{0.0f}; /**< 簇级置信度 */
  bool any_jammed{false}; /**< 簇内是否存在受扰观测 */
};

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_PIPELINE_TYPES_H_
