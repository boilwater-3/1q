/**
 * @file FusionConfig.h
 * @brief 定义多源融合配置（权重/门限/窗口）。
 */

#ifndef ONEQ_FUSION_FUSION_CONFIG_H_
#define ONEQ_FUSION_FUSION_CONFIG_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace fusion {

/**
 * @brief 多源融合配置。
 * @note 融合置信度 = Σ 判决值 × 质量归一化 × 权重（滑窗内精确求和，不归一化）；
 *       权重按 source_id 索引，缺失时按 1.0 计。
 */
struct ONEQ_API FusionConfig {
  double position_radius_m{1000.0};    /**< 空间门限（带位置记录，单位：m） */
  double bearing_beamwidth_deg{5.0};   /**< 方位相干门限（仅方位记录，单位：deg） */
  double feature_threshold{0.0};       /**< 特征相似度门限（≤ 0 = 不启用特征门） */
  std::size_t window_size{10U};        /**< 每源每航迹滑窗量测数（超出驱逐并重算置信度） */
  std::size_t max_missed_cycles{5U};   /**< 失跟删除周期数（连续无量测超过该值删除航迹） */
  std::vector<double> source_weights{}; /**< 按 source_id 索引的源权重（空或缺项 = 1.0） */
};

}  // namespace fusion

#endif  // ONEQ_FUSION_FUSION_CONFIG_H_
