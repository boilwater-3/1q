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

  // ---- 航迹滤波（默认开启；置 false 时行为与 P2 前关联/置信度一致） ----
  bool enable_track_filtering{true};         /**< 是否启用逐航迹无迹滤波（关闭时无运动学估计） */
  double track_cycle_period_sec{1.0};        /**< 周期时长（滤波 dt = 周期差 × 该值，单位：s） */
  double track_process_noise{1.0};           /**< CV 过程噪声扩散系数 q（common/estimation 语义） */
  double track_initial_position_std_m{2000.0};      /**< 位置记录起始先验 1-σ（单位：m） */
  double track_initial_velocity_std_m_per_s{300.0}; /**< 速度起始先验 1-σ（单位：m/s） */
  double track_bearing_init_range_m{100000.0};      /**< 方位+原点记录起始距离先验（沿 LOS，单位：m） */
  double track_bearing_init_range_std_m{300000.0};  /**< 方位+原点起始距离先验 1-σ（各向同性，单位：m） */
  double default_position_noise_std_m{50.0};        /**< 位置量测噪声默认 1-σ（单位：m） */
  double default_bearing_noise_sigma_rad{2.0e-4};   /**< 方位量测噪声默认 1-σ（单位：rad，az/el 同 σ） */
  std::size_t confirm_hits{3U};              /**< 航迹确认门：累计命中数达该值转 confirmed */
};

}  // namespace fusion

#endif  // ONEQ_FUSION_FUSION_CONFIG_H_
