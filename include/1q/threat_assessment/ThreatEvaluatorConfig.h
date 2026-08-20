/**
 * @file ThreatEvaluatorConfig.h
 * @brief 定义威胁评估配置（属性权重与归一化参考值）。
 */

#ifndef ONEQ_THREAT_ASSESSMENT_THREAT_EVALUATOR_CONFIG_H_
#define ONEQ_THREAT_ASSESSMENT_THREAT_EVALUATOR_CONFIG_H_

#include "1q/api.hpp"

namespace threat_assessment {

/**
 * @brief 威胁评估配置（归一化加权和）。
 * @details 每个属性经分段线性归一化到 [0,1] 后按权重加权求和得威胁分。
 *          权重按**相对值**解释：评估器内部钳制非负后归一化（Σ 权重 = 1），
 *          配置时无需人为保证权重和为 1。归一化断点语义：
 *          距离 ≤ range_near_m → 1.0、≥ range_far_m → 0.0（越近越危险）；
 *          速度 ≤ speed_min_mps → 0.0、≥ speed_max_mps → 1.0（越快越危险）；
 *          加速度 ≥ acceleration_max_mps2 → 1.0（线性 0→1，机动越大越危险）；
 *          RCS ≤ rcs_min_sqm → 0.0、≥ rcs_max_sqm → 1.0（越大越危险）；
 *          类型概率与融合置信度直通 [0,1]（融合置信度先钳制）。
 */
struct ONEQ_API ThreatEvaluatorConfig {
  float weight_range{0.30f};               /**< 距离属性权重（默认最大） */
  float weight_speed{0.25f};               /**< 速度属性权重 */
  float weight_acceleration{0.10f};        /**< 加速度属性权重 */
  float weight_rcs{0.10f};                 /**< RCS 属性权重 */
  float weight_target_probability{0.15f};  /**< 类型概率属性权重 */
  float weight_fusion_confidence{0.10f};   /**< 融合置信度属性权重 */

  float range_near_m{20000.0f};     /**< 距离归一化上断点（单位：m） */
  float range_far_m{200000.0f};     /**< 距离归一化下断点（单位：m） */
  float speed_min_mps{50.0f};       /**< 速度归一化下断点（单位：m/s） */
  float speed_max_mps{500.0f};      /**< 速度归一化上断点（单位：m/s） */
  float acceleration_max_mps2{50.0f}; /**< 加速度归一化上断点（单位：m/s^2） */
  float rcs_min_sqm{0.5f};          /**< RCS 归一化下断点（单位：m^2） */
  float rcs_max_sqm{10.0f};         /**< RCS 归一化上断点（单位：m^2） */

  float high_threshold{0.70f};      /**< 威胁等级 HIGH 阈值（威胁分 ≥ 此值） */
  float medium_threshold{0.40f};    /**< 威胁等级 MEDIUM 阈值（威胁分 ≥ 此值且 < HIGH 阈值） */
};

}  // namespace threat_assessment

#endif  // ONEQ_THREAT_ASSESSMENT_THREAT_EVALUATOR_CONFIG_H_
