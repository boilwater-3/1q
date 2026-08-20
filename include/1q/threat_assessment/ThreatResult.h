/**
 * @file ThreatResult.h
 * @brief 定义威胁评估输出结果。
 */

#ifndef ONEQ_THREAT_ASSESSMENT_THREAT_RESULT_H_
#define ONEQ_THREAT_ASSESSMENT_THREAT_RESULT_H_

#include <cstdint>

#include "1q/api.hpp"

namespace threat_assessment {

/**
 * @brief 威胁等级（按等级递增排序：低 < 中 < 高，供升级判定比较）。
 */
enum class ONEQ_API ThreatLevel {
  kLow = 0,    /**< 低威胁 */
  kMedium,     /**< 中威胁 */
  kHigh        /**< 高威胁 */
};

/**
 * @brief 每属性归一化贡献分解（可解释性：威胁分由哪些属性构成）。
 * @note 当配置的权重和为 1 时，六项贡献之和等于威胁分。
 */
struct ONEQ_API AttributeContribution {
  float range{0.0f};               /**< 距离贡献（权重 × 距离归一化值） */
  float speed{0.0f};               /**< 速度贡献 */
  float acceleration{0.0f};        /**< 加速度贡献 */
  float rcs{0.0f};                 /**< RCS 贡献 */
  float target_probability{0.0f};  /**< 类型概率贡献 */
  float fusion_confidence{0.0f};   /**< 融合置信度贡献 */
};

/**
 * @brief 单目标威胁评估结果。
 */
struct ONEQ_API ThreatResult {
  std::uint64_t key{0U};                /**< 目标库内键（与输入对齐） */
  float threat_score{0.0f};             /**< 威胁分（单位：1，[0,1]） */
  ThreatLevel level{ThreatLevel::kLow}; /**< 威胁等级（按配置阈值映射） */
  AttributeContribution contributions{}; /**< 每属性贡献分解 */
};

}  // namespace threat_assessment

#endif  // ONEQ_THREAT_ASSESSMENT_THREAT_RESULT_H_
