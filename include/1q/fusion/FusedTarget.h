/**
 * @file FusedTarget.h
 * @brief 定义融合目标态势输出记录。
 */

#ifndef ONEQ_FUSION_FUSED_TARGET_H_
#define ONEQ_FUSION_FUSED_TARGET_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace fusion {

/**
 * @brief 单源通道探测状态与最近量测。
 */
struct ONEQ_API ChannelMeasurement {
  std::uint32_t source_id{0U};                 /**< 源通道标识 */
  std::size_t sample_count{0U};                /**< 滑窗内该源量测数 */
  double latest_verdict{0.0};                  /**< 最近一次判决值 */
  double latest_quality{0.0};                  /**< 最近一次质量归一化值 */
  bool has_position{false};                    /**< 该源是否提供位置量测 */
  oneq::coordinate::LlaPositionDegM position{}; /**< 最近一次位置量测 */
  bool has_bearing{false};                     /**< 该源是否提供方位量测 */
  double bearing_az_deg{0.0};                  /**< 最近一次方位角（单位：deg） */
  double bearing_el_deg{0.0};                  /**< 最近一次俯仰角（单位：deg） */
};

/**
 * @brief 融合目标态势记录。
 */
struct ONEQ_API FusedTarget {
  std::uint64_t key{0U};                  /**< 航迹库内键（身份键，或引擎为无身份航迹合成的键） */
  std::vector<ChannelMeasurement> channels{}; /**< 各源探测状态与量测 */
  double confidence{0.0};                 /**< 融合置信度（滑窗内 Σ 判决值 × 质量 × 权重） */
  std::uint64_t last_update_cycle{0U};    /**< 最近一次收到量测的周期号 */
};

}  // namespace fusion

#endif  // ONEQ_FUSION_FUSED_TARGET_H_
