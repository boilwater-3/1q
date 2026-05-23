/**
 * @file EmitterHypothesis.h
 * @brief 定义电子侦察输出侧的辐射源假设与威胁评估类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_MODEL_EMITTER_HYPOTHESIS_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_MODEL_EMITTER_HYPOTHESIS_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace model {

/**
 * @brief EsrEmitterMode 表示辐射源工作模式假设。
 */
enum class ONEQ_API EsrEmitterMode {
  kUnknown = 0, /**< 未知模式 */
  kSearch,      /**< 搜索模式 */
  kTracking,    /**< 跟踪模式 */
  kGuidance     /**< 制导模式 */
};

/**
 * @brief EsrThreatLevel 表示威胁等级。
 */
enum class ONEQ_API EsrThreatLevel {
  kLow = 0, /**< 低威胁 */
  kMedium,  /**< 中威胁 */
  kHigh     /**< 高威胁 */
};

/**
 * @brief EmitterHypothesis 描述单个辐射源假设。
 * @note 该结构不应包含场景真值标识字段。
 */
struct ONEQ_API EmitterHypothesis {
  std::uint64_t hypothesis_id{0U};              /**< 假设记录唯一标识 */
  std::vector<std::string> candidate_classes{}; /**< 候选类别列表（按置信度降序） */
  EsrEmitterMode mode{EsrEmitterMode::kUnknown};      /**< 工作模式假设 */
  EsrThreatLevel threat_level{EsrThreatLevel::kLow};  /**< 威胁等级 */
  float bearing_az_deg{0.0f};                   /**< 方位线方位角（单位：deg） */
  float bearing_el_deg{0.0f};                   /**< 方位线俯仰角（单位：deg） */
  float bearing_std_deg{0.0f};                  /**< 方位测量标准差（单位：deg） */
  float confidence{0.0f};                       /**< 假设置信度，范围 [0, 1] */
  std::uint32_t last_seen_cycle{0U};            /**< 最近命中周期号 */
};

/** @brief EmitterHypothesisList 表示辐射源假设列表。 */
using EmitterHypothesisList = std::vector<EmitterHypothesis>;

}  // namespace model
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_MODEL_EMITTER_HYPOTHESIS_H_
