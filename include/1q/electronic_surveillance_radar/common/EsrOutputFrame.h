/**
 * @file EsrOutputFrame.h
 * @brief 定义电子侦察单周期输出帧及其三通道结构。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_COMMON_ESR_OUTPUT_FRAME_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_COMMON_ESR_OUTPUT_FRAME_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/common/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/common/EmitterObservation.h"

namespace electronic_surveillance_radar {
namespace common {

/**
 * @brief TruthAssociationRecord 表示观测记录与真值辐射源的评估关联。
 */
struct ONEQ_API TruthAssociationRecord {
  std::uint64_t observation_id{0U}; /**< 观测记录标识 */
  std::string truth_emitter_id{};   /**< 真值辐射源标识 */
  bool matched{false};              /**< 该观测是否匹配到真值辐射源 */
  float confidence{0.0f};           /**< 评估关联置信度，范围 [0, 1] */
};

/** @brief TruthAssociationRecordList 表示评估关联记录列表。 */
using TruthAssociationRecordList = std::vector<TruthAssociationRecord>;

/**
 * @brief ObservationOutputFrame 表示观测输出通道。
 */
struct ONEQ_API ObservationOutputFrame {
  std::uint32_t cycle_index{0U};         /**< 当前周期号 */
  std::uint64_t batch_id{0U};            /**< 当前批次号 */
  EmitterObservationList observations{}; /**< 当前周期观测记录 */
};

/**
 * @brief EmitterOutputFrame 表示侦察输出通道。
 */
struct ONEQ_API EmitterOutputFrame {
  std::uint32_t cycle_index{0U};      /**< 当前周期号 */
  std::uint64_t batch_id{0U};         /**< 当前批次号 */
  EmitterHypothesisList hypotheses{}; /**< 当前周期辐射源假设 */
};

/**
 * @brief TruthEvaluationFrame 表示真值评估输出通道。
 */
struct ONEQ_API TruthEvaluationFrame {
  std::uint32_t cycle_index{0U};             /**< 当前周期号 */
  std::uint64_t batch_id{0U};                /**< 当前批次号 */
  TruthAssociationRecordList associations{}; /**< 观测与真值关联记录 */
  std::size_t matched_count{0U};             /**< 匹配成功的观测数 */
  std::size_t total_observation_count{0U};   /**< 参与评估的观测总数 */
};

/**
 * @brief EsrOutputFrame 表示电子侦察模块的三通道聚合输出。
 * @warning `emitter_output` 不应包含任何真值直通字段。
 */
struct ONEQ_API EsrOutputFrame {
  ObservationOutputFrame observation_output{};    /**< 传感器观测输出通道 */
  EmitterOutputFrame emitter_output{};            /**< 侦察假设输出通道 */
  TruthEvaluationFrame truth_evaluation_output{}; /**< 真值评估输出通道 */
};

}  // namespace common
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_COMMON_ESR_OUTPUT_FRAME_H_
