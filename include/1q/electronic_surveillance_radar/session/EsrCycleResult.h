/**
 * @file EsrCycleResult.h
 * @brief 定义电子侦察单周期输出帧与聚合结果类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_RESULT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_RESULT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
#include "1q/electronic_surveillance_radar/model/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/model/EmitterObservation.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"

namespace electronic_surveillance_radar {
namespace session {

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
  std::size_t raw_observation_count{0U};               /**< 预处理前原始检测记录数 */
  std::size_t cluster_count{0U};                       /**< 聚类后簇数 */
  model::EmitterObservationList observations{};        /**< 当前周期观测记录 */
};

/**
 * @brief EmitterOutputFrame 表示侦察输出通道。
 */
struct ONEQ_API EmitterOutputFrame {
  model::EmitterHypothesisList hypotheses{}; /**< 当前周期辐射源假设 */
};

/**
 * @brief TruthEvaluationFrame 表示真值评估输出通道。
 */
struct ONEQ_API TruthEvaluationFrame {
  TruthAssociationRecordList associations{}; /**< 观测与真值关联记录 */
};

/**
 * @brief EsrOutputFrame 表示电子侦察模块的三通道聚合输出。
 * @warning `emitter_output` 不应包含任何真值直通字段。
 */
struct ONEQ_API EsrOutputFrame {
  std::uint32_t cycle_index{0U};                 /**< 当前周期号 */
  std::uint64_t batch_id{0U};                    /**< 当前批次号 */
  ObservationOutputFrame observation_output{};    /**< 传感器观测输出通道 */
  EmitterOutputFrame emitter_output{};            /**< 侦察假设输出通道 */
  TruthEvaluationFrame truth_evaluation_output{}; /**< 真值评估输出通道 */
};

/**
 * @brief EsrCycleResult 描述电子侦察会话单周期聚合结果。
 */
struct ONEQ_API EsrCycleResult {
  EsrOutputFrame output_frame{};               /**< 当前周期输出帧 */
  session::ValidationIssueList validation_issues{}; /**< 当前周期输入校验结果 */
  bool has_validation_error{false};                    /**< 是否存在 error 级输入问题 */
  bool executed_this_cycle{false};                     /**< 当前调用是否真正执行了 pipeline */
  bool reused_previous_output{false};                  /**< 当前 output_frame 是否复用了上一有效周期输出 */
  extension::EsrPipelineAbortReason abort_reason{
      extension::EsrPipelineAbortReason::kNone};       /**< 若 downstream 链路 abort，给出结构化原因 */
};

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_RESULT_H_
