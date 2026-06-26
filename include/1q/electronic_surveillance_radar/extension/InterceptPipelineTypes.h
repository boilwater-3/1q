/**
 * @file InterceptPipelineTypes.h
 * @brief 定义电子侦察公共输出通道与周期终止原因类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_INTERCEPT_PIPELINE_TYPES_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_INTERCEPT_PIPELINE_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/model/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/model/EmitterObservation.h"

namespace electronic_surveillance_radar {
namespace extension {

/**
 * @brief TruthAssociationRecord 表示观测记录与真值辐射源的评估关联。
 */
struct ONEQ_API TruthAssociationRecord {
  std::uint64_t observation_id{0U};
  std::uint64_t truth_emitter_id{0U};
  bool matched{false};
  float confidence{0.0f};
};

/** @brief TruthAssociationRecordList 表示评估关联记录列表。 */
using TruthAssociationRecordList = std::vector<TruthAssociationRecord>;

/**
 * @brief ObservationOutputFrame 表示观测输出通道。
 */
struct ONEQ_API ObservationOutputFrame {
  std::size_t raw_observation_count{0U};
  std::size_t cluster_count{0U};
  model::EmitterObservationList observations{};
};

/**
 * @brief EmitterOutputFrame 表示侦察输出通道。
 */
struct ONEQ_API EmitterOutputFrame {
  model::EmitterHypothesisList hypotheses{};
};

/**
 * @brief TruthEvaluationFrame 表示真值评估输出通道。
 */
struct ONEQ_API TruthEvaluationFrame {
  TruthAssociationRecordList associations{};
};

/**
 * @brief EsrPipelineAbortReason 表示单周期核心管线流产原因。
 */
enum class EsrPipelineAbortReason {
  kNone = 0,                    /**< 正常执行完成，未中断 */
  kValidationRejected,          /**< 因输入级严重校验问题（Error）而主动放弃计算 */
  kRuntimeStateRestoreRejected, /**< 因运行时状态回滚失败引发阻断 */
  kOutputContractViolation      /**< 下游计算返回的契约非法或状态错乱 */
};

}  // namespace extension
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_INTERCEPT_PIPELINE_TYPES_H_
