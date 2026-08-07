/**
 * @file EsrOutputTypes.h
 * @brief 定义电子侦察公共输出通道与周期终止原因类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_TYPES_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/session/EmitterObservation.h"
#include "1q/foundation/validation_types.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief ObservationOutputFrame 表示观测输出通道。
 */
struct ONEQ_API ObservationOutputFrame {
  std::size_t raw_observation_count{0U};
  std::size_t cluster_count{0U};
  double receiver_center_frequency_hz{0.0}; /**< 本周期接收调谐中心频率（单位：Hz）。 */
  double receiver_bandwidth_hz{0.0};        /**< 本周期接收调谐带宽（单位：Hz）。 */
  bool receiver_saturated{false};           /**< 本周期是否超过最大线性输入功率。 */
  session::EmitterObservationList observations{};
};

/**
 * @brief EmitterOutputFrame 表示侦察输出通道。
 */
struct ONEQ_API EmitterOutputFrame {
  session::EmitterHypothesisList hypotheses{};
};

/**
 * @brief ESR 问题条目严重等级。
 */
enum class ONEQ_API EsrIssueSeverity : std::uint8_t {
  kInfo = 0,    /**< 信息 */
  kWarning = 1, /**< 警告 */
  kError = 2    /**< 错误 */
};

/**
 * @brief ESR 问题条目来源阶段（统一问题列表模型，session_contract.md 规则 14）。
 * @note 结构化来源判别字段；状态判断仍以 `status`/`abort_reason` 为准，phase 不改变状态语义。
 */
enum class ONEQ_API EsrIssuePhase : std::uint8_t {
  kInputValidation = 0, /**< 输入校验阶段（调用方输入问题） */
  kExecution = 1,       /**< 管线执行阶段（含关机等运行态条件） */
  kOutputContract = 2   /**< 输出违反内部契约（ESR 当前不产出，契约三值预留） */
};

/**
 * @brief EsrIssue 描述单周期问题条目（统一问题列表模型，session_contract.md 规则 14）。
 * @note 承载输入校验问题（phase=kInputValidation）与执行诊断（phase=kExecution/kOutputContract）；
 *       code 带模块前缀（如 "esr.rf_receiver_rejected"、"esr.validation.invalid_cycle_delta_time"），
 *       机器消费只认 code；不用于调用方状态判断。
 */
struct ONEQ_API EsrIssue {
  EsrIssueSeverity severity{EsrIssueSeverity::kInfo};
  EsrIssuePhase phase{EsrIssuePhase::kExecution};
  std::string code{};
  std::string message{};
  oneq::foundation::ValidationLocation location{}; /**< 可选定位；kind==kGlobal 表示无定位 */
  std::string field{}; /**< 可选定位；为空表示无关联字段（跨字段或域级问题） */
};

/** @brief ESR 问题条目列表。 */
using EsrIssueList = std::vector<EsrIssue>;

/**
 * @brief EsrPipelineAbortReason 表示单周期核心管线流产原因。
 * @note 枚举编号 2/3 空缺为 replay 兼容保留（已删除的两个旧 abort 值，
 *       解码侧 IsValidAbortReason 按显式白名单拒绝）。
 */
enum class EsrPipelineAbortReason {
  kNone = 0,             /**< 正常执行完成，未中断 */
  kValidationRejected,   /**< 因输入级严重校验问题（Error）而主动放弃计算 */
  kSensorPoweredOff = 4, /**< 设备关机，pipeline 未执行 */
  kRfReceiverRejected = 5 /**< RF v2 接收机链路前置条件不成立 */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_TYPES_H_
