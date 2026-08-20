/**
 * @file EosOutputTypes.h
 * @brief EOS 公共输出记录、归属记录与周期终止原因类型。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_TYPES_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"

namespace electro_optical_sensor {
namespace output {

/**
 * @brief EosDetectionRecord 表示单条 EOS 传感器探测输出。
 */
struct ONEQ_API EosDetectionRecord {
  std::uint64_t detection_id{0U};  /**< 本输出帧内的探测记录标识 */
  float range_m{0.0f};             /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};         /**< 方位角（单位：deg） */
  float elevation_deg{0.0f};       /**< 仰角（单位：deg） */
  float infrared_snr_linear{0.0f}; /**< 红外通道线性 SNR */
  float visible_snr_linear{0.0f};  /**< 可见光通道线性 SNR */
  float fused_snr_linear{0.0f};    /**< 融合线性 SNR */
  float fused_snr_db{0.0f};        /**< 融合 dB SNR */
  bool detected{false};            /**< 是否通过探测门限判决 */
};

/** @brief EosDetectionRecordList 表示单周期探测结果列表。 */
using EosDetectionRecordList = std::vector<EosDetectionRecord>;

}  // namespace output

namespace attribution {

/**
 * @brief EosDetectionAttributionRecord 表示仿真归属映射。
 * @note 该类型不属于真实传感器输出；只用于 StepWithResult、调试视图、生命周期和 replay 诊断。
 */
struct ONEQ_API EosDetectionAttributionRecord {
  std::uint64_t detection_id{0U}; /**< 对应 EosDetectionRecord::detection_id */
  std::uint64_t target_id{0U};    /**< 仿真输入目标 ID */
  std::string target_name{};      /**< 仿真输入目标名称 */
};

/** @brief EosDetectionAttributionRecordList 表示探测记录到仿真目标的归属映射集合。 */
using EosDetectionAttributionRecordList = std::vector<EosDetectionAttributionRecord>;

}  // namespace attribution

namespace session {

/**
 * @brief EOS 问题条目严重等级。
 */
enum class ONEQ_API EosIssueSeverity : std::uint8_t {
  kInfo = 0,    /**< 信息 */
  kWarning = 1, /**< 警告 */
  kError = 2    /**< 错误 */
};

/**
 * @brief EOS 问题条目来源阶段（统一问题列表模型，session_contract.md 规则 14）。
 * @note 结构化来源判别字段；状态判断仍以 `status`/`abort_reason` 为准，phase 不改变状态语义。
 */
enum class ONEQ_API EosIssuePhase : std::uint8_t {
  kInputValidation = 0, /**< 输入校验阶段（调用方输入问题） */
  kExecution = 1,       /**< 管线执行阶段（含关机等运行态条件） */
  kOutputContract = 2   /**< 输出违反内部契约 */
};

/**
 * @brief EOS 问题条目门内归因（规则 13b 门内归因条款，session_contract.md）。
 * @note 仅排除诊断（"eos.target_out_of_fov"）使用：视场门排除时标识越界轴；
 *       不替代 code（机器键仍只认 code），不用于状态判断。
 */
enum class ONEQ_API EosIssueCause : std::uint8_t {
  kNone = 0,        /**< 无归因（非排除诊断） */
  kAzOutside,       /**< 仅方位越出视场 */
  kElOutside,       /**< 仅俯仰越出视场 */
  kBothAxesOutside, /**< 方位与俯仰均越出视场 */
  kUnknown          /**< 无法判定主因 */
};

/**
 * @brief EosIssue 描述单周期问题条目（统一问题列表模型，session_contract.md 规则 14）。
 * @note 承载输入校验问题（phase=kInputValidation）与执行诊断（phase=kExecution/kOutputContract）；
 *       code 带模块前缀（如 "eos.pipeline_contract_violation"、
 *       "eos.validation.invalid_target_range"），机器消费只认 code；不用于调用方状态判断。
 */
struct ONEQ_API EosIssue {
  EosIssueSeverity severity{EosIssueSeverity::kInfo};
  EosIssuePhase phase{EosIssuePhase::kExecution};
  std::string code{};
  std::string message{};
  oneq::foundation::ValidationLocation location{}; /**< 可选定位；kind==kGlobal 表示无定位 */
  std::string field{}; /**< 可选定位；为空表示无关联字段（跨字段或域级问题） */
  EosIssueCause cause{EosIssueCause::kNone}; /**< 可选归因；仅排除诊断使用（规则 13b） */
};

/** @brief EOS 问题条目列表。 */
using EosIssueList = std::vector<EosIssue>;

/**
 * @brief EosPipelineAbortReason 描述核心管线周期终止原因。
 */
enum class ONEQ_API EosPipelineAbortReason {
  kNone = 0,
  kValidationRejected,
  kOutputContractViolation,
  kRuntimeStateRestoreRejected,
  kSensorPoweredOff /**< 设备关机，核心 pipeline 未执行 */
};

/**
 * @brief EosCycleStatus 描述单周期高层执行状态。
 * @note 与 ArCycleStatus / EsrCycleExecutionStatus 对齐的强类型枚举。
 */
enum class ONEQ_API EosCycleStatus : std::uint8_t {
  kCompleted = 0,           /**< 周期正常完成 */
  kPoweredOff,              /**< 设备关机，核心 pipeline 未执行 */
  kRejectedInvalidInput,    /**< 输入校验失败 */
  kRejectedExecution        /**< 执行失败（含输出契约违规、运行时状态恢复失败） */
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_TYPES_H_
