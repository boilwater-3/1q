/**
 * @file SarCycleResult.h
 * @brief 定义 SAR 会话单周期输出与诊断结果。
 */

#ifndef ONEQ_SAR_SESSION_SAR_CYCLE_RESULT_H_
#define ONEQ_SAR_SESSION_SAR_CYCLE_RESULT_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"

namespace sar {
namespace session {

/**
 * @brief SAR 管线周期终止原因（强类型枚举）。
 * @note 粗粒度结构性原因，与 AR/ESR/EOS/SBIRS 对齐（~6 值）。
 *       细粒度失败信息由 `SarIssue::code`（如 "sar.snr_below_minimum"）
 *       和 `PROJECT_LOG_ERROR` 双写承载，不进入 public abort_reason。
 */
enum class SarPipelineAbortReason : std::uint16_t {
  kNone = 0,                    /**< 正常执行，无中止 */
  kValidationRejected,          /**< 输入/配置校验失败 */
  kPipelineExecutionFailed,     /**< 管线内部执行失败 */
  kExternalInputRejected,       /**< 外部原始 IQ 输入校验失败 */
  kRuntimeStateRestoreRejected, /**< 运行时状态恢复失败 */
  kSensorPoweredOff             /**< 设备关机：管线短路，本周期未执行（COMMON-OQ-4 字段提升） */
};

/**
 * @brief SarCycleStatus 描述单周期高层执行状态。
 * @note 与 ArCycleStatus / EsrCycleExecutionStatus / EosCycleStatus 对齐。
 */
enum class SarCycleStatus : std::uint8_t {
  kCompleted = 0,           /**< 周期正常完成 */
  kRejectedInvalidInput,    /**< 输入/配置校验失败 */
  kRejectedExecution,       /**< 执行失败 */
  kPoweredOff               /**< 设备关机：本周期未执行，输出为默认空帧 */
};

/**
 * @brief SAR 处理阶段状态。
 */
enum class SarProcessingStage {
  kNone = 0,
  kRawEcho = 1,
  kL1RdaImage = 2,
  kL3BpImage = 3
};

/**
 * @brief SAR 问题严重等级。
 */
enum class SarIssueSeverity { kInfo = 0, kWarning = 1, kError = 2 };

/**
 * @brief SAR 问题来源阶段标签（统一问题列表模型，session_contract.md 规则 14b）。
 * @note phase 是结构化来源判别字段；状态判断仍以 `status`/`abort_reason` 为准，
 *       phase 不改变状态语义。
 */
enum class SarIssuePhase : std::uint8_t {
  kInputValidation = 0, /**< 输入校验阶段：调用方输入/运行期配置问题 */
  kExecution = 1,       /**< 管线执行阶段：含关机等运行态条件 */
  kOutputContract = 2   /**< 输出违反内部契约 */
};

/**
 * @brief 公共聚焦图像的生成来源。
 */
enum class SarFocusedImageSource { kNone = 0, kL1Rda = 1, kL3Bp = 2 };

/** @brief 原始相位历史的执行来源。 */
enum class SarRawPhaseHistorySource {
  kNone = 0,
  kInternallyGenerated = 1,
  kExternalRawIq = 2
};

/**
 * @brief SAR 聚焦图像相位参考摘要。
 */
enum class SarPhaseReferenceMode { kNative = 0, kCenterBroadside = 1 };

/**
 * @brief SAR 图像质量主瓣判定方法摘要。
 */
enum class SarMainlobeEstimationMethod { k3dB = 0, k20dB = 1 };

/**
 * @brief SAR 问题条目门内归因（规则 13b 门内归因条款，session_contract.md）。
 * @note SAR 无逐目标门控排除（13b 空洞条款，集体成像模型），本模块恒 kNone；
 *       枚举仅为五模块 `*Issue` 结构逐字同构保留。
 */
enum class SarIssueCause : std::uint8_t {
  kNone = 0, /**< 无归因 */
  kUnknown   /**< 无法判定主因（本模块不使用） */
};

/**
 * @brief SAR 统一问题条目（规则 14）。
 * @note `location.kind == kGlobal` 或 `field` 为空表示无定位；定位只服务人读与
 *       replay 保真，不用于状态判断。
 */
struct ONEQ_API SarIssue {
  SarIssueSeverity severity{SarIssueSeverity::kInfo}; /**< 问题严重级别 */
  SarIssuePhase phase{SarIssuePhase::kExecution};     /**< 来源阶段标签 */
  std::string code{};                                 /**< 结构化码（带 "sar." 前缀） */
  std::string message{};                              /**< 面向调用方的人读说明 */
  oneq::foundation::ValidationLocation location{};    /**< 可选定位（kGlobal=无） */
  std::string field{};                                /**< 触发问题的字段名；为空表示无定位 */
  SarIssueCause cause{SarIssueCause::kNone};          /**< 可选归因；SAR 恒 kNone（13b 空洞条款） */
};

using SarIssueList = std::vector<SarIssue>;

/**
 * @brief 与内部矩阵实现解耦的行主序复数聚焦图像。
 * @note Cycle-result replay 完整保存来源、尺寸、占位状态与复数像素，并按精确值比较。
 *       `row_count` 对应方位向（L1 RDA = `azimuth_pulse_count`，L3 BP = `azimuth_pulse_count`），
 *       `column_count` 对应距离向（= `range_sample_count`）。
 */
struct ONEQ_API SarFocusedImage {
  SarFocusedImageSource source{SarFocusedImageSource::kNone};
  std::uint32_t row_count{0U};
  std::uint32_t column_count{0U};
  std::vector<double> real_values{};
  std::vector<double> imaginary_values{};
  bool is_placeholder{false};
};

/**
 * @brief 实际进入本周期处理链的完整孔径原始相位历史。
 * @note I/Q 向量均为 pulse-major 行主序，长度必须等于
 *       `pulse_count * samples_per_pulse`。
 */
struct ONEQ_API SarRawPhaseHistory {
  SarRawPhaseHistorySource source{SarRawPhaseHistorySource::kNone};
  std::uint32_t pulse_count{0U};
  std::uint32_t samples_per_pulse{0U};
  std::vector<double> i_values{};
  std::vector<double> q_values{};
};

/**
 * @brief SAR 输出帧元数据。
 * @note `range_sample_count`、`azimuth_pulse_count`、`center_slant_range_m` 是配置回显
 *       （从 `SarMissionConfig` 拷贝），非测量值。
 */
struct ONEQ_API SarOutputFrame {
  std::uint32_t cycle_index{0U};
  SarProcessingStage completed_stage{SarProcessingStage::kNone};
  std::uint32_t range_sample_count{0U};
  std::uint32_t azimuth_pulse_count{0U};
  double center_slant_range_m{0.0};
  double estimated_snr_db{0.0};
  SarPhaseReferenceMode phase_reference_mode{SarPhaseReferenceMode::kNative};
  SarMainlobeEstimationMethod image_quality_mainlobe_method{SarMainlobeEstimationMethod::k3dB};
  double range_width_3db_bins{0.0};
  double azimuth_width_3db_bins{0.0};
  double range_resolution_3db_m{0.0};
  double azimuth_resolution_3db_m{0.0};
  double image_entropy_nats{0.0};
  double image_contrast{0.0};
  bool has_raw_echo{false};
  bool has_range_compressed_echo{false}; /**< RDA/BP 已实际完成内部距离压缩的事实摘要。当前不返回独立距离压缩载荷 */
  bool has_l1_image{false};
  bool has_l3_bp_image{false};
  bool has_image_quality_metrics{false};
  bool image_resolution_m_valid{false};
  bool phase_reference_applied{false};
};

/**
 * @brief SAR 单周期聚合结果。
 * @note `output_frame`、`focused_image` 与质量指标只有在 `status == kCompleted`
 *       时才代表本周期有效计算结果；非执行周期返回默认空帧，不复用上一有效输出，
 *       不能按真实零值参与统计。
 */
struct ONEQ_API SarCycleResult {
  std::uint32_t input_cycle_index{0U};
  SarOutputFrame output_frame{};
  SarFocusedImage focused_image{};
  SarRawPhaseHistory raw_phase_history{};
  SarIssueList issues{}; /**< 统一问题列表（规则 14）：校验问题（kInputValidation）与执行诊断 */
  SarCycleStatus status{SarCycleStatus::kRejectedInvalidInput};
  SarPipelineAbortReason abort_reason{SarPipelineAbortReason::kNone};
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_CYCLE_RESULT_H_
