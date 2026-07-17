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

namespace sar {
namespace session {

/**
 * @brief SAR 处理阶段状态。
 */
enum class SarProcessingStage {
  kNone = 0,
  kRawEcho = 1,
  kRangeCompression = 2, /**< 距离压缩阶段标记。当前 Phase 1 不产出独立可消费的距离压缩产物，真实距离压缩在 RDA / BP 内部完成；该值表示会话已声明完成该内部步骤、满足 L3 BP 前置条件门，并作为 replay 摘要保真。它不是终端成像阶段，仅当 RDA/BP 未启用时才可能成为 completed_stage 的最终值 */
  kL1RdaImage = 3,
  kL3BpImage = 4
};

/**
 * @brief SAR 诊断等级。
 */
enum class SarDiagnosticSeverity { kInfo = 0, kWarning = 1, kError = 2 };

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
 * @brief SAR 诊断条目。
 */
struct ONEQ_API SarDiagnosticIssue {
  SarDiagnosticSeverity severity{SarDiagnosticSeverity::kInfo};
  std::string code{};
  std::string message{};
};

using SarDiagnosticIssueList = std::vector<SarDiagnosticIssue>;

/**
 * @brief 与内部矩阵实现解耦的行主序复数聚焦图像。
 * @note Phase 1 replay 仍只保存摘要；该载荷仅由本次实时执行结果返回。
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
  bool has_range_compressed_echo{false}; /**< 距离压缩摘要标志，与 `SarProcessingStage::kRangeCompression` 同义。当前 Phase 1 不返回独立距离压缩载荷；该标志在 `enable_range_compression` 为真时置位，表示 RDA/BP 内部距离压缩步骤已声明完成，并非可消费的独立输出 */
  bool has_l1_image{false};
  bool has_l3_bp_image{false};
  bool has_image_quality_metrics{false};
  bool image_resolution_m_valid{false};
  bool phase_reference_applied{false};
};

/**
 * @brief SAR 单周期聚合结果。
 * @note `output_frame`、`focused_image` 与质量指标只有在 `executed_this_cycle=true`
 *       时才代表本周期有效计算结果；失败/abort 周期会保留默认值或上一有效输出，
 *       不能按真实零值参与统计。
 */
struct ONEQ_API SarCycleResult {
  std::uint32_t input_cycle_index{0U};
  SarOutputFrame output_frame{};
  SarFocusedImage focused_image{};
  SarRawPhaseHistory raw_phase_history{};
  SarDiagnosticIssueList diagnostics{};
  bool has_error{false};
  bool executed_this_cycle{false};
  bool reused_previous_output{false};
  std::string abort_reason{};
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_CYCLE_RESULT_H_
