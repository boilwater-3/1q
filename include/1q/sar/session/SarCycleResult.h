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
  kRangeCompression = 2,
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
enum class SarFocusedImageSource {
  kNone = 0,
  kL1Rda = 1,
  kL3Bp = 2
};

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
 * @brief SAR 输出帧元数据。
 */
struct ONEQ_API SarOutputFrame {
  std::uint32_t cycle_index{0U};
  SarProcessingStage completed_stage{SarProcessingStage::kNone};
  std::uint32_t range_sample_count{0U};
  std::uint32_t azimuth_pulse_count{0U};
  double center_slant_range_m{0.0};
  double estimated_snr_db{0.0};
  bool has_raw_echo{false};
  bool has_range_compressed_echo{false};
  bool has_l1_image{false};
  bool has_l3_bp_image{false};
};

/**
 * @brief SAR 单周期聚合结果。
 */
struct ONEQ_API SarCycleResult {
  std::uint32_t input_cycle_index{0U};
  SarOutputFrame output_frame{};
  SarFocusedImage focused_image{};
  SarDiagnosticIssueList diagnostics{};
  bool has_error{false};
  bool executed_this_cycle{false};
  bool reused_previous_output{false};
  std::string abort_reason{};
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_CYCLE_RESULT_H_
