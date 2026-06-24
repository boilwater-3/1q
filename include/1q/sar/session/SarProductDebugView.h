/**
 * @file SarProductDebugView.h
 * @brief 定义 SAR 产品开发调试视图构建工具。
 */

#ifndef ONEQ_SAR_SESSION_SAR_PRODUCT_DEBUG_VIEW_H_
#define ONEQ_SAR_SESSION_SAR_PRODUCT_DEBUG_VIEW_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

// 前向声明：Build 参数为 const 引用，header 无需完整类型，避免拉入 SarCycleInput 重依赖。
struct SarCycleInput;

struct ONEQ_API SarDebugPointTarget {
  std::uint64_t target_id{0U};
  std::string target_name{};
  double radar_cross_section_dbsm{0.0};
};

struct ONEQ_API SarProductDebugView {
  std::uint32_t input_cycle_index{0U};
  std::uint32_t output_cycle_index{0U};
  bool executed_this_cycle{false};
  bool reused_previous_output{false};
  bool has_error{false};
  std::string abort_reason{};
  SarProcessingStage completed_stage{SarProcessingStage::kNone};
  bool has_raw_echo{false};
  bool has_range_compressed_echo{false};
  bool has_l1_image{false};
  bool has_l3_bp_image{false};
  bool has_focused_pixels{false};
  double estimated_snr_db{0.0};
  std::uint32_t range_sample_count{0U};
  std::uint32_t azimuth_pulse_count{0U};
  std::vector<SarDebugPointTarget> point_targets{};
  SarDiagnosticIssueList diagnostics{};
};

/**
 * @brief 把原始产品输出、执行结果与输入点目标合成为开发可读视图。
 *
 * 该构建器只读组合，不反向影响成像 pipeline。实现见 .cpp。
 */
class ONEQ_API SarProductDebugViewBuilder {
 public:
  static SarProductDebugView Build(const SarCycleInput& input, const SarCycleResult& result);
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_PRODUCT_DEBUG_VIEW_H_
