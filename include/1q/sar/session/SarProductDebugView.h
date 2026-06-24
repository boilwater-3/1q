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
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

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

class ONEQ_API SarProductDebugViewBuilder {
 public:
  static SarProductDebugView Build(const SarCycleInput& input, const SarCycleResult& result) {
    SarProductDebugView view;
    view.input_cycle_index = result.input_cycle_index;
    view.output_cycle_index = result.output_frame.cycle_index;
    view.executed_this_cycle = result.executed_this_cycle;
    view.reused_previous_output = result.reused_previous_output;
    view.has_error = result.has_error;
    view.abort_reason = result.abort_reason;
    view.completed_stage = result.output_frame.completed_stage;
    view.has_raw_echo = result.output_frame.has_raw_echo;
    view.has_range_compressed_echo = result.output_frame.has_range_compressed_echo;
    view.has_l1_image = result.output_frame.has_l1_image;
    view.has_l3_bp_image = result.output_frame.has_l3_bp_image;
    view.has_focused_pixels =
        !result.focused_image.real_values.empty() || !result.focused_image.imaginary_values.empty();
    view.estimated_snr_db = result.output_frame.estimated_snr_db;
    view.range_sample_count = result.output_frame.range_sample_count;
    view.azimuth_pulse_count = result.output_frame.azimuth_pulse_count;
    view.diagnostics = result.diagnostics;
    view.point_targets.reserve(input.point_targets.size());
    for (const SarPointTarget& target : input.point_targets) {
      SarDebugPointTarget debug_target;
      debug_target.target_id = target.target_id;
      debug_target.target_name = target.target_name;
      debug_target.radar_cross_section_dbsm = target.radar_cross_section_dbsm;
      view.point_targets.push_back(debug_target);
    }
    return view;
  }
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_PRODUCT_DEBUG_VIEW_H_
