/**
 * @file SarDebugViewToJson.h
 * @brief SAR DebugView → JSON 字符串序列化（header-only，无第三方依赖）。
 *
 * 对应契约 docs/common/session_contract.md 三层输出模型规则 12：本库不提供跨周期
 * 状态查询接口，"到目前为止"的累积信息由调用方将每周期 DebugView 以结构化格式
 * （如 JSON）写入自己的日志/事件系统获得。
 *
 * 集成方 copy 本文件 + debug_view_json.h（共享原语，可合并为一个文件）：
 * 每周期调用 SarDebugViewToJson() 得到一条 JSON 记录，写入你们自己的日志即可；
 * 字段名与格式可按需调整。
 */

#ifndef EXAMPLES_COMMON_SAR_DEBUG_VIEW_TO_JSON_H_
#define EXAMPLES_COMMON_SAR_DEBUG_VIEW_TO_JSON_H_

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "1q/sar/session/SarProductDebugView.h"
#include "debug_view_json.h"

namespace {

const char* SarStageName(sar::session::SarProcessingStage stage) {
  switch (stage) {
    case sar::session::SarProcessingStage::kNone:
      return "none";
    case sar::session::SarProcessingStage::kRawEcho:
      return "raw_echo";
    case sar::session::SarProcessingStage::kL1RdaImage:
      return "l1_rda_image";
    case sar::session::SarProcessingStage::kL3BpImage:
      return "l3_bp_image";
  }
  return "unknown";
}

}  // namespace

/**
 * @brief 把单周期 SAR 调试视图序列化为一条 JSON 记录（帧快照）。
 * @param[in] view 由 SarProductDebugViewBuilder::Build() 产出的调试视图。
 * @return JSON 字符串，可直接写入调用方自己的日志/事件系统。
 */
inline std::string SarDebugViewToJson(const sar::session::SarProductDebugView& view) {
  std::ostringstream out;
  out << "{\"input_cycle_index\":" << view.input_cycle_index
      << ",\"output_cycle_index\":" << view.output_cycle_index
      << ",\"executed_this_cycle\":" << (view.executed_this_cycle ? "true" : "false")
      << ",\"has_error\":" << (view.has_error ? "true" : "false") << ",\"abort_reason\":\""
      << JsonEscape(view.abort_reason) << '"' << ",\"completed_stage\":\""
      << SarStageName(view.completed_stage) << '"'
      << ",\"has_raw_echo\":" << (view.has_raw_echo ? "true" : "false")
      << ",\"has_range_compressed_echo\":" << (view.has_range_compressed_echo ? "true" : "false")
      << ",\"has_l1_image\":" << (view.has_l1_image ? "true" : "false")
      << ",\"has_l3_bp_image\":" << (view.has_l3_bp_image ? "true" : "false")
      << ",\"has_focused_pixels\":" << (view.has_focused_pixels ? "true" : "false")
      << ",\"estimated_snr_db\":" << view.estimated_snr_db
      << ",\"range_sample_count\":" << view.range_sample_count
      << ",\"azimuth_pulse_count\":" << view.azimuth_pulse_count << ",\"point_targets\":[";
  for (std::size_t i = 0U; i < view.point_targets.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    out << "{\"target_id\":" << view.point_targets[i].target_id << ",\"target_name\":\""
        << JsonEscape(view.point_targets[i].target_name) << '"'
        << ",\"radar_cross_section_dbsm\":" << view.point_targets[i].radar_cross_section_dbsm
        << '}';
  }
  WriteIssuesJson(out, view.diagnostics);
  return out.str();
}

#endif  // EXAMPLES_COMMON_SAR_DEBUG_VIEW_TO_JSON_H_
