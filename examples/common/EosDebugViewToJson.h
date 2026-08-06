/**
 * @file EosDebugViewToJson.h
 * @brief EOS DebugView → JSON 字符串序列化（header-only，无第三方依赖）。
 *
 * 对应契约 docs/common/session_contract.md 三层输出模型规则 12：本库不提供跨周期
 * 状态查询接口，"到目前为止"的累积信息由调用方将每周期 DebugView 以结构化格式
 * （如 JSON）写入自己的日志/事件系统获得。
 *
 * 集成方 copy 本文件 + debug_view_json.h（共享原语，可合并为一个文件）：
 * 每周期调用 EosDebugViewToJson() 得到一条 JSON 记录，写入你们自己的日志即可；
 * 字段名与格式可按需调整。
 */

#ifndef EXAMPLES_COMMON_EOS_DEBUG_VIEW_TO_JSON_H_
#define EXAMPLES_COMMON_EOS_DEBUG_VIEW_TO_JSON_H_

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "debug_view_json.h"

namespace {

const char* EosAbortReasonName(electro_optical_sensor::session::EosPipelineAbortReason reason) {
  switch (reason) {
    case electro_optical_sensor::session::EosPipelineAbortReason::kNone:
      return "none";
    case electro_optical_sensor::session::EosPipelineAbortReason::kValidationRejected:
      return "validation_rejected";
    case electro_optical_sensor::session::EosPipelineAbortReason::kOutputContractViolation:
      return "output_contract_violation";
    case electro_optical_sensor::session::EosPipelineAbortReason::kRuntimeStateRestoreRejected:
      return "runtime_state_restore_rejected";
    case electro_optical_sensor::session::EosPipelineAbortReason::kSensorPoweredOff:
      return "sensor_powered_off";
  }
  return "unknown";
}

const char* EosTargetStatusName(electro_optical_sensor::session::EosDebugTargetStatus status) {
  switch (status) {
    case electro_optical_sensor::session::EosDebugTargetStatus::kDetected:
      return "detected";
    case electro_optical_sensor::session::EosDebugTargetStatus::kObservedBelowThreshold:
      return "observed_below_threshold";
    case electro_optical_sensor::session::EosDebugTargetStatus::kNotInOutput:
      return "not_in_output";
    case electro_optical_sensor::session::EosDebugTargetStatus::kCycleNotExecuted:
      return "cycle_not_executed";
  }
  return "unknown";
}

}  // namespace

/**
 * @brief 把单周期 EOS 调试视图序列化为一条 JSON 记录（帧快照）。
 * @param[in] view 由 EosOutputDebugViewBuilder::Build() 产出的调试视图。
 * @return JSON 字符串，可直接写入调用方自己的日志/事件系统。
 */
inline std::string EosDebugViewToJson(
    const electro_optical_sensor::session::EosOutputDebugView& view) {
  std::ostringstream out;
  out << "{\"input_cycle_index\":" << view.input_cycle_index
      << ",\"output_cycle_index\":" << view.output_cycle_index
      << ",\"executed_this_cycle\":" << (view.executed_this_cycle ? "true" : "false")
      << ",\"has_validation_error\":" << (view.has_validation_error ? "true" : "false")
      << ",\"abort_reason\":\"" << EosAbortReasonName(view.abort_reason) << '"' << ",\"targets\":[";
  for (std::size_t i = 0U; i < view.targets.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    const electro_optical_sensor::session::EosDebugTargetState& target = view.targets[i];
    out << "{\"target_id\":" << target.target_id << ",\"target_name\":\""
        << JsonEscape(target.target_name) << '"' << ",\"status\":\""
        << EosTargetStatusName(target.status) << '"'
        << ",\"present_in_input\":" << (target.present_in_input ? "true" : "false")
        << ",\"has_raw_output_record\":" << (target.has_raw_output_record ? "true" : "false")
        << ",\"detected\":" << (target.detected ? "true" : "false")
        << ",\"range_m\":" << target.range_m << ",\"azimuth_deg\":" << target.azimuth_deg
        << ",\"elevation_deg\":" << target.elevation_deg
        << ",\"fused_snr_db\":" << target.fused_snr_db << '}';
  }
  WriteDiagnosticsJson(out, view.diagnostics);
  return out.str();
}

#endif  // EXAMPLES_COMMON_EOS_DEBUG_VIEW_TO_JSON_H_
