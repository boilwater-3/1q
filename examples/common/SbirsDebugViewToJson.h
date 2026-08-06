/**
 * @file SbirsDebugViewToJson.h
 * @brief SBIRS DebugView → JSON 字符串序列化（header-only，无第三方依赖）。
 *
 * 对应契约 docs/common/session_contract.md 三层输出模型规则 12：本库不提供跨周期
 * 状态查询接口，"到目前为止"的累积信息由调用方将每周期 DebugView 以结构化格式
 * （如 JSON）写入自己的日志/事件系统获得。
 *
 * 集成方 copy 本文件 + debug_view_json.h（共享原语，可合并为一个文件）：
 * 每周期调用 SbirsDebugViewToJson() 得到一条 JSON 记录，写入你们自己的日志即可；
 * 字段名与格式可按需调整。
 */

#ifndef EXAMPLES_COMMON_SBIRS_DEBUG_VIEW_TO_JSON_H_
#define EXAMPLES_COMMON_SBIRS_DEBUG_VIEW_TO_JSON_H_

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"
#include "debug_view_json.h"

namespace {

const char* SbirsAbortReasonName(sbirs_sensor::session::SbirsPipelineAbortReason reason) {
  switch (reason) {
    case sbirs_sensor::session::SbirsPipelineAbortReason::kNone:
      return "none";
    case sbirs_sensor::session::SbirsPipelineAbortReason::kValidationRejected:
      return "validation_rejected";
    case sbirs_sensor::session::SbirsPipelineAbortReason::kSensorPoweredOff:
      return "sensor_powered_off";
  }
  return "unknown";
}

const char* SbirsTargetStatusName(sbirs_sensor::session::SbirsDebugTargetStatus status) {
  switch (status) {
    case sbirs_sensor::session::SbirsDebugTargetStatus::kDetected:
      return "detected";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kObservedBelowThreshold:
      return "observed_below_threshold";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kCoasting:
      return "coasting";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kNotInOutput:
      return "not_in_output";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kCycleNotExecuted:
      return "cycle_not_executed";
  }
  return "unknown";
}

const char* SbirsTrackingSourceName(sbirs_sensor::attribution::SbirsTrackingSource source) {
  switch (source) {
    case sbirs_sensor::attribution::SbirsTrackingSource::kNotApplicable:
      return "not_applicable";
    case sbirs_sensor::attribution::SbirsTrackingSource::kEstimated:
      return "estimated";
    case sbirs_sensor::attribution::SbirsTrackingSource::kStrictTruthAssisted:
      return "strict_truth_assisted";
    case sbirs_sensor::attribution::SbirsTrackingSource::kSensorLikeTruthAssisted:
      return "sensor_like_truth_assisted";
  }
  return "unknown";
}

const char* SbirsObservationStageName(sbirs_sensor::output::SbirsObservationStage stage) {
  switch (stage) {
    case sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch:
      return "wide_field_search";
    case sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition:
      return "narrow_field_acquisition";
    case sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack:
      return "narrow_field_track";
  }
  return "unknown";
}

}  // namespace

/**
 * @brief 把单周期 SBIRS 调试视图序列化为一条 JSON 记录（帧快照）。
 * @param[in] view 由 SbirsOutputDebugViewBuilder::Build() 产出的调试视图。
 * @return JSON 字符串，可直接写入调用方自己的日志/事件系统。
 */
inline std::string SbirsDebugViewToJson(const sbirs_sensor::session::SbirsOutputDebugView& view) {
  std::ostringstream out;
  out << "{\"input_cycle_index\":" << view.input_cycle_index
      << ",\"output_cycle_index\":" << view.output_cycle_index
      << ",\"executed_this_cycle\":" << (view.executed_this_cycle ? "true" : "false")
      << ",\"has_validation_error\":" << (view.has_validation_error ? "true" : "false")
      << ",\"abort_reason\":\"" << SbirsAbortReasonName(view.abort_reason) << '"'
      << ",\"targets\":[";
  for (std::size_t i = 0U; i < view.targets.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    const sbirs_sensor::session::SbirsDebugTargetState& target = view.targets[i];
    out << "{\"target_id\":" << target.target_id << ",\"target_name\":\""
        << JsonEscape(target.target_name) << '"' << ",\"status\":\""
        << SbirsTargetStatusName(target.status) << '"'
        << ",\"present_in_input\":" << (target.present_in_input ? "true" : "false")
        << ",\"has_raw_output_record\":" << (target.has_raw_output_record ? "true" : "false")
        << ",\"detected\":" << (target.detected ? "true" : "false") << ",\"tracking_source\":\""
        << SbirsTrackingSourceName(target.tracking_source) << '"'
        << ",\"estimated_range_m\":" << target.estimated_range_m
        << ",\"has_estimation_nis\":" << (target.has_estimation_nis ? "true" : "false")
        << ",\"estimation_nis\":" << target.estimation_nis << ",\"estimation_nis_gate_exceeded\":"
        << (target.estimation_nis_gate_exceeded ? "true" : "false")
        << ",\"nfov_channel_id\":" << target.nfov_channel_id
        << ",\"has_nfov_tracking_diagnostics\":"
        << (target.has_nfov_tracking_diagnostics ? "true" : "false")
        << ",\"nfov_pointing_error_deg\":" << target.nfov_pointing_error_deg
        << ",\"nfov_geometry_gate_passed\":"
        << (target.nfov_geometry_gate_passed ? "true" : "false")
        << ",\"nfov_snr_gate_passed\":" << (target.nfov_snr_gate_passed ? "true" : "false")
        << ",\"nfov_tracking_gate_failure_count\":" << target.nfov_tracking_gate_failure_count
        << ",\"nfov_tracking_coasting\":" << (target.nfov_tracking_coasting ? "true" : "false")
        << ",\"azimuth_deg\":" << target.azimuth_deg
        << ",\"elevation_deg\":" << target.elevation_deg
        << ",\"infrared_snr_linear\":" << target.infrared_snr_linear << ",\"observation_stage\":\""
        << SbirsObservationStageName(target.observation_stage) << "\"}";
  }
  WriteDiagnosticsJson(out, view.diagnostics);
  return out.str();
}

#endif  // EXAMPLES_COMMON_SBIRS_DEBUG_VIEW_TO_JSON_H_
