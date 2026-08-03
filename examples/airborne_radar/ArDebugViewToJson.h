/**
 * @file ArDebugViewToJson.h
 * @brief AR DebugView → JSON 字符串序列化（header-only，无第三方依赖）。
 *
 * 对应契约 docs/common/session_contract.md 三层输出模型规则 12：本库不提供跨周期
 * 状态查询接口，"到目前为止"的累积信息由调用方将每周期 DebugView 以结构化格式
 * （如 JSON）写入自己的日志/事件系统获得。
 *
 * 集成方直接 copy 本文件：每周期调用 ArDebugViewToJson() 得到一条 JSON 记录，
 * 写入你们自己的日志即可；字段名与格式可按需调整。
 */

#ifndef EXAMPLES_AIRBORNE_RADAR_DEBUG_VIEW_TO_JSON_H_
#define EXAMPLES_AIRBORNE_RADAR_DEBUG_VIEW_TO_JSON_H_

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"

namespace {

std::string ArJsonEscape(const std::string& text) {
  std::ostringstream out;
  for (char ch : text) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20U) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<unsigned int>(static_cast<unsigned char>(ch)) << std::dec;
        } else {
          out << ch;
        }
    }
  }
  return out.str();
}

const char* ArSeverityName(airborne_radar::session::ArDiagnosticSeverity severity) {
  switch (severity) {
    case airborne_radar::session::ArDiagnosticSeverity::kInfo:
      return "info";
    case airborne_radar::session::ArDiagnosticSeverity::kWarning:
      return "warning";
    case airborne_radar::session::ArDiagnosticSeverity::kError:
      return "error";
  }
  return "unknown";
}

const char* ArImpairmentName(airborne_radar::session::ArReceiverImpairment impairment) {
  switch (impairment) {
    case airborne_radar::session::ArReceiverImpairment::kNone:
      return "none";
    case airborne_radar::session::ArReceiverImpairment::kSaturated:
      return "saturated";
  }
  return "unknown";
}

const char* ArTrackStatusName(airborne_radar::session::ArDebugTrackStatus status) {
  switch (status) {
    case airborne_radar::session::ArDebugTrackStatus::kConfirmed:
      return "confirmed";
    case airborne_radar::session::ArDebugTrackStatus::kTentative:
      return "tentative";
    case airborne_radar::session::ArDebugTrackStatus::kLost:
      return "lost";
    case airborne_radar::session::ArDebugTrackStatus::kNotInOutput:
      return "not_in_output";
    case airborne_radar::session::ArDebugTrackStatus::kCycleNotCompleted:
      return "cycle_not_completed";
  }
  return "unknown";
}

}  // namespace

/**
 * @brief 把单周期 AR 调试视图序列化为一条 JSON 记录（帧快照）。
 * @param[in] view 由 ArTrackOutputDebugViewBuilder::Build() 产出的调试视图。
 * @return JSON 字符串，可直接写入调用方自己的日志/事件系统。
 */
inline std::string ArDebugViewToJson(const airborne_radar::session::ArTrackOutputDebugView& view) {
  std::ostringstream out;
  out << "{\"world_cycle_index\":" << view.world_cycle_index
      << ",\"output_cycle_index\":" << view.output_cycle_index
      << ",\"completed_this_cycle\":" << (view.completed_this_cycle ? "true" : "false")
      << ",\"receiver_impairment\":\"" << ArImpairmentName(view.receiver_impairment) << '"'
      << ",\"tracks\":[";
  for (std::size_t i = 0U; i < view.tracks.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    const airborne_radar::session::ArDebugTrackState& track = view.tracks[i];
    out << "{\"external_target_id\":" << track.external_target_id << ",\"target_name\":\""
        << ArJsonEscape(track.target_name) << '"' << ",\"status\":\""
        << ArTrackStatusName(track.status) << '"'
        << ",\"present_in_input\":" << (track.present_in_input ? "true" : "false")
        << ",\"has_track\":" << (track.has_track ? "true" : "false")
        << ",\"association_key\":" << track.association_key
        << ",\"position_x\":" << track.position_x << ",\"position_y\":" << track.position_y
        << ",\"position_z\":" << track.position_z << ",\"speed\":" << track.speed
        << ",\"rcs\":" << track.rcs << ",\"hit_count\":" << track.hit_count
        << ",\"miss_count\":" << track.miss_count << ",\"target_type\":\""
        << ArJsonEscape(track.target_type) << "\"}";
  }
  out << "],\"diagnostics\":[";
  for (std::size_t i = 0U; i < view.diagnostics.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    out << "{\"severity\":\"" << ArSeverityName(view.diagnostics[i].severity) << '"'
        << ",\"code\":\"" << ArJsonEscape(view.diagnostics[i].code) << '"' << ",\"message\":\""
        << ArJsonEscape(view.diagnostics[i].message) << "\"}";
  }
  out << "]}";
  return out.str();
}

#endif  // EXAMPLES_AIRBORNE_RADAR_DEBUG_VIEW_TO_JSON_H_
