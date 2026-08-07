/**
 * @file demo_output.cpp
 * @brief 输出落盘与事件消费实现（见 demo_output.h）。
 *
 * 事件日志（events.csv + 控制台事件行）由组件源文件内日志宏承担
 * （components/demo_log.h）——本文件只负责周期落盘与决策事件链。
 */

#include "demo_output.h"

#include <algorithm>
#include <iostream>
#include <sstream>

#include "ArDebugViewToJson.h"
#include "EosDebugViewToJson.h"
#include "SbirsDebugViewToJson.h"
#include "components/demo_log.h"
#include "core/events.h"
#include "demo_config.h"

namespace component_attachment {
namespace demo {
namespace {

/// 规则 14 问题严重度 → 稳定字符串（与 DebugView JSON 序列化的取值一致）。
template <typename Severity>
const char* IssueSeverityName(Severity severity) {
  switch (severity) {
    case Severity::kInfo:
      return "info";
    case Severity::kWarning:
      return "warning";
    case Severity::kError:
      return "error";
    default:
      break;
  }
  return "unknown";
}

/// 规则 14 问题来源阶段 → 稳定字符串（kInputValidation/kExecution/kOutputContract）。
template <typename Phase>
const char* IssuePhaseName(Phase phase) {
  switch (phase) {
    case Phase::kInputValidation:
      return "input_validation";
    case Phase::kExecution:
      return "execution";
    case Phase::kOutputContract:
      return "output_contract";
    default:
      break;
  }
  return "unknown";
}

/// 规则 14 机器消费路径：把某模块本周期 DebugView 的问题列表逐条写进
/// issues.csv（severity/phase/code/message，message 经 CSV 转义；与 JSONL
/// 同源——DebugView 承载的即周期结果 issues 的转写）。
template <typename IssueList>
void WriteIssuesCsv(examples::CsvWriter& csv, std::uint64_t cycle, const char* module,
                    const IssueList& issues, std::size_t* row_count) {
  for (const auto& issue : issues) {
    csv.WriteRow(Fmt("%llu,%s,%s,%s,%s,%s", static_cast<unsigned long long>(cycle), module,
                     IssueSeverityName(issue.severity), IssuePhaseName(issue.phase),
                     examples::EscapeCsvField(issue.code).c_str(),
                     examples::EscapeCsvField(issue.message).c_str()));
    if (row_count != nullptr) {
      ++(*row_count);
    }
  }
}

}  // namespace

DemoOutputs::DemoOutputs(const std::string& output_dir)
    : platform_csv_(output_dir + "/platform_track.csv",
                    "cycle,t_sec,lat_deg,lon_deg,alt_m,heading_deg,speed_mps,wp_index"),
      issues_csv_(output_dir + "/issues.csv", "cycle,module,severity,phase,code,message"),
      sbirs_debug_jsonl_(output_dir + "/sbirs_debug_view.jsonl"),
      ar_delta_jsonl_(output_dir + "/ar_track_status_deltas.jsonl"),
      eos_debug_jsonl_(output_dir + "/eos_debug_view.jsonl") {
  // 规则 12 落盘示范：SBIRS 每周期调试视图 JSONL（一行一条帧快照，含按目标
  // 状态与规则 13b kInfo 排除诊断），与平台轨迹 CSV 同级输出。
  if (!sbirs_debug_jsonl_) {
    std::cerr << "Failed to open sbirs_debug_view.jsonl\n";
    valid_ = false;
  }
  // 三落盘模式之二（跨周期增量）：AR 调试视图只落状态变化行（首次出现即
  // 记录），状态表由调用方持有（只增不减）——"变更才落盘"的瘦数据形态。
  if (!ar_delta_jsonl_) {
    std::cerr << "Failed to open ar_track_status_deltas.jsonl\n";
    valid_ = false;
  }
  // 三落盘模式之三（降频落盘）：EOS 每 10 周期一次全帧，其余周期只落
  // 周期号 + 问题列表（含规则 13b kInfo 排除诊断）。
  if (!eos_debug_jsonl_) {
    std::cerr << "Failed to open eos_debug_view.jsonl\n";
    valid_ = false;
  }
}

void DemoOutputs::RecordPlatformRow(std::uint32_t cycle, double t_sec,
                                    const FlightComponent& flight) {
  platform_csv_.WriteRow(Fmt("%u,%.2f,%.7f,%.7f,%.1f,%.1f,%.1f,%zu", cycle, t_sec,
                             flight.position().latitude_deg, flight.position().longitude_deg,
                             flight.position().altitude_m, flight.heading_deg(),
                             flight.speed_mps(), flight.next_waypoint_index()));
  ++platform_rows_;
}

void DemoOutputs::RecordDebugViews(std::uint32_t cycle, const ArSensorComponent& ar,
                                   const EosSensorComponent& eos,
                                   const SbirsSensorComponent& sbirs) {
  // 规则 12：SBIRS 全帧（每周期一行 JSON）。
  sbirs_debug_jsonl_ << SbirsDebugViewToJson(sbirs.LastDebugView()) << '\n';
  ++sbirs_debug_rows_;

  // 三落盘模式之二：AR 跨周期增量（状态变化/首次出现各一行，其余周期静默）。
  // 行数经缓冲统计（增量行数随状态变化而变，不保证每周期一行）。
  std::ostringstream ar_delta_buffer;
  ArWriteTrackStatusDeltas(ar.LastDebugView(), ar_prev_status_, ar_delta_buffer);
  const std::string ar_delta_text = ar_delta_buffer.str();
  ar_delta_rows_ += static_cast<std::size_t>(
      std::count(ar_delta_text.begin(), ar_delta_text.end(), '\n'));
  ar_delta_jsonl_ << ar_delta_text;

  // 三落盘模式之三：EOS 降频（每 10 周期全帧 + 其余周期只落问题列表）。
  EosWriteDownsampledView(eos.LastDebugView(), eos_debug_jsonl_, 10U);
  ++eos_debug_rows_;

  // 规则 14 机器消费路径：三通道问题列表逐条落 CSV（与 JSONL 同源）。
  WriteIssuesCsv(issues_csv_, cycle, "ar", ar.LastDebugView().issues, &issues_rows_);
  WriteIssuesCsv(issues_csv_, cycle, "eos", eos.LastDebugView().issues, &issues_rows_);
  WriteIssuesCsv(issues_csv_, cycle, "sbirs", sbirs.LastDebugView().issues, &issues_rows_);
}

void DemoOutputs::Flush() {
  platform_csv_.Flush();
  sbirs_debug_jsonl_.flush();
  ar_delta_jsonl_.flush();
  eos_debug_jsonl_.flush();
  issues_csv_.Flush();
}

DecisionListener::DecisionListener(World& world) : world_(world) {
  world_.signals().on_fusion_updated.connect([this](const FusionUpdatedEvent& e) {
    if (e.confidence >= kHighThreatConfidence && !issued_) {
      issued_ = true;
      CommandIssuedEvent command;
      command.cycle = e.cycle;
      command.command = "ENABLE_ANTI_FALSE_TARGET_DISCRIMINATION";
      // 事件日志：字符串就地填充（发布处记录，与组件宏同模式）。
      CA_LOG_EVENT(world_, "command_issued", "cmd=%s", command.command.c_str());
      world_.signals().on_command_issued(command);
    }
  });
}

}  // namespace demo
}  // namespace component_attachment
