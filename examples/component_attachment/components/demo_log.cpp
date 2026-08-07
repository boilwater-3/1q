/**
 * @file demo_log.cpp
 * @brief 事件日志宏设施实现（见 demo_log.h）。
 *
 * 背后设施为进程级单例（延迟创建 events.csv；未初始化时 LogEvent 静默
 * 跳过——单元测试链接本文件但不调用 InitEventLog，组件宏调用安全无副作用）。
 * 控制台事件行格式与既有事件流一致：`  [type] detail`。
 */

#include "demo_log.h"

#include <cstring>
#include <iostream>
#include <memory>

#include "csv_writer.h"

namespace component_attachment {
namespace demo {
namespace {

std::unique_ptr<examples::CsvWriter> g_event_csv;  ///< events.csv（main InitEventLog 延迟创建）
std::size_t g_event_count = 0U;
std::size_t g_sbirs_count = 0U;
std::size_t g_sar_count = 0U;

}  // namespace

void InitEventLog(const std::string& output_dir) {
  if (g_event_csv != nullptr) {
    return;  // 幂等
  }
  // CsvWriter 构造失败即 abort（与既有 events.csv 语义一致）。
  g_event_csv =
      std::make_unique<examples::CsvWriter>(output_dir + "/events.csv",
                                            "cycle,t_sec,event_type,detail");
}

void LogEvent(std::uint64_t cycle, double t_sec, const char* type,
              const std::string& detail) {
  if (g_event_csv == nullptr) {
    return;  // 未初始化（单元测试）：静默跳过
  }
  ++g_event_count;
  if (std::strcmp(type, "sbirs_detection") == 0) {
    ++g_sbirs_count;
  }
  if (std::strcmp(type, "sar_product") == 0) {
    ++g_sar_count;
  }
  std::cout << "  [" << type << "] " << detail << "\n";
  // detail 为自由文本（含逗号/括号）：按 RFC 4180 转义，保证列结构完整。
  g_event_csv->WriteRow(Fmt("%llu,%.2f,%s,%s", static_cast<unsigned long long>(cycle), t_sec,
                            type, examples::EscapeCsvField(detail).c_str()));
}

void FlushEventLog() {
  if (g_event_csv != nullptr) {
    g_event_csv->Flush();
  }
}

std::size_t EventCount() {
  return g_event_count;
}

std::size_t SbirsEventCount() {
  return g_sbirs_count;
}

std::size_t SarProductEventCount() {
  return g_sar_count;
}

}  // namespace demo
}  // namespace component_attachment
