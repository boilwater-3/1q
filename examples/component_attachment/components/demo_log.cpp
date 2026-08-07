/**
 * @file demo_log.cpp
 * @brief 集成端日志设施实现（见 demo_log.h）。
 *
 * 两个日志模块的装配：库内部日志（spdlog 默认 logger → 1q_library.log）与
 * 集成端日志（命名 logger "integration" → stdout + integration.log）。背后
 * 设施为进程级单例（延迟创建；未初始化时 LogEvent / LogViewSummary 静默
 * 跳过——单元测试链接本文件但不调用 InitIntegrationLog，组件宏调用安全无
 * 副作用）。
 */

#include "demo_log.h"

#include <cstring>
#include <memory>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>

namespace component_attachment {
namespace demo {
namespace {

std::shared_ptr<spdlog::logger> g_integration_logger;  ///< integration.log + stdout（InitIntegrationLog 延迟创建）
std::size_t g_event_count = 0U;
std::size_t g_sbirs_count = 0U;
std::size_t g_sar_count = 0U;
std::size_t g_ar_view_count = 0U;
std::size_t g_eos_view_count = 0U;
std::size_t g_sbirs_view_count = 0U;

}  // namespace

void InitIntegrationLog(const std::string& output_dir) {
  if (g_integration_logger != nullptr) {
    return;  // 幂等
  }
  // 库内部日志：库内 PROJECT_LOG_* 走 spdlog 默认 logger，装配为
  // 1q_library.log 文件 sink（人读：默认 pattern 含时间戳 + 级别 + 消息）。
  auto library_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      output_dir + "/1q_library.log", /*truncate=*/true);
  spdlog::set_default_logger(
      std::make_shared<spdlog::logger>("", library_sink));

  // 集成端日志：命名 logger "integration"（stdout + integration.log）。
  // pattern 仅为消息体（%v）：事件行为 "[event:type] ..."、视图摘要行为
  // "[view:module] ..."，全部为人读文本（日志给人读，示例不做结构化落盘）。
  auto integration_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      output_dir + "/integration.log", /*truncate=*/true);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  g_integration_logger = std::make_shared<spdlog::logger>(
      "integration", spdlog::sinks_init_list{console_sink, integration_sink});
  g_integration_logger->set_pattern("%v");
}

void LogEvent(std::uint64_t cycle, double t_sec, const char* type,
              const std::string& detail) {
  if (g_integration_logger == nullptr) {
    return;  // 未初始化（单元测试）：静默跳过
  }
  ++g_event_count;
  if (std::strcmp(type, "sbirs_detection") == 0) {
    ++g_sbirs_count;
  }
  if (std::strcmp(type, "sar_product") == 0) {
    ++g_sar_count;
  }
  // 事件行自含周期/时间戳（时序随行内嵌，不依赖日志文件级时间戳）；detail
  // 为自由文本（事件字符串归属组件源文件，此处仅做组装）。
  g_integration_logger->info("[event:{}] cycle={} t_sec={:.2f} {}", type, cycle,
                              t_sec, detail);
}

void LogViewSummary(const char* module, const std::string& text) {
  if (g_integration_logger == nullptr) {
    return;  // 未初始化（单元测试）：静默跳过
  }
  if (std::strcmp(module, "ar") == 0) {
    ++g_ar_view_count;
  } else if (std::strcmp(module, "eos") == 0) {
    ++g_eos_view_count;
  } else if (std::strcmp(module, "sbirs") == 0) {
    ++g_sbirs_view_count;
  }
  g_integration_logger->info("[view:{}] {}", module, text);  // 人读摘要行
}

void FlushIntegrationLog() {
  if (g_integration_logger != nullptr) {
    g_integration_logger->flush();
  }
  if (spdlog::default_logger_raw() != nullptr) {
    spdlog::default_logger_raw()->flush();
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

std::size_t ArViewCount() {
  return g_ar_view_count;
}

std::size_t EosViewCount() {
  return g_eos_view_count;
}

std::size_t SbirsViewCount() {
  return g_sbirs_view_count;
}

}  // namespace demo
}  // namespace component_attachment
