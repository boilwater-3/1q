/**
 * @file logger.cpp
 * @brief 集成端日志设施实现（见 logger.h）。
 *
 * 1. 两个日志模块装配：库内部日志（spdlog 默认 logger → 1q_library.log）与集成
 *    端日志（拆两个命名 logger：事件行 → integration_events.log、视图行 →
 *    integration_views.log，均带 stdout，中文人读行）；
 * 2. 事件模式二（AGGREGATE）：事件按周期聚合，周期边界落一行（类型+次数）；
 * 3. 背后设施为进程级单例（延迟创建；未初始化时 LogEvent / LogViewSummary 静默
 *    跳过——单元测试链接本文件但不调用 InitIntegrationLog，组件宏调用安全无副作用）。
 */

#include "logger/logger.h"

#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>

namespace component_attachment {
namespace demo {
namespace {

std::shared_ptr<spdlog::logger> g_event_logger;  ///< integration_events.log + stdout（InitIntegrationLog 延迟创建）
std::shared_ptr<spdlog::logger> g_view_logger;   ///< integration_views.log + stdout（InitIntegrationLog 延迟创建）
std::size_t g_event_count = 0U;
std::size_t g_sbirs_count = 0U;
std::size_t g_sar_count = 0U;
std::size_t g_ar_view_count = 0U;
std::size_t g_eos_view_count = 0U;
std::size_t g_sbirs_view_count = 0U;
std::size_t g_sar_view_count = 0U;
std::size_t g_threat_view_count = 0U;

#if defined(CA_EVENT_LOG_MODE_AGGREGATE)
/// 事件类型 → 中文名（聚合行人读显示）。
const char* EventTypeName(const char* type) {
  struct Entry {
    const char* type;
    const char* name;
  };
  static const Entry kNames[] = {
      {"target_confirmed", "目标确认"},
      {"target_lost", "目标丢失"},
      {"emitter_hypothesis", "辐射源假设"},
      {"eos_detection", "光电探测"},
      {"sbirs_detection", "天基探测"},
      {"sar_product", "SAR 产品"},
      {"fusion_updated", "融合更新"},
      {"waypoint_reached", "航点到达"},
      {"platform_state", "平台状态"},
      {"command_issued", "指令下发"},
      {"exclusion_cause", "排除原因变化"},
  };
  for (const auto& entry : kNames) {
    if (std::strcmp(entry.type, type) == 0) {
      return entry.name;
    }
  }
  return type;  // 未知类型：退回稳定标识
}

std::vector<std::pair<std::string, std::size_t>> g_aggregate;  ///< 当前周期聚合缓冲（type → 次数）
std::uint64_t g_aggregate_cycle = 0U;                          ///< 聚合中的周期号
bool g_aggregate_cycle_set = false;

/// 落一行聚合记录（当前周期缓冲 → 人读行，随后清空；事件文件）。
void FlushAggregate() {
  if (!g_aggregate_cycle_set || g_event_logger == nullptr) {
    return;
  }
  std::string parts;
  std::size_t total = 0U;
  for (const auto& slot : g_aggregate) {
    if (!parts.empty()) {
      parts += ", ";
    }
    parts += spdlog::fmt_lib::format("{}×{}", EventTypeName(slot.first.c_str()),
                                     slot.second);
    total += slot.second;
  }
  g_event_logger->info("[事件聚合] 周期={} 事件数={} [{}]", g_aggregate_cycle,
                       total, parts);
  g_aggregate.clear();
  g_aggregate_cycle_set = false;
}
#endif  // CA_EVENT_LOG_MODE_AGGREGATE

}  // namespace

void InitIntegrationLog(const std::string& output_dir) {
  if (g_event_logger != nullptr) {
    return;  // 幂等
  }
  // 库内部日志：库内 PROJECT_LOG_* 走 spdlog 默认 logger，装配为
  // 1q_library.log 文件 sink（人读：默认 pattern 含时间戳 + 级别 + 消息）。
  auto library_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      output_dir + "/1q_library.log", /*truncate=*/true);
  spdlog::set_default_logger(
      std::make_shared<spdlog::logger>("", library_sink));

  // 集成端日志拆两个命名 logger（stdout + 文件，pattern 仅为消息体）：
  // 事件行（[事件:...] / [事件聚合]）→ integration_events.log；视图行
  // （[视图:...]）→ integration_views.log。全部为中文人读文本。
  auto event_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      output_dir + "/integration_events.log", /*truncate=*/true);
  auto view_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      output_dir + "/integration_views.log", /*truncate=*/true);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  g_event_logger = std::make_shared<spdlog::logger>(
      "integration_events", spdlog::sinks_init_list{console_sink, event_sink});
  g_event_logger->set_pattern("%v");
  g_view_logger = std::make_shared<spdlog::logger>(
      "integration_views", spdlog::sinks_init_list{console_sink, view_sink});
  g_view_logger->set_pattern("%v");
}

void LogEvent(std::uint64_t cycle, double t_sec, const char* type,
              const std::string& detail) {
  if (g_event_logger == nullptr) {
    return;  // 未初始化（单元测试）：静默跳过
  }
  ++g_event_count;
  if (std::strcmp(type, "sbirs_detection") == 0) {
    ++g_sbirs_count;
  }
  if (std::strcmp(type, "sar_product") == 0) {
    ++g_sar_count;
  }
#if defined(CA_EVENT_LOG_MODE_AGGREGATE)
  // 周期边界：先落上一周期聚合行，再把本事件计入当前周期缓冲。
  if (g_aggregate_cycle_set && g_aggregate_cycle != cycle) {
    FlushAggregate();
  }
  g_aggregate_cycle = cycle;
  g_aggregate_cycle_set = true;
  for (auto& slot : g_aggregate) {
    if (slot.first == type) {
      ++slot.second;
      return;
    }
  }
  g_aggregate.emplace_back(type, 1U);
#else
  // 事件行自含周期/时间戳（时序随行内嵌，不依赖日志文件级时间戳）；detail 为
  // 组件源文件就地的中文文本（事件字符串归属组件，此处仅做组装）。
  g_event_logger->info("[事件:{}] 周期={} 时间={:.2f}s {}", type, cycle,
                       t_sec, detail);
#endif  // CA_EVENT_LOG_MODE_AGGREGATE
}

void LogViewSummary(const char* module, const std::string& text) {
  if (g_view_logger == nullptr) {
    return;  // 未初始化（单元测试）：静默跳过
  }
  if (std::strcmp(module, "ar") == 0) {
    ++g_ar_view_count;
  } else if (std::strcmp(module, "eos") == 0) {
    ++g_eos_view_count;
  } else if (std::strcmp(module, "sbirs") == 0) {
    ++g_sbirs_view_count;
  } else if (std::strcmp(module, "sar") == 0) {
    ++g_sar_view_count;
  } else if (std::strcmp(module, "threat") == 0) {
    ++g_threat_view_count;
  }
  g_view_logger->info("[视图:{}] {}", module, text);  // 中文人读摘要行
}

void FlushIntegrationLog() {
#if defined(CA_EVENT_LOG_MODE_AGGREGATE)
  FlushAggregate();  // 会话结束：落最后一周期聚合行
#endif
  if (g_event_logger != nullptr) {
    g_event_logger->flush();
  }
  if (g_view_logger != nullptr) {
    g_view_logger->flush();
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

std::size_t SarViewCount() {
  return g_sar_view_count;
}

std::size_t ThreatViewCount() {
  return g_threat_view_count;
}

}  // namespace demo
}  // namespace component_attachment
