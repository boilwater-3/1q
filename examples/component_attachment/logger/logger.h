/**
 * @file logger.h
 * @brief 集成端日志设施（外部集成惯用法示范，双后端）。
 *
 * 与库内部日志（PROJECT_LOG_*）区分成两个日志模块：
 * 1. 库内部调试日志默认关闭（ONEQ_ENABLE_FILE_LOG / CA_LIBRARY_FILE_LOG）。
 *    仅 macOS 显式打开时装配 1q_library.log；Windows 保持关闭。
 * 2. 集成端日志：spdlog 平台拆两个命名 logger（均带 stdout，pattern 仅消息
 *    体）；Windows 走 std::ofstream 文件后端（仅落文件，行文案与 spdlog 分支
 *    完全一致）——事件行（"[事件:type] 周期=... 时间=...s 中文详情"）→
 *    integration_events.log，各组件每周期调试视图行（"[视图:module] 中文摘要"）
 *    → integration_views.log，中文人读，不做结构化落盘，结构化持久化由外部
 *    集成方自接（规则 12）。
 * 组件源文件内直接调日志宏、字符串就地填充（fmt 风格 {} 语法，经 CA_FMT_FORMAT
 * 格式化——spdlog 平台为 spdlog::fmt_lib，Windows 为 logger/logger_format.h 的
 * 迷你实现），字符串归属组件源文件（事件产生处）；cycle/t_sec 由宏从 world 共享
 * 场景状态取。日志三模式见 logger_modes.h 模式选择区。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_H_

// ============================ 日志模式选择区（编译期） ============================
// DebugView 每周期都会产生，落盘多少、怎么落由集成方决定——本示例示范三种常见
// 写入方式，用宏门控（未选中的模式不参与编译）。每次只启用一个视图模式 + 一个
// 事件模式，重新编译后运行 demo 即可分别验证对应写入方式。
// 模式宏定义见 logger_modes.h（组件头文件也包含它，用于门控成员声明）。
#include "logger/logger_modes.h"
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <string>

// CA_FMT_FORMAT 双后端门面：spdlog 平台转发 spdlog::fmt_lib，Windows 文件后端
// 分支为迷你格式化（CA_LOG_BACKEND_SPDLOG 由 CMake 注入，见 logger_format.h）。
#include "logger_format.h"

namespace component_attachment {
namespace demo {

/// 初始化集成端日志（事件/视图两个文件。库内部 1q_library.log 仅在
/// CA_LIBRARY_FILE_LOG 打开时装配）。幂等。同时把各层验收文件与 RIR
/// 天线/波位 CSV 钉到同一 `output_dir`。main 装配时在会话创建前调用一次；
/// 未初始化时 LogEvent / LogViewSummary 静默跳过。
void InitIntegrationLog(const std::string& output_dir);

/// 视图摘要每隔多少周期写一行（求余：cycle % n == 0）。n<=1 表示每周期都写。
void SetViewLogEveryCycles(std::uint32_t every_cycles);

/// 本周期开始时调用，供 CA_LOG_VIEW 求余。未调用时（单测）视为每周期都写。
void BeginViewLogCycle(std::uint64_t cycle);

/// 本周期是否该写视图行（cycle % every == 0；every<=1 或未 Begin 则为是）。
bool ShouldWriteViewLog();

/// 按周期数与间隔算出应落盘的视图拍数（冒烟用）。
std::uint32_t ViewLogExpectedTicks(std::uint32_t cycles, std::uint32_t every_cycles);

/// 记录一行事件（宏背后）：integration_events logger 事件行 + 计数。未初始化时
/// 静默跳过。
void LogEvent(std::uint64_t cycle, double t_sec, const char* type,
              const std::string& detail);

/// 记录一行调试视图摘要（宏背后；中文人读文本行，写 integration_views logger）。
/// 未初始化时静默跳过。
/// @param[in] module 模块稳定名（"ar"/"eos"/"sbirs"/"sar"，用于按模块计数）。
/// @param[in] text   人读摘要文本（中文，如 "周期=5 完成=是 目标=2 问题=0"）。
void LogViewSummary(const char* module, const std::string& text);

/// 刷盘（demo 结束前调用；库默认 logger 与集成端 logger 一并刷出）。
void FlushIntegrationLog();

/// 事件计数（结束摘要与冒烟断言；按事件发生次数计，与日志模式无关）。
std::size_t EventCount();
std::size_t SbirsEventCount();
std::size_t SarProductEventCount();

/// 调试视图摘要行计数（按 module 名分类；结束摘要与冒烟断言）。
std::size_t ArViewCount();
std::size_t EosViewCount();
std::size_t SbirsViewCount();
std::size_t SarViewCount();
std::size_t ThreatViewCount();
std::size_t RirViewCount();

}  // namespace demo
}  // namespace component_attachment

/**
 * @brief 事件日志宏（关键事件）：组件源文件内就地记录，字符串归属事件产生处。
 *
 * cycle/t_sec 从 world 共享场景状态取（组件发布事件均以 scene 值填充，二者同源）；
 * detail 为 fmt 风格格式化串（{} 占位；spdlog 分支持留字面量编译期检查）。
 * 模式一（KEY）下仍逐条落盘；模式二（AGGREGATE）下并入周期聚合行。
 * @param[in] world World 左值引用（取 scene_state().cycle / .t_sec）
 * @param[in] type  事件类型稳定字符串（如 "target_confirmed"）
 * @param[in] ...   fmt 风格格式串与参数
 */
#define CA_LOG_EVENT(world, type, ...)                                         \
  ::component_attachment::demo::LogEvent(                                      \
      (world).scene_state().cycle, (world).scene_state().t_sec, (type),        \
      CA_FMT_FORMAT(__VA_ARGS__))

#if defined(CA_EVENT_LOG_MODE_KEY)
/**
 * @brief 事件日志宏（周期性重复事件）：与 CA_LOG_EVENT 的区别仅在模式一（KEY）
 * 下不落盘（信号照常发布）——适用于每周期平台状态、kUpdated/kProductSustained
 * 等更新类事件。模式二（AGGREGATE）/模式三（ALL）下行为同 CA_LOG_EVENT。
 * @param[in] world World 左值引用（取 scene_state().cycle / .t_sec）
 * @param[in] type  事件类型稳定字符串
 * @param[in] ...   fmt 风格格式串与参数
 */
#define CA_LOG_EVENT_DUP(world, type, ...) ((void)0)
#else
#define CA_LOG_EVENT_DUP(world, type, ...)                                     \
  ::component_attachment::demo::LogEvent(                                      \
      (world).scene_state().cycle, (world).scene_state().t_sec, (type),        \
      CA_FMT_FORMAT(__VA_ARGS__))
#endif

/**
 * @brief 视图摘要日志宏：组件源文件内就地记录（每周期行，中文人读摘要，字符串
 * 归属组件）。视图三模式的分支选择在组件 Step 内完成（#if 按模式宏）。
 * @param[in] module 模块稳定名（"ar"/"eos"/"sbirs"/"sar"，用于按模块计数）
 * @param[in] ...    fmt 风格格式串与参数（中文，如 "周期={} 完成=是 目标=2 问题=0"）
 */
#define CA_LOG_VIEW(module, ...)                                               \
  do {                                                                         \
    if (::component_attachment::demo::ShouldWriteViewLog()) {                  \
      ::component_attachment::demo::LogViewSummary(                            \
          (module), CA_FMT_FORMAT(__VA_ARGS__));                               \
    }                                                                          \
  } while (0)

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_H_
