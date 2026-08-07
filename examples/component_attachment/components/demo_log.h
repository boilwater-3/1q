/**
 * @file demo_log.h
 * @brief 自定义实体-组件示例：事件日志宏设施（外部集成惯用法示范）。
 *
 * 外部集成中事件日志的典型形态：组件源文件内直接调日志宏、detail 字符串
 * 就地填充（宏背后接消费方自己的日志/落盘设施）。本文件提供对应的演示
 * 实现——CA_LOG_EVENT 宏 + 背后设施（控制台打印 + events.csv 结构化落盘
 * + 事件计数）。事件类型名 → 稳定字符串，detail 为可读摘要文本；字符串
 * 归属组件源文件（事件产生处），集成侧不再集中拼接。
 *
 * Fmt 为示例共享格式化原语（snprintf 风格，按返回值动态分配避免截断），
 * 宏展开与 demo_output 侧共用。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_DEMO_LOG_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_DEMO_LOG_H_

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

namespace component_attachment {
namespace demo {

#if defined(__GNUC__) || defined(__clang__)
#define CA_LOG_PRINTF_ATTR __attribute__((format(printf, 1, 2)))
#else
#define CA_LOG_PRINTF_ATTR
#endif

/// snprintf 风格格式化辅助（按返回值动态分配，避免截断；printf 格式检查）。
inline std::string Fmt(const char* format, ...) CA_LOG_PRINTF_ATTR;
inline std::string Fmt(const char* format, ...) {
  va_list args;
  va_start(args, format);
  va_list args_copy;
  va_copy(args_copy, args);
  const int length = std::vsnprintf(nullptr, 0, format, args_copy);
  va_end(args_copy);
  std::string out(static_cast<std::size_t>(length), '\0');
  std::vsnprintf(out.data(), out.size() + 1, format, args);
  va_end(args);
  return out;
}

/// 初始化事件日志（打开 events.csv；幂等）。main 装配时调用一次；
/// 未初始化时 LogEvent 静默跳过（单元测试不初始化也可安全编译运行）。
void InitEventLog(const std::string& output_dir);

/// 记录一行事件（宏背后）：控制台 + events.csv + 计数。未初始化时静默跳过。
void LogEvent(std::uint64_t cycle, double t_sec, const char* type,
              const std::string& detail);

/// 刷盘（demo 结束前调用）。
void FlushEventLog();

/// 事件计数（结束摘要与冒烟断言）。
std::size_t EventCount();
std::size_t SbirsEventCount();
std::size_t SarProductEventCount();

}  // namespace demo
}  // namespace component_attachment

/**
 * @brief 事件日志宏：组件源文件内就地记录（字符串归属事件产生处）。
 *
 * cycle/t_sec 从 world 共享场景状态取（组件发布事件均以 scene 值填充，
 * 二者同源）；detail 为 printf 风格格式化串。
 * @note world 仅允许传左值引用（宏内求值两次：cycle 与 t_sec）；"未初始化
 * 静默跳过"指跳过落盘/计数，格式化成本（vsnprintf + 分配）每次调用仍会
 * 发生——单元测试路径每周期会为每个事件付一次格式化。
 * @param[in] world World 左值引用（取 scene_state().cycle / .t_sec）
 * @param[in] type  事件类型稳定字符串（如 "target_confirmed"）
 * @param[in] ...   printf 风格格式串与参数
 */
#define CA_LOG_EVENT(world, type, ...)                                         \
  ::component_attachment::demo::LogEvent(                                      \
      (world).scene_state().cycle, (world).scene_state().t_sec, (type),        \
      ::component_attachment::demo::Fmt(__VA_ARGS__))

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_DEMO_LOG_H_
