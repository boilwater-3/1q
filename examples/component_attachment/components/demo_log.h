/**
 * @file demo_log.h
 * @brief 自定义实体-组件示例：集成端日志设施（外部集成惯用法示范）。
 *
 * 与库内部日志（src/common/logging/ProjectLog.h 的 PROJECT_LOG_* 宏，走 spdlog
 * 默认 logger）区分成两个日志模块，输出到两个文件：
 *   1) 库内部日志：库内 PROJECT_LOG_* 调用 → spdlog 默认 logger；本设施把默认
 *      logger 装配为文件 sink（1q_library.log，时间戳 + 级别 + 消息），库日志
 *      因此落到独立文件，宿主（集成端）拥有 logger 生命周期。
 *   2) 集成端日志：命名 logger "integration"（stdout + integration.log，pattern
 *      仅为消息体 %v），承载事件行（"[event:type] cycle=... t_sec=... detail"）
 *      与各组件每周期调试视图摘要行（"[view:module] cycle=... 关键计数"）——
 *      日志给人读，不做结构化落盘；结构化持久化由外部集成方接入自己的
 *      日志/事件系统（规则 12），示例不再内置 JSON 序列化器。
 *
 * 组件源文件内直接调日志宏、字符串就地填充（fmt 风格 {} 语法，经
 * spdlog::fmt_lib 格式化——兼容 spdlog 外部/内置 fmt 两种构建形态），字符串
 * 归属组件源文件（事件产生处），集成侧不再集中拼接。cycle/t_sec 由 CA_LOG_EVENT
 * 宏从 world 共享场景状态取（组件发布事件均以 scene 值填充，二者同源）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_DEMO_LOG_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_DEMO_LOG_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include <spdlog/spdlog.h>

namespace component_attachment {
namespace demo {

/// 初始化集成端日志（装配 1q_library.log 默认 logger + integration.log 命名
/// logger；幂等）。main 装配时在会话创建前调用一次（库日志入库文件，避免
/// spdlog 自动创建 stdout 默认 logger）；未初始化时 LogEvent / LogViewSummary
/// 静默跳过（单元测试不初始化也可安全编译运行）。
void InitIntegrationLog(const std::string& output_dir);

/// 记录一行事件（宏背后）：integration logger 事件行 + 计数。未初始化时静默跳过。
void LogEvent(std::uint64_t cycle, double t_sec, const char* type,
              const std::string& detail);

/// 记录一行调试视图摘要（宏背后；人读文本行）。未初始化时静默跳过。
/// @param[in] module 模块稳定名（"ar"/"eos"/"sbirs"，用于按模块计数）。
/// @param[in] text   人读摘要文本（如 "cycle=5 completed=true tracks=2 issues=0"）。
void LogViewSummary(const char* module, const std::string& text);

/// 刷盘（demo 结束前调用；库默认 logger 与集成端 logger 一并刷出）。
void FlushIntegrationLog();

/// 事件计数（结束摘要与冒烟断言）。
std::size_t EventCount();
std::size_t SbirsEventCount();
std::size_t SarProductEventCount();

/// 调试视图摘要行计数（按 module 名分类；结束摘要与冒烟断言）。
std::size_t ArViewCount();
std::size_t EosViewCount();
std::size_t SbirsViewCount();

}  // namespace demo
}  // namespace component_attachment

/**
 * @brief 事件日志宏：组件源文件内就地记录（字符串归属事件产生处）。
 *
 * cycle/t_sec 从 world 共享场景状态取（组件发布事件均以 scene 值填充，
 * 二者同源）；detail 为 fmt 风格格式化串（{} 占位，编译期格式检查）。
 * @note world 仅允许传左值引用（宏内求值两次：cycle 与 t_sec）；"未初始化
 * 静默跳过"指跳过落盘/计数，格式化成本（fmt::format）每次调用仍会发生——
 * 单元测试路径每周期会为每个事件付一次格式化。
 * @param[in] world World 左值引用（取 scene_state().cycle / .t_sec）
 * @param[in] type  事件类型稳定字符串（如 "target_confirmed"）
 * @param[in] ...   fmt 风格格式串与参数
 */
#define CA_LOG_EVENT(world, type, ...)                                         \
  ::component_attachment::demo::LogEvent(                                      \
      (world).scene_state().cycle, (world).scene_state().t_sec, (type),        \
      ::spdlog::fmt_lib::format(__VA_ARGS__))

/**
 * @brief 视图摘要日志宏：组件源文件内就地记录（每周期一行人读摘要，字符串
 * 归属组件；日志给人读，示例不做结构化落盘）。
 * @param[in] module 模块稳定名（"ar"/"eos"/"sbirs"，用于按模块计数）
 * @param[in] ...    fmt 风格格式串与参数（如 "cycle={} tracks={} issues={}"）
 */
#define CA_LOG_VIEW(module, ...)                                               \
  ::component_attachment::demo::LogViewSummary(                                \
      (module), ::spdlog::fmt_lib::format(__VA_ARGS__))

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_DEMO_LOG_H_
