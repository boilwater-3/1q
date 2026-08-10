/**
 * @file logger_modes.h
 * @brief 集成端日志模式选择区（纯宏定义，零依赖）。
 *
 * 独立成头：组件头文件（ar/eos/sbirs_sensor_component.h 的模式二 prev 状态表
 * 成员按模式门控）与 logger.h 共享同一份模式定义，避免头文件解析顺序问题。
 *
 * DebugView 每周期都会产生，落盘多少、怎么落由集成方决定——本示例示范三种
 * 常见写入方式，用宏门控（未选中的模式不参与编译）。每次只启用一个视图模式
 * + 一个事件模式，重新编译后运行 demo 即可分别验证对应写入方式。
 *
 * 模式选择两条途径（互斥，编译期生效）：
 * 1. CMake 构建时控制（推荐，无需改源码）：cmake 传
 *    -DCA_VIEW_LOG_MODE=summary|nonnominal|delta
 *    -DCA_EVENT_LOG_MODE=all|key|aggregate
 *    （见 examples/component_attachment/CMakeLists.txt；不传则用下方源码默认）；
 * 2. 调试时直接改本文件：取消注释对应宏（每次只留一个）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_MODES_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_MODES_H_

// 视图模式（三选一）：未被 CMake 或本文件指定时默认模式二（跨周期增量）。
#if !defined(CA_VIEW_LOG_MODE_SUMMARY) && !defined(CA_VIEW_LOG_MODE_NONNOMINAL) && \
    !defined(CA_VIEW_LOG_MODE_DELTA)
#define CA_VIEW_LOG_MODE_DELTA  // 模式二（默认）：只落跨周期状态变化行
#endif

// 事件模式（三选一）：未被 CMake 或本文件指定时默认模式一（只记关键事件）。
#if !defined(CA_EVENT_LOG_MODE_ALL) && !defined(CA_EVENT_LOG_MODE_KEY) && \
    !defined(CA_EVENT_LOG_MODE_AGGREGATE)
#define CA_EVENT_LOG_MODE_KEY  // 模式一（默认）：只记关键事件（重复事件不落盘）
#endif

// 调试时改源码切换（改完重新编译；与 CMake 途径互斥，见文件头注释）：
// #define CA_VIEW_LOG_MODE_SUMMARY         // 模式三：每周期一行人读摘要
// #define CA_VIEW_LOG_MODE_NONNOMINAL      // 模式一：只落非标称目标行
// #define CA_VIEW_LOG_MODE_DELTA           // 模式二：只落跨周期状态变化行
// #define CA_EVENT_LOG_MODE_ALL            // 模式三：逐条全量
// #define CA_EVENT_LOG_MODE_KEY            // 模式一：只记关键事件
// #define CA_EVENT_LOG_MODE_AGGREGATE      // 模式二：周期聚合

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_MODES_H_
