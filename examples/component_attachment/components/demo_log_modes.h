/**
 * @file demo_log_modes.h
 * @brief 集成端日志模式选择区（纯宏定义，零依赖）。
 *
 * 独立成头：组件头文件（ar/eos/sbirs_sensor_component.h 的模式二 prev 状态表
 * 成员按模式门控）与 demo_log.h 共享同一份模式定义，避免头文件解析顺序问题。
 * DebugView 每周期都会产生，落盘多少、怎么落由集成方决定——本示例示范三种
 * 常见写入方式，用宏门控（未选中的模式不参与编译）。每次只启用一个视图模式
 * + 一个事件模式，重新编译后运行 demo 即可分别验证对应写入方式。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_DEMO_LOG_MODES_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_DEMO_LOG_MODES_H_

// 视图模式：
#define CA_VIEW_LOG_MODE_SUMMARY         // 模式三（默认）：每周期一行人读摘要
// #define CA_VIEW_LOG_MODE_NONNOMINAL  // 模式一：只落非标称目标行
// #define CA_VIEW_LOG_MODE_DELTA       // 模式二：只落跨周期状态变化行

// 事件模式：
#define CA_EVENT_LOG_MODE_ALL            // 模式三（默认）：逐条全量
// #define CA_EVENT_LOG_MODE_KEY        // 模式一：只记关键事件（重复事件不落盘）
// #define CA_EVENT_LOG_MODE_AGGREGATE  // 模式二：周期聚合（每周期一行：类型+次数）

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_DEMO_LOG_MODES_H_
