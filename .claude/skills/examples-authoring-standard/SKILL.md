---
name: examples-authoring-standard
description: Use when creating, editing, or reviewing any code under examples/ in the 1Q repo（示例层编写规范）——components、scenes、app、core、logger、common 下的 .cpp/.h，或评审意见涉及示例组件。Trigger on "示例"、"examples 组件"、"场景 exe"、"给集成方的示例"、"demo 写法"，以及任何 examples/ 文件的新增/修改/评审任务。规范来源：2026-08 用户对 rir_sensor_component 的三轮评审纠正。
---

# 1Q Examples 编写规范（集成方视角）

examples/ 是给**集成方**的样板代码：读者是 VS2015 / C++11 / 无 fmt 环境的外部工程师，不是库内部开发者。一切写法以"集成方打开 .cpp 就能看懂、直接搬用"为第一原则。以下每条规则都来自用户明确的评审裁定，违反即返工。

## 读者与措辞

- 假设读者不了解库内部（模块契约、规则编号、内部术语）。出现"他需要知道 X 才能懂"的注释就是错的。
- **永不写「规则 N」等内部契约编号**进示例注释（如"规则 10/12/13e"）——契约号只留在库内源码与设计文档。
- 注释保持简洁、按模块说明：行为注释一两句白话讲清"谁→谁、什么沿、什么效果"，不长篇大论。
- 跨文件数据流给精确指针：「见 <文件> 的 <函数>（谁每周期注入）」，不让读者一步步回翻。
- 集成方桥接措辞：写明示例内机制 + 集成方等价物。例："融合组件每周期直接来取（示例内是组件引用拉取；集成方对应把记录发给融合组件的消息）"。

## 日志（CA_LOG_EVENT / CA_LOG_VIEW）

- **每处 CA_LOG_* 调用上方保留搬用串**：`const std::string xxx_log = std::string("…") + std::to_string(…) + …;`，配注释"与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）"。这是给无 fmt 客户的配方，**永不删除或改写**（用户纠正过一次：删块被要求原样恢复）。搬用串随其日志行同生同灭——行删了串才删。
- 行内片段（如摘要行的逐目标 parts）不是日志行，不加搬用串；在其声明处注释说明"拼进下方 XX 行的 YY 字段随行写日志"。
- 视图日志三密度模式（编译期宏三选一，见 logger/logger_modes.h）：
  - nonnominal / delta：无事发生的周期**整周期静默**，连"全部正常""无状态变化"这类兜底行也不写（用户明确：兜底行也是刷屏）；
  - summary（默认）：每周期恰好一行完整摘要。
- `LogDebugView` 函数头给三模式输出示例注释块，样例数值取自**真实运行日志**（跑一次场景，从 integration_views.log 抄真实行）。
- 事件沿（确认/丢失/作废/排除原因变化）只在迁移沿写，不逐周期重复。

## 头文件与类布局

- .h 文件头与类文档只留一行 `@brief`；实现细节的长描述放对应 .cpp 的文件头，两处不重复（用户裁定删除 .h 长段）。
- **私有成员平铺**：相关成员相邻排列、不同用途组之间空一行（库会话/记录器 → 固定事实 → 本周期产物 → 跨周期沿状态 → 开关与计数）。**不收嵌套结构体**（用户纠正过一次：嵌套 struct 的双层前缀更绕）。
- 每成员保留 `/**< … */` 行尾说明；成员语义要写"是什么+干什么用"（例：驻留时长 → "单次识别驻留时长（mission 配置，s）：每周期喂给会话的 RF 发射窗口长度"），不用只有作者懂的缩写。
- 中文 Doxygen 风格遵循 cpp-chinese-doxygen skill（Javadoc `/** */`、`@brief`、`/**< */`）。

## 组件结构模式（与兄弟组件同形）

- `TryApplyRuntimeConfig` 必须同步 `powered_on_`：`if (applied && patch.has_sensor_enabled) powered_on_ = patch.sensor_enabled;`（组件层电源门控，六个传感器组件统一）。
- `Step()` 结构：关机早退 → 单一可变场景引用（`auto& scene = static_cast<AppSceneState&>(…)`，不用 const/mutable 双重 cast）→ 组输入 → 驱动会话 → 每周期构建调试视图快照（视图行的数据源）→ 发布区块。
- 发布区块每个调用带行尾注释（`PublishXxx(…)  // 谁的什么沿 → 去向`）；每个 Publish 函数头部一两句行为说明。
- 会话输入组装（BuildCycleInput）处解释"为什么这个字段要过函数"（如 RF 世界共享、剔除自身发射）。

## 完成检查

- [ ] 注释里无「规则 N」、无库内部术语、无长篇段落
- [ ] 每个新增 CA_LOG_* 行都有配套搬用串；没有删除任何既有搬用串
- [ ] nonnominal/delta 分支无兜底行；LogDebugView 头有三模式真实示例
- [ ] .h 只有一行 brief；成员平铺空行分组、无嵌套 struct
- [ ] 与兄弟组件（ar/esr/eos/sbirs/sar_sensor_component）同形的部分已对齐
- [ ] 三种视图模式都编译验证（默认构建 + 对 NONNOMINAL/DELTA 各做一次 -fsyntax-only），场景 exe 冒烟输出与预期一致
