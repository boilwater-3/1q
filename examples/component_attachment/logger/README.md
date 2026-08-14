# 集成端日志设施（logger/）

> 本目录是示例的**集成端日志设施**——示范"外部集成方怎么自己组织日志/落盘"，不属于
> oneq 库的 public surface。直接使用 conanfile 的 spdlog 依赖（fmt 风格 `{}` 格式化，
> 编译期格式检查）。
>
> 想先建立整体认知（三层输出模型 → 两种日志 → 三模式）再读本文件，见使用教程
> [`docs/practice/output_view_and_logging_guide.md`](../../../../docs/practice/output_view_and_logging_guide.md)。

## 两个日志模块

与库内部 `src/common/logging/ProjectLog.h` 区分——库日志走 spdlog 默认 logger，宿主
拥有 logger 生命周期：

| 模块 | 后端 | 输出文件 |
| --- | --- | --- |
| **库内部日志** | 库内 `PROJECT_LOG_*` 宏 → spdlog 默认 logger（`InitIntegrationLog` 装配为文件 sink） | `1q_library.log`（时间戳 + 级别 + 消息） |
| **集成端日志** | spdlog 命名 logger `"integration_events"` / `"integration_views"`（均带 stdout，pattern 仅为消息体） | `integration_events.log`（事件行）+ `integration_views.log`（各组件每周期调试视图行） |

## 文件

| 文件 | 作用 |
| --- | --- |
| `logger.h` / `logger.cpp` | 日志设施主体：`InitIntegrationLog`（装配三个 logger）、`LogEvent` / `LogViewSummary`（宏背后）、事件/视图计数器；对外宏 `CA_LOG_EVENT` / `CA_LOG_EVENT_DUP` / `CA_LOG_VIEW` |
| `logger_modes.h` | 日志模式选择区（纯宏定义，零依赖）：视图/事件各三模式，编译期门控 |
| `logger_i18n.h` | issue code → 中文名适配表（纯查表零依赖；不翻译/不解析 message，量值走 DebugView 结构化字段；未知 code 回退英文原文） |

## 字符串归属与写入方式

集成端日志的字符串**归属组件源文件**（事件产生处），通过日志宏就地填充——外部集成的
典型形态（宏背后接消费方自己的日志/落盘设施）。**日志内容为中文人读文本**（给人类看，
不做结构化落盘；规则 12 的结构化持久化由外部集成方接入自己的日志/事件系统）：

```cpp
// 组件源文件内（发布信号前）
CA_LOG_EVENT(world, "target_confirmed", "目标={} 位置=({:.5f},{:.5f})",
             static_cast<unsigned long long>(confirmed.target_id),
             confirmed.position.latitude_deg, confirmed.position.longitude_deg);
world.signals().on_target_confirmed(confirmed);
```

`CA_LOG_EVENT(world, type, ...)` 的 cycle/t_sec 取自共享场景状态（与事件字段同源），背后
设施把事件行（`[事件:type] 周期=... 时间=...s 中文详情`）写入 `integration_events.log`
并打印控制台，另维护事件计数（摘要/冒烟断言用）。

事件宏分两类：
- `CA_LOG_EVENT`（关键事件：确认/丢失/首发现/产出/失败/航点/指令等）
- `CA_LOG_EVENT_DUP`（周期性重复事件：每周期平台状态、`kUpdated`/`kProductSustained`
  更新类、辐射源假设、融合更新——仅在事件模式一（KEY）下不落盘，信号照常发布）

集成方替换该设施即接入自己的日志系统；单元测试不初始化日志设施，宏调用静默跳过
（no-op）。

**调试视图落盘在组件内直写**（规则 12）：各传感器组件（AR/EOS/SBIRS/SAR）的 `Step`
在构建 `LastDebugView()` 后直写中文人读行到集成端视图日志（`integration_views.log`）——
日志给人读，示例不做结构化落盘：`session_contract.md` 规则 12 的"调用方结构化持久化
DebugView"由外部集成方接入自己的日志/事件系统实现，结构化格式与字段布局由调用方自定
（参考 `*OutputDebugView` 字段集合直接转写）。

**Lifecycle 事件字符串化**：库内 `*LifecycleRecorder` 产出的生命周期事件
（`GetLastEvents()`，如 AR 首确认/失跟、EOS/SBIRS 首发现/丢失、SAR 产品事件）为纯
struct、库内无字符串化工具——其"转字符串写日志"由各组件源文件内宏的手写中文格式串
承担（`"类型=首发现 探测ID={} 目标={} 信噪比={:.1f}dB 方位={:.1f}°"` 等，kind/status
枚举在组件内做中文名映射），字符串归属组件（组件自描述）；DebugView 同理以组件内摘要
行直写，示例层不内置 JSON 序列化器。

## 日志三模式（宏门控，编译期）

DebugView 每周期都会产生，落盘多少、怎么落由集成方决定——`logger_modes.h` 顶部"模式
选择区"示范三种常见写入方式，未选中的模式**不参与编译**。模式选择有两条途径（互斥）：

1. **CMake 构建时控制**（推荐，无需改源码）：`-DCA_VIEW_LOG_MODE=summary|nonnominal|delta`
   `-DCA_EVENT_LOG_MODE=all|key|aggregate`（不传则用源码默认；非法值 FATAL_ERROR）；
2. **源码调试时**：改 `logger_modes.h` 里的注释（每次只启用一个视图模式 + 一个事件模式）
   重新编译。

默认模式：**视图模式二（跨周期增量）+ 事件模式一（只记关键事件）**：

| 模式 | 宏 | 行为 |
| --- | --- | --- |
| 视图模式一（只落非标称行） | `CA_VIEW_LOG_MODE_NONNOMINAL` | 每周期只把非标称目标（AR 非 `kConfirmed`；EOS/SBIRS 非 `kDetected`）逐行写日志，全标称时写一行"全部正常"；日志量 ∝ 异常数 |
| 视图模式二（跨周期状态增量，**默认**） | `CA_VIEW_LOG_MODE_DELTA` | 只写状态与上一周期不同的目标行（上一周期状态表由组件持有）；无变化时写一行"无状态变化"；日志量 ∝ 变化数 |
| 视图模式三（每周期摘要行） | `CA_VIEW_LOG_MODE_SUMMARY` | 每周期一行中文摘要（周期/完成与否/目标状态明细带**结构化量值**——方位/俯仰/距离/RCS，库 DebugView 输入实体回填，未检测也可见；问题列表为 **code + 中文名**（`logger_i18n.h` 查表，未知 code 回退英文 message 原文，不翻译/不解析 message），日志量恒定） |
| 事件模式一（只记关键事件，**默认**） | `CA_EVENT_LOG_MODE_KEY` | `CA_LOG_EVENT` 逐条落盘，`CA_LOG_EVENT_DUP`（周期性重复事件）不落盘 |
| 事件模式二（周期聚合） | `CA_EVENT_LOG_MODE_AGGREGATE` | 每周期把全部事件聚合为一行（`[事件聚合] 周期=N 事件数=M [中文名×次数, ...]`） |
| 事件模式三（逐条全量） | `CA_EVENT_LOG_MODE_ALL` | 事件逐条落盘 |

SAR 为**阶段型视图**（无逐目标状态），不适用目标级三模式落盘，只实现每周期摘要行
（执行状态/完成阶段/L1/L3 成像标志/SNR/点目标数/问题列表）。
