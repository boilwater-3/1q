# 02 — 核心协调层

本层是调度编排中枢，连接公共 API 层与下游执行层（信号处理、决策、环境）。

## 职责

- **周期调度**：驱动单周期处理流程 Input → Signal → Decision → Output
- **上下文管理**：封装周期输入输出的双向通道
- **事件分发**：提供即时同步与周期双缓冲两种消息传递语义
- **输出装配**：将内部轨迹快照整理为稳定的对外输出视图

## 关键模块设计概述

### RadarController — 调度器

- **模式**：Mediator + PIMPL，作为各执行层之间的唯一协调点
- **依赖注入**：通过构造函数注入四个核心接口
  - `IRadarContext` — 周期上下文（输入读取 + 命令提交）
  - `ISignalPipeline` — 信号处理管线
  - `ITacticalDecisionEngine` — 战术决策引擎
  - `IEnvironmentService` — 环境建模服务
- **周期调度循环**：`RunOnce()` 执行单周期，依次驱动各接口完成处理
- **输出访问**：实现 `IRadarOutputReader`，提供 `GetLatestTrackOutputFrame()` 只读查询

### IRadarContext / MutableRadarContext — 周期上下文

- **IRadarContext**（抽象接口）：定义周期上下文的读写契约
  - 查询：`GetTargetFeatures()`、`GetPlatformAttitude()`、`GetCycleDeltaTimeSec()`
  - 提交：`SubmitControlCommand(RadarCommand)`
  - 可选：`UpdateRadarControlProfile()` — 保存控制真值供下周期使用
- **MutableRadarContext**（默认实现）：
  - `BeginCycle(RadarCycleInput)` — 加载输入、清空输出缓冲
  - 内部维护目标列表、平台姿态、时间步、已提交命令、最新控制规约

### EventBus / CycleEventBus — 事件总线

详见 [eventbus-architecture.md](./eventbus-architecture.md)。

- **IEventBus**（抽象接口）：模板化的类型安全订阅/发布模型，使用 `EventToken` 作为统一撤销凭证
- **EventBus**（即时总线）：基于 `eventpp::EventDispatcher` 的同步即时分发，`Publish()` 立即触发所有订阅者
- **CycleEventBus**（周期总线）：基于 `eventpp::EventQueue` 的双缓冲队列
  - `BeginCycle()` → 切换读写队列索引
  - 业务层 `Enqueue()` → 事件进入写队列
  - `DispatchCurrentCycle()` → 处理读队列中的事件
  - 防止周期内事件反馈环路，确保仿真周期边界清晰

### DataOutputManager — 输出管理

详见 [output-architecture.md](./output-architecture.md)。

- **IDataOutputManager**（抽象接口）：定义输出帧装配契约
  - `BuildTrackOutputFrame()` — 将轨迹快照装配为中性输出帧（`TrackOutputFrame`）
  - `BuildDecisionInputFrame()` — 融合轨迹帧、ECCM 信息、关联/感知质量指标，构建决策输入帧
- **DataOutputManager**（默认实现）：计算汇总指标（轨迹计数、状态标记），解耦信号层与决策层

## 模块交互

```text
RadarSession.Step(input)
  └─→ RadarController.RunOnce()
        ├─→ MutableRadarContext.BeginCycle(input)
        ├─→ IEnvironmentService.Update(scene)
        ├─→ ISignalPipeline.Process(context)
        │     └─→ 输出 DecisionTrackSnapshotList
        ├─→ DataOutputManager.BuildTrackOutputFrame(snapshots)
        ├─→ DataOutputManager.BuildDecisionInputFrame(frame, eccm, quality)
        ├─→ ITacticalDecisionEngine.Process(decision_input)
        │     └─→ 输出 RadarCommand[]
        ├─→ IRadarContext.SubmitControlCommand(cmd)
        └─→ 缓存 TrackOutputFrame 供外部查询
```
