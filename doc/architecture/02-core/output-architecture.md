# 数据输出管理模块设计

## 0. 当前实现状态（2026-03）

- 一期已落地中性输出结构 `TrackOutputFrame`
- `RadarController` 已通过输出管理模块统一装配决策输入
- `TracksUpdatedEvent` 仍保留兼容；`TrackOutputPublishedEvent` 仅保留契约，当前默认不发布
- `RadarController` 已对外提供最新 `TrackOutputFrame` 的只读查询能力，支持外部组件主动拉取

## 1. 设计背景

当前代码里已经存在三种“导出”路径，但职责仍然分散：

1. `TrackLifecycleManager::BuildFeatureSnapshot()` 导出 `TargetFeatureList`，主要服务事件广播与外围观测。
2. `TrackLifecycleManager::BuildDecisionSnapshot()` 导出 `DecisionTrackSnapshotList`，来源于内部对象池，但已经做了轻量只读封装。
3. `TrackLifecycleManager::BuildDecisionFrame()` 与 `RadarController::BuildDecisionFrameFromFeatures(...)` 负责拼装 `DecisionInputFrame`，主要服务决策层。

这意味着“对外输出”虽然已经有数据来源，但还没有一个统一的输出管理模块来回答以下问题：

- 哪些字段属于稳定对外输出，哪些字段只属于决策内部语义？
- 如何从内部对象池取数而不泄露 `TrackState`、对象池和滤波运行态？
- 如何同时满足事件总线、记录回放和外部接口等多个消费方，而不把 `RadarController` 继续做胖？

## 2. 已有事实与边界

当前设计里已经有两条重要边界，不建议打破：

### 2.1 对象池不能直接作为输出契约

`common::TrackState` 是对象池内复用的重对象，包含生命周期状态、代次、Gaussian 状态等内部运行态。它的职责是服务 `TrackLifecycleManager`，不是对外输出契约。

因此，输出管理模块**可以以内部对象池为信息来源**，但**不能以 `TrackState` 作为对外结构**。

### 2.2 `DecisionTrackSnapshot` 已经是可复用的轻量快照

`signal-data-contracts.md` 已明确约束：

- `DecisionTrackSnapshot` 是从 Lifecycle 当前活跃轨迹导出的稳定只读快照
- 它不是对象池重对象本身
- 内部对象池、状态机计数、滤波运行态仍保持封装

这说明如果要做输出管理，最自然的基础载荷不是 `TrackState`，而是 `DecisionTrackSnapshot`。

### 2.3 `DecisionInputFrame` 带有明显决策域语义

`DecisionInputFrame` 当前除 `tracks` 外，还包含：

- `environment_jamming_detected`
- `eccm_source_info`
- `association_quality_info`
- `perception_quality_info`

这些字段的意义不是“通用数据输出”，而是“供战术决策链使用的周期输入事实”。把它直接升级为统一输出契约，会让输出模块和决策模块强绑定。

## 3. 方案对比

| 方案 | 可读性 | 安全性/封装 | 性能 | 复杂度 | 可维护性 | 结论 |
|------|--------|-------------|------|--------|----------|------|
| 直接对外暴露 `TrackState` / 对象池 | 最差，外部会看到内部运行态 | 最差，泄露生命周期与滤波细节 | 表面最好，但收益不成立 | 最低 | 最差 | 不采用 |
| 直接复用 `DecisionInputFrame` 作为统一输出结构 | 中等，字段对“输出”并不自解释 | 中等，未泄露对象池，但暴露决策语义 | 好 | 最低 | 中等偏差，输出跟随决策演化抖动 | 仅适合临时过渡 |
| 新建输出帧，复用 `DecisionTrackSnapshot` 作为轨迹载荷 | 最好，输出语义独立 | 最好，继续隔离对象池 | 好，仍是轻量快照复制 | 中等 | 最好 | 推荐 |

## 4. 推荐方案

推荐采用“**专用输出帧 + 复用 `DecisionTrackSnapshot`**”的方式。

核心判断如下：

1. **复用轨迹快照，不复用整帧语义。**
   `DecisionTrackSnapshot` 已经满足“内部对象池 -> 外部轻量快照”的需求，可以直接复用。
2. **输出模块应拥有自己的帧语义。**
   输出模块面对的是事件、记录、接口下发，不应把 `DecisionInputFrame` 的决策字段当成默认必备字段。
3. **决策层仍保留自己的输入结构。**
   `DecisionInputFrame` 继续作为决策层契约，由适配器或控制器在输出快照之上补齐环境/关联/感知摘要。

## 5. 建议模块职责

建议把“数据输出管理模块”设计成**核心层的应用服务**，位于 `RadarController` 与对外消费方之间，而不是继续塞进 `TrackLifecycleManager` 或对象池。

建议职责：

- 从 `ITrackLifecycleManager` 拉取只读轨迹快照
- 组装统一的周期输出帧
- 按消费方策略裁剪、排序或过滤输出内容
- 将输出分发到事件总线、记录器、外部接口适配器

明确不做的事：

- 不推进轨迹生命周期
- 不直接操作对象池 `Acquire/Release`
- 不承担决策推理
- 不直接改写 `SignalPipeline` 或 `TrackLifecycleManager` 内部状态

## 6. 推荐数据分层

### 6.1 基础层：复用现有 `DecisionTrackSnapshot`

基础轨迹输出建议直接复用 `DecisionTrackSnapshotList`，因为它已经满足：

- 来源于内部轨迹对象池
- 只读
- 字段稳定
- 不暴露滤波与对象池实现细节

### 6.2 输出层：定义中性的周期输出帧

建议新增一个**中性输出帧**（名称可后续确认，例如 `OutputFrame` / `TrackOutputFrame`），它至少包含：

| 字段 | 作用 |
|------|------|
| `cycle_index` | 标识输出所属周期 |
| `batch_id` | 标识输出所属批次 |
| `tracks` | 轨迹快照列表，建议直接使用 `DecisionTrackSnapshotList` |
| `published_track_count` | 输出后实际发布的轨迹数 |
| `confirmed_track_count` | 已确认轨迹数，便于外围系统快速判定稳定性 |
| `contains_lost_tracks` | 是否包含 lost 轨迹，便于订阅方选择显示/忽略 |

上表是**设计建议字段**，不是当前已实现 API。

### 6.3 决策层：继续使用 `DecisionInputFrame`

决策层仍消费 `DecisionInputFrame`，但其构造来源应改为：

```text
对象池 / Lifecycle
  -> DecisionTrackSnapshotList
  -> 输出管理模块组装中性输出帧
  -> 决策适配层补齐环境/关联/感知摘要
  -> DecisionInputFrame
```

这样做的好处是，`tracks` 部分只维护一份快照语义，而决策附加事实仍然归决策链所有。

## 7. 与现有代码的协作方式

当前 `RadarController::RunOnce()` 在生命周期更新后同时做三件事：

1. 调 `BuildFeatureSnapshot()` 给 `TracksUpdatedEvent`
2. 调 `BuildDecisionFrame()` 给决策层
3. 自己拼装环境/关联/感知摘要

这说明控制器现在兼任了“输出拼装器”和“决策装配器”两种职责。推荐改造成：

```text
SignalPipeline.RunCycle()
  -> TrackLifecycleManager.Update()
  -> DataOutputManager.BuildFrameFromLifecycle()
  -> EventBus / Recorder / External Adapter
  -> DecisionFrameAdapter.ToDecisionInputFrame()
  -> TacticalDecisionEngine
```

这样 `RadarController` 只保留流程编排，不再维护多套导出逻辑。

## 8. 是否复用决策输入结构

结论：**不建议直接复用整个 `DecisionInputFrame` 作为数据输出管理模块的统一结构；建议复用其内部的 `DecisionTrackSnapshot`。**

原因：

1. `DecisionInputFrame` 的命名和语义都明确指向“决策输入”，不是通用输出。
2. 其中的 ECCM、关联质量、探测质量字段对很多输出消费方是冗余甚至误导信息。
3. 一旦决策层新增字段，输出契约会被被动放大，导致对外接口频繁波动。
4. 输出管理模块的核心职责是“稳定对外表达”，而不是“复用一切现有字段”。

可接受的折中方案：

- **短期过渡**：内部可以继续借用 `DecisionInputFrame` 参与决策调用，避免大改。
- **正式对外**：对事件、记录、接口等输出场景，引入独立输出帧。
- **共享部分**：统一复用 `DecisionTrackSnapshot`，必要时后续再把它抽象成更中性的 `TrackSnapshot`。

## 9. 建议落地步骤

1. 第一步只新增输出管理模块，输入为 `ITrackLifecycleManager::BuildDecisionSnapshot()`，不改对象池和生命周期实现。
2. 第二步让事件总线与记录器改消费中性输出帧，而不是 `TargetFeatureList`。
3. 第三步把 `DecisionInputFrame` 的构造收敛为“输出帧 + 决策附加摘要”的适配过程。
4. 第四步再评估是否需要把 `DecisionTrackSnapshot` 重命名或上提为更中性的公共快照类型。

## 10. 最终建议

如果目标是“从内部目标对象池稳定对外输出”，最小风险方案是：

- **数据源**：内部对象池，但只能经 `TrackLifecycleManager` 导出
- **共享载荷**：复用 `DecisionTrackSnapshot`
- **输出帧**：新建中性输出结构
- **决策输入**：继续保留 `DecisionInputFrame`，通过适配层补齐

这样既复用了已有正确抽象，又避免让输出模块被决策语义反向绑架。
