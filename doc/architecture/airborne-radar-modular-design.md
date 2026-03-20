# 机载雷达仿真库：模块化架构设计

本文档说明 1Q 机载雷达仿真库的模块化设计原理，聚焦**为什么这样分层**以及**模块之间如何解耦**。各层详细设计见对应子目录文档。

## 1. 五层架构与设计原则

库采用五层分层架构，每层职责单一、边界清晰：

| 层级 | 职责 | 核心设计模式 |
|------|------|-------------|
| [01-api](./01-api/) | 面向外部调用方的门面封装 | Facade、PIMPL、Builder、Factory |
| [02-core](./02-core/) | 周期调度与编排中枢 | Mediator、依赖注入、双缓冲 EventBus |
| [03-signal](./03-signal/) | 单周期信号处理链路 | Pipeline、Strategy（关联组件可替换）、Observer（种子回灌） |
| [04-decision](./04-decision/) | 战术评估与控制真值生成 | 评估器管线、Proposal 归约、域级状态机 |
| [05-environment](./05-environment/) | 环境事实建模 | Repository（特征数据库）、Service（环境快照） |

贯穿全部层级的核心设计原则：

1. **依赖倒置（DIP）**：各层通过抽象接口交互（`ISignalPipeline`、`ITacticalDecisionEngine`、`IEnvironmentService`、`IRadarContext`），具体实现可替换
2. **单一职责（SRP）**：Controller 只做编排，不承载业务逻辑；环境层只产事实，不做策略选择；决策层只输出控制意图，不直接修改探测器参数
3. **PIMPL 隐藏实现**：`RadarSession`、`RadarController`、`SignalPipeline` 使用 PIMPL 保证二进制兼容性
4. **值类型传递**：跨层数据全部使用 POD/聚合类型（`TargetFeature`、`TrackOutputFrame`、`RadarControlProfile`），无共享可变状态

![模块化设计架构图](./airborne-radar-modular-design.png)

*(PlantUML 源文件：`airborne-radar-modular-design.puml`)*

## 2. 双层公共 API

```text
┌─────────────────────────────────────────────────┐
│ Layer 1: RadarSession（门面）                     │ ← 80% 场景
│   Step(input) → TrackOutputFrame                │
│   ConfigPresets / RadarSessionConfigBuilder      │
│   TrackOutputQueries / RadarInputValidation      │
└─────────────────────────────────────────────────┘
                    │ 穿透
                    ▼
┌─────────────────────────────────────────────────┐
│ Layer 2: RadarController + 接口注入              │ ← 20% 高级定制
│   自定义 ISignalPipeline / ITacticalDecision-   │
│   Engine / IEnvironmentService / IRadarContext   │
└─────────────────────────────────────────────────┘
```

- **Layer 1** 通过 `RadarSession` PIMPL 门面封装全部内部装配，外部只需 `Step(input)`
- **Layer 2** 暴露 4 个核心接口的注入点，允许替换任意执行层
- **Builder 体系**：`RadarSessionConfigBuilder`（系统配置）、`TargetFeatureBuilder`（目标输入）、`EnvironmentSceneBuilder`（场景构造）提供链式 API，避免多参数位置耦合
- **边界守卫**：公共头严格收敛——信号层仅 3 个公共头，决策层仅 2 个公共头；内部实现头全部在 `src/`

## 3. 核心协调层：Mediator 编排

`RadarController` 作为中介者，是各执行层之间的唯一协调点：

```text
RadarController.RunOnce()
  ├── IRadarContext.BeginCycle(input)       ← 加载输入
  ├── IEnvironmentService.Update(scene)     ← 更新环境
  ├── ISignalPipeline.RunCycle(targets, env)← 探测/关联/跟踪
  ├── DataOutputManager.BuildFrames(...)    ← 装配输出
  ├── ITacticalDecisionEngine.Evaluate(...) ← 战术决策
  ├── IRadarContext.SubmitControlCommand()  ← 缓冲命令
  └── 缓存 TrackOutputFrame                 ← 供外部查询
```

关键约束：
- Controller **不承载**业务逻辑（不计算探测、不做分类、不管理轨迹状态机）
- Controller **不持有**可变共享状态，通过 `IRadarContext` 双向通信
- 事件总线提供两种语义：`EventBus`（同步即时）和 `CycleEventBus`（双缓冲周期延迟），防止周期内事件回调重入

## 4. 信号处理层：领域核心

信号层是计算密集的领域核心，内部按职责拆分为四个子域：

```text
探测域                          关联域
┌────────────────────┐    ┌────────────────────────┐
│ TargetGeometry-    │    │ DistanceMetric（马氏）   │
│   Resolver         │    │ Gater（波门裁剪）        │
│ BeamControlResolver│    │ Hypothesiser（假设生成） │
│ SignalDetector     │───→│ LapjvSolver（最优指派）  │
│ MeasurementError-  │    └────────────────────────┘
│   Model            │              │
└────────────────────┘              ▼
                            跟踪域
                    ┌────────────────────────┐
                    │ TrackLifecycleManager  │
                    │   ├── 状态机（建轨/确认/│
                    │   │    丢失/回收）      │
                    │   ├── Kalman / EKF /   │
                    │   │    IMM 滤波        │
                    │   └── ITrackPool       │
                    │        对象池复用       │
                    └────────────────────────┘
                              │
                    编排域（SignalPipeline）
                    统一单周期 orchestration
```

设计要点：
- **关联组件可替换**：`DistanceMetric`、`Gater`、`Hypothesiser`、`AssignmentSolver` 均为抽象接口，参照 Stone Soup 架构拆分
- **滤波器三级并存**：标准 Kalman → EKF（`ITransitionModel` / `IMeasurementModel` 虚函数注入）→ IMM（Bar-Shalom 四步，每轨一份运行态）
- **对象池与域服务分离**：`ITrackPool` 仅负责内存复用（Acquire/Release），`TrackLifecycleManager` 承载状态机、批号、回收策略
- **种子回灌机制**：Lifecycle 导出 `AssociationTrackSeed` → Controller 注入 → 下周期关联消费；无种子时自动退回 stateless 模式
- **公共边界极简**：对外仅暴露 `ISignalPipeline` 接口和配置类型（3 个公共头），全部内部实现对外不可见

## 5. 决策层：评估器管线

决策层采用三级评估器串行管线，而非互斥策略选择：

```text
DecisionInputFrame
  → ThreatAssessmentEvaluator（威胁识别 + 跨周期记忆）
  → EmissionControlEvaluator（LPI 控制建议）
  → SurvivabilityEvaluator（ECCM 组合策略建议）
  → TacticalProposal[]
  → ControlReducer（归约 + 域级 hold/cooldown + 冲突裁决）
  → RadarControlProfile
```

设计要点：
- **评估器串行有因果**：先分类才能判断是否降低辐射，先判断辐射威胁才能选择抗干扰组合
- **Proposal 可叠加**：ECCM 策略按四个正交维度（空域/频域/时域/能量域）输出，`ControlReducer` 只做冲突限幅，不理解干扰物理
- **跨周期记忆**：`TacticalStateStore` 持有威胁分数缓存、置信度缓存和域级 hold 计数器，支持滞后保持和冷却退出
- **三条关键边界**：环境层只产事实 → 决策层只定策略 → 信号层只执行真值

## 6. 环境层：事实基础设施

环境层作为底层基础设施，向信号层和决策层提供只读环境事实：

- **IEnvironmentService**：每周期采样一次环境快照（传播损耗、杂波、干扰存在性）
- **EccmSourceInfo + EccmJammerSourceInfoList**：多源干扰事实（强度、角域、频率重叠、PRF 锁定风险、干扰类型）
- **IFeatureRepository**：目标特征分类数据库，内存级高速查询，防止流水线 I/O 阻塞
- **SceneManager**：统一管理场景实体与干扰源列表

约束：环境层拥有"物理事实"的解释权，不拥有"战术动作"的解释权。

## 7. 跨层数据流

### 7.1 主数据流（每周期）

```text
RadarCycleInput（目标 + 姿态 + 时间步）
  → IRadarContext                                    [API → Core]
  → ISignalPipeline.RunCycle()                       [Core → Signal]
  → DecisionTrackSnapshotList                        [Signal → Core]
  → DataOutputManager.BuildTrackOutputFrame()        [Core 内部]
  → ITacticalDecisionEngine.Evaluate()               [Core → Decision]
  → RadarControlProfile                              [Decision → Core]
  → ISignalPipeline.SetControlProfile()              [Core → Signal，下周期生效]
  → TrackOutputFrame                                 [Core → API → 外部]
```

### 7.2 事件总线（松耦合通知）

- **EventBus**（即时同步）：`Publish()` 立即触发所有订阅者
- **CycleEventBus**（周期延迟）：`Enqueue()` 进入写队列，下周期 `DispatchCurrentCycle()` 消费，防止回调重入

### 7.3 多源干扰事实流

```text
EnvironmentService（干扰事实）
  → EccmSourceInfo                                   [Environment → Core]
  → DecisionInputFrame.eccm_source_info              [Core → Decision]
  → SurvivabilityEvaluator（优先级调整）              [Decision 内部]
  → TacticalProposal[]                               [Decision → Core]
  → RadarControlProfile                              [Core → Signal]
  → ApplyControlProfileToConfig()                    [Signal 内部，映射到探测/关联/跟踪]
```

### 7.4 关联质量补位流

```text
DataAssociationEngine（关联质量）
  → AssociationQualityMetrics                        [Signal → Core]
  → DecisionInputFrame.association_quality_info      [Core → Decision]
  → TacticalCoordinator（补位触发 ECCM）              [Decision 内部]
```

当环境层无干扰信号时，若关联质量同时满足欺骗/转发语义 + 严重度/压力门限，则补位触发 ECCM。
