# AR 模块设计与架构审查（2026-04-26）

> **审查范围**: `src/airborne_radar/`、`include/1q/airborne_radar/`
> **审查方法**: 仅以当前代码为依据，不依赖文档
> **关注点**: 过度设计、边界不清晰、职责混乱

---

## 一、Session ↔ Controller 双重编排

### 1.1 问题：两套平行的周期执行与回滚机制

`RadarSession::StepWithResult` 和 `RadarController::RunOnce` 都各自实现了完整的**校验 → 快照 → 执行 → 回滚/提交**流程，且两者在同一调用链上串行执行。

**Session 层** ([RadarSession.cpp:187-221](file:///Users/aurora/Code/1q/src/airborne_radar/session/RadarSession.cpp#L187-L221)):
```
ValidateInput → CaptureRuntimeState(context/env/controller)
  → CommitPendingRuntimeConfig → BeginCycle → controller.RunOnce()
  → 失败时 RollbackFailedCycle → 成功时 FinalizePendingRuntimeConfig
```

**Controller 层** ([RadarController.cpp:102-171](file:///Users/aurora/Code/1q/src/airborne_radar/runtime/RadarController.cpp#L102-L171)):
```
RadarCycleOutcomeRecorder.CaptureSnapshot → ResetPerCycleFlags
  → ValidateRadarCycleDeltaTime + ValidateRadarSceneTargets
  → FreezeEnvironment → orchestrator.Execute
  → 失败时 RestoreFromFailedCycle → 成功时 CommitSuccessfulCycle
```

两层都做：
- **输入校验**：Session 调 `ValidateRadarCycleInput`，Controller 再调 `ValidateRadarCycleDeltaTime` + `ValidateRadarSceneTargets`
- **状态快照/回滚**：Session 快照 context + env + controller，Controller 内部用 `RadarCycleOutcomeRecorder` 再快照 env + pipeline
- **结果装配**：Session 的 `BuildCycleResult` 从 controller 查询状态再包装，Controller 自己也维护 `runtime_state`

**问题本质**：Session 把 Controller 当黑盒调用，但同时自己也在做 Controller 该做的事。Controller 暴露了太多内部状态查询（`ExecutedLatestCycle`、`ReusedPreviousTrackOutputLatestCycle`、`GetLastSignalCycleAbortReason` 等 7 个 getter），Session 的 `BuildCycleResult` 就是把这些 getter 逐个查询再组装——这不是门面模式，这是**职责泄漏后的手动拼装**。

### 1.2 RadarCycleOutcomeRecorder：引用参数包的伪 RAII

[RadarCycleOutcomeRecorder](file:///Users/aurora/Code/1q/src/airborne_radar/runtime/components/RadarCycleOutcomeRecorder.h#L25-L53) 接收 7 个引用参数（包括 `bool&`、`uint32_t&`），本质上只是把 `RadarController::Impl` 的字段打包传递。它不持有状态、不管理资源，只是把"快照-重置-回滚-提交"四个动作搬到了另一个文件里。

```cpp
RadarCycleOutcomeRecorder(
    environment::IEnvironmentService& environment_service,
    extension::ISignalPipeline& signal_pipeline,
    RuntimeCycleState<...>& runtime_state,
    std::uint32_t& cycle_index,
    bool& last_cycle_executed,
    bool& last_cycle_reused_previous_output,
    SignalCycleAbortReason& last_signal_abort_reason);
```

这些应该是 Controller::Impl 的私有方法，不值得独立成类。

### 1.3 建议

- **消除双重校验**：Session 层仅做"Session-specific"的校验（如 runtime config patch 合法性），domain-level 校验（dt、targets）应仅在 Controller 内完成
- **统一回滚所有权**：要么 Session 管回滚（让 Controller 变纯粹），要么 Controller 自管回滚（Session 只转发），当前两层都管是多余的
- **内联 OutcomeRecorder**：将其逻辑合并回 `RadarController::Impl` 的私有方法

---

## 二、RadarController 作为第二组装根

### 2.1 问题：Controller 在构造函数中硬编码组件创建

[RadarController::Impl 构造函数](file:///Users/aurora/Code/1q/src/airborne_radar/runtime/RadarController.cpp#L57-L71) 直接 `new` 出 `TacticalCoordinator`、`ControlReducer`、`ControlCommandMapper`、`RadarCycleOrchestrator`：

```cpp
Impl(IRadarContext& ctx, ISignalPipeline& sig, IEnvironmentService& env)
    : ...,
      owned_decision_engine(new TacticalCoordinator()),
      control_reducer(new ControlReducer()),
      command_mapper(new ControlCommandMapper(*control_reducer, ctx, ctx)),
      cycle_orchestrator(new RadarCycleOrchestrator(sig, ...)) {}
```

同时 `RadarSessionCompositionRoot::ComposeDefault` 也在创建 Controller 和其依赖。结果是**两个地方都在做组装**：
- `CompositionRoot` 创建 context、pipeline、env、controller
- `Controller::Impl` 创建 decision_engine、reducer、mapper、orchestrator

这违反了"单一组装点"原则。Controller 应该接收所有依赖，不应自己创建。

### 2.2 IRadarContext 多继承导致的接口滥用

[IRadarContext](file:///Users/aurora/Code/1q/include/1q/airborne_radar/extension/IRadarContext.h#L43-L45) 同时继承了三个接口：

```cpp
class IRadarContext : public IRadarContextReader,   // 3 方法：GetSceneTargets/GetPlatformAttitude/GetCycleDeltaTimeSec
                      public IRadarCommandBus,       // 2 方法：SubmitControlCommand/GetSubmittedCommands
                      public IRadarControlProfileStore  // 3 方法：UpdateRadarControlProfile/Has/Get
```

加上自身的 `BeginCycle`、`CaptureRuntimeState`、`RestoreRuntimeState`，共 11 个方法。但实际使用场景中：

- `ControlCommandMapper` 只需要 `IRadarCommandBus` + `IRadarControlProfileStore`（构造时传的就是 `ctx, ctx`）
- `RadarController::RunOnce` 只需要 `IRadarContextReader`（读 targets/attitude/dt）
- Session 需要生命周期管理

三个子接口的拆分是正确的，但通过 `IRadarContext` 多继承又合回去了，导致 Controller 构造 `ControlCommandMapper` 时把整个 context 传了两次（`command_bus` 和 `profile_store` 实际指向同一对象）。

### 2.3 建议

- 将 decision_engine/reducer/mapper/orchestrator 的创建移入 `CompositionRoot`，Controller 通过构造函数接收
- Controller 构造签名改为接收窄接口而非 `IRadarContext&`

---

## 三、Signal Pipeline 内部碎片化

### 3.1 四层命名空间嵌套

Signal pipeline 内部文件的命名空间深度过大：

| 文件 | 命名空间 |
|------|----------|
| [RuntimeAssemblySupport.h](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/RuntimeAssemblySupport.h) | `signal::pipeline::assembly::internal` |
| [SignalComponentFactory.h](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/SignalComponentFactory.h) | `signal::pipeline::assembly::internal` |
| [ScanScheduleResolver.h](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/ScanScheduleResolver.h) | `signal::pipeline::core::internal` |
| [CycleExecutor.h](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/CycleExecutor.h) | `signal::pipeline::internal` |

这些文件全在同一个目录 `signal/pipeline/` 下，但用了 `assembly::internal`、`core::internal`、`internal` 三个不同的子命名空间。目录结构是扁平的，命名空间却是深层嵌套的——两者不一致。

### 3.2 "Phase Output" 结构体只是引用的引用

[CycleExecutor.h](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/CycleExecutor.h#L92-L129) 定义了四个 `*PhaseOutput` 结构体，每个都只持有 `const std::vector<...>&` 引用：

```cpp
struct DetectionPhaseOutput {       // 持有 4 个 const vector& 引用
struct AssociationPhaseOutput {     // 持有 2 个 const vector& 引用
struct MeasurementBuildPhaseOutput {// 持有 2 个 const vector& 引用
struct EnvironmentPhaseOutput {     // 持有 2 个 float 值
```

这些引用全部指向 `CycleExecutionScratch` 的字段。它们存在的目的是在 phase 之间传递数据，但实际上这些数据已经在 scratch 里了。Phase 函数直接从 scratch 读取即可，不需要再包装一层。

### 3.3 CycleExecutionScratch + CycleWorkspace 双重缓冲

[CycleExecutionScratch](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/CycleExecutor.h#L29-L44) 和 [CycleWorkspace](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/CycleContextSupport.h#L28-L42) 描述的是**同一组数据**，前者持有值，后者持有指向前者字段的指针：

```cpp
struct CycleExecutionScratch {
    std::vector<float> signal_term_db;           // 值
    std::vector<float> detection_margin_db;      // 值
    ...
};
struct CycleWorkspace {
    std::vector<float>* signal_term_db{nullptr};           // 指向 scratch 的指针
    std::vector<float>* detection_margin_db{nullptr};      // 指向 scratch 的指针
    ...
};
```

`CycleWorkspace` 本质上就是 `CycleExecutionScratch` 的"指针视图"，没有独立意义。

### 3.4 TrackMeasurementBuildContext / TrackFilterApplyContext 参数爆炸

[TrackMeasurementBuildContext](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/TrackMeasurementProcessing.h#L28-L66) 构造函数有 **11 个参数**，[TrackFilterApplyContext](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/TrackMeasurementProcessing.h#L77-L107) 有 **9 个参数**。这些 context 结构体本意是减少函数参数，但它们的构造本身就需要传入全部参数，只是把参数从函数签名搬到了结构体构造——复杂度没有减少，反而增加了一个间接层。

### 3.5 建议

- 统一命名空间为 `signal::pipeline::internal`，去掉 `assembly` 和 `core` 子空间
- 删除 `*PhaseOutput` wrapper，phase 函数直接操作 scratch
- 合并 `CycleWorkspace` 到 `CycleExecutionScratch`
- 简化 context 结构体，或直接把 scratch 引用传给 phase 函数

---

## 四、Decision 层边界泄漏

### 4.1 DecisionFrameBuilders 在 signal 层构建决策输入

[DecisionFrameBuilders.h](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/DecisionFrameBuilders.h) 位于 `signal/pipeline/` 目录，但它的函数签名：

```cpp
model::EccmSourceInfo BuildEccmSourceInfo(const EnvironmentSnapshot&);
model::AssociationQualityInfo BuildAssociationQualityInfo(const AssociationQualityMetrics&);
model::PerceptionQualityInfo BuildPerceptionQualityInfo(size_t, const AssociationQualityMetrics&);
```

这些函数输出的类型（`EccmSourceInfo`、`AssociationQualityInfo`、`PerceptionQualityInfo`）全部是决策层的 `model::` 类型。Signal pipeline 不应该知道决策层需要什么格式的输入——这是**下游格式需求向上泄漏**。

### 4.2 ITrackLifecycleManager 承担了决策帧构建职责

[ITrackLifecycleManager](file:///Users/aurora/Code/1q/src/airborne_radar/signal/tracking/ITrackLifecycleManager.h#L23-L75) 有一个方法：

```cpp
virtual model::DecisionInputFrame BuildDecisionFrame(
    uint32_t cycle_index, uint64_t batch_id,
    bool environment_jamming_detected) const = 0;
```

生命周期管理器负责构建**决策输入帧**，这是跨层职责。它需要了解 `DecisionInputFrame` 的结构（包含 `EccmSourceInfo`、`AssociationQualityInfo` 等决策概念），这把 tracking 和 decision 耦合了。`BuildDecisionFrame` 应由编排层（orchestrator 或 pipeline）负责，lifecycle manager 只需导出 track snapshots。

### 4.3 TacticalCoordinator 的合成干扰源逻辑过于精巧

[TacticalCoordinator::Evaluate](file:///Users/aurora/Code/1q/src/airborne_radar/decision/TacticalCoordinator.cpp#L265-L384) 中的"关联质量安全网回填"逻辑（L296-L329）在无实际干扰源时，根据关联语义创建合成 `EccmJammerSourceInfo`，手动填入 `jammer_power_db=8.0f`、`jammer_to_signal_db=6.0f` 等硬编码值：

```cpp
synthetic.jammer_power_db = 8.0f;
synthetic.jammer_to_signal_db = 6.0f;
synthetic.frequency_overlap_ratio = association_quality_info.jamming_severity;
```

这实质上是 TacticalCoordinator 在**伪造环境数据**来触发 EccmEvaluator。更干净的做法是让 EccmEvaluator 直接接受"关联压力触发"的概念，而非通过伪造干扰源来绕行。

### 4.4 建议

- 将 `DecisionFrameBuilders` 移入 decision 层或放在 orchestrator 层
- `ITrackLifecycleManager` 移除 `BuildDecisionFrame`，只保留 `BuildTrackStateSnapshots` 和 `BuildAssociationSeeds`
- 将合成干扰源逻辑提取为显式的 `AssociationPressureToEccmAdapter`

---

## 五、Tracking 接口粒度

### 5.1 IKalmanPredictor / IKalmanUpdater 的价值审视

前次审查已指出这两个单方法接口的问题。当前代码中，它们的实际实现有：

| 接口 | 实现 |
|------|------|
| `IKalmanPredictor` | `KalmanPredictor`、`SrifPredictor`、`UdkfPredictor` |
| `IKalmanUpdater` | `KalmanUpdater`、`SrifUpdater`、`UdkfUpdater` |

三种后端（EKF、SRIF、UDKF）通过 `config::engineering::KalmanUpdateBackend` 枚举在工厂中静态选择。运行时不会在周期间切换后端。但这个接口层确实支撑了 IMM 多模型路径——`ImmFilter` 持有多个 `IKalmanPredictor*` 和 `IKalmanUpdater*`。

**结论修正**：对于 IMM 场景，`IKalmanPredictor`/`IKalmanUpdater` 是必要的（不同模型用不同 noise coefficient 的 predictor）。

`ITrackPool` 同样**应当保留**：`SignalComponentFactory::BuildLifecycleAssemblyArtifacts` 存在两条明确的业务路径——`kSingleThreadNoLock` 直接传 `BoostTrackPool*`，`kMultiThreadGlobalLock` 传 `SynchronizedTrackPool*`（用于外部多线程访问场景）。`TrackLifecycleManager` 通过 `ITrackPool&` 接受两者，是真实使用的多态分发，不是过度设计。

### 5.2 TrackFilter 的双重身份

[TrackFilter.h](file:///Users/aurora/Code/1q/src/airborne_radar/signal/tracking/TrackFilter.h) 定义了两套并行的预测/更新抽象：

1. **轻量级 Pipeline 滤波**：`ITrackPredictor` → `IdentityTrackPredictor`，`ITrackUpdater` → `SimpleTrackUpdater`，组合成 `TrackFilter`
2. **Kalman 滤波**：`IKalmanPredictor`/`IKalmanUpdater`，用在 `TrackLifecycleManager` 里

`TrackFilter` 的 `ITrackPredictor` 接口只有一个实现 `IdentityTrackPredictor`（恒等变换），`ITrackUpdater` 只有一个实现 `SimpleTrackUpdater`。这两个接口从未被替换过，`TrackFilter` 直接硬持有具体类型：

```cpp
class TrackFilter final {
  IdentityTrackPredictor predictor_{};  // 具体类型，不是接口指针
  SimpleTrackUpdater updater_{};        // 具体类型，不是接口指针
};
```

接口定义了但未通过多态使用，是**僵尸抽象**。

### 5.3 建议

- 保留 `IKalmanPredictor`/`IKalmanUpdater`（IMM 需要，4 种具体实现多态分发）
- 保留 `ITrackPool`（两种线程安全模式的多态分发，非过度设计）
- ✅ **已完成**：删除 `ITrackPredictor`/`ITrackUpdater`（僵尸抽象）——`TrackFilter` 持有具体类型成员，接口从未用于多态，移除后无运行时影响

---

## 六、Config 映射链与运行期配置

### 6.1 InternalExecutionConfig 的膨胀

[InternalExecutionConfig](file:///Users/aurora/Code/1q/src/airborne_radar/config/InternalExecutionConfig.h#L73-L104) 有 **20+ 个字段**，加上嵌套的 `JammingEffectsConfig`（33 个字段）和 `ControlProfileEffectsConfig`（8 个字段），总共 **60+ 个配置参数**。这个结构体被传递到 pipeline 的几乎每一个内部函数，成为了事实上的"god config"。

问题不在于参数多（仿真系统参数本来就多），而在于它被当作一个整体传递——`DetectionExecution` 只需要探测相关的参数，却收到了包含跟踪、关联、IMM 全部参数的完整 config。

### 6.2 RuntimeConfigState 在两层重复定义

- `config::mapping::RuntimeConfigState`（[RuntimePatchMapper.h:21-27](file:///Users/aurora/Code/1q/src/airborne_radar/config/mapping/RuntimePatchMapper.h#L21-L27)）：Session 层使用
- `SignalPipeline.cpp` 中的匿名 `RuntimeConfigState`（[SignalPipeline.cpp:40-46](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/SignalPipeline.cpp#L40-L46)）：Pipeline 内部使用

两者都叫 `RuntimeConfigState`，都包含 `ExecutionConfig` + `RadarControlProfile`，但分属不同命名空间，没有继承关系。这是语义重复。

### 6.3 Session 配置更新的 dynamic_cast

[RadarSessionCompositionRoot.cpp:39-48](file:///Users/aurora/Code/1q/src/airborne_radar/session/RadarSessionCompositionRoot.cpp#L39-L48) 和 [RadarSession.cpp:95-107](file:///Users/aurora/Code/1q/src/airborne_radar/session/RadarSession.cpp#L95-L107) 都包含同样的 `dynamic_cast<SignalPipeline*>` 分支：

```cpp
SignalPipeline* concrete = dynamic_cast<SignalPipeline*>(&signal_pipeline);
if (concrete != nullptr) {
    concrete->UpdateExecutionConfig(config);  // 内部配置路径
} else {
    signal_pipeline.UpdateConfig(session_config);  // 公开接口路径
}
```

这意味着 `ISignalPipeline::UpdateConfig` 的语义不完整——它只接受 `RadarSessionConfig`，但内部实现实际需要 `InternalExecutionConfig`。Session 层不得不 `dynamic_cast` 来绕过接口。正确做法是让接口直接支持所需的配置粒度，或者将配置转换完全封装在 Pipeline 内部。

### 6.4 建议

- 将 `InternalExecutionConfig` 拆分为 detection/association/tracking/lifecycle 子 config，各 phase 函数只接收自己需要的子集
- 统一 `RuntimeConfigState` 定义
- 移除 `dynamic_cast` 分支：让 `ISignalPipeline::UpdateConfig` 接受 `InternalExecutionConfig`，或在 `SignalPipeline::UpdateConfig(RadarSessionConfig)` 内部完成转换

---

## 七、其他杂项问题

### 7.1 CycleTelemetryLogger 是一个伪类

[CycleTelemetryLogger](file:///Users/aurora/Code/1q/src/airborne_radar/runtime/CycleTelemetryLogger.h#L53-L60) 只有一个 `static` 方法。`CycleTelemetryPayload` 是一个 8 参数的结构体，在调用点 ([RadarController.cpp:157-164](file:///Users/aurora/Code/1q/src/airborne_radar/runtime/RadarController.cpp#L157-L164)) 被就地构造。这应该是一个自由函数。

### 7.2 SynchronizedTrackPool 的定位

`SynchronizedTrackPool` 是 `BoostTrackPool` 的 mutex 包装，通过 `ITrackPool` 接口在 `SignalComponentFactory::BuildLifecycleAssemblyArtifacts` 中按 `TrackPoolThreadSafetyMode` 动态选择：`kSingleThreadNoLock` → 直接传 `BoostTrackPool*`，`kMultiThreadGlobalLock` → 传 `SynchronizedTrackPool*`。两条路径均有使用，不是死代码。

### 7.3 IOverrideControlStrategy 的挂载点不在公共组装路径上

[IOverrideControlStrategy](file:///Users/aurora/Code/1q/include/1q/airborne_radar/extension/IOverrideControlStrategy.h) 是公共头文件中定义的接口，但它只能通过 `TacticalCoordinator` 的构造函数注入。而 `TacticalCoordinator` 的创建发生在 `RadarController::Impl` 内部（见第二节），外部用户通过 `RadarSessionFactory::Create` 无法注入自定义策略。这个扩展点事实上不可达（除非绕过 Session/Controller 层直接构造）。

### 7.4 IFeatureRepository 仍然存在 YAGNI 问题

前次审查已指出 `ConnectDataSource` / `ReloadFromDataSource` 永远返回 false。当前代码未变——这两个方法仍然是占位符。

---

## 总结：问题模式分布

| 模式 | 严重度 | 出现位置 | 影响 |
|------|--------|----------|------|
| **双重编排/回滚** | 🔴 高 | Session ↔ Controller | 理解成本翻倍，bug 容易藏在两层回滚的交互中 |
| **双重组装** | 🔴 高 | CompositionRoot ↔ Controller::Impl | 依赖注入不完整，扩展点不可达 |
| **跨层泄漏** | 🟠 中 | DecisionFrameBuilders、ITrackLifecycleManager::BuildDecisionFrame | 违反分层，修改决策输入格式时牵连 signal 层 |
| **碎片化拆分** | 🟠 中 | PhaseOutput wrappers、CycleWorkspace、Context 结构体 | 增加间接层但不增加抽象能力 |
| **僵尸抽象** | 🟡 低 | ~~ITrackPredictor/ITrackUpdater~~（已清除）、IFeatureRepository 的两个占位方法 | 代码噪音，新人困惑 |
| **命名空间不一致** | 🟡 低 | assembly::internal vs core::internal vs internal | 在同目录下，认知负担 |
| **dynamic_cast 绕行** | 🟠 中 | Session → Pipeline 配置更新 | 接口契约不完整的信号 |

### 核心诊断

AR 模块的**高层分层（session → runtime → signal/decision/environment）是清晰的**。问题集中在两个方面：

1. **Session 和 Controller 的边界模糊**：两者都试图拥有"周期执行"的完整控制权，导致校验、快照、回滚逻辑在两层重复
2. **Signal pipeline 内部过度结构化**：把一个 350 行的 `ExecuteCycle` 函数拆成了 phase output、workspace、context、scratch 四层间接，但每层都只是对同一组数据的不同视图

建议的优先级排序：
1. 🔴 **统一 Session/Controller 的职责边界**（最高优先级，影响面最大）
2. 🔴 **将 Controller 改为纯依赖注入**，移除内部组装
3. 🟠 **简化 signal pipeline 内部间接层**
4. 🟠 **修正跨层泄漏**（DecisionFrameBuilders 归属）
5. 🟡 **清理僵尸抽象和命名空间**
