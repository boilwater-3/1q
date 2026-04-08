# AR 模块架构审查报告

审查对象：`airborne_radar` 模块  
审查视角：架构师视角，只读评估  
审查日期：2026-04-08

## 结论

`airborne_radar` 模块整体是一个“分层清楚、公共边界有意识、运行链路可测试”的设计，当前适合作为单进程仿真模型库继续演进。

主要风险不在算法细节，而在编排层逐渐变胖、公共 API 面偏宽、构建系统没有强制模块依赖方向。

综合评价：`7.5/10`。

## 架构概览

当前主链路基本可以概括为：

```text
RadarSession
  -> RadarController
      -> ISignalPipeline / SignalPipeline
      -> IEnvironmentService / EnvironmentService
      -> ITacticalDecisionEngine / TacticalCoordinator
      -> ControlReducer
```

源码分层大体对应：

```text
include/1q/airborne_radar/   公共 API 与扩展接口
src/airborne_radar/common/   共享模型、输出、工具
src/airborne_radar/core/     会话、上下文、控制器编排
src/airborne_radar/signal/   探测、关联、跟踪、流水线运行
src/airborne_radar/decision/ 决策评估、控制归并
src/airborne_radar/environment/ 环境场景与传播模型
```

## 主要优点

- 分层基本成立：`core / signal / decision / environment / common` 拆分清晰，`RadarSession -> RadarController -> SignalPipeline + Environment + Decision` 的主链路容易理解。
- 对外门面明确：`RadarSession` 提供一步一帧和聚合结果接口，适合外部仿真调用。参考 `include/1q/airborne_radar/core/session/RadarSession.h`。
- 有依赖倒置意识：`IRadarContext`、`ISignalPipeline`、`IEnvironmentService`、`ITacticalDecisionEngine` 都能注入，利于替换和测试。参考 `include/1q/airborne_radar/core/controller/RadarController.h`。
- PIMPL 用得比较克制：`RadarSession`、`RadarController` 隐藏实现细节，有利于公共头稳定。参考 `src/airborne_radar/core/session/RadarSession.cpp`。
- `SignalPipeline` 的单周期执行已经被阶段化：环境采样、探测、关联、量测、滤波、输出装配的顺序在 `CycleExecutor` 中非常直观。参考 `src/airborne_radar/signal/pipeline/CycleExecutor.cpp`。
- 公共 API 有契约守护：公共头白名单、禁止公共头暴露 Boost/Eigen、include 风格检查都存在。参考 `tests/contract/check_public_api_boundary.cmake` 和 `tests/contract/check_airborne_include_style.cmake`。

## 主要问题

### 1. `RadarController::RunOnce` 是编排热点

`RadarController::RunOnce` 同时做输入校验、环境冻结、信号流水线调用、决策、控制归并、上下文写回、命令提交和日志输出。短期可维护，长期会成为“中枢大类”。

参考文件：`src/airborne_radar/core/controller/RadarController.cpp`

### 2. `RadarSession` 职责偏多

`RadarSession` 同时承担门面、默认装配、局部依赖注入、运行期配置传播、结果聚合。构造重载较多，引用注入和内部持有混在一个 `Impl` 里，后续扩展会继续增加装配复杂度。

参考文件：

- `include/1q/airborne_radar/core/session/RadarSession.h`
- `src/airborne_radar/core/session/RadarSession.cpp`

### 3. 公共 API 暴露面偏宽

`RadarController`、`IRadarContext`、`ISignalPipeline`、`IEnvironmentService` 都作为公共扩展点暴露。这对测试和高级接入友好，但也会把内部架构锁死成外部兼容性承诺。

参考文件：

- `include/1q/airborne_radar/core/controller/RadarController.h`
- `include/1q/airborne_radar/core/context/IRadarContext.h`
- `include/1q/airborne_radar/signal/pipeline/ISignalPipeline.h`
- `include/1q/airborne_radar/environment/IEnvironmentService.h`

### 4. `SignalPipeline` 子系统规模偏大

`src/airborne_radar/signal` 下文件数量明显高于其他分层。`CycleExecutor` 的阶段化是好事，但 `CycleExecutionContext` 是共享可变工作区，阶段之间靠约定读写字段，缺少更强的阶段契约。

参考文件：

- `src/airborne_radar/signal/pipeline/SignalPipeline.cpp`
- `src/airborne_radar/signal/pipeline/CycleExecutor.h`
- `src/airborne_radar/signal/pipeline/CycleExecutor.cpp`

### 5. 构建系统没有真正强制架构边界

CMake 把 `airborne_common/signal/decision/core` 做成对象库，最终合进一个核心库，并给各对象库相同 include 目录和依赖。这样“看起来分层”，但不能防止错误依赖方向。

参考文件：`src/CMakeLists.txt`

### 6. 配置传播仍偏手工

`RadarSessionConfig` 直接由 signal 的 detection/beam/tracking/lifecycle 配置和 environment 配置组成，`ApplyRuntimeConfig` 手动 patch 深层字段。后续运行期可变项增加时，这部分会继续膨胀。

参考文件：

- `include/1q/airborne_radar/core/session/RadarSession.h`
- `src/airborne_radar/core/session/RadarSession.cpp`
- `include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h`

### 7. 控制闭环时序需要显式文档化

当前控制器在本周期信号处理前注入旧 `control_profile`，决策后再更新 profile 和提交命令，语义上更像“下一周期生效”。如果这是设计意图，应在接口或架构文档里明确；否则容易被调用方误解。

参考文件：`src/airborne_radar/core/controller/RadarController.cpp`

## 建议优先级

1. 先拆 `RadarController::RunOnce`：抽出 `RadarCycleOrchestrator`、`ControlCommandMapper`、`CycleTelemetryLogger`，让 `RadarController` 回到薄编排入口。
2. 建一个明确的装配根：把 `RadarSession` 的默认组件构造和局部注入逻辑迁到 `RadarSessionFactory` 或内部 `RadarSessionCompositionRoot`，`RadarSession` 只保留门面职责。
3. 收窄公共 API 分层：把“一般用户 API”和“高级扩展 SPI”分开，例如 `airborne_radar.hpp` 只暴露 `RadarSession`、配置、输入输出类型；控制器/流水线接口作为 advanced/extension 区域。
4. 给构建系统加架构约束：至少增加 include-direction 检查，例如禁止 `signal` include `core`，禁止 `environment` include `decision/signal`，`core` 只能依赖抽象接口。
5. 明确单周期数据契约：把 `CycleExecutionContext` 拆成阶段输入/输出 DTO，或者为每个 phase 定义最小读写范围，减少共享可变状态带来的隐性耦合。
6. 把运行期配置更新集中化：引入 `RuntimeConfigResolver` 或每模块 `ApplyPatch`，避免 `RadarSession::ApplyRuntimeConfig` 持续知道 signal/environment 的深层结构。
7. 补一份架构文档：重点写清楚周期时序、控制 profile 生效延迟、线程安全模型、公共 API 稳定性级别。

## 风险判断

短期风险：中等。当前结构有足够测试和契约守护，常规功能迭代不至于失控。

中期风险：偏高。如果继续增加探测、跟踪、决策和运行期控制能力，而不拆薄 `RadarController` 与 `RadarSession`，编排逻辑会变成事实上的架构中心，后续改动容易产生连锁影响。

建议先做“边界强化”和“编排拆薄”，再做更大范围的领域模型或算法扩展。

按选项 2：忽略当前 dirty worktree，只按架构目标评价目录结构。

**结论**
需要调整。现在 `ar` 的目录结构有两个问题并存：

- 小目录过多：很多目录只有 1-3 个文件，例如 `common/config`、`common/output`、`core/session/config`、`environment/scene`、`environment/simulation`。
- 大目录没有继续按责任拆清：`signal/pipeline` 和 `signal/tracking` 文件数明显偏多，且 `pipeline` 里混有执行编排、运行期配置、输出装配、干扰影响等不同职责。

这导致当前结构的层级成本大于表达收益。允许破坏性重构的话，我建议做一次“按 API 边界 + 运行责任”重排。

**推荐原则**
- 去掉泛化层：`core`、`common` 这类词容易变成筐，公共 API 里尤其不应该过度使用。
- 不为 1-2 个文件单独建目录：例如 `session/config`、`environment/scene` 这类目录可以先合并。
- 公共 API 和内部实现不要镜像目录：`include/` 应按用户认知组织，`src/` 应按实现责任组织。
- 默认用户 API 和高级扩展 SPI 分开：普通用户不要默认看到 `RadarController`、`IRadarContext`、`ISignalPipeline` 等内部扩展点。
- 大模块保留一级拆分：`signal/detection`、`signal/association`、`signal/tracking` 有足够复杂度，可以保留。

**建议的公共头结构**
把现在的：

```text
include/1q/airborne_radar/
  common/model/
  common/output/
  common/utils/
  config/
  core/context/
  core/controller/
  core/session/
  decision/pipeline/
  environment/
  extension/control/
  signal/config/
  signal/pipeline/
  tools/
```

重构为：

```text
include/1q/airborne_radar/
  airborne_radar.hpp

  session/
    RadarSession.h
    RadarCycleResult.h

  config/
    RadarSessionConfig.h
    RadarSessionConfigBuilder.h
    RadarRuntimeConfigBuilder.h
    RadarSessionConfigPresets.h
    SignalConfig.h
    EnvironmentConfig.h

  model/
    TargetFeature.h
    TargetFeatureBuilder.h
    TargetFeatureUtils.h
    TargetCategory.h
    RadarOrientationConfig.h
    DecisionFrame.h
    DecisionSourceInfo.h
    DecisionTrackSnapshot.h
    JammingSemantics.h

  output/
    TrackOutputFrame.h
    TrackOutputQueries.h

  environment/
    EnvironmentTypes.h
    EnvironmentSceneBuilder.h
    EnvironmentDefaultConfigBuilder.h

  control/
    RadarCommand.h
    RadarControlProfile.h
    ControlDirective.h

  trace/
    RadarTraceSession.h

  extension/
    airborne_radar_extension.hpp
    IRadarContext.h
    RadarController.h
    IRadarOutputReader.h
    ISignalPipeline.h
    SignalPipelineResultTypes.h
    IEnvironmentService.h
    ITacticalDecisionEngine.h
    ControlReducerTypes.h
```

这里的关键变化：

- `core/session` 变成 `session`：用户不需要知道 `core` 这个内部概念。
- `common/model` 变成 `model`：这些是 AR 领域模型，不是跨全项目 common。
- `common/output` 变成 `output`。
- `extension/control` 变成 `control`：控制命令/控制真值是领域概念，不一定只是扩展点。
- `signal/config` 合并进 `config/SignalConfig.h` 或少量配置头：普通用户不应该理解 signal 内部目录树。
- `decision/pipeline`、`signal/pipeline` 的接口移动到 `extension/`：它们是高级替换点，不是默认用户 API。

默认入口 `airborne_radar.hpp` 建议只 include：

```text
session/*
config/*
model/*
output/*
environment/*
control/*
trace/*
```

不要 include：

```text
extension/IRadarContext.h
extension/RadarController.h
extension/ISignalPipeline.h
extension/IEnvironmentService.h
extension/ITacticalDecisionEngine.h
```

高级用户显式 include：

```cpp
#include "1q/airborne_radar/extension/airborne_radar_extension.hpp"
```

**建议的内部源码结构**
把现在的：

```text
src/airborne_radar/
  common/
  core/context/
  core/controller/
  core/session/
  core/session/config/
  decision/evaluators/
  decision/pipeline/
  environment/database/
  environment/scene/
  environment/simulation/
  signal/assembly/
  signal/association/
  signal/detection/
  signal/pipeline/
  signal/runtime/
  signal/tracking/
```

重构为：

```text
src/airborne_radar/
  session/
    RadarSession.cpp
    RadarTraceSession.cpp
    RadarSessionConfigBuilder.cpp
    ConfigPresets.cpp

  runtime/
    RadarController.cpp
    MutableRadarContext.cpp
    RadarInputValidation.cpp
    RadarCycleOrchestrator.cpp
    ControlCommandMapper.cpp
    CycleTelemetryLogger.cpp

  model/
    TargetFeature.cpp
    TargetFeatureUtils.cpp
    DecisionTrackSnapshot.cpp
    TrackOutputQueries.cpp

  environment/
    EnvironmentService.cpp
    EnvironmentSceneBuilder.cpp
    EnvironmentSceneBuilder.h
    EnvironmentService.h
    SceneManager.cpp
    SceneManager.h
    PropagationModel.cpp
    PropagationModel.h
    FeatureRepository.cpp
    FeatureRepository.h
    IFeatureRepository.h

  decision/
    TacticalCoordinator.cpp
    TacticalCoordinator.h
    TacticalEvaluation.h
    ControlReducer.cpp
    ControlReducer.h
    ThreatAssessmentEvaluator.cpp
    ThreatAssessmentEvaluator.h
    EmissionControlEvaluator.cpp
    EmissionControlEvaluator.h
    SurvivabilityEvaluator.cpp
    SurvivabilityEvaluator.h
    SurvivabilityEvaluatorHelpers.cpp
    SurvivabilityEvaluatorHelpers.h

  signal/
    detection/
    association/
    tracking/
    pipeline/
```

`signal/` 里我建议先不要强行扁平化，因为它确实是大头：

```text
src/airborne_radar/signal/
  detection/      探测与雷达方程
  association/    数据关联、门控、分配
  tracking/       Kalman/IMM/生命周期/track pool
  pipeline/       单周期流水线编排、运行期配置解析、输出收集
```

但是可以把 `signal/runtime` 合并进 `signal/pipeline`，因为它服务的是 pipeline 运行期装配，不是独立架构层。

**重点取舍**
我不建议把所有东西都压平成：

```text
src/airborne_radar/*.cpp
```

那会让信号处理算法区失去边界。更好的方向是：

- 公共 API 扁平一些，按用户概念组织。
- 内部实现保留领域模块，但删除 1-2 个文件的小层级。
- 大模块 `signal` 保留子目录，但减少 `pipeline/runtime/assembly` 这种职责交叉的目录。

**最值得先改的目录**
优先级如下：

1. `include/1q/airborne_radar/core/*`：公共 API 中的 `core` 应该消失或被降级到 `extension`。
2. `include/1q/airborne_radar/common/*`：AR 自己的领域类型不应叫 `common`，改成 `model`、`output`。
3. `src/airborne_radar/core/*`：改成 `session/` + `runtime/`，让“会话门面”和“周期运行编排”分开。
4. `src/airborne_radar/environment/database|scene|simulation`：文件太少，先合并到 `environment/`，用类名表达职责。
5. `src/airborne_radar/signal/runtime`：合并到 `signal/pipeline` 或改名为 `signal/assembly`，不要和顶层运行时概念冲突。

**迁移策略**
允许破坏性重构的话，我建议一次完成，不保留旧路径适配头：

1. 先改公共头路径和 umbrella header。
2. 更新所有 include。
3. 调整 `src/CMakeLists.txt` 源文件列表和安装头列表。
4. 更新 contract tests 的 public header 白名单。
5. 更新 consumer tests，区分 `ar_session_consumer` 和 `ar_extension_consumer`。
6. 最后加 include-direction 检查，防止重构后目录再次漂移。

**一句话建议**
把公共 API 从“内部结构镜像”改成“用户概念分组”，把内部实现从“过细目录层级”改成“少数稳定责任区”。这会破坏路径兼容，但对后续架构演进是值得的。
