<!--
  文件说明：说明 1q 作为外部库时的公共头文件边界与目录约束。
-->

# 公共 API 与内部实现边界

## 设计原则

- `include/1q/...`：安装后对外可见，供外部项目直接 `#include`。
- `src/...`：库内部实现细节，不参与安装，也不承诺二进制或源码兼容性。
- 目录路径必须与命名空间语义一致，遵守 `Namespace-directory mapping`。

## 应作为外部可视的头文件

以下头文件属于“外部项目可以直接依赖”的稳定公共 API，同时也是安装白名单：

- 根头：
  - `include/1q/api.hpp`
- 公共 DTO / config：
  - `include/1q/airborne_radar/common/*`
- 上下文与高层接入：
  - `include/1q/airborne_radar/core/context/IRadarContext.h`
  - `include/1q/airborne_radar/core/context/RadarCycleInput.h`
  - `include/1q/airborne_radar/core/context/MutableRadarContext.h`
  - `include/1q/airborne_radar/core/context/RadarInputValidation.h`
  - `include/1q/airborne_radar/core/controller/RadarController.h`
  - `include/1q/airborne_radar/core/session/RadarSession.h`
  - `include/1q/airborne_radar/core/session/RadarCycleResult.h`
- 输出读取与查询：
  - `include/1q/airborne_radar/core/output/IRadarOutputReader.h`
  - `include/1q/airborne_radar/core/output/TrackOutputQueries.h`
- 决策层公开契约与控制归并 types：
  - `include/1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h`
  - `include/1q/airborne_radar/decision/pipeline/ControlReducerTypes.h`
- 环境层公开接口：
  - `include/1q/airborne_radar/environment/IEnvironmentService.h`
  - `include/1q/airborne_radar/environment/EnvironmentService.h`
  - `include/1q/airborne_radar/environment/EnvironmentSceneBuilder.h`
  - `include/1q/airborne_radar/environment/database/IFeatureRepository.h`
- 信号层公开入口与公共类型：
  - `include/1q/airborne_radar/signal/detection/DetectionTypes.h`
  - `include/1q/airborne_radar/signal/pipeline/ISignalPipeline.h`
  - `include/1q/airborne_radar/signal/pipeline/SignalPipeline.h`
  - `include/1q/airborne_radar/signal/tracking/LifecycleConfig.h`
  - `include/1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h`
  - `include/1q/airborne_radar/signal/tracking/GaussianTrackState.h`
  - `include/1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h`

这些头要么定义外部交互契约，要么构成推荐接入路径的稳定入口。公共层只暴露“如何接入雷达仿真链路”和“如何配置/读取结果”，不再暴露默认内部实现件。

## 面向外部库用户的推荐接入路径

- 轻量组件路径：当外部项目已经自持有调度与依赖注入框架时，优先组合 `MutableRadarContext + SignalPipeline + EnvironmentService + RadarController`，并配合 `RadarCycleInput`、`TargetFeatureUtils`、`EnvironmentSceneBuilder`、`TrackOutputQueries` 降低样板代码。
- 高层门面路径：当外部项目只需要“按周期喂输入并拿输出”时，优先使用 `RadarSession`；它托管默认 `MutableRadarContext + SignalPipeline + EnvironmentService + RadarController` 装配，并保留 `TrackOutputFrame`、控制命令、控制真值和关联质量指标的读取能力。
- 推荐快速接入顺序：`ConfigPresets -> RadarCycleInput -> ValidateRadarCycleInput -> RadarSession::StepWithResult -> TrackOutputQueries`。这样可以先显式校验输入，再一次性取回当前周期输出、命令、控制真值和关联质量指标。

这两条路径都属于稳定公共 API。前者适合已有宿主框架的工程，后者适合快速接入、示例程序和测试夹具。

## 应保留在 src 的头文件

以下头文件属于算法细节、装配细节或白盒测试依赖，不应作为安装接口：

- 事件体系：`src/airborne_radar/core/event/*`
- 输出装配内部缝：`src/airborne_radar/core/output/IDataOutputManager.h`
- 责任链模板：`src/airborne_radar/core/pipeline/IChainProcessor.h`
- 默认决策实现件：`src/airborne_radar/decision/classifier/*`、`src/airborne_radar/decision/eccm/*`、`src/airborne_radar/decision/lpi/*`、`src/airborne_radar/decision/pipeline/TacticalCoordinator.h`
- reducer 私有实现：`src/airborne_radar/decision/pipeline/ControlReducer.h`
- 默认仓储实现：`src/airborne_radar/environment/database/FeatureRepository.h`
- 探测物理内部件：`src/airborne_radar/signal/detection/RadarEquations.h`、`src/airborne_radar/signal/detection/BeamwidthResolution.h`
- 其余信号/环境内部实现：`src/airborne_radar/signal/association/*`、`src/airborne_radar/signal/detection/BeamControlResolver.h`、`src/airborne_radar/signal/detection/MeasurementErrorModel.h`、`src/airborne_radar/signal/detection/SignalDetector.h`、`src/airborne_radar/signal/detection/TargetGeometryResolver.h`、`src/airborne_radar/signal/detection/TargetLookResolver.h`、`src/airborne_radar/signal/pipeline/SignalComponentFactory.h`、`src/airborne_radar/signal/tracking/TrackLifecycleManager.h`、`src/airborne_radar/signal/tracking/ITrackPool.h`、`src/airborne_radar/signal/tracking/BoostTrackPool.h`、`src/airborne_radar/signal/tracking/SynchronizedTrackPool.h`、`src/airborne_radar/signal/tracking/KalmanPredictor.h`、`src/airborne_radar/signal/tracking/KalmanUpdater.h`、`src/airborne_radar/common/TrackTypes.h`、`src/airborne_radar/signal/tracking/EkfFilter.h`、`src/airborne_radar/signal/tracking/ImmFilter.h`、`src/airborne_radar/signal/tracking/TrackFilter.h`、`src/airborne_radar/environment/scene/*`、`src/airborne_radar/environment/simulation/*`

这些文件可以被库内部源码和白盒测试引用，但外部项目不应把它们当成稳定接口。特别是事件体系、默认 evaluator/coordinator、默认仓储实现和探测物理工具，现在都只服务内部实现与测试，不再属于安装承诺。

## 维护规则

- 新增头文件时，先判断它是“外部契约/默认入口”还是“内部实现”。
- 任何被公共头文件签名直接引用的类型，都必须能在 `include/1q/...` 下找到定义或可用声明。
- 安装阶段只导出显式白名单中的公共头；新增公共头必须手动加入安装列表。
- 不要让测试对 `src/` 的依赖反向决定公共 API；白盒测试允许依赖私有头，但安装接口只能由库设计决定。
