<!--
  文件说明：说明 1q 作为外部库时的公共头文件边界与目录约束。
-->

# 公共 API 与内部实现边界

## 设计原则

- `include/1q/...`：安装后对外可见，供外部项目直接 `#include`。
- `src/...`：库内部实现细节，不参与安装，也不承诺二进制或源码兼容性。
- 目录路径必须与命名空间语义一致，遵守 `Namespace-directory mapping`。

## 应作为外部可视的头文件

以下头文件属于“外部项目可以直接依赖”的 API：

- 核心抽象与数据类型：`include/1q/airborne_radar/common/*`
- 外部输入与上下文默认实现：
  - `include/1q/airborne_radar/core/context/IRadarContext.h`
  - `include/1q/airborne_radar/core/context/RadarCycleInput.h`
  - `include/1q/airborne_radar/core/context/MutableRadarContext.h`
- 核心上下文与事件契约：`include/1q/airborne_radar/core/context/*`
- 核心输出读取与装配契约：`include/1q/airborne_radar/core/output/*`
- 轨迹输出查询辅助：
  - `include/1q/airborne_radar/core/output/TrackOutputQueries.h`
- 核心事件接口与默认实现：
  - `include/1q/airborne_radar/core/event/IEventBus.h`
  - `include/1q/airborne_radar/core/event/EventBus.h`
  - `include/1q/airborne_radar/core/event/CycleEventBus.h`
  - `include/1q/airborne_radar/core/event/RadarEvents.h`
  - `include/1q/airborne_radar/core/event/TrackEvents.h`
- 控制与决策入口：
  - `include/1q/airborne_radar/core/controller/RadarController.h`
  - `include/1q/airborne_radar/core/session/RadarSession.h`
  - `include/1q/airborne_radar/decision/pipeline/ITacticalProcessor.h`
  - `include/1q/airborne_radar/decision/classifier/TargetClassifier.h`
  - `include/1q/airborne_radar/decision/lpi/LpiController.h`
  - `include/1q/airborne_radar/decision/eccm/EccmController.h`
- 环境层接口与默认实现：
  - `include/1q/airborne_radar/environment/IEnvironmentService.h`
  - `include/1q/airborne_radar/environment/EnvironmentService.h`
  - `include/1q/airborne_radar/environment/EnvironmentSceneBuilder.h`
  - `include/1q/airborne_radar/environment/database/IFeatureRepository.h`
  - `include/1q/airborne_radar/environment/database/FeatureRepository.h`
- 信号层公共入口与可选默认实现：
  - `include/1q/airborne_radar/signal/pipeline/ISignalPipeline.h`
  - `include/1q/airborne_radar/signal/pipeline/SignalPipeline.h`
  - `include/1q/airborne_radar/signal/tracking/LifecycleConfig.h`
  - `include/1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h`
  - `include/1q/airborne_radar/signal/tracking/GaussianTrackState.h`
  - `include/1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h`
  - `include/1q/airborne_radar/signal/detection/*.h`
- 目标输入辅助：
  - `include/1q/airborne_radar/common/TargetFeatureUtils.h`

这些头要么定义了外部交互契约，要么提供了外部工程配置和驱动默认仿真链路所需的稳定入口。公共层应暴露“做什么”和“如何配置”，而不是把对象池、滤波器拼装细节一并暴露出去。

## 面向外部库用户的推荐接入路径

- 轻量组件路径：当外部项目已经自持有调度与依赖注入框架时，优先组合 `MutableRadarContext + SignalPipeline + EnvironmentService + RadarController`，并配合 `RadarCycleInput`、`TargetFeatureUtils`、`EnvironmentSceneBuilder`、`TrackOutputQueries` 降低样板代码。
- 高层门面路径：当外部项目只需要“按周期喂输入并拿输出”时，优先使用 `RadarSession`；它托管默认 `MutableRadarContext + SignalPipeline + EnvironmentService + RadarController` 装配，并保留 `TrackOutputFrame`、控制命令、控制真值和关联质量指标的读取能力。

这两条路径都属于稳定公共 API。前者适合已有宿主框架的工程，后者适合快速接入、示例程序和测试夹具。

## 应保留在 src 的头文件

以下头文件属于算法细节、装配细节或白盒测试依赖，不应作为安装接口：

- `src/airborne_radar/signal/association/*`
- `src/airborne_radar/signal/detection/BeamControlResolver.h`
- `src/airborne_radar/signal/detection/MeasurementErrorModel.h`
- `src/airborne_radar/signal/detection/SignalDetector.h`
- `src/airborne_radar/signal/detection/TargetGeometryResolver.h`
- `src/airborne_radar/signal/detection/TargetLookResolver.h`
- `src/airborne_radar/signal/pipeline/SignalComponentFactory.h`
- `src/airborne_radar/signal/tracking/TrackLifecycleManager.h`
- `src/airborne_radar/signal/tracking/ITrackPool.h`
- `src/airborne_radar/signal/tracking/BoostTrackPool.h`
- `src/airborne_radar/signal/tracking/SynchronizedTrackPool.h`
- `src/airborne_radar/signal/tracking/KalmanPredictor.h`
- `src/airborne_radar/signal/tracking/KalmanUpdater.h`
- `src/airborne_radar/common/TrackTypes.h`
- `src/airborne_radar/signal/tracking/EkfFilter.h`
- `src/airborne_radar/signal/tracking/ImmFilter.h`
- `src/airborne_radar/signal/tracking/TrackFilter.h`
- `src/airborne_radar/environment/database` 与 `src/airborne_radar/environment/scene|simulation` 下未进入 `include/` 的实现细节

这些文件可以被库内部源码和白盒测试引用，但外部项目不应把它们当成稳定接口。特别是 `TrackLifecycleManager` 与其依赖的对象池/Kalman 组件，现在仅作为内部默认实现存在；外部只依赖 `ITrackLifecycleManager` 契约和 `LifecycleConfig` 配置。

## 维护规则

- 新增头文件时，先判断它是“外部契约/默认入口”还是“内部实现”。
- 任何被公共头文件签名直接引用的类型，都必须能在 `include/1q/...` 下找到定义或可用声明。
- 不要让测试对 `src/` 的依赖反向决定公共 API；白盒测试允许依赖私有头，但安装接口只能由库设计决定。
