<!--
  文件说明：说明 1q 作为外部库时的稳定公共头边界、目录约束与安装规则。
-->

# 公共 API 与内部实现边界

## 设计原则

- `include/1q/...` 只承载安装后对外可见的稳定公共契约。
- `src/...` 承载默认实现、内部 SPI、算法细节与白盒测试依赖，不参与安装。
- 所有公共头与内部头都必须遵守 `Namespace-directory mapping`。
- `include/1q` 不允许保留空目录；目录结构必须与实际公共命名空间边界一致。
- 公共头不得暴露 `Boost`、`Eigen` 等重依赖实现细节。

## 安装白名单

以下头文件构成当前安装后的稳定公共 API。

- 根头：
  - `include/1q/api.hpp`
- 通用 DTO / helper：
  - `include/1q/airborne_radar/common/*`
- 默认稳定接入路径：
  - `include/1q/airborne_radar/core/context/RadarCycleInput.h`
  - `include/1q/airborne_radar/core/context/RadarInputValidation.h`
  - `include/1q/airborne_radar/core/output/IRadarOutputReader.h`
  - `include/1q/airborne_radar/core/output/TrackOutputQueries.h`
  - `include/1q/airborne_radar/core/session/RadarSession.h`
  - `include/1q/airborne_radar/core/session/RadarCycleResult.h`
  - `include/1q/airborne_radar/environment/EnvironmentSceneBuilder.h`
  - `include/1q/airborne_radar/environment/EnvironmentTypes.h`
  - `include/1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h`
- 高级稳定扩展路径：
  - `include/1q/airborne_radar/core/context/IRadarContext.h`
  - `include/1q/airborne_radar/core/controller/RadarController.h`
  - `include/1q/airborne_radar/environment/IEnvironmentService.h`
  - `include/1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h`
  - `include/1q/airborne_radar/decision/pipeline/ControlReducerTypes.h`
  - `include/1q/airborne_radar/signal/pipeline/ISignalPipeline.h`
- 轻量支撑类型：
  - `include/1q/airborne_radar/signal/detection/DetectionTypes.h`

其中 `DetectionTypes.h` 仍保留在公共层，是因为 `SignalPipelineTypes.h` 中的探测域配置直接依赖其轻量探测配置结构；它不再代表默认实现扩展点。

## 推荐接入路径

### 默认路径

默认库用户应优先使用 `RadarSession`：

- 入口：`ConfigPresets -> RadarCycleInput -> ValidateRadarCycleInput -> RadarSession::StepWithResult -> TrackOutputQueries`
- 特点：默认装配开箱即用、依赖轻量、屏蔽默认实现件与内部装配细节。

### 高级路径

仅当宿主系统需要自持有调度、上下文或策略实现时，才使用以下稳定抽象接口：

- `IRadarContext`
- `IEnvironmentService`
- `ISignalPipeline`
- `ITacticalDecisionEngine`
- `RadarController`

高级路径只承诺抽象契约稳定，不承诺默认实现类稳定。

## 明确下沉到 `src` 的内部头

以下类型已经从公共边界移除，仅允许库内部和白盒测试使用：

- 默认上下文/环境/信号实现：
  - `src/airborne_radar/core/context/MutableRadarContext.h`
  - `src/airborne_radar/environment/EnvironmentService.h`
  - `src/airborne_radar/signal/pipeline/SignalPipeline.h`
- 内部 SPI：
  - `src/airborne_radar/environment/database/IFeatureRepository.h`
  - `src/airborne_radar/decision/pipeline/TacticalEvaluation.h`
- 生命周期与跟踪内部类型：
  - `src/airborne_radar/signal/tracking/ITrackLifecycleManager.h`
  - `src/airborne_radar/signal/tracking/LifecycleConfig.h`
  - `src/airborne_radar/signal/tracking/GaussianTrackState.h`
  - `src/airborne_radar/signal/tracking/TrackLifecycleTypes.h`
  - `src/airborne_radar/signal/tracking/TrackLifecycleManager.h`
- 其余算法与装配内部件：
  - `src/airborne_radar/core/event/*`
  - `src/airborne_radar/core/output/IDataOutputManager.h`
  - `src/airborne_radar/core/pipeline/IChainProcessor.h`
  - `src/airborne_radar/decision/classifier/*`
  - `src/airborne_radar/decision/eccm/*`
  - `src/airborne_radar/decision/lpi/*`
  - `src/airborne_radar/decision/pipeline/TacticalCoordinator.h`
  - `src/airborne_radar/decision/pipeline/ControlReducer.h`
  - `src/airborne_radar/environment/database/FeatureRepository.h`
  - `src/airborne_radar/environment/scene/*`
  - `src/airborne_radar/environment/simulation/*`
  - `src/airborne_radar/signal/association/*`
  - `src/airborne_radar/signal/detection/BeamControlResolver.h`
  - `src/airborne_radar/signal/detection/BeamwidthResolution.h`
  - `src/airborne_radar/signal/detection/MeasurementErrorModel.h`
  - `src/airborne_radar/signal/detection/RadarEquations.h`
  - `src/airborne_radar/signal/detection/SignalDetector.h`
  - `src/airborne_radar/signal/detection/TargetGeometryResolver.h`
  - `src/airborne_radar/signal/detection/TargetLookResolver.h`
  - `src/airborne_radar/signal/pipeline/SignalComponentFactory.h`
  - `src/airborne_radar/signal/tracking/ITrackPool.h`
  - `src/airborne_radar/signal/tracking/BoostTrackPool.h`
  - `src/airborne_radar/signal/tracking/SynchronizedTrackPool.h`
  - `src/airborne_radar/signal/tracking/KalmanPredictor.h`
  - `src/airborne_radar/signal/tracking/KalmanUpdater.h`
  - `src/airborne_radar/signal/tracking/EkfFilter.h`
  - `src/airborne_radar/signal/tracking/ImmFilter.h`
  - `src/airborne_radar/signal/tracking/TrackFilter.h`
  - `src/airborne_radar/common/TrackTypes.h`

这些文件不属于安装承诺，外部项目不应直接依赖。

## 边界守卫

为防止公共边界回退，当前仓库要求：

- 安装阶段只导出显式白名单中的公共头。
- `tests/check_public_api_boundary.cmake` 必须通过：
  - 公共头集合与白名单一致；
  - `include/1q` 下不存在空目录；
  - 公共头不重新引入 `Boost` / `Eigen`。
- `tests/public_headers_smoke_test.cpp` 必须通过，验证稳定公共头集合可统一包含并完成最小用法编译。
- `tests/install_consumer/` 提供安装后 consumer fixture，用于验证默认路径和高级扩展路径都可通过 `find_package(1q)` 接入。

## 维护规则

- 新增头文件时，先判断它是“稳定公共契约”还是“内部实现”。
- 任何被公共头文件签名直接引用的类型，都必须可在 `include/1q/...` 下找到定义。
- 默认实现类、内部 SPI、生命周期细节、对象池与算法细节不得为了测试便利重新提升为公共头。
- 若某个公共头被移除或新增，必须同步更新：
  - `src/CMakeLists.txt` 安装白名单
  - `tests/check_public_api_boundary.cmake`
  - 本文档
