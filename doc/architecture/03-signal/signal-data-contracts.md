# Signal Data Contracts

本文记录 Signal 层当前稳定的数据契约、封装边界与失败策略，避免这些内容继续挤进 `signal-architecture.md`。

## 1. Public vs Internal Boundary

### 对外公共头（仅 3 个）

- `include/1q/airborne_radar/signal/pipeline/ISignalPipeline.h` — 信号处理抽象接口
- `include/1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h` — 全部配置结构 + 关联质量指标 + 周期输出
- `include/1q/airborne_radar/signal/detection/DetectionTypes.h` — 雷达系统配置 + Swerling 模型

### 库内部实现头

- `src/airborne_radar/signal/pipeline/SignalPipeline.h` — 默认流水线实现（PIMPL）
- `src/airborne_radar/signal/pipeline/SignalComponentFactory.h` — 配置映射与组件装配
- `src/airborne_radar/signal/tracking/ITrackLifecycleManager.h` — 生命周期管理抽象
- `src/airborne_radar/signal/tracking/TrackLifecycleManager.h` — 默认生命周期实现
- `src/airborne_radar/signal/tracking/TrackLifecycleTypes.h` — 量测与关联种子内部契约
- `src/airborne_radar/signal/tracking/LifecycleConfig.h` — 生命周期内部配置
- `src/airborne_radar/signal/tracking/GaussianTrackState.h` — 高斯状态类型
- `src/airborne_radar/signal/tracking/TrackFilter.h` — 轨迹滤波抽象层
- `src/airborne_radar/signal/tracking/ITrackPool.h` — 对象池接口
- `src/airborne_radar/signal/tracking/BoostTrackPool.h` — 对象池实现
- `src/airborne_radar/signal/tracking/SynchronizedTrackPool.h` — 线程安全包装
- `src/airborne_radar/signal/tracking/IKalmanPredictor.h` / `IKalmanUpdater.h` — Kalman 接口
- `src/airborne_radar/signal/tracking/KalmanPredictor.h` / `KalmanUpdater.h` — 标准 Kalman 实现
- `src/airborne_radar/signal/tracking/EkfFilter.h` — EKF（含 ITransitionModel / IMeasurementModel 接口）
- `src/airborne_radar/signal/tracking/ImmFilter.h` — IMM 交互多模型
- `src/airborne_radar/signal/association/*` — 关联引擎全部组件
- `src/airborne_radar/signal/detection/*` — 探测链内部组件

边界原则：

- 对外只暴露”如何配置”（`SignalPipelineTypes.h`）和”如何调用”（`ISignalPipeline.h`）。
- 所有原先独立的公共头（`SignalPipeline.h`、`ITrackLifecycleManager.h`、`GaussianTrackState.h`、`TrackLifecycleTypes.h`、`LifecycleConfig.h`）已移入 `src/`，配置参数合并到 `SignalPipelineTypes.h`。
- 对象池、默认滤波器拼装、Lifecycle 内部重对象都不作为安装接口承诺。

## 2. Input Contracts

### `common::TargetFeature`

- `position_x / position_y / position_z` 在 Signal 层内的正式语义是“雷达局部笛卡尔坐标”。
- 当前位置空间关联是唯一正式主路径，因此成功探测目标必须具备可信的笛卡尔位置。
- 若仅有速度、RCS 而没有位置，当前不再静默退回历史特征空间关联路径。

### `environment::IEnvironmentService`

- 每个 `RunCycle()` 周期读取一次环境快照。
- 环境服务只提供只读环境条件，不参与修改输入目标状态。

### `AssociationTrackSeed`

- 当前是位置空间关联的唯一外部先验载荷。
- 若 `has_gaussian_state = true`，则 `gaussian_state` 必须是完整有效状态。
- 若 seeds 被显式注入，即使集合为空，也表示本周期按“无活跃先验”的 stateless 语义执行。

## 3. Internal Exchange Contracts

### `TrackMeasurement`

`TrackMeasurement` 被明确拆成两段：

- `raw_measurement`
  - 位置量测
  - 关联键
  - 关联代价
  - `measurement_covariance`
  - 是否使用 position association / external seeds
- `filtered_feature`
  - 速度
  - 加速度
  - RCS
  - 干扰标记

这个拆分的目的，是避免一个平面结构同时承载“原始量测语义”和“滤波后特征语义”。

### `DecisionTrackSnapshot`

- 这是从 Lifecycle 当前活跃轨迹导出的稳定只读快照。
- 它面向决策层，只保留位置、速度、加速度、RCS、干扰标记以及轨迹标识等必要字段。
- 它不是对象池重对象本身，也不是 `TrackMeasurement` 的完整暴露；内部对象池、状态机计数、滤波运行态仍保持封装。

### `GaussianTrackState`

- 目前仍属于公共类型，因为 `AssociationTrackSeed` 需要携带状态种子。
- 但它只是一种状态表达，不代表 `TrackLifecycleManager`、对象池或 Kalman 组件本身对外可见。

## 4. Failure Strategy

Signal 层当前对关键契约采用 fail-fast：

- 成功探测目标缺位置量测时，中止执行
- external seeds 缺位置或缺高斯状态时，中止执行
- Lifecycle 自动装配配置非法时，中止执行

原因很直接：这些都属于“继续算只会产生错误结果”的输入/配置错误，不适合静默降级。

## 5. Concurrency Semantics

- 当前信号层默认仍按主循环单线程驱动。
- `LifecycleConfig.track_pool_thread_safety_mode` 配置声明对象池包装策略。若选择多线程安全模式，`SignalComponentFactory` 会在装配时引入 `SynchronizedTrackPool`，保证对象池和底层对象创建时的状态安全。
- 后续若做 IMM 多线程优化，只能优先考虑每轨独立的计算阶段并行，容器写入、回收和运行态创建仍应保持串行边界。

## 6. Current Known Constraints

- `AssociationTrackSeed` 仍直接暴露 `GaussianTrackState`（两者均已移入 `src/`，不再是公共类型）
- `kGroundStabilized` 当前仍按对惯性空间稳定近似处理
- 长耗时 `IMM` 批量 Debug 压测已临时从默认测试目标中移除，待多线程优化后恢复
- 探测链路当前仍输出单标量 `angle_error_std_rad`，量测协方差在 LOS 正交平面上采用各向同性近似
