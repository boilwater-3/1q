# Signal Architecture

## 1. Overview

Signal 层是机载雷达仿真中的信号处理核心，负责把输入目标特征与环境快照收敛为单周期探测结果、关联结果、跟踪量测和稳定航迹快照。

- 输入：
  - `common::TargetFeatureList`
  - `environment::IEnvironmentService`
  - 可选的 Lifecycle 关联种子
- 输出：
  - 探测/关联后的 `TrackMeasurement`
  - 关联质量指标
  - 供决策层消费的轨迹特征快照
- 设计原则：
  - 显式步骤编排：`SignalPipeline` 只负责单周期 orchestration
  - 数据契约分层：探测、关联、滤波、生命周期之间通过结构化数据交互
  - 公共接口最小化：对外暴露配置与契约，不暴露对象池与滤波实现拼装细节

## 2. Directory Layout

```text
src/airborne_radar/signal/
├── association/                  # 位置主路径关联、波门、假设生成、最优指派
├── detection/                    # 几何解析、方向图、误差建模、物理探测
├── pipeline/                     # 单周期编排与组件装配
└── tracking/                     # 生命周期、Kalman/EKF/IMM、对象池

include/1q/airborne_radar/signal/
├── detection/                    # 公共探测配置与雷达方程
├── pipeline/                     # ISignalPipeline / SignalPipeline
└── tracking/
    ├── GaussianTrackState.h      # 公开的高斯状态类型
    ├── ITrackLifecycleManager.h  # 生命周期管理抽象
    ├── LifecycleConfig.h         # 生命周期公开配置
    └── TrackLifecycleTypes.h     # 量测与关联种子契约
```

当前信号层的公共边界已经收紧：`TrackLifecycleManager`、`ITrackPool`、`BoostTrackPool`、`KalmanPredictor`、`KalmanUpdater` 等默认实现与对象池细节均留在 `src/`，只供库内部和白盒测试使用。

## 3. Diagram Index

| Diagram ID | Purpose | Source | Export |
|------------|---------|--------|--------|
| D1 | 主处理链路 | `./signal-processing-flow.puml` | `./signal-processing-flow.png` |
| D2 | 模块分层 | `./signal-module-layering.puml` | `./signal-module-layering.png` |

## 4. Main Processing Flow

Signal 层当前的正式主链路如下：

1. `SignalPipeline` 读取输入目标列表并采样环境快照。
2. 若启用物理探测链，则通过 `TargetGeometryResolver -> BeamControlResolver -> SignalDetector -> MeasurementErrorModel` 生成探测与误差结果；否则走经验探测路径。
3. `DataAssociationEngine` 使用位置空间主路径完成关联；若控制器注入 Lifecycle seeds，则按外部先验运行，否则按 stateless 模式运行。
4. `SignalPipeline` 把关联结果转换为 `TrackMeasurement(raw_measurement + filtered_feature)`。
5. 若上层绑定了 `ITrackLifecycleManager`，由 Lifecycle 推进状态机并导出下一周期 seeds 与当前稳定快照。
6. `RadarController` 在下周期开始前把 seeds 回灌到 `SignalPipeline`。

推荐配合图示阅读：

- 主链路图：[signal-processing-flow.puml](/Users/aurora/Code/1q/doc/architecture/signal/signal-processing-flow.puml)
- 主链路导出图：[signal-processing-flow.png](/Users/aurora/Code/1q/doc/architecture/signal/signal-processing-flow.png)

## 5. Key Mechanisms

### 5.1 探测链路前置解析

- `TargetGeometryResolver` 统一 `range / position / look` 真值源。
- `BeamControlResolver` 解析有效波束宽度、波束指向与方向图增益。
- `MeasurementErrorModel` 根据有效波束宽度和 SNR 生成距离/角度误差。
- `SignalDetector` 只保留功率、噪声、SNR、检测概率与判决这条纯探测物理职责。

### 5.2 位置空间关联

- 当前位置空间关联是唯一正式主路径。
- `DataAssociationEngine` 不再保留内部历史先验入口，只消费外部 seeds 或直接走 stateless。
- 动态量测协方差 `R` 由 `SignalPipeline` 前移构造，并被关联与 Lifecycle 更新共享。

### 5.3 生命周期自动装配

- `SignalPipeline` 通过 `SignalComponentFactory` 统一做配置映射与组件装配。
- 当 `SignalLifecycleConfig.enable_auto_lifecycle_manager = true` 时，`SignalPipeline` 可创建默认 Lifecycle 服务。
- 默认 Lifecycle 服务内部仍使用 `TrackLifecycleManager`、对象池、Kalman 或每轨 IMM，但这些实现不再是公共安装接口。

### 5.4 关键路径日志与失败策略

- 关联契约失败、非法 lifecycle 装配配置等场景保持 fail-fast。
- `DataAssociationEngine`、`TrackLifecycleManager`、`RadarController` 已补充关键路径摘要日志，便于定位先验来源、匹配质量与周期推进结果。

### 5.5 控制真值如何作用于 ECCM 探测

信号层并不决定是否启用 ECCM，而是在每周期开始时读取 `RadarControlProfile`，并通过 `ApplyControlProfileToConfig()` 把控制真值映射为运行时配置。

当前已落地的映射关系如下：

| 控制真值 | 主要影响 |
|----------|----------|
| `enable_sidelobe_canceller` | 压低 `jam_noise_w`、`clutter_noise_w`，降低方向图旁瓣电平 |
| `enable_adaptive_beamforming` | 提高主瓣增益、缩窄波束、降低量测噪声 |
| `enable_agility_frequency` | 调整载频，并提高关联/跟踪保守度 |
| `enable_eccm_rejitter` | 调整 PRF，并提高关联/跟踪保守度 |
| `eccm_burnthrough_gain` | 提升有效探测能力、放宽关联保留、降低量测噪声 |

因此，Decision 层输出的是“策略真值”，Signal 层负责把它们落到三个执行面：

1. 探测面：`SignalDetector` 的频率、PRF、主瓣/旁瓣、噪声项。
2. 量测面：`BeamControlResolver` 与 `MeasurementErrorModel` 的有效波束宽度和误差。
3. 跟踪面：关联代价、Kalman/IMM 噪声和失配容忍。

在当前版本里，Signal 层不会把多源干扰事实原样传给 `TrackFilter` 和 `TrackLifecycleManager`，而是先压缩成“主导干扰类型 + 残余干扰强度”两项摘要语义：

- 欺骗式 / 转发式干扰会更明显地抬高关联与量测统计的不确定性。
- `TrackFilter` 在这两类干扰下会减少对失配周期的误惩罚，避免把关联抖动直接解释成目标机动衰减。
- `TrackLifecycleManager` 会基于上一命中周期记录的摘要语义，为欺骗式 / 转发式干扰提供一个小的本地失配容忍 bonus；压制式干扰不享受这项 bonus，因为它更接近真实探测能量下降。
- `AssociationQualityMetrics` 现在同时导出 `dominant_jamming_semantic`、`jamming_severity` 和 `association_stress`，用于区分“当前关联变差”到底更像是普通失配，还是受类型化干扰驱动的关联压力上升。
- `RadarController` 已把这组关联质量摘要上送到 `DecisionInputFrame`；决策层可在环境层事实不足时，把高关联压力视为补充证据触发 ECCM，并进一步只修正频率捷变 / 重频抖动 / 自适应波束形成的优先级，而不是把它整体伪装成完整环境干扰事实。
- `RadarController` 还会基于 `input_targets` 与 `detection_count` 生成 `PerceptionQualityInfo`，让决策层能区分“探测阶段掉量”与“关联阶段抖动”，避免把所有感知退化都解释成同一种 ECCM 问题。

## 6. Contracts and Boundaries

详细字段契约见 [signal-data-contracts.md](/Users/aurora/Code/1q/doc/architecture/signal/signal-data-contracts.md)。这里仅保留架构级边界：

- 对外公共头：
  - `ISignalPipeline` / `SignalPipeline`
  - `ITrackLifecycleManager`
  - `LifecycleConfig`
  - `TrackLifecycleTypes`
  - `GaussianTrackState`
  - 探测域的公共配置与雷达方程
- 内部实现头：
  - `TrackLifecycleManager`
  - `ITrackPool` / `BoostTrackPool`
  - `KalmanPredictor` / `KalmanUpdater`
  - `EkfFilter` / `ImmFilter` / `TrackFilter`
  - `SignalComponentFactory`
  - `association/*`

显式边界的目的不是隐藏算法存在，而是避免外部项目直接依赖默认实现的拼装方式、对象池策略和内部重对象结构。

## 7. Collaboration with Other Modules

- `RadarController`
  - 在每周期开始前把 platform attitude 与 lifecycle seeds 注入 `SignalPipeline`
  - 在每周期结束后消费关联质量与稳定航迹快照
- `environment::IEnvironmentService`
  - 为探测链和误差建模提供环境快照
- 决策层
  - 消费 Lifecycle 输出的稳定轨迹特征，不直接依赖信号层内部重对象
  - 通过 `RadarControlProfile` 间接控制信号层，不直接写探测器参数

## 8. Test Coverage

| Test Suite | Coverage | Key Invariants |
|-----------|----------|----------------|
| `signal_environment_test.cpp` | `SignalPipeline` 公共周期接口、环境交互、seeds 注入 | 平台姿态更新、stateless / external seeds 语义一致 |
| `signal_association_test.cpp` | 位置空间关联与动态协方差 | 缺位置或缺高斯状态时 fail-fast |
| `signal_detection_test.cpp` | 物理探测链与误差链 | SNR/Pd/beamwidth 影响保持一致 |
| `track_lifecycle_test.cpp` | Lifecycle 状态机、每轨 IMM、时间步长策略 | 确认/丢失/回收、IMM 激活策略、对象池包装策略 |
| `kalman_filter_test.cpp` / `advanced_filter_test.cpp` | Kalman、EKF、IMM 数学正确性 | Joseph 形式协方差稳定、模型权重演化合理 |
| `core_controller_test.cpp` | Controller 与 Signal/Lifecycle 集成 | 周期注入、事件发布、自动装配行为 |

补充说明：

- `signal_bulk_data_test.cpp` 目前已从默认 Debug 测试目标中禁用。
- 原因是其中包含长耗时的 IMM Debug 压测，计划待 IMM 多线程优化后再恢复。

## 9. Status and Next Steps

### Done

- `SignalPipeline` 显式步骤编排已落地
- 物理探测链、动态量测协方差、位置主路径关联已贯通
- Lifecycle external seeds -> Association 桥接已稳定
- 生命周期自动装配、每轨 IMM、关键路径日志已接入
- 公共接口与私有实现边界已按封装性重新收口
- 多源干扰事实已继续传播到 TrackFilter / Lifecycle 的汇总级干扰语义

### Next

- 把类型化干扰对关联质量观测的影响继续显式化，而不只停留在运行时参数和本地 miss tolerance
- 为 IMM 多线程优化重新评估 `TrackLifecycleManager` 的阶段划分和可并行区间
- 优化长耗时批量 IMM 测试后，再恢复 `signal_bulk_data_test.cpp`
- 视需要继续收紧 `AssociationTrackSeed` 对高斯状态的公开暴露
