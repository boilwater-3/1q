# 03 — 信号处理层

Signal 层负责机载雷达仿真中的单周期信号处理，输入为目标特征与环境快照，输出为探测、关联、跟踪量测和稳定航迹快照。

## 模块设计概述

### 处理链路

```text
TargetFeatureList + IEnvironmentService
  -> 探测（TargetGeometryResolver → BeamControlResolver → SignalDetector → MeasurementErrorModel）
  -> 关联（DataAssociationEngine：波门 → 假设生成 → LAPJV 最优指派）
  -> 跟踪量测（TrackMeasurement = raw_measurement + filtered_feature）
  -> 生命周期（TrackLifecycleManager：状态机推进 + Kalman/EKF/IMM 滤波）
  -> 输出（DecisionTrackSnapshotList + AssociationQualityMetrics）
```

### 公共边界

当前公共头仅 **3 个**，所有内部实现均在 `src/`：

| 公共头 | 职责 |
|--------|------|
| `ISignalPipeline.h` | 信号处理抽象接口：`RunCycle`、平台姿态更新、控制真值注入 |
| `SignalPipelineTypes.h` | 全部配置结构（探测/波束/关联/跟踪/生命周期）、`AssociationQualityMetrics`、`SignalCycleResult` |
| `DetectionTypes.h` | `RadarSystemConfig`（发射/天线/接收/检测策略）、`SwerlingModel` |

### 关键抽象

- **ISignalPipeline**（公共接口）：单周期 `RunCycle()` → `SignalCycleResult`
- **SignalPipeline**（内部实现，PIMPL）：编排探测 → 关联 → 跟踪全链路
- **ITrackLifecycleManager**（内部接口）：轨迹状态机推进 + 快照导出 + 关联种子导出
- **DataAssociationEngine**（内部）：位置空间唯一主路径关联，支持 external seeds / stateless 两种模式
- **SignalComponentFactory**（内部）：配置映射与组件自动装配

### 滤波器层次

```text
Level 1: KalmanPredictor + KalmanUpdater（标准线性 Kalman，3D 恒速）
Level 2: EkfPredictor + EkfUpdater（扩展 Kalman，ITransitionModel / IMeasurementModel 注入）
Level 3: ImmFilter（Bar-Shalom 四步 IMM，每轨一份运行态）
Level 4: TrackFilter（轨迹级轻量滤波抽象：ITrackPredictor + ITrackUpdater）
```

### 控制真值作用

Signal 层不决定是否启用 ECCM，只执行 `RadarControlProfile` 映射到三个执行面：
1. **探测面**：频率、PRF、主瓣/旁瓣、噪声项
2. **量测面**：有效波束宽度和误差模型
3. **跟踪面**：关联代价、Kalman/IMM 噪声和失配容忍

## 文件说明

| 文件 | 说明 |
|------|------|
| `signal-architecture.md` | **核心文档**：模块定位、目录分层、处理流程、关键机制、边界与测试 |
| `signal-algorithms.md` | 探测、关联、Kalman/EKF/IMM 算法公式与实现细节 |
| `signal-data-contracts.md` | 公共契约、封装边界、失败策略与并发语义 |
| `signal-antenna-pattern.md` | 天线方向图建模语义与工程限制 |
| `signal-processing-flow.puml` | 主处理链路图源文件 |
| `signal-processing-flow.png` | 主处理链路图导出图 |
| `signal-module-layering.puml` | 模块分层图源文件 |
| `signal-module-layering.png` | 模块分层图导出图 |

## 建议阅读顺序

1. `signal-architecture.md` — 架构总览与关键机制
2. `signal-module-layering.puml/png` — 模块分层视图
3. `signal-processing-flow.puml/png` — 主处理链路视图
4. `signal-data-contracts.md` — 数据契约与封装边界
5. `signal-algorithms.md` — 算法公式与实现细节
6. `signal-antenna-pattern.md` — 天线方向图工程近似

落代码时，优先对照以下入口：
- `include/.../signal/pipeline/ISignalPipeline.h` — 公共接口
- `include/.../signal/pipeline/SignalPipelineTypes.h` — 全部公共配置与类型
- `include/.../signal/detection/DetectionTypes.h` — 雷达系统公共配置
- `src/.../signal/pipeline/SignalPipeline.h` — 默认流水线实现
- `src/.../signal/pipeline/SignalComponentFactory.h` — 组件装配工厂
- `src/.../signal/association/DataAssociation.h` — 关联引擎
- `src/.../signal/tracking/TrackLifecycleManager.h` — 生命周期管理

## 流程图预览

![Signal Processing Flow](./signal-processing-flow.png)

## 分层图预览

![Signal Module Layering](./signal-module-layering.png)
