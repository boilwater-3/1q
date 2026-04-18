# Airborne Radar Config

本目录定义机载雷达对外公开的配置 API。

设计目标：

- `hardware/mission/policy/environment`：公开稳定四域配置语言
- `semantic/`：公开语义档位枚举（Builder 输入语义）
- `presets/`：提供可直接使用的预设组合
- `expert/`：内部专家物理参数定义，由 Builder 内部装配，不作为外部入口
- 根目录仅保留聚合壳、Builder 与统一入口

## 目录结构（公开层）

```text
config/
|-- RadarSessionConfig.h
|-- RadarSessionConfigBuilder.h
|-- RadarDetailedSessionConfigBuilder.h
|-- RadarRuntimeConfigBuilder.h
|-- airborne_radar_config.hpp
|-- PipelineConfig.h                     (内部装配过渡壳)
|-- expert/
|-- semantic/
|   |-- AntennaProfiles.h
|   |-- DetectionProfiles.h
|   |-- TrackingProfiles.h
|   `-- LifecycleProfiles.h
`-- presets/
```

## 核心类型

### `session::RadarSessionConfig`

[`RadarSessionConfig.h`](RadarSessionConfig.h)

会话初始化配置壳（四域）：

- `hardware`：硬件固有能力参数（探测链路固有能力）
- `mission`：任务态与波束运行态（工作子模式、指向与扫描状态）
- `policy`：调度/关联/跟踪/生命周期策略（波束调度、数据关联、跟踪滤波、航迹生命周期、IMM）
- `environment`：环境默认参数

### `config::PipelineConfig`

[`PipelineConfig.h`](PipelineConfig.h)

内部装配过渡壳，当前用于将四域会话配置映射到信号流水线。外部调用方不应直接使用。

## 配置分层

### `semantic/`

语义档位枚举层（Builder 输入）：

- `AntennaProfiles.h`
- `DetectionProfiles.h`
- `TrackingProfiles.h`
- `LifecycleProfiles.h`

适用：任务编排、场景驱动、低心智负担配置。

### `expert/`

内部专家物理参数层：

- `detection/`、`beam/`、`tracking/`、`lifecycle/`
- `ExpertPipelineConfig.h` 聚合各子域

适用：由 Builder 内部装配使用，外部调用方应通过 `semantic` 档位或 `RadarDetailedSessionConfigBuilder` 间接配置。

### `presets/`

典型预设入口：

- `MakeDefaultRadarSessionConfig()`
- `MakeDetectionMissionRadarSessionConfig()`

## Builder 选择

### 语义 Builder

[`RadarSessionConfigBuilder.h`](RadarSessionConfigBuilder.h)

- 输入：`semantic::...Profile` 枚举
- 输出：`RadarSessionConfig`（落到 `hardware/mission/policy/environment`）

适用：业务/任务层快速配置，推荐首选。

### 详细 Builder

[`RadarDetailedSessionConfigBuilder.h`](RadarDetailedSessionConfigBuilder.h)

- 输入：显式细粒度工程参数
- 输出：`RadarSessionConfig`（显式写入 `hardware/mission/policy/environment`）

适用：细粒度建模、工程调参、体制复现。

## Runtime Patch

[`RadarRuntimeConfigBuilder.h`](RadarRuntimeConfigBuilder.h)

支持运行期在不重建 `RadarSession` 的前提下热更新参数：

- 整域覆盖：`mission`、`policy`、`environment_runtime_config`
- 叶子覆盖：工作子模式、扫描中心、驻留中心、指令态波束宽度

规则：

- 同时存在整域和叶子时，先整域后叶子，叶子具有最终优先级
- 运行期补丁不等价于重建 `RadarSession`

## 使用建议

- 业务/任务层优先：`semantic/profiles + RadarSessionConfigBuilder`
- 细粒度建模优先：`RadarDetailedSessionConfigBuilder`
- 不要依赖内部 `src/.../engineering` 头
- 不要直接使用 `PipelineConfig`，它是内部装配过渡壳

## 推荐入口

- [`airborne_radar_config.hpp`](airborne_radar_config.hpp)
- [`RadarSessionConfig.h`](RadarSessionConfig.h)
- [`presets/RadarSessionConfigPresets.h`](presets/RadarSessionConfigPresets.h)
