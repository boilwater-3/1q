# Airborne Radar Config

本目录定义机载雷达对外公开的配置 API。

设计目标：

- `hardware/mission/policy/environment`：公开稳定领域配置语言
- `expert/`：保留底层参数定义，作为内部装配过渡层
- `semantic/`：公开语义档位枚举（Builder 输入语义）
- `presets/`：提供可直接使用的预设组合
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

[`RadarSessionConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfig.h)

会话初始化配置壳（四域）：

- `hardware`：硬件固有能力参数
- `mission`：任务态与波束运行态
- `policy`：调度/关联/跟踪/生命周期策略
- `environment`：环境默认参数

### `config::PipelineConfig`

[`PipelineConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/PipelineConfig.h)

内部装配过渡壳，当前用于将四域会话配置映射到信号流水线：

- `expert`：内部专家参数聚合
- `orientation`：内部波束与扫描运行态

## 配置分层

### `semantic/`

语义档位枚举层（Builder 输入）：

- `AntennaProfiles.h`
- `DetectionProfiles.h`
- `TrackingProfiles.h`
- `LifecycleProfiles.h`

适用：任务编排、场景驱动、低心智负担配置。

### `expert/`

专家物理参数层：

- `detection/`、`beam/`、`tracking/`、`lifecycle/`
- `ExpertPipelineConfig.h` 聚合各子域

适用：工程调参、体制复现、精确建模。

### `presets/`

典型预设入口：

- `MakeDetectionMissionPipelineConfig()`
- `MakeTrackingMissionPipelineConfig()`
- `MakeHighRobustnessPipelineConfig()`
- `MakeDefaultRadarSessionConfig()`
- `MakeDetectionMissionRadarSessionConfig()`

## Builder 选择

### 语义 Builder

[`RadarSessionConfigBuilder.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfigBuilder.h)

- 输入：`semantic::...Profile` 枚举
- 输出：`RadarSessionConfig`（落到 `hardware/mission/policy/environment`）

### 详细 Builder

[`RadarDetailedSessionConfigBuilder.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarDetailedSessionConfigBuilder.h)

- 输入：显式细粒度工程参数
- 输出：`RadarSessionConfig`（显式写入 `hardware/mission/policy/environment`）

## Runtime Patch

[`RadarRuntimeConfigBuilder.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h)

支持：

- 整域覆盖：`pipeline_config`、`environment_runtime_config`
- 叶子覆盖：工作子模式、扫描中心、驻留中心、指令态波束宽度

规则：

- 同时存在整域和叶子时，先整域后叶子
- 运行期补丁不等价于重建 `RadarSession`

## 使用建议

- 业务/任务层优先：`semantic/profiles + RadarSessionConfigBuilder`
- 细粒度建模优先：`RadarDetailedSessionConfigBuilder`
- 不要依赖内部 `src/.../engineering` 头

## 推荐入口

- [`airborne_radar_config.hpp`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/airborne_radar_config.hpp)
- [`PipelineConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/PipelineConfig.h)
- [`RadarSessionConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfig.h)
- [`presets/RadarSessionConfigPresets.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/presets/RadarSessionConfigPresets.h)
