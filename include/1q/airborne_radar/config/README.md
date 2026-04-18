# Airborne Radar Config

本目录定义机载雷达对外公开的配置 API，以 `hardware / mission / policy / environment` 四域模型为公开稳定主路径。

## 推荐公开主路径

调用方应仅通过以下头文件完成配置：

```text
config/
|-- RadarHardwareConfig.h                 硬件固有能力（探测链路参数）
|-- RadarMissionConfig.h                  任务态与波束运行态
|-- RadarPolicyConfig.h                   调度/关联/跟踪/生命周期策略
|-- RadarEnvironmentConfig.h              环境默认参数
|-- RadarSessionConfig.h                  会话初始化配置壳（四域聚合）
|-- RadarRuntimeConfigPatch.h             运行期可变参数补丁
|-- RadarRuntimeConfigBuilder.h           运行期补丁 Builder
|-- RadarSessionConfigBuilder.h           语义 Builder（Profile 输入）
|-- RadarDetailedSessionConfigBuilder.h   详细 Builder（工程参数输入）
|-- RadarSessionConfigPresets.h           预设工厂
|-- airborne_radar_config.hpp             统一入口头（聚合以上全部）
```

调用方不需要直接 include `PipelineConfig.h`、`config/expert/*` 或 `model/RadarOrientationConfig.h`。

## 语义档位

```text
semantic/
|-- AntennaProfiles.h
|-- DetectionProfiles.h
|-- TrackingProfiles.h
`-- LifecycleProfiles.h
```

语义档位是 Builder 输入材料，服务于 `RadarSessionConfigBuilder` 与预设工厂。不属于独立配置入口。

## 遗留/内部过渡头（不推荐直接使用）

以下头短期保留在 `include` 树中供内部装配路径使用，**不作为对外推荐入口**：

```text
PipelineConfig.h                         内部装配过渡壳
expert/                                   内部专家物理参数
  |-- ExpertPipelineConfig.h
  |-- beam/
  |-- detection/
  |-- lifecycle/
  `-- tracking/
presets/
  |-- PipelineConfigPresets.h             内部装配预设
  `-- RadarSessionConfigPresets.h         历史入口（等同根目录版本）
```

外部调用方应避免直接依赖上述文件。后续里程碑将逐步将其下沉为内部实现头。

## 核心类型

### 四域公开配置

| 域 | 头文件 | 说明 |
| --- | --- | --- |
| `hardware` | `RadarHardwareConfig.h` | 硬件固有能力（探测链路参数） |
| `mission` | `RadarMissionConfig.h` | 任务态与波束运行态（工作子模式、指向与扫描） |
| `policy` | `RadarPolicyConfig.h` | 调度/关联/跟踪/生命周期策略 |
| `environment` | `RadarEnvironmentConfig.h` | 环境默认参数 |

### `session::RadarSessionConfig`

[`RadarSessionConfig.h`](RadarSessionConfig.h)

会话初始化配置壳（四域聚合）：

- `hardware`：硬件固有能力参数
- `mission`：任务态波束与扫描运行态
- `policy`：调度/关联/跟踪/生命周期策略
- `environment`：环境默认参数

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

[`RadarRuntimeConfigPatch.h`](RadarRuntimeConfigPatch.h) / [`RadarRuntimeConfigBuilder.h`](RadarRuntimeConfigBuilder.h)

支持运行期在不重建 `RadarSession` 的前提下热更新参数：

- 整域覆盖：`mission`、`policy`、`environment_runtime_config`
- 叶子覆盖：工作子模式、扫描中心、驻留中心、指令态波束宽度

规则：

- 同时存在整域和叶子时，先整域后叶子，叶子具有最终优先级
- 运行期补丁不等价于重建 `RadarSession`

## 使用建议

- 业务/任务层优先：`semantic/profiles + RadarSessionConfigBuilder`
- 细粒度建模优先：`RadarDetailedSessionConfigBuilder`
- 不要直接使用 `PipelineConfig`，它是内部装配过渡壳
- 不要直接 include `expert/*`，其类型已通过四域头间接导出

## 推荐入口

- [`airborne_radar_config.hpp`](airborne_radar_config.hpp)
- [`RadarSessionConfig.h`](RadarSessionConfig.h)
- [`RadarSessionConfigPresets.h`](RadarSessionConfigPresets.h)
