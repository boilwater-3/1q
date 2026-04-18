# Airborne Radar Config

本目录定义机载雷达对外公开的配置 API。

目标很简单：

- `semantic/` 负责表达调用方意图
- `expert/` 负责表达专家级工程参数
- `presets/` 负责提供可直接使用的预设组合
- 根目录只保留聚合壳、builder 和统一入口

## 目录结构

```text
config/
|-- ConfigModel.h
|-- PipelineConfig.h
|-- RadarSessionConfig.h
|-- RadarSessionConfigBuilder.h
|-- RadarExpertSessionConfigBuilder.h
|-- RadarRuntimeConfigBuilder.h
|-- airborne_radar_config.hpp
|-- semantic/
|-- expert/
`-- presets/
```

## 核心类型

### `config::PipelineConfig`

[`PipelineConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/PipelineConfig.h)

信号流水线聚合配置壳。

它同时容纳两套配置输入：

- `semantic` 路径：`detection / beam_control / tracking / lifecycle`
- `expert` 路径：`expert`

通过 `model` 字段决定当前使用哪条路径：

- `PipelineConfigModel::kSemantic`
- `PipelineConfigModel::kExpert`

规则：

- `kSemantic` 时，流水线读取四个 `semantic::*Config`
- `kExpert` 时，流水线读取 `expert::ExpertPipelineConfig`

### `session::RadarSessionConfig`

[`RadarSessionConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfig.h)

会话初始化配置壳。

它比 `PipelineConfig` 多一层会话维度数据：

- `pipeline_config_model`
- semantic pipeline fields
- `expert_pipeline_config`
- `environment_default_config`

可以把它理解成：

- `PipelineConfig` 解决“雷达流水线怎么跑”
- `RadarSessionConfig` 解决“整个 RadarSession 怎么初始化”

## 三层边界

### `semantic/`

语义配置层，给任务层、场景编排层、普通调用方使用。

代表文件：

- [`semantic/DetectionConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/DetectionConfig.h)
- [`semantic/BeamControlConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/BeamControlConfig.h)
- [`semantic/TrackingConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/TrackingConfig.h)
- [`semantic/LifecycleConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/LifecycleConfig.h)
- [`semantic/profiles/DetectionProfiles.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/profiles/DetectionProfiles.h)

特点：

- 面向意图，不直接暴露全部工程细参
- 倾向稳定、易用、组合成本低
- 适合 mission preset、联调、常规仿真

典型字段：

- `RadarHardwareProfile`
- `DetectionIntentProfile`
- `TrackingPolicyProfile`
- `LifecyclePolicyProfile`

### `expert/`

专家配置层，给需要精确建模雷达体制的调用方使用。

代表文件：

- [`expert/ExpertPipelineConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/expert/ExpertPipelineConfig.h)
- [`expert/detection/DetectionConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/expert/detection/DetectionConfig.h)
- [`expert/beam/BeamControlConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/expert/beam/BeamControlConfig.h)
- [`expert/tracking/TrackingConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/expert/tracking/TrackingConfig.h)
- [`expert/tracking/AssociationConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/expert/tracking/AssociationConfig.h)
- [`expert/lifecycle/ImmConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/expert/lifecycle/ImmConfig.h)

特点：

- 面向工程参数
- 允许显式控制 detection / beam / tracking / lifecycle 子域
- 适合专家调参、体制对比、严格建模

说明：

- `expert` 是公开稳定合同
- 内部 `src/.../config/engineering/...` 仍然是实现合同
- 外部不要依赖内部 `engineering` 头

### `presets/`

预设层，给“直接拿来用”的典型组合。

代表文件：

- [`presets/PipelineConfigPresets.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/presets/PipelineConfigPresets.h)
- [`presets/RadarSessionConfigPresets.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/presets/RadarSessionConfigPresets.h)

公开入口：

- `MakeDetectionMissionPipelineConfig()`
- `MakeTrackingMissionPipelineConfig()`
- `MakeHighRobustnessPipelineConfig()`
- `MakeDefaultRadarSessionConfig()`
- `MakeDetectionMissionRadarSessionConfig()`

适用场景：

- 先用预设启动
- 再通过 builder 做少量覆盖

## Builder 选择

### 语义 builder

[`RadarSessionConfigBuilder.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfigBuilder.h)

使用场景：

- 你要的是语义配置
- 你只想覆盖 profile 或少量高频字段

示例：

```cpp
auto config =
    airborne_radar::config::RadarSessionConfigBuilder(
        airborne_radar::config::presets::MakeDetectionMissionRadarSessionConfig())
        .Detection()
        .WithHardwareProfile(
            airborne_radar::config::semantic::RadarHardwareProfile::kLongRangeHighPower)
        .WithDetectionIntentProfile(
            airborne_radar::config::semantic::DetectionIntentProfile::kDetectionPriority)
        .End()
        .Lifecycle()
        .EnableImmFusion(true)
        .End()
        .Build();
```

### 专家 builder

[`RadarExpertSessionConfigBuilder.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarExpertSessionConfigBuilder.h)

使用场景：

- 你要显式指定峰值功率、频率、PRF、跟踪噪声、生命周期阈值等工程参数

示例：

```cpp
auto config =
    airborne_radar::config::RadarExpertSessionConfigBuilder(
        airborne_radar::config::presets::MakeDetectionMissionRadarSessionConfig())
        .Detection()
        .EnablePhysicsDetection(true)
        .WithPeakPowerW(5.0e6f)
        .WithFrequencyHz(9.3e9f)
        .WithPrfHz(220.0f)
        .End()
        .Build();
```

## Runtime Patch

[`RadarRuntimeConfigBuilder.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h)

运行期补丁支持两类更新：

- 整域覆盖：`pipeline_config`、`environment_runtime_config`
- 叶子覆盖：工作子模式、扫描中心、驻留中心、指令态波束宽度

规则：

- 整域和叶子同时存在时，先应用整域，再应用叶子
- 运行期补丁不等价于重建 `RadarSession`

示例：

```cpp
auto patch =
    airborne_radar::config::RadarRuntimeConfigBuilder()
        .WithRadarWorkSubMode(airborne_radar::model::RadarWorkSubMode::kStt)
        .WithScanCenterDeg({10.0f, 2.0f})
        .Build();
```

## 使用建议

### 什么时候选 `semantic`

- 你关注任务行为，不关注全部工程细节
- 你希望配置易读、稳定、默认值明确
- 你正在做场景驱动仿真或上层编排

### 什么时候选 `expert`

- 你需要复现具体雷达体制
- 你需要精确控制 detection / tracking / lifecycle 参数
- 你愿意承担更高的配置复杂度

## 不要混用

建议遵守下面的约束：

- 不要在同一个 `PipelineConfig` 中同时手动维护完整 semantic 字段和 expert 字段，再期待两者共同生效
- 不要依赖 `model` 以外的隐式优先级
- 不要直接包含内部 `src/airborne_radar/config/engineering/...` 头

推荐做法：

- semantic 用户：只维护 semantic 字段，并把 `model` 设为 `kSemantic`
- expert 用户：只维护 expert 字段，并把 `model` 设为 `kExpert`

## 推荐入口

如果你只想使用稳定公开 API，优先从这几个头开始：

- [`airborne_radar_config.hpp`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/airborne_radar_config.hpp)
- [`PipelineConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/PipelineConfig.h)
- [`RadarSessionConfig.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfig.h)
- [`presets/RadarSessionConfigPresets.h`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/presets/RadarSessionConfigPresets.h)

