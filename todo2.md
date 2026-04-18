**AR Config 重构落地记录**

目标：把 AR `config` 从旧的平铺式 public surface 收敛成“聚合壳 + semantic/expert/presets 分层”的结构，并为后续 `beam expert`、`association expert`、`IMM expert` 预留稳定位置。

当前状态：主要重构已完成。本文档不再描述“计划做什么”，而是记录“已经落地了什么、与原规划的偏差是什么、还剩哪些收尾项”。

---

**已完成原则**
1. 根目录只保留聚合壳、builder、统一入口。
2. `semantic`、`expert`、`presets` 已成为显式目录与命名空间。
3. `expert` 已按领域拆分。
4. 旧 `Signal*` 兼容头已删除，不再保留双路径兼容层。
5. `PipelineConfig` 体系已替代旧的 `SignalPipelineConfig` 命名。

---

**当前目录**
```text
include/1q/airborne_radar/config/
|-- README.md
|-- airborne_radar_config.hpp
|-- ConfigModel.h
|-- PipelineConfig.h
|-- RadarSessionConfig.h
|-- RadarRuntimeConfigBuilder.h
|-- RadarSessionConfigBuilder.h
|-- RadarExpertSessionConfigBuilder.h
|-- presets/
|   |-- RadarSessionConfigPresets.h
|   `-- PipelineConfigPresets.h
|-- semantic/
|   |-- AntennaPatternConfig.h
|   |-- BeamControlConfig.h
|   |-- DetectionConfig.h
|   |-- LifecycleConfig.h
|   |-- TrackingConfig.h
|   `-- profiles/
|       |-- DetectionProfiles.h
|       |-- LifecycleProfiles.h
|       `-- TrackingProfiles.h
`-- expert/
    |-- ExpertPipelineConfig.h
    |-- detection/
    |   |-- DetectionConfig.h
    |   |-- TransmitterConfig.h
    |   |-- ReceiverConfig.h
    |   |-- AntennaConfig.h
    |   |-- AntennaPatternConfig.h
    |   |-- DetectionPolicyConfig.h
    |   `-- RcsPhysicsConfig.h
    |-- beam/
    |   |-- BeamControlConfig.h
    |   |-- BeamSchedulerConfig.h
    |   `-- BeamPointingConfig.h
    |-- tracking/
    |   |-- TrackingConfig.h
    |   |-- AssociationConfig.h
    |   `-- KalmanConfig.h
    `-- lifecycle/
        |-- LifecycleConfig.h
        `-- ImmConfig.h
```

---

**命名空间设计**
- 顶层聚合：`airborne_radar::config`
- semantic：`airborne_radar::config::semantic`
- semantic profile：`airborne_radar::config::semantic`
- expert 总壳：`airborne_radar::config::expert`
- expert 子域：
  - `airborne_radar::config::expert::detection`
  - `airborne_radar::config::expert::beam`
  - `airborne_radar::config::expert::tracking`
  - `airborne_radar::config::expert::lifecycle`
- presets：`airborne_radar::config::presets`

说明：
- `semantic/profiles/` 是相对原方案的一个偏差。
- 原因是 profile enum 的复用范围比最初预期更大，单独拆出后，`semantic::*Config` 负责结构，`profiles/*` 负责枚举与语义档位，include 关系更清楚。

---

**当前顶层类型**

`[ConfigModel.h]`
- 定义 `PipelineConfigModel`

`[PipelineConfig.h]`
- 当前聚合壳字段：
  - `model`
  - `semantic::DetectionConfig`
  - `semantic::BeamControlConfig`
  - `semantic::TrackingConfig`
  - `semantic::LifecycleConfig`
  - `expert::ExpertPipelineConfig`

`[RadarSessionConfig.h]`
- 当前 session 聚合壳字段：
  - `pipeline_config_model`
  - `detection`
  - `beam_control`
  - `tracking`
  - `lifecycle`
  - `expert_pipeline_config`
  - `environment_default_config`

`[expert/ExpertPipelineConfig.h]`
- 当前 expert 聚合壳字段：
  - `detection::DetectionConfig`
  - `beam::BeamControlConfig`
  - `tracking::TrackingConfig`
  - `tracking::AssociationConfig`
  - `lifecycle::LifecycleConfig`
  - `lifecycle::ImmConfig`

---

**已完成迁移**

1. semantic 目录化
- 已落地：
  - `semantic/AntennaPatternConfig.h`
  - `semantic/DetectionConfig.h`
  - `semantic/BeamControlConfig.h`
  - `semantic/TrackingConfig.h`
  - `semantic/LifecycleConfig.h`
  - `semantic/profiles/*.h`

2. expert 目录化
- 已落地：
  - `expert/ExpertPipelineConfig.h`
  - `expert/detection/*`
  - `expert/beam/*`
  - `expert/tracking/*`
  - `expert/lifecycle/*`

3. presets 目录化
- 已落地：
  - `presets/PipelineConfigPresets.h`
  - `presets/RadarSessionConfigPresets.h`

4. 聚合壳改造
- 已落地：
  - `PipelineConfig.h`
  - `RadarSessionConfig.h`
  - `airborne_radar_config.hpp`

5. builder 改造
- 已落地：
  - `RadarSessionConfigBuilder.h`
  - `RadarExpertSessionConfigBuilder.h`
  - `RadarRuntimeConfigBuilder.h`

6. 命名收口
- 已完成：
  - `SignalPipelineConfig` -> `PipelineConfig`
  - `SignalPipelineConfigModel` -> `PipelineConfigModel`
  - `ExpertSignalPipelineConfig` -> `ExpertPipelineConfig`
  - `Make*SignalPipelineConfig` -> `Make*PipelineConfig`
  - `WithSignalPipelineConfig(...)` -> `WithPipelineConfig(...)`

7. 兼容层删除
- 已删除旧 public wrapper：
  - `AntennaPatternConfig.h`
  - `SignalDetectionConfig.h`
  - `SignalBeamControlConfig.h`
  - `SignalTrackingConfig.h`
  - `SignalLifecycleConfig.h`
  - `ExpertSignalPipelineConfig.h`
  - `RadarSessionConfigPresets.h`

8. public 契约与 smoke 测试更新
- 已完成：
  - `tests/contract/check_public_api_boundary.cmake`
  - `tests/contract/public_headers_smoke_test.cpp`
  - `tests/contract/ar_public_api_convenience_test.cpp`

9. 文档补齐
- 已新增：
  - `include/1q/airborne_radar/config/README.md`

---

**与原规划的主要偏差**

1. `semantic/profiles/` 被保留
- 原规划里 `semantic` 更粗粒度，没有单独 profile 目录。
- 现在保留 `profiles/`，因为：
  - detection / tracking / lifecycle profile enum 被 builder、preset、resolver、trace 多处直接使用；
  - 单独拆出后职责更清楚；
  - 后续继续加 semantic profile 时更稳。

2. `PipelineConfig` 进一步替代了原计划中的 `SignalPipelineConfig`
- 原规划里这个 rename 是“视情况再做”。
- 现在已直接落地，不再保留旧 public 命名。

3. `ExpertPipelineConfig` 也同步收口
- 原规划里主要强调 semantic/expert 目录化。
- 当前实现顺手把 expert 聚合壳命名也统一到 `PipelineConfig` 体系，避免 `PipelineConfig` / `ExpertSignalPipelineConfig` 混搭。

4. 内部文件命名只做了部分收口
- public config 已全面切到 `PipelineConfig` 体系。
- 但内部 `signal/pipeline` 仍保留部分 `SignalPipeline*` 文件名，例如：
  - `SignalPipelineExecutionConfig.h`
  - `SignalPipelinePresetSemantics.h`
  - `SignalPipelineRuntimeTypes.h`
- 这些目前不构成 public surface 问题，因此暂未继续扩大改动面。

---

**关键文件现状**

public:
- [README.md](/Users/aurora/Code/1q/include/1q/airborne_radar/config/README.md)
- [ConfigModel.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/ConfigModel.h)
- [PipelineConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/PipelineConfig.h)
- [RadarSessionConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfig.h)
- [RadarSessionConfigBuilder.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfigBuilder.h)
- [RadarExpertSessionConfigBuilder.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarExpertSessionConfigBuilder.h)
- [RadarRuntimeConfigBuilder.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h)
- [airborne_radar_config.hpp](/Users/aurora/Code/1q/include/1q/airborne_radar/config/airborne_radar_config.hpp)

semantic:
- [semantic/DetectionConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/DetectionConfig.h)
- [semantic/BeamControlConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/BeamControlConfig.h)
- [semantic/TrackingConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/TrackingConfig.h)
- [semantic/LifecycleConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/LifecycleConfig.h)
- [semantic/profiles/DetectionProfiles.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/semantic/profiles/DetectionProfiles.h)

expert:
- [expert/ExpertPipelineConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/expert/ExpertPipelineConfig.h)
- [expert/tracking/AssociationConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/expert/tracking/AssociationConfig.h)
- [expert/lifecycle/ImmConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/expert/lifecycle/ImmConfig.h)

presets:
- [presets/PipelineConfigPresets.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/presets/PipelineConfigPresets.h)
- [presets/RadarSessionConfigPresets.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/presets/RadarSessionConfigPresets.h)

internal mapping:
- [InternalPipelineConfig.h](/Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/config/InternalPipelineConfig.h)
- [RadarSessionCompositionRoot.cpp](/Users/aurora/Code/1q/src/airborne_radar/session/RadarSessionCompositionRoot.cpp)
- [RadarTraceSession.cpp](/Users/aurora/Code/1q/src/airborne_radar/session/RadarTraceSession.cpp)

---

**验证状态**

已验证通过：

```bash
cmake --build --preset llvm-ninja-debug-local
ctest --preset llvm-ninja-debug-local --output-on-failure -R "RadarSessionConfigBuilderTest|ArRuntimeConfigResolverTest|PublicHeadersSmokeTest|public_api_boundary_guard|PublicApiConvenienceTest"
```

---

**剩余事项**

1. 内部文件名是否继续去掉 `SignalPipeline*`
- 当前 public surface 已经统一。
- 是否继续把内部：
  - `SignalPipelineExecutionConfig.h`
  - `SignalPipelinePresetSemantics.h`
  - `SignalPipelineRuntimeTypes.h`
  做进一步 rename，需要单独判断收益和改动面。

2. `README.md` 是否上挂到仓库主文档
- 当前说明文件只放在 `include/1q/airborne_radar/config/README.md`
- 如果希望外部用户更容易发现，可以在仓库根 README 或 examples 索引中补链接。

3. 是否继续收口 session 层命名
- 目前 `PipelineConfig` 和 `RadarSessionConfig` 边界已经明确。
- 是否还要进一步抽象 session init / runtime patch 文案，不是当前阻塞项。

---

**结论**

原规划的主线已经完成：

- 目录结构已经按 `semantic / expert / presets / aggregate` 分层
- 旧 `Signal*` public surface 已清除
- public 命名已统一到 `PipelineConfig` 体系
- 文档、builder、preset、tests、contract 已同步

当前更适合把后续工作视为“收尾与细化”，而不是继续做大规模结构迁移。

