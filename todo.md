结论：`config` 该从“按历史功能堆头文件”改成“按配置层级 + 领域”组织。  
如果后面要公开 `beam expert`、`association expert`、`IMM expert`，继续在根目录加文件会很快失控。

我建议把 AR config 重构成 4 层：

- `config/`：只放顶层聚合入口和总装配壳
- `config/semantic/`：语义配置
- `config/expert/`：专家配置
- `config/presets/`：预设与 profile 映射入口

**目标**
1. 让“semantic”和“expert”成为一级概念
2. 让 beam/detection/tracking/lifecycle/association 成为稳定领域
3. 让 public API 目录结构和命名空间一致
4. 给后续 expert 扩展留足空间，不再出现“大而全头文件”

---

**建议目录**
```text
include/1q/airborne_radar/config/
|-- airborne_radar_config.hpp                 public convenience umbrella
|-- PipelineConfig.h                    top-level pipeline aggregate
|-- RadarSessionConfig.h                      top-level session aggregate
|-- RadarRuntimeConfigBuilder.h               runtime patch entry
|-- RadarSessionConfigBuilder.h               semantic builder
|-- RadarExpertSessionConfigBuilder.h         expert builder
|-- ConfigModel.h                             PipelineConfigModel
|-- semantic/
|   |-- DetectionConfig.h
|   |-- BeamControlConfig.h
|   |-- TrackingConfig.h
|   |-- LifecycleConfig.h
|   |-- AntennaPatternConfig.h
|   `-- profiles/
|       |-- DetectionProfiles.h
|       |-- TrackingProfiles.h
|       `-- LifecycleProfiles.h
|-- expert/
|   |-- ExpertPipelineConfig.h
|   |-- detection/
|   |   |-- DetectionConfig.h
|   |   |-- TransmitterConfig.h
|   |   |-- ReceiverConfig.h
|   |   |-- AntennaConfig.h
|   |   |-- AntennaPatternConfig.h
|   |   |-- DetectionPolicyConfig.h
|   |   `-- RcsPhysicsConfig.h
|   |-- beam/
|   |   |-- BeamControlConfig.h
|   |   |-- BeamSchedulerConfig.h
|   |   `-- BeamPointingConfig.h
|   |-- tracking/
|   |   |-- TrackingConfig.h
|   |   |-- AssociationConfig.h
|   |   `-- KalmanConfig.h
|   `-- lifecycle/
|       |-- LifecycleConfig.h
|       `-- ImmConfig.h
`-- presets/
    |-- RadarSessionConfigPresets.h
    `-- PipelineConfigPresets.h
```

---

**建议命名空间**
- 顶层聚合：`airborne_radar::config`
- 语义层：`airborne_radar::config::semantic`
- 专家层：`airborne_radar::config::expert`
- 预设层：`airborne_radar::config::presets`

不要把 semantic 类型继续直接平铺在 `config` 根命名空间里。  
现在 `SignalDetectionConfig`、`SignalTrackingConfig` 这种名字太泛，后面 expert 同名概念会越来越拧巴。

更建议变成：

- `semantic::DetectionConfig`
- `semantic::TrackingConfig`
- `semantic::LifecycleConfig`
- `semantic::BeamControlConfig`

和

- `expert::detection::DetectionConfig`
- `expert::tracking::TrackingConfig`
- `expert::tracking::AssociationConfig`
- `expert::lifecycle::ImmConfig`

---

**顶层聚合该怎么长**
`ConfigModel.h`
```cpp
namespace airborne_radar::config {
enum class PipelineConfigModel {
  kSemantic = 0,
  kExpert = 1,
};
}
```

`PipelineConfig.h`
```cpp
namespace airborne_radar::config {

struct PipelineConfig {
  PipelineConfigModel model{PipelineConfigModel::kSemantic};

  semantic::DetectionConfig detection{};
  semantic::BeamControlConfig beam_control{};
  semantic::TrackingConfig tracking{};
  semantic::LifecycleConfig lifecycle{};

  expert::ExpertPipelineConfig expert{};
};

}
```

这个聚合壳保留是对的，因为：
- runtime patch 已经围绕它建模了
- session/composition root 已经围绕它传递了

但它应该只做“总装配壳”，不要再承载一堆子类型定义。

---

**expert 层怎么拆**
你后面提到的扩展点，天然就对应这些目录：

1. `beam expert`
- `expert/beam/BeamControlConfig.h`
- `expert/beam/BeamSchedulerConfig.h`
- `expert/beam/BeamPointingConfig.h`

2. `association expert`
- `expert/tracking/AssociationConfig.h`
- 包含 gating、cost、unassigned cost、seed policy 之类
- 因为 association 现在在实现上隶属 tracking/pipeline，放 `tracking/` 比单开根目录更稳

3. `IMM expert`
- `expert/lifecycle/ImmConfig.h`
- 因为 IMM 在你的实现里更偏 lifecycle activation 和 lifecycle runtime assembly，而不是独立域

4. detection 继续细分
- `TransmitterConfig`
- `ReceiverConfig`
- `AntennaConfig`
- `DetectionPolicyConfig`
- `RcsPhysicsConfig`

这比把它们都塞在 [ExpertPipelineConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/ExpertPipelineConfig.h) 里强得多。

---

**semantic 层怎么拆**
semantic 不需要拆得像 expert 那么细。  
semantic 的本质是“调用方意图输入”，不是工程参数库。

所以 semantic 层控制在 4 到 6 个文件最合理：

- `semantic/DetectionConfig.h`
- `semantic/BeamControlConfig.h`
- `semantic/TrackingConfig.h`
- `semantic/LifecycleConfig.h`
- `semantic/AntennaPatternConfig.h`

不要把 semantic 再拆成 transmitter/receiver 这种工程子结构，那会回到专家配置思路。

---

**当前文件的去留建议**
保留：
- [PipelineConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/PipelineConfig.h)
- [RadarSessionConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfig.h)
- [RadarRuntimeConfigBuilder.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h)
- [RadarSessionConfigBuilder.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarSessionConfigBuilder.h)
- [RadarExpertSessionConfigBuilder.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarExpertSessionConfigBuilder.h)
- [airborne_radar_config.hpp](/Users/aurora/Code/1q/include/1q/airborne_radar/config/airborne_radar_config.hpp)

迁移/拆分：
- [ExpertPipelineConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/ExpertPipelineConfig.h)
- [SignalDetectionConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/SignalDetectionConfig.h)
- [SignalTrackingConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/SignalTrackingConfig.h)
- [SignalLifecycleConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/SignalLifecycleConfig.h)
- [SignalBeamControlConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/SignalBeamControlConfig.h)
- [AntennaPatternConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/AntennaPatternConfig.h)

其中 `Signal*` 这个前缀我建议也一起处理，因为它已经不够清楚了。

---

**命名建议**
当前：
- `SignalDetectionConfig`
- `SignalTrackingConfig`
- `SignalLifecycleConfig`
- `SignalBeamControlConfig`

建议改成：
- `semantic::DetectionConfig`
- `semantic::TrackingConfig`
- `semantic::LifecycleConfig`
- `semantic::BeamControlConfig`

原因：
- `Signal*` 没有提供有效区分
- 后面 expert 也会有 detection/tracking/lifecycle/beam
- 靠 `semantic::` / `expert::` 区分更稳，更清楚

expert 侧建议：
- `expert::detection::DetectionConfig`
- `expert::tracking::TrackingConfig`
- `expert::tracking::AssociationConfig`
- `expert::lifecycle::LifecycleConfig`
- `expert::lifecycle::ImmConfig`
- `expert::beam::BeamControlConfig`

---

**和内部 engineering 的关系**
不要把 `src/airborne_radar/config/engineering/SignalEngineeringConfig.h` 直接 public 化。  
应该继续保持：

- public expert 是“稳定外部合同”
- internal engineering 是“内部执行合同”
- 中间用 resolver 转换

这样以后你内部想把一个 `TrackingConfig` 拆成更多 runtime 子项，不会把 public API 一起拖着改。

---

**我建议的重构原则**
1. public config 不直接镜像 internal engineering
2. semantic/expert 是一级维度，不是注释说明
3. 领域类型放领域目录，聚合类型放根目录
4. root header 只做装配，不定义大量细节结构
5. runtime/session builder 继续围绕聚合壳工作

---

**最值得避免的设计**
1. 所有 expert 类型继续堆一个文件
- 后面一定失控

2. semantic 和 expert 都平铺在 `config/`
- 名字冲突和认知冲突会越来越重

3. 直接暴露 `engineering::...`
- 会把内部实现锁死

---

**落地顺序**
1. 先引入 `config/ConfigModel.h`
2. 拆 `semantic/`
3. 拆 `expert/`
4. 把根目录聚合头改成 include 聚合，不再定义子类型
5. 最后统一 builder include 和命名空间

如果你要，我下一步可以直接给你一份“可执行重构方案”，精确到：
- 哪些文件新建
- 哪些旧文件 move/rename
- 哪些命名空间和 include 要怎么改
- 哪一批可以一次提交完成，哪一批要分两步迁移