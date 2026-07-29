# 远程识别需求文档审查报告

Status: draft

**审查日期:** 2026-07-24
**基准代码:** `codex/esr-rf-v2-receiver` 分支（最新 AR 架构）
**审查文档:** `docs/review/remote_recognition_design(1)(1).md`

---

## 审查摘要

文档整体架构设计与当前代码库高度一致，主要数据流插入点正确。以下按章节列出需要更新的具体项，按严重程度分级：

- 🔴 **必须修正** — 与当前代码架构冲突，不改会导致实现错误
- 🟡 **建议对齐** — 与当前命名/结构不一致，不改会导致风格混乱
- 🟢 **补充完善** — 缺少对现有机制的引用，不改会导致设计遗漏

---

## 第2章：现有能力与设计约束

### 🔴 2.1 数据流描述需更新

**原文：**
> 场景目标 -> 探测 -> 关联 -> 航迹 -> 决策输入帧 -> 威胁评估 -> 航迹输出

**问题：** 当前实际数据流比这更复杂，包含 RF 干扰、接收机观测等独立通道。建议更新为与 `design.md` §1.5 一致的描述：

```
场景目标 + RF干扰 → 探测(SINR/Pd/Monte Carlo) → 关联(LAPJV) → 航迹(KF/IMM/生命周期)
  → DecisionInputFrame → 威胁评估/LPI/ECCM → ControlReducer → ArControlProfile
  → 下一周期发射控制
```

### 🟡 2.2 FeatureRepository 引用

**原文：**
> 当前内部 `FeatureRepository` 仅覆盖速度、RCS、干扰三项特征

**确认：** 正确。当前 `FeatureRepository` 位于 `src/airborne_radar/environment/`（内部实现），其 `FeatureVector` 使用 `map<string,float>` 键值对，内建键为 `"speed"`、`"rcs"`、`"jamming"`。默认记录仅包含三类目标（`HIGH_THREAT_FIGHTER`、`LOW_THREAT_TARGET`、`UNKNOWN`），权重为 0.45/0.35/0.20。接口 `IFeatureRepository` 也是内部类型，不在 public API 中。

**建议：** 明确标注 `FeatureRepository` 的文件路径 `src/airborne_radar/environment/IFeatureRepository.h` 和 `FeatureRepository.h`。

### 🟡 2.3 设计约束引用应精确化

**原文：**
> 识别以 `association_key` 关联航迹，不依赖可缺失或重复的 `external_target_id`

**确认：** `association_key` 定义在 `TrackStateSnapshot` 中为 `std::uint64_t`（第31行），是单调递增的内部键。`external_target_id` 同样是 `std::uint64_t`（0 表示未知）。当前 `TrackOutputFrame` 已提供 `BuildTrackMapByAssociationKey()` 查询函数。文档约束正确。

### 🔴 2.4 capture/restore 机制需扩展

**原文：**
> 识别状态与航迹状态均跨周期累积，必须纳入 `ArSession` 的 capture/restore 回滚边界

**问题：** 当前 capture/restore 机制覆盖**四类快照**：`ArContextRuntimeState`、`SignalPipelineRuntimeState`、`EnvironmentServiceRuntimeState`、`ArControllerRuntimeState`。详见 `design.md` §2.2（第393-405行）。识别状态是一个新的跨周期状态域，需要：
1. 决定是新增第五类快照 `RecognitionRuntimeState` 还是将识别状态归入 `ArControllerRuntimeState`
2. 在 `CommitPendingRuntimeConfig` 失败时回滚识别状态
3. 设备关机 (`kSensorPoweredOff`) 时需要在 finalize 前恢复识别快照

**建议：** 推荐将 `RecognitionTrackState` 纳入 `ArControllerRuntimeState`（因识别在 controller 层执行），避免扩大快照矩阵。

### 🟡 2.5 replay 要求需具体化

**原文：**
> 首期数据库采用版本化本地 JSON 文件加载，作为仿真输入的一部分参与 trace/replay

**确认：** 当前 trace/replay 使用 FlatBuffers 编解码（`ArReplayFlatbufferCodec`），trace 通过 `ArTraceSession` 记录，replay 通过 `ReplayArTrace()` 逐周期比较。JSON 数据库路径 `database_path` 需要作为 session config 的一部分进入 replay state（`ArSessionReplayState`）以保证回放一致性。

---

## 第3章：工作模式与运行流程

### 🔴 3.1 ArWorkMode 枚举位置

**原文：**
> 在 `ArWorkMode` 中增加 `kLrr`

**确认：** `ArWorkMode` 定义在 `include/1q/airborne_radar/config/ArOrientationConfig.h` 第65-70行：

```cpp
enum class ONEQ_API ArWorkMode {
  kStby = 0,  // 待机
  kTas = 1,   // 目标捕获扫描
  kTws = 2,   // 边扫边跟踪
  kStt = 3    // 单目标跟踪
};
```

**建议更新：** 
```cpp
enum class ONEQ_API ArWorkMode {
  kStby = 0,
  kTas = 1,
  kTws = 2,
  kStt = 3,
  kLrr = 4   // 远程识别 (long-range recognition)
};
```

注意：该枚举同时被 `ArRuntimeConfigPatch::work_mode` 和 `ArOrientationConfig::work_mode` 引用。`ArRuntimeConfigPatch` 已有 `has_work_mode` 叶子覆盖，添加 `kLrr` 后无需额外 runtime patch 字段即可通过 `work_mode` 切换。

### 🟡 3.2 ArController 不是 public API

**原文：**
> 识别执行点位于 `SignalPipeline` 生成 `DecisionInputFrame` 之后、`ArController` 调用战术决策之前

**问题：** `ArController` 是内部类型（位于 `src/airborne_radar/runtime/`），不在 public API 中。`SignalPipeline` 同样是内部接口 `ISignalPipeline`。

**建议：** 改为描述公开可见的语义边界："识别执行点位于 SignalPipeline 生成 DecisionInputFrame 之后、TacticalCoordinator 调用 ThreatAssessmentEvaluator 之前"，并引用 `design.md` §1.4 时序图中的 `Controller->>Decision: Evaluate(frame, state_store)` 执行点。

### 🟡 3.3 周期数据流中的类型名称

**原文：**
> ArSceneTarget 目标特征真值 -> 探测/跟踪链路 -> 已确认 TrackStateSnapshot -> ...

**确认：** 类型名正确。但需要注意：
- `ArSceneTarget` 在 `include/1q/airborne_radar/session/ArSceneTypes.h`
- `TrackStateSnapshot` 在 `include/1q/airborne_radar/session/TrackStateSnapshot.h`
- `DecisionInputFrame` 在 `include/1q/airborne_radar/session/DecisionInputFrame.h`
- `TrackOutputFrame` 在 `include/1q/airborne_radar/session/ArTrackOutput.h`

### 🟢 3.4 缺少 batch_id 引用

周期级标识不仅有 `cycle_index`（`uint32_t`），还有 `batch_id`（`uint64_t`）。两者共同出现在 `DecisionInputFrame`、`TrackOutputFrame` 和 `ArCycleResult` 中。识别结果应同时携带 `cycle_index` 和 `batch_id` 以便回放溯源。

---

## 第4章：配置与公共输出

### 🔴 4.1 ArRecognitionConfig 放置位置

**原文：**
> 建议新增 `ArRecognitionConfig` 并聚合至 `ArSessionConfig::policy`

**确认：** 正确。当前四域所有权模型（`design.md` §1.1 和 `contract.md` §四域配置所有权）规定：
- `hardware` — 物理能力
- `mission` — 开关机、工作模式、扫描几何
- `policy` — 判决规则、门限、策略参数
- `environment` — 外部环境事实

识别配置（权重、门限、窗口、数据库路径）属于判决规则和策略参数，归入 `policy` 正确。数据库 JSON 路径属于仿真输入，可放在 `ArRecognitionConfig` 中。

**当前 `ArPolicyConfig` 结构** (`include/1q/airborne_radar/config/ArPolicyConfig.h`):
```cpp
struct ArPolicyConfig {
  ArDetectionPolicyConfig detection{};
  BeamControlConfig beam_control{};
  AssociationConfig association{};
  TrackingConfig tracking{};
  LifecycleConfig lifecycle{};
  DecisionControlConfig decision_control{};
};
```

建议在 `ArPolicyConfig` 中新增：
```cpp
ArRecognitionConfig recognition{};  // 远程识别策略配置
```

### 🔴 4.2 TrackStateSnapshot 字段名不匹配

**原文（§4.2表）：**
> 在 `TrackStateSnapshot` 中增加独立 `ArRecognitionResult recognition` 字段

**确认：** 当前 `TrackStateSnapshot` (`TrackStateSnapshot.h`) 的实际字段：
```cpp
std::uint64_t association_key{0};
std::uint64_t external_target_id{0};
std::string target_name{};
TrackStatus status{TrackStatus::kTentative};
float position_x/y/z, velocity_x/y/z, speed;
float acceleration_x/y/z, acceleration;
float rcs{0.0f};                    // 单位：m²（平方米）
std::uint32_t hit_count, miss_count;
std::string target_type{"UNKNOWN"};  // 威胁分类
float target_probability{0.0f};      // [0,1]
```

文档中对 `rcs` 的描述应与实际单位一致：当前存储为 **m²（平方米）**，不是 dBsm。识别结果中的 RCS 特征应保持 dBsm 以避免混淆，但需要在输出时注明单位差异。

### 🟡 4.3 ArRecognitionResult 字段建议

**原文（§4.2表）中的字段：** 结构合理。建议补充以下字段与现有 `TrackStateSnapshot` 对齐：

| 建议新增字段 | 类型 | 说明 |
|---|---|---|
| `source_cycle_index` | `uint32_t` | 产生此结论的 cycle_index |
| `source_batch_id` | `uint64_t` | 产生此结论的 batch_id |

这与 `ArCycleResult` 中 `applied_decision_cycle_index` / `applied_decision_batch_id` 的 provenance 模式一致。

### 🔴 4.4 ArCycleResult 新增识别效能摘要

**原文：**
> `ArCycleResult` 应新增识别效能摘要

**确认：** 当前 `ArCycleResult` (`ArCycleResult.h`) 结构已包含多个聚合摘要字段（`AssociationQualityMetrics`、`SignalCycleAbortReason` 等）。新增识别摘要字段应遵循相同模式：

```cpp
// 建议在 ArCycleResult 中新增
bool has_recognition_summary{false};
ArRecognitionCycleSummary recognition_summary{};
```

### 🟡 4.5 bandwidth_hz 来源

**原文：**
> 有效带宽取自发射机 `bandwidth_hz`

**确认：** `bandwidth_hz` 在 `TransmitterConfig` 中（`ArHardwareConfig.h` 第171行），默认值 4.5 MHz。识别模块应只读取该字段，不修改。

### 🟡 4.6 配置字段表与命名规范

**原文表4.1中：**
- `database_path` → 建议改为 `database_path`（无需变更），与 `ArSessionConfig` 无特定命名后缀要求一致（配置路径不以 `_m` 等物理单位后缀结尾）
- `feature_weights` → 建议明确为 `feature_weights` 的结构体，包含 `rcs_weight`、`motion_weight`、`polarization_weight`、`range_profile_weight`，每个 `float` 范围 [0,1]，和为1

### 🟢 4.7 缺少 RuntimePatch 接入说明

`ArRuntimeConfigPatch` 支持两种粒度的更新：整域覆盖（`has_policy`）和叶子覆盖（`has_work_mode` 等）。识别配置的运行期可调项需明确属于哪一级。建议：
- `enabled`、`acceptance_score`、`minimum_margin` 等可调参数通过 `has_policy` 整域覆盖（与 `ArPolicyConfig` 中的 `ArRecognitionConfig` 一起提交）
- 切换 `kLrr` 模式通过已有的 `has_work_mode` 叶子覆盖

---

## 第5章：特征观测与提取

### 🔴 5.0 ArSceneTarget 需大规模扩展

**原文：**
> 现有 `ArSceneTarget::rcs` 是单一等效 RCS...启用远程识别的场景目标应扩展识别特征描述子

**确认：** 当前 `ArSceneTarget` 结构（`ArSceneTypes.h` 第23-48行）：
```cpp
struct ArSceneTarget {
  std::uint64_t external_target_id{0};
  std::string target_name{};
  float velocity_x/y/z;       // m/s
  float rcs{0.0f};            // m² — 单一标量
  float range_m{0.0f};
  float position_x/y/z;       // m
  int target_swerling_type{0};
};
```

文档提出的三个新字段是净新增：
- `aspect_rcs_samples` — `vector<AspectRcsSample>`
- `polarization_rcs_samples` — `vector<PolarizationRcsSample>`
- `range_rcs_scatterers` — `vector<RangeRcsScatterer>`

**约束：** 这些字段仅在启用 `kLrr` 模式时需要。对于 `kTws`/`kTas`/`kStt` 模式，它们应为空（默认构造），不影响现有探测链路。建议：
1. 将这些字段作为 `ArSceneTarget` 的可选扩展，而非必填
2. 在 `ArInputValidation` 中增加条件校验：`kLrr` 模式下若需要识别但无特征数据则 warn

### 🟡 5.1 RCS 特征中的单位

**原文：**
> RCS 使用 dBsm 存储和比对

**确认：** 当前 `ArSceneTarget::rcs` 和 `TrackStateSnapshot::rcs` 均为 **m²**。文档中新增的 `AspectRcsSample::rcs_dbsm` 和 `PolarizationRcsSample::channel_*_rcs_dbsm` 使用 dBsm — 这是有意的设计选择，应在文档中显式标注"新增字段使用 dBsm，不同于现有 public 字段的 m²"。

### 🟡 5.2 运动特征来源

**原文：**
> 运动特征由滤波航迹多周期估计，不直接采用场景输入速度真值

**确认：** `TrackStateSnapshot` 已包含所需字段：
- `speed` — 速度模长（m/s）
- `acceleration` — 加速度模长（m/s²）
- `velocity_x/y/z` — 速度分量
- `acceleration_x/y/z` — 加速度分量

但**缺少的直接可用字段**：
- `altitude_m` — 目标绝对高度。当前需要从 `position_z` 和平台高度换算。文档应明确换算来源（`ArCycleInput::platform` 中的平台位姿）。

建议在 `TrackStateSnapshot` 中不新增 `altitude_m`（保持快照为雷达局部坐标），而在 `RecognitionObservationBuilder` 中自行换算。

### 🟡 5.3 双通道极化特征 — 需对齐硬件配置

**原文：**
> 通道定义和顺序必须由任务配置固定并写入数据库元数据，例如 `H/V`、`HH/VV`

**确认：** 当前 AR 硬件配置中极化定义在 `ReceiverConfig::polarization`（`RfPolarization` 枚举，默认 `kHorizontal`）和 `ReceiverConfig::scene_polarization`。文档中的双通道极化是一种**新的能力**——需要在硬件层增加第二极化通道配置，或者在识别配置中单独声明极化通道对。

### 🟢 5.4 距离像 — 需引用实际 bandwidth_hz 路径

**原文：**
> 根据有效带宽计算距离单元宽度

**确认：** 当前 `bandwidth_hz` 路径：`ArSessionConfig::hardware::transmitter.bandwidth_hz`。文档应显式标注此引用路径，避免实现时硬编码。

---

## 第6章：多周期融合与识别判定

### 🟡 6.1 RecognitionTrackState 生命周期

**原文：**
> 每个活跃 `association_key` 对应一个 `RecognitionTrackState`

**确认：** `association_key` 由 `DataAssociationEngine` 管理，是单调递增的 `uint64_t`。航迹回收后 key 被释放。识别状态的生命周期应与 `TrackLifecycleManager` 中的航迹生命周期对齐——航迹 `kRecycled` 时应同步清理对应的 `RecognitionTrackState`。

### 🟢 6.2 判定规则阈值

文档中的判定规则（`acceptance_score`、`minimum_margin`、`result_hold_sec`）设计合理，与当前 `LifecycleConfig` 的模式一致（确认/丢失/回收阈值）。建议参照 `LifecycleConfig` 的命名风格，例如 `confirm_hits` → `min_observation_count` 已含义一致。

---

## 第7章：目标特征数据库设计

### 🟡 7.1 文件路径

**原文：**
> 建议存放于 `examples/configs/recognition/`

**确认：** 当前 `examples/configs/` 目录下为：
```
airborne_radar.json  electro_optical.json  electronic_warfare.json  sar.json  README.md
```

无 `recognition/` 子目录。建议创建 `examples/configs/recognition/`，与现有结构一致。

### 🟡 7.2 数据库加载流程与 replay 约束

**原文：**
> 数据库加载应为全量原子替换

**确认：** 此设计需与 `design.md` §2.2 的运行时提交/回滚语义对齐。参考 `EnvironmentScenarioConfig` 的 pending/active 双缓冲模式（场景配置先进入 pending，在 `BeginCycle` 时提交）。数据库替换建议采用类似模式：
1. 新路径通过 runtime patch 提交
2. 在校验周期进行全量校验
3. 校验通过后原子替换
4. 失败时保持旧库

### 🟢 7.3 JSON schema 与 FlatBuffer 回放

JSON 数据库的 `database_id` + `version` 需进入 replay state（`ArSessionReplayState`），确保回放时加载的库版本与录制时一致。`ArReplayCycleRecord` 中应记录每个识别结果的 `database_version`。

---

## 第8章：效能模型

### 🔴 8.1 干扰术语需对齐

**原文：**
> 干扰和 ECCM

**确认：** 当前 AR 公开术语：
- `ArInterferenceObservation` — 接收机 J/N 门控后的干扰观测（`ArInterferenceObservation.h`）
- 最近提交 `94e803e9` 将 **ESR** 模块的 "interference" 重命名为 "rf_emissions"，但 **AR** 模块仍使用 "interference" 命名
- `receiver_saturated` — 前端饱和损伤（`ArReceiverImpairment::kSaturated`）
- ECCM 措施：频率捷变、重频抖动、旁瓣对消、自适应波束、烧穿

文档中"干扰和 ECCM"的引用正确。但应注明：识别质量因子应在 ECCM 激活导致实际发射参数改变后反映在下一周期的特征质量中，不直接读取 ECCM intent。

### 🟡 8.2 效能因素表与现有字段对照

| 文档因素 | 现有相关字段/机制 |
|---|---|
| 斜距和传播损耗 | `ArSceneTarget::range_m`, `EnvironmentService`/`PropagationModel` |
| 驻留时间和脉冲积累 | `ArDetectionPolicyConfig::pulse_count`, 识别专用 `recognition_dwell_sec` |
| 有效带宽 | `TransmitterConfig::bandwidth_hz` |
| 视角覆盖 | 由 `ArOrientationConfig` + 平台姿态 + target look angle 派生 |
| 航迹协方差 | KF/IMM 内部 `KalmanUpdateResult::innovation_covariance` (内部字段) |
| 干扰和 ECCM | `ArInterferenceObservation::jammer_to_noise_db`, `ArControlProfile` ECCM 字段 |
| 目标机动 | `TrackStateSnapshot::acceleration`, IMM 模式概率（内部） |

---

## 第9章：失败、降级与可解释性

### 🟡 9.1 周期 abort 语义需对齐

**原文：**
> 周期 abort 或运行期配置提交失败：恢复识别缓存、最新输出和数据库引用

**确认：** 当前 abort 类型（`SignalCycleAbortReason`）：
- `kNone`, `kLifecycleUnavailable`, `kInvalidEnvironmentCycle`, `kRuntimePreparationFailed`, `kValidationRejected`, `kSensorPoweredOff`

识别降级需与这些 abort reason 对齐。特别是：
- `kRuntimePreparationFailed` — 识别状态必须在回滚中恢复
- `kSensorPoweredOff` — 识别应保持最后结论至 `result_hold_sec`，之后标记 `kStale`
- `kValidationRejected` — 不推进识别积累

### 🟢 9.2 关联键重分配语义

**原文：**
> 重新分配的关联键视为新目标，不继承旧结论

**确认：** 此语义与 `TrackLifecycleManager` 的 recycle 行为一致——回收后 `association_key` 从 `tracks_by_key_` 中移除，新目标获得新 key，旧结论自然失效。但需注意：同一个 `track_id` 可能在不同生命周期获得不同 `association_key`，识别状态应绑定 `association_key` 而非 `track_id`。

---

## 第10章：实施阶段与验收

### 🟡 10.1 需对齐的测试文件

建议新增/扩展以下测试，与现有测试结构对齐：

| 测试域 | 现有参考测试 |
|---|---|
| DTO 与配置 | `tests/unit/airborne_radar/ar_session_config_builder_test.cpp` |
| 运行时 patch | `tests/unit/airborne_radar/ar_runtime_patch_mapper_test.cpp` |
| 特征提取 | 新增 `tests/unit/airborne_radar/ar_recognition_feature_test.cpp` |
| 数据库加载 | 新增 `tests/unit/airborne_radar/ar_recognition_database_test.cpp` |
| 链路集成 | `tests/unit/airborne_radar/ar_core_controller_test.cpp` |
| trace/replay | `tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp` |
| 公共 API | `tests/contract/airborne_radar/ar_public_api_convenience_test.cpp` |
| 跨域命名 | `tests/contract/check_cross_domain_naming.cmake` |

### 🟢 10.2 跨域命名对齐

`ArWorkMode` 命名已对齐 ESR/EOS 的 `EsrWorkMode`/`EosWorkMode`（去掉 `Sub` 后缀，由 `check_cross_domain_naming.cmake` 守护）。新增 `ArRecognitionResult`、`ArRecognitionConfig` 等类型应遵循 `Ar*` 前缀约定，不在 public API 中使用 `Radar*` 前缀。

---

## 附录：关键文件索引（供实施参考）

| 关注点 | 文件路径 |
|---|---|
| **ArWorkMode 枚举** | `include/1q/airborne_radar/config/ArOrientationConfig.h:65-70` |
| **TrackStateSnapshot** | `include/1q/airborne_radar/session/TrackStateSnapshot.h:30-61` |
| **ArSceneTarget** | `include/1q/airborne_radar/session/ArSceneTypes.h:23-48` |
| **ArSessionConfig** | `include/1q/airborne_radar/config/ArSessionConfig.h:23-28` |
| **ArPolicyConfig** | `include/1q/airborne_radar/config/ArPolicyConfig.h:144-151` |
| **ArRuntimeConfigPatch** | `include/1q/airborne_radar/config/ArRuntimeConfigPatch.h:49-76` |
| **DecisionInputFrame** | `include/1q/airborne_radar/session/DecisionInputFrame.h:45-61` |
| **ArCycleResult** | `include/1q/airborne_radar/session/ArCycleResult.h:42-65` |
| **TrackOutputFrame** | `include/1q/airborne_radar/session/ArTrackOutput.h:20-24` |
| **ArInterferenceObservation** | `include/1q/airborne_radar/session/ArInterferenceObservation.h:22-35` |
| **TransmitterConfig** | `include/1q/airborne_radar/config/ArHardwareConfig.h:167-179` |
| **FeatureRepository (内部)** | `src/airborne_radar/environment/IFeatureRepository.h` |
| **ArController (内部)** | `src/airborne_radar/runtime/ArController.h` |
| **ArSession (public)** | `include/1q/airborne_radar/session/ArSession.h` |
| **设计文档** | `docs/airborne_radar/design.md` |
| **跨模块契约** | `docs/common/contract.md` |

---

## 总结

文档的核心设计思路（独立识别链路、四类特征、JSON数据库、动态质量加权）与当前架构兼容。主要修正点集中于：

1. **类型路径和字段名** — 对齐实际头文件位置和字段定义
2. **capture/restore 快照** — 明确识别状态归属（建议归入 `ArControllerRuntimeState`）
3. **RuntimePatch 接入** — 明确识别配置通过 `has_policy` 整域覆盖
4. **命名约定** — 保持 `Ar*` 前缀、物理单位后缀、batch_id provenance
5. **ArSceneTarget 扩展** — 标注新增字段为可选，仅 kLrr 模式需要
6. **Replay 一致性** — database_version 进入 replay state、recognition 结果携带 cycle/batch provenance
