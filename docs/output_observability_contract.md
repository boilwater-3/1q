# EOS/ESR/SAR 三层输出与可观测性契约

Date: 2026-06-25
Status: 阶段 9 (Batch G-J) 落地。EOS raw output 已去真值化(detection_id + 独立 attribution)；
        EOS/ESR/SAR 调试/生命周期实现已从 header-only 收敛到 .cpp；
        ESR/SAR 输出边界合同测试已补齐。本文已同步实际落地状态。

## 目的

本文冻结 EOS、ESR、SAR 三个模块的三层输出模型、实体名称、生命周期事件、未发现原因诊断、Trace/Replay 和日志边界。后续实现必须以本文为公共契约，分批落地，不在单个批次中混合无关重构。

## 总体原则

1. `Step()` 是外部简化入口，只返回该模块的主系统输出帧。
2. `StepWithResult()` 是结构化执行结果入口，返回主输出帧和本周期执行状态。
3. 开发调试视图由独立工具类从输入实体表、系统输出帧、生命周期事件和执行结果中组合出来。
4. 数值 ID 是唯一稳定关联键；名称字段只用于人读、调试、Trace/Replay 和报告展示。
5. 真实系统输出和仿真真值辅助必须隔离。仿真真值不得混入面向外部系统的主输出通道。
6. 未发现原因诊断默认关闭；只有显式开启时才记录 gate 级失败原因。
7. 日志只用于人读运行信息；状态判断不得依赖解析日志文本。

## 三层模型

| 层级 | 责任 | 典型类型 | 可被系统消费 | 可含调试便利字段 |
|------|------|----------|--------------|------------------|
| 原始系统输出层 | 展示、融合、记录该传感器或产品真实会输出的数据 | `EosOutputFrame`、`EsrOutputFrame`、`SarOutputFrame` 或兼容增强类型 | 是 | 否，除非字段本身是系统输出 |
| 结构化执行结果层 | 表达本周期是否执行、为何失败、是否复用上一帧、输入校验和诊断摘要 | `EosCycleResult`、`EsrCycleResult`、`SarCycleResult` | 是 | 可含结构化诊断摘要 |
| 开发调试视图层 | 把系统输出和输入实体表解析成开发人员可快速阅读的目标/产品状态 | `EosOutputViewBuilder`、`EsrOutputViewBuilder`、`SarOutputViewBuilder` | 否，默认调试用途 | 是 |

## 兼容策略

- 首选增强现有 `*OutputFrame`，不要轻易替换 `Step()` 返回类型。
- 新增字段必须有默认值，旧调用方零初始化后行为不变。
- 若新增类型不可避免，保留旧 `Step()` 和旧 `*OutputFrame` 路径，并提供显式 adapter。
- Replay schema 新字段必须是可选/默认空语义；旧 trace 的读取和比较不应因空名称字段失败。
- Contract tests 必须覆盖 public header 可用性、C++11 兼容性、无第三方类型泄露。

## 实体名称契约

### 公共语义

每个可输入实体应有数值 ID 和可选名称：

```cpp
std::uint64_t target_id_or_emitter_id{0U};
std::string target_name_or_emitter_name{};
```

名称字段语义：

- 空字符串表示未设置。
- 名称不参与唯一性判断。
- 同名不同 ID 是合法输入。
- 输出、Trace/Replay 和调试视图应尽量透传名称；若输出通道代表真实系统估计且不应包含真值直通字段，名称只能出现在调试视图或独立诊断事件中。
- 如果管线中间态无法稳定保留名称，调试视图必须能通过输入实体表按 ID 回填名称。

### 模块字段建议

| 模块 | 输入实体 | 名称字段 | 主关联键 |
|------|----------|----------|----------|
| EOS | `EosSceneTarget` / `EosExternalTargetInput` | `target_name` | `target_id` |
| ESR | `EsrSceneEmitter` / `EsrExternalEmitterInput` | `emitter_name` | `emitter_id` |
| SAR | 点目标、场景或采集任务 | `target_name`、`scene_name`、`collect_name` | 目标 ID 或产品/采集 ID |

SAR 不强行使用“目标航迹”语义。SAR 主流程优先记录成像产品和采集任务名称；点目标名称只用于仿真场景、质量评估和调试视图。

## 原始系统输出层字段契约

### EOS

EOS 原始系统输出应表达光电设备对目标的探测结果。**阶段 9A 去真值化后，
`EosDetectionRecord` 不再携带任何仿真目标语义字段（无 `target_id`/`target_name`），
改用 `detection_id` 表示传感器侧探测记录；仿真归属通过独立的 attribution 层承载。**

必备字段（`EosDetectionRecord`，传感器探测语义）：

- `cycle_index`（帧级，`EosOutputFrame`）
- `scan_azimuth_deg`（帧级，`EosOutputFrame`）
- 每个检测记录的 `detection_id`（传感器侧唯一探测标识，**不是**仿真 `target_id`）
- `range_m`
- `azimuth_deg`
- `elevation_deg`
- `infrared_snr_linear`
- `visible_snr_linear`
- `fused_snr_linear`
- `fused_snr_db`
- `detected`

仿真归属层（`EosDetectionAttributionRecord`，**不进入** `EosOutputFrame`，
由 `EosCycleResult.detection_attributions` 承载）：

- `detection_id` → `target_id` + `target_name` 的映射
- 这是从系统输出回到输入仿真实体的唯一桥梁，属于仿真辅助，不是真实传感器输出。

边界：

- 真实光电设备天然不知道外部仿真实体 ID；`target_id`/`target_name` 不得出现在
  `EosDetectionRecord` 或 `EosOutputFrame` 中。
- 从输出回到输入实体必须经 attribution 层（`EosCycleResult.detection_attributions`）
  或 debug/lifecycle 工具回填。
- `EosExternalOutputAdapter` 只导出 `detection_id` 与传感器测量字段。
- 未检测目标不应默认进入主检测列表，除非输出类型明确支持 negative report。
- 边界由 `tests/unit/eos_cycle_output_builder_test.cpp` 与 replay 严格相等比较守护。

### ESR

ESR 原始系统输出保持三通道结构：

1. `observation_output`
2. `emitter_output`
3. `truth_evaluation_output`

`observation_output` 接近设备观测上报：

- `observation_id`
- `timestamp` 或 `cycle_index`
- `emitter_id/name` 不进入当前 ESR 真实输出通道；需要人读映射时通过 debug view 从输入表和 truth evaluation 关联回填
- `rf_hz`
- `bandwidth_hz`
- `pulse_width_s`
- `pri_s`
- `azimuth_deg`
- `elevation_deg`
- `snr_db`
- `quality`
- `suppression/deception` 标记或质量影响摘要

`emitter_output` 是系统估计：

- `hypothesis_id`
- `status`
- `estimated_rf_hz`
- `estimated_pw_s`
- `estimated_pri_s`
- `estimated_aoa`
- `confidence`
- `hit_count/miss_count`
- `emitter_name` 只允许通过仿真归属或 debug view 回填，不得伪装成真实侦察系统字段。

`truth_evaluation_output` 只用于仿真评估：

- 允许包含 truth emitter id、匹配关系、评估置信度。
- 不得被外部系统当作真实侦察输出消费。
- `EsrOutputFrame` 中必须继续保留“真值评估通道与系统输出通道分离”的边界。

当前落地类型：

- EOS：`EosOutputDebugViewBuilder`、`EosDetectionLifecycleRecorder`
- ESR：`EsrOutputDebugViewBuilder`、`EsrEmitterLifecycleRecorder`
- SAR：`SarProductDebugViewBuilder`、`SarProductLifecycleRecorder`

### SAR

SAR 原始系统输出是成像产品，不是目标航迹。

必备字段：

- `cycle_index`
- `completed_stage`
- `range_sample_count`
- `azimuth_pulse_count`
- `center_slant_range_m`
- `estimated_snr_db`
- `phase_reference_mode`
- `range_resolution_3db_m`
- `azimuth_resolution_3db_m`
- `image_entropy_nats`
- `image_contrast`
- `has_raw_echo`
- `has_range_compressed_echo`
- `has_l1_image`
- `has_l3_bp_image`
- `has_image_quality_metrics`
- `image_resolution_m_valid`
- `product_name` 或 `collect_name`，可选
- `quality_flags`

边界：

- 点目标解释、峰值归属、真值对比属于 debug/evidence view，不进入主图像产品字段。
- `SarFocusedImage` 仍是实时结果载荷；Replay 可继续只保存摘要，除非后续 schema 冻结允许完整载荷。

## 结构化执行结果层契约

所有模块的 `*CycleResult` 应能回答以下问题：

- 本次输入周期号是什么？
- 本周期主输出帧是什么？
- 输入校验是否有 error？
- 本周期是否实际执行 pipeline？
- 是否复用上一有效输出？
- 若失败或中止，结构化原因是什么？
- 是否有模块诊断摘要？
- 是否产出了生命周期事件？

字段建议：

```cpp
std::uint32_t input_cycle_index{0U};
OutputFrame output_frame{};
ValidationIssueList validation_issues{};
bool has_validation_error{false};
bool executed_this_cycle{false};
bool reused_previous_output{false};
AbortReason abort_reason{};
DiagnosticIssueList diagnostics{};
LifecycleEventList lifecycle_events{};
```

兼容要求：

- EOS/ESR 当前已有结构化 abort enum，应沿用并扩展。
- SAR 当前 `abort_reason` 是字符串；阶段实现时需评估是否新增 enum，同时保留字符串字段用于兼容和可读说明。
- 生命周期事件列表可先为空，不能影响旧路径。

## 开发调试视图层契约

调试视图类只读输入和输出，不驱动 pipeline，不修改 session 状态。

**阶段 H 落地**：所有 debug view builder 与 lifecycle recorder 已从 header-only
内联实现收敛到「public header 只留类型与声明 + .cpp 实现」。其中 EOS/ESR 的
lifecycle recorder 采用 PImpl(`std::unique_ptr<Impl>`) 完全隔离 `<unordered_map>`
私有状态，确保 public header 不暴露重依赖。

### EOS 调试视图

落地类型（`include/1q/electro_optical_sensor/session/`）：

- `EosOutputDebugView` / `EosDebugTargetState` / `EosOutputDebugViewBuilder`（`EosOutputDebugView.h`）
- `EosDetectionLifecycleRecorder` / `EosDetectionLifecycleEvent`（`EosDetectionLifecycleRecorder.h`）

每个目标视图字段：

- `target_id`
- `target_name`（经 attribution 从输入表回填）
- `present_in_input`
- `has_raw_output_record`
- `detected`
- `range_m` / `azimuth_deg` / `elevation_deg` / `fused_snr_db`
- `status`（`kDetected` / `kObservedBelowThreshold` / `kNotInOutput` / `kCycleNotExecuted`）

### ESR 调试视图

落地类型（`include/1q/electronic_surveillance_radar/session/`）：

- `EsrOutputDebugView` / `EsrDebugEmitterState` / `EsrOutputDebugViewBuilder`（`EsrOutputDebugView.h`）
- `EsrEmitterLifecycleRecorder` / `EsrEmitterLifecycleEvent`（`EsrEmitterLifecycleRecorder.h`）

每个辐射源视图字段：

- `emitter_id`
- `emitter_name`（经 truth evaluation 关联从输入表回填）
- `present_in_input`
- `matched_observation` / `observation_id` / `confidence`
- `status`（`kObserved` / `kNotObserved` / `kNotEmitting` / `kCycleNotExecuted`）

### SAR 调试视图

落地类型（`include/1q/sar/session/`）：

- `SarProductDebugView` / `SarDebugPointTarget` / `SarProductDebugViewBuilder`（`SarProductDebugView.h`）
- `SarProductLifecycleRecorder` / `SarProductLifecycleEvent`（`SarProductLifecycleRecorder.h`）

产品视图字段：

- `completed_stage` / `has_raw_echo` / `has_range_compressed_echo` / `has_l1_image` / `has_l3_bp_image`
- `estimated_snr_db` / `range_sample_count` / `azimuth_pulse_count`
- `has_focused_pixels` / `diagnostics`
- `point_targets`（点目标 `target_id`/`target_name` 只在此 debug 视图出现，不进入产品输出）

产品生命周期事件按成像产品语义记录：`kImageProduced` / `kProductUpdated` /
`kProductLost` / `kProcessingFailed` / `kNoProduct`（无产品事件需显式开启）。

## 生命周期事件契约

### 公共事件类型

```cpp
enum class LifecycleEventType {
  kInputObserved,
  kFirstDetected,
  kConfirmed,
  kUpdated,
  kCoasted,
  kLost,
  kDropped,
  kNotDetected,
  kProductStarted,
  kProductCompleted,
  kProductFailed,
  kQualityAccepted,
  kQualityRejected
};
```

### 公共原因类型

```cpp
enum class LifecycleReason {
  kNone,
  kSensorDisabled,
  kInputInvalid,
  kOutOfFov,
  kOutOfRange,
  kBelowSnrThreshold,
  kBelowDetectionProbability,
  kAssociationFailed,
  kTooManyMisses,
  kRuntimeAborted,
  kQualityGateFailed,
  kProductUnavailable,
  kUnknown
};
```

### 事件字段

```cpp
struct LifecycleEvent {
  std::uint32_t cycle_index{0U};
  double sim_time_sec{0.0};
  std::uint64_t entity_id{0U};
  std::string entity_name{};
  std::uint64_t track_or_product_id{0U};
  LifecycleEventType type{LifecycleEventType::kInputObserved};
  LifecycleReason reason{LifecycleReason::kNone};
  std::string detail{};
};
```

实现策略：

- 可先定义模块内类型，后续再提升到 `oneq::foundation`，避免第一批过度抽象。
- 事件列表可放入 `*CycleResult`，同时由 `*LifecycleRecorder` 提供跨周期 summary。
- 未发现原因需要配置开启：

```cpp
struct LifecycleDiagnosticsConfig {
  bool enabled{false};
  bool record_not_detected_reasons{false};
  bool include_gate_details{false};
};
```

## Trace/Replay 契约

必须记录：

- 输入实体名称字段（`EosTargetState.target_name` / `EsrSceneEmitter.emitter_name` /
  `SarPointTarget.target_name`）。
- 系统输出帧新增字段。
- 诊断配置是否启用。

兼容要求：

- 新字段默认值不应改变旧 trace 读取行为。
- Debug-only 字段不应进入严格输出相等比较，除非该字段已被声明为系统输出契约。
- 归属/生命周期事件若进入 trace，必须使用独立 payload type，避免污染主输出 payload。

### 当前落地（阶段 9）

- **主 output payload 只保存系统输出层字段。** EOS `EosOutputFrame` 的 replay 载荷
  只含 `detections`（`detection_id` + 传感器测量），不含任何仿真 name/id。
- **仿真归属使用独立 payload。** EOS `detection_attributions` 是独立
  `EosDetectionAttributionRecord` table，挂在 `EosCycleResult` 上与 `EosOutputFrame`
  分离；replay 严格相等比较同时覆盖 raw output 和 attribution（见
  `EosReplaySession.cpp` 的 `EosDetectionAttributionListEqual`）。
- **ESR/SAR name 不进入输出 payload。** name 只在输入侧载荷中出现；真实输出三通道
  （ESR）与产品输出帧（SAR）不含 name。
- **生命周期事件暂不进入 trace。** lifecycle recorder 是独立工具类，由调用方按需
  调用，当前不强制写入 replay trace。这一决策避免默认路径 trace 体积膨胀；
  若未来需要把 lifecycle 事件纳入 replay，必须作为独立 event type
  （如 `payload_type="EosDetectionLifecycleEvents"`），不得塞入 cycle_output payload。

## 日志边界

当前日志宏位于 `src/common/logging/ProjectLog.h`。当构建宏 `PROJECT_LOG_BACKEND_SPDLOG=1` 时，宏转发到 spdlog；否则为空操作。

静态库消费者启用日志的最小方式：

```cpp
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

auto logger = spdlog::basic_logger_mt("oneq", "oneq.log");
spdlog::set_default_logger(logger);
spdlog::set_level(spdlog::level::debug);
spdlog::flush_on(spdlog::level::warn);
```

规则：

- 不得通过解析日志判断目标状态。
- 内部状态判断必须来自 `StepWithResult()`、生命周期事件、Trace/Replay 或显式诊断对象。
- 后续可新增 `oneq::logging` public helper，降低消费者直接接触 spdlog 的成本。
- 直接调用 `spdlog::*` 的历史代码应逐步迁移到 `PROJECT_LOG_*` 或 public logging facade。

## 分批实施顺序

1. Batch A：名称字段和兼容测试。
2. Batch B：EOS 输出视图与生命周期记录。
3. Batch C：ESR 输出视图与生命周期记录。
4. Batch D：SAR 产品视图与产品生命周期记录。
5. Batch E：Trace/Replay schema 扩展。
6. Batch F：日志 facade、文档、consumer examples、contract tests。

阶段 9（未上线边界优先重构，兼容性让位于边界干净）：

7. Batch G：EOS raw output 去真值化（`EosDetectionRecord` 改用 `detection_id`，
   新增 `EosDetectionAttributionRecord`，replay schema 按新边界重排）。
8. Batch H：调试/生命周期实现从 header-only 收敛到 .cpp（recorder 用 PImpl）。
9. Batch I：ESR/SAR 输出边界合同测试（编译期 trivially-copyable 哨兵 + 运行时
   pipeline 边界断言）。
10. Batch J：Trace/Replay 归属独立 payload 确认 + 契约文档收口（本文档）。

每批要求：

- 单独可构建。
- 单独有测试。
- 不放宽现有测试阈值。
- 不扩大 skip 或 known-limit 范围来制造绿色。
- header isolation guard、public API guard、replay roundtrip 均通过。

## 阶段 1 决策结论

| 问题 | 决策 |
|------|------|
| 是否引入跨模块公共基类 | 暂不引入。采用公共概念和命名约束，各模块保留领域专用类型。 |
| `Step()` 返回类型是否替换 | 暂不替换。优先增强现有 `*OutputFrame`，必要时新增 adapter。 |
| 名称字段是否参与关联 | 不参与。ID 仍是唯一稳定键。 |
| ESR 真值是否进入系统输出 | 不进入。继续保留 truth evaluation 独立通道。 |
| SAR 生命周期主语义 | 使用产品/采集生命周期，点目标解释进入 debug/evidence view。 |
| 日志是否可作状态判断 | 不可。日志只用于人读辅助。 |

## 阶段 9 决策结论

| 问题 | 决策 |
|------|------|
| EOS raw output 是否携带 `target_id`/`target_name` | 不携带。改用 `detection_id` 表示传感器探测记录，仿真归属用独立 `EosDetectionAttributionRecord` 承载。 |
| 未上线阶段是否优先边界干净 | 是。兼容性让位于真实输出/仿真归属/调试层边界清晰，允许 replay/schema/API 破坏性调整。 |
| 调试/生命周期是否保持 header-only | 否。实现下沉到 .cpp，public header 只留类型与声明；recorder 用 PImpl 隔离私有状态。 |
| 输出边界如何防止回归 | 编译期 `static_assert(std::is_trivially_copyable<...>)` 哨兵 + 运行时 pipeline 边界断言双重守护。 |
| 生命周期事件是否进入 replay trace | 暂不进入。lifecycle recorder 按需调用，避免默认路径 trace 体积膨胀；未来若纳入必须用独立 event type。 |
