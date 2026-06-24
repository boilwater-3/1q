# 三层输出可观测性改造迁移指南

Date: 2026-06-25
Status: 适用于 `refactor/eos-esr-sar-output-observability` 分支

本文面向已基于旧 API 集成 EOS/ESR/SAR/AR 的调用方，说明三层输出可观测性
改造带来的变更与迁移路径。设计契约见 `docs/output_observability_contract.md`。

## 总览：三层输出模型

改造后四个传感器模块统一具备三层输出：

| 层级 | 入口 | 内容 | 可被外部系统消费 |
|------|------|------|------------------|
| 原始系统输出层 | `Step()` 返回的 `*OutputFrame` | 传感器/产品真实输出 | 是 |
| 结构化执行结果层 | `StepWithResult()` 返回的 `*CycleResult` | 输出帧 + 执行状态 + 校验 + abort reason | 是 |
| 开发调试视图层 | `*OutputDebugViewBuilder` / `*LifecycleRecorder` | 人读目标/产品状态、生命周期事件 | 否（调试用途） |

**关键不变量**：`Step()` 行为不变，仍返回主系统输出帧。旧调用方零改动即可继续工作。

## 非破坏性变更（向后兼容）

### 1. 实体名称字段（所有模块）

输入实体新增可选 `target_name` / `emitter_name` 字段：

```cpp
// EOS
eos::session::EosSceneTarget target;
target.target_id = 1U;
target.target_name = "alpha";  // 新增，可选，默认空

// ESR
esr::session::EsrSceneEmitter emitter;
emitter.emitter_id = 7U;
emitter.emitter_name = "bravo";  // 新增，可选

// SAR
sar::session::SarPointTarget point;
point.target_id = 9U;
point.target_name = "charlie";  // 新增，可选

// AR
airborne_radar::session::RadarSceneTarget target;
target.external_target_id = 42U;
target.target_name = "delta";  // 新增，可选
```

**迁移要点**：
- name 默认空字符串，不提供时行为与旧版完全一致。
- **name 不参与关联**，ID（`target_id` / `emitter_id` / `external_target_id`）仍是唯一稳定键。
- 同名不同 ID 是合法输入，不会互相干扰。
- name 仅用于人读、trace、报告展示和调试视图。

### 2. 结构化执行结果（所有模块）

`StepWithResult()` 返回的 `*CycleResult` 一直是执行状态入口，本次只是补齐字段：

- `input_cycle_index` / `executed_this_cycle` / `reused_previous_output`
- `has_validation_error` / `validation_issues` / `abort_reason`

旧调用方若已用 `StepWithResult()`，无需改动。

### 3. 开发调试视图与生命周期记录器（所有模块，新增）

新增只读工具类，**不影响主输出路径**，按需调用：

```cpp
// debug view：把输出 + 输入实体表合成为开发可读状态
const auto view = eos::session::EosOutputDebugViewBuilder::Build(input, result);

// lifecycle recorder：跨周期记录首次发现/更新/丢失
eos::session::EosDetectionLifecycleRecorder recorder;
auto events = recorder.Update(input, result);
```

四个模块的对应类型：

| 模块 | DebugViewBuilder | LifecycleRecorder |
|------|------------------|-------------------|
| EOS | `EosOutputDebugViewBuilder` | `EosDetectionLifecycleRecorder` |
| ESR | `EsrOutputDebugViewBuilder` | `EsrEmitterLifecycleRecorder` |
| SAR | `SarProductDebugViewBuilder` | `SarProductLifecycleRecorder` |
| AR | `RadarTrackOutputDebugViewBuilder` | `RadarTrackLifecycleRecorder` |

未发现/未观测/无产品原因默认关闭，通过 recorder config 显式开启：

```cpp
eos::session::EosDetectionLifecycleRecorder recorder(
    eos::session::EosDetectionLifecycleRecorderConfig{/*emit_not_detected_events=*/true});
```

## 破坏性变更（仅 EOS，阶段 9A 去真值化）

> 项目当前未上线，阶段 9 允许破坏性重构以换取边界干净。以下变更影响 EOS replay
> schema 与 raw output 结构，旧 EOS trace 无法直接回放。

### EOS raw output 去真值化

**变更前**：`EosDetectionRecord` 直接携带 `target_id` / `target_name`（仿真真值字段
混入了传感器原始输出）。

**变更后**：
- `EosDetectionRecord` 改用 `detection_id`（传感器侧探测记录标识），**移除**
  `target_id` / `target_name`。
- 仿真归属用独立的 `EosDetectionAttributionRecord` 承载，挂在
  `EosCycleResult.detection_attributions`，表达 `detection_id → target_id + target_name` 映射。

**迁移步骤**：

1. **访问检测记录的目标身份**：旧代码若直接读 `record.target_id`，改为经 attribution：
   ```cpp
   // 旧（已删除）
   // std::uint64_t tid = record.target_id;

   // 新：从 CycleResult 的 attribution 列表按 detection_id 查
   for (const auto& attr : result.detection_attributions) {
     if (attr.detection_id == record.detection_id) {
       // attr.target_id / attr.target_name 是仿真归属
     }
   }
   ```

2. **使用 debug view 代替手写归属查找**（推荐）：
   ```cpp
   const auto view = eos::session::EosOutputDebugViewBuilder::Build(input, result);
   // view.targets[i].target_name / .target_id 已通过 attribution 回填
   ```

3. **Replay trace**：旧 EOS replay trace（含 `target_id`/`target_name` 在 detection
   record 中的版本）与新 schema 不兼容。需重新生成 trace。

### 边界原则

- 真实系统输出层（`*OutputFrame`）不得携带仿真输入实体名称。
- 从输出回到输入实体必须经 attribution（EOS）或 debug view（所有模块）。
- ESR/SAR/AR 的 name 只在输入侧和 debug view 中出现，不在真实输出帧中（ESR/SAR）
  或作为人读标签附在系统估计上（AR track）。

## 日志

- 日志用法已统一：全仓使用 `PROJECT_LOG_*` 宏，不再直接调用 `spdlog::`。
- **不引入 public logging facade**：spdlog 保持 PRIVATE 依赖，`ProjectLog.h` 保持
  内部头。静态库消费者启用日志的最小方式见 `docs/output_observability_contract.md`
  日志边界小节。
- **状态判断不得依赖解析日志文本**：必须用 `StepWithResult()`、生命周期事件、
  Trace/Replay 或显式诊断对象。

## 验证

迁移后可用以下方式验证：

1. **consumer examples**：`tests/consumer/*_output_observability_consumer.cpp` 演示
   三层输出的外部消费方式（安装后冒烟）。
2. **边界合同测试**：`tests/unit/*_output_boundary_contract_test.cpp` 锁定真实输出
   不被 name 污染的边界。
3. **replay roundtrip**：`1q_replay_fast_tests` 验证 name 字段与 attribution 的
   序列化/反序列化。
4. **public api convenience**：`tests/contract/*_public_api_convenience_test.cpp`
   验证各模块 public API 可达性（含 SAR）。
