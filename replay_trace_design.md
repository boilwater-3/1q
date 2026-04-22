# Replay Trace 复现采集设计

## 1. 背景

业务目标不是简单记录运行摘要，而是支持这个流程：

1. A 电脑创建场景并长时间仿真。
2. A 电脑在某个时刻出现问题。
3. 将采集数据拷贝到 B 电脑。
4. B 电脑重建同一个 session，并按同一时间线 replay，直到复现问题或定位首次分歧。

因此，采集器必须记录“可重新驱动模型的数据”。当前 Windows trace 只记录 `target_feature_count`、`detection_count` 这类摘要，无法复现场景、输入、运行时配置变化、随机状态和模型内部状态。

## 2. 设计目标

- 能在 B 电脑重建 A 电脑的仿真 session。
- 能按原始顺序回放每个外部输入。
- 能用 A 电脑输出作为 replay 校验基准。
- 能为长时间仿真提供 checkpoint，避免每次从 cycle 0 开始重放。
- 能在故障发生时保存最后窗口数据。
- trace 格式必须版本化、可校验、可检查。
- 模块业务字段由模块 serializer 负责，通用 writer 只处理事件 envelope。

## 3. 非目标

- 第一版不承诺跨编译器、跨 CPU 的 bit-exact 复现。
- 不把所有内部私有状态直接暴露为公共 API。
- 不把 replay trace 混成普通日志系统。

## 4. 采集产物结构

Replay trace 建议是目录，而不是单个大文件：

```text
trace-<trace_id>/
  manifest.json
  events/
    000000.events.jsonl
    000001.events.jsonl
  checkpoints/
    checkpoint-000000.json
    checkpoint-001000.json
  crash/
    failure.json
    last-window.events.jsonl
  indexes/
    cycles.idx
    checkpoints.idx
```

`manifest.json` 保存运行身份和兼容性信息。`events/*.jsonl` 保存按序排列的 replay 事件。`checkpoints/*` 保存周期性状态快照。`crash/*` 保存故障现场。`indexes/*` 用于快速定位 cycle 和 checkpoint。

## 5. 事件 Envelope

所有事件统一使用 envelope：

```json
{
  "schema_version": 1,
  "trace_id": "20260421-...",
  "sequence": 42,
  "module": "airborne_radar",
  "event_type": "cycle_input",
  "cycle_index": 1001,
  "sim_time_sec": 1001.0,
  "wall_time_ms": 1776770000000,
  "payload_type": "RadarCycleInput",
  "payload_encoding": "json",
  "payload": {},
  "payload_hash": "sha256:...",
  "previous_event_hash": "sha256:..."
}
```

字段含义：

- `schema_version`: replay trace schema 版本。
- `trace_id`: 本次仿真的唯一 ID。
- `sequence`: 全局递增事件序号。
- `module`: `airborne_radar`、`electronic_surveillance_radar` 或 `electro_optical_sensor`。
- `event_type`: 事件类型。
- `cycle_index`: 当前周期编号；没有周期语义时可为空或为最近周期。
- `sim_time_sec`: 仿真时间。
- `wall_time_ms`: 真实墙钟时间，用于排查长时间运行问题。
- `payload_type`: 业务 payload 的具体类型。
- `payload_encoding`: 第一版使用 `json`，后续可扩展 `flexbuffers`、压缩二进制。
- `payload`: 完整业务数据。
- `payload_hash`: payload 内容哈希，用于拷贝和读取校验。
- `previous_event_hash`: 前一事件哈希，用于发现丢事件、乱序或截断。

## 6. 必须记录的事件

`session_config`:
完整 session 配置。必须是解析默认值之后的完整配置，包括硬件、任务、策略、环境、扫描、检测、跟踪等字段。

`initial_scene`:
A 电脑创建的初始场景真值。它不是摘要，而是重建场景的源数据。

`cycle_input`:
每次传入 `Step` 或 `StepWithResult` 的完整输入。

`scene_state`:
独立传入的场景状态，例如 AR 的 `EnvironmentSceneState`。

`runtime_config_patch`:
每次 `ApplyRuntimeConfig` 的完整 patch，并记录发生时的 cycle 和仿真时间。

`external_command`:
外部控制、人工注入、暂停/恢复、场景突变、测试驱动事件等会影响后续行为的事件。

`rng_state`:
随机种子和可序列化的随机引擎状态。至少要记录 seed；存在随机噪声、虚警、欺骗、量测误差时，需要更强的 RNG 状态保存。

`cycle_output`:
每次模型输出的完整结果。输出不用于驱动 replay，但用于对比 B 电脑是否复现 A 电脑行为。

`checkpoint`:
指向 checkpoint 文件的事件。大 payload 放在 `checkpoints/` 下，不直接塞进事件流。

`failure_marker`:
故障标记，包括错误码、异常信息、断言位置、最后 cycle、最后 event sequence、相关诊断文件。

## 7. Manifest 内容

`manifest.json` 至少包括：

- `trace_id`
- `created_wall_time_ms`
- `module`
- `scenario_id`
- `schema_version`
- `serializer_version`
- `git_commit`
- `git_dirty`
- `build_type`
- `compiler`
- `compiler_version`
- `platform`
- `cpu_arch`
- `library_version`
- `dependency_versions`
- `float_policy`
- `default_tolerances`
- `checkpoint_interval_cycles`
- `event_chunk_size`

B 电脑 replay 时必须校验 `schema_version`、`serializer_version`、`git_commit`。不匹配时不一定禁止 replay，但必须给出明确警告。

## 8. 各模块必须保存的业务数据

AR 机载雷达：

- 完整 `RadarSessionConfig`。
- 完整 `RadarCycleInput`: `dt_sec`、`platform_pose`、每个 `TargetFeature`。
- 使用场景态 Step 时，完整 `EnvironmentSceneState`。
- 完整 `RadarRuntimeConfigPatch`。
- 完整 `RadarCycleResult`: `TrackOutputFrame`、validation issues、submitted commands、control profile、abort reason、association metrics。
- checkpoint 需要覆盖 track pool、track lifecycle、filter state、last output、cycle counters、control profile state、environment runtime state、random state。

ESR 电子侦察雷达：

- 完整 `EsrSessionConfig`。
- 完整 `EsrCycleInput`: `cycle_index`、`dt_sec`、`platform_pose`、每个 `EmitterTruthState`、`environment_observation`。
- 完整 `EsrRuntimeConfigPatch`。
- 完整 `EsrCycleResult` 和 `EsrOutputFrame`。
- checkpoint 需要覆盖 observation history、hypothesis association state、cluster/track state、scan state、environment runtime state、random state。

EOS 光电传感器：

- 完整 `EosSessionConfig`。
- 完整 `EosCycleInput`: `cycle_index`、`dt_sec`、`platform_pose`、太阳参数、云量、昼夜类型、背景温度、每个 `EosTargetState`。
- 完整 `EosRuntimeConfigPatch`。
- 完整 `EosCycleResult` 和 `EosOutputFrame`。
- checkpoint 需要覆盖 scan state、last output、cadence/frame scheduler state、environment model state、random state。

## 9. 采集顺序

Session wrapper 应按如下顺序记录：

```text
construct session
  write manifest
  write session_config
  write initial_scene

ApplyRuntimeConfig(patch)
  write runtime_config_patch
  apply patch
  optionally write runtime_config_applied

Step(input)
  write cycle_input
  write scene_state when present
  write rng_state when available
  execute model
  write cycle_output
  write checkpoint when triggered
```

输入必须在执行前写入。这样即使模型在本周期内部崩溃，B 电脑仍然能拿到触发崩溃的输入。

## 10. Checkpoint 策略

长时间仿真不能只依赖从头 replay。checkpoint 用来缩短复现路径。

第一版可以先支持从 cycle 0 完整 replay，并写 checkpoint 元信息。第二版再支持真正恢复内部状态。

触发条件：

- 每 `N` 个周期。
- runtime config 变化前后。
- 用户显式请求。
- 出现 validation error 或异常 abort reason。
- 发生故障且进程仍可写文件。

checkpoint 文件应包含：

- `checkpoint_id`
- `cycle_index`
- `sim_time_sec`
- `last_event_sequence`
- `session_config_hash`
- `rng_state`
- module-specific internal state
- `last_output`

## 11. 故障窗口

采集器应维护一个内存 ring buffer，保存最后 `N` 个事件。故障发生时写入：

- 最近 checkpoint id。
- 最后 `N` 个事件。
- 触发故障的 input。
- 最后一个成功 output。
- failure metadata。
- process/build metadata。

B 电脑优先从最近 checkpoint 恢复，再 replay 最后窗口。

## 12. 序列化方案

第一版使用 JSON 作为 durable replay 格式，因为它可检查、便于测试，并且当前非 Windows trace 已有完整 JSON serializer 的雏形。

不要继续扩展当前 summary-style `TraceSink`。建议新增 replay 专用层：

```text
ReplayTraceWriter
  WriteManifest(...)
  WriteEvent(...)
  WriteCheckpoint(...)
  Flush()

ReplayTraceReader
  ReadManifest(...)
  IterateEvents(...)
  LoadCheckpoint(...)

ReplaySerializer<T>
  ToJson(T)
  FromJson(Json)
```

当前 `TraceSink` 可以保留为轻量 telemetry。Replay capture 是更强语义的功能，应该独立。

## 13. B 电脑 Replay 流程

```text
open trace directory
read manifest
verify schema/build compatibility
create session from session_config
load initial_scene
optionally restore nearest checkpoint
iterate ordered events
apply runtime_config_patch at recorded sequence
feed cycle_input and scene_state into Step/StepWithResult
compare cycle_output with recorded output
stop at first divergence or reproduce failure_marker
write replay_report.json
```

输出对比需要模块级浮点容差。报告要区分：完全一致、容差内一致、超出容差、缺事件、不支持 checkpoint、构建不兼容。

## 14. 实施阶段

Phase 1: 从 cycle 0 完整 replay。

- 增加 replay trace schema 和 writer。
- 序列化完整 config、input、runtime patch、output。
- 增加 AR/ESR/EOS replay 示例。
- 增加“记录短仿真再 replay 并对比输出”的测试。

Phase 2: 故障包和工程化。

- 增加最后窗口 ring buffer。
- 增加分片 event 文件和索引。
- 增加 hash 链和 manifest 兼容性检查。
- 增加显式 failure marker API。

Phase 3: 真正 checkpoint/restore。

- 为各模块增加内部状态 snapshot/export 接口。
- 增加 checkpoint restore 路径。
- 增加从 checkpoint replay 的测试。

Phase 4: 二进制与压缩。

- 增加可选 FlexBuffers 或 FlatBuffers payload encoding。
- 为长时间仿真增加压缩分片。
- 保留 JSON debug/export 格式。

## 15. 关键决策

- telemetry trace 和 replay trace 分离。
- replay 事件必须记录完整 typed payload，不能记录摘要。
- input 在执行前记录，output 在执行后记录。
- output 是校验数据，不是 replay 输入。
- checkpoint 用来缩短 replay 时间，不能替代事件流。
- 所有兼容性都显式写进 manifest。

## 16. 待确认问题

- replay trace 是否需要加密，因为它会包含完整场景真值。
- 最大仿真时长和可接受 trace 体积是多少。
- 业务上需要 bit-exact 复现，还是容差内行为复现即可。
- 第一阶段先落哪个模块：AR、ESR 还是 EOS。
- 是否要求跨操作系统 replay，还是只要求同平台 replay。

## 17. AR 模块 Phase 1 实现说明

### 17.1 完成度判断

AR 模块的 replay 主链路已经达到 Phase 1 可落地状态：A 电脑运行仿真时可以把重建同一条仿真路径所需的关键外部输入记录到 replay trace；B 电脑可以读取 trace、重建 `RadarSession`、按事件顺序回放输入，并用 A 电脑记录的输出做一致性校验。

当前完成的是“从 cycle 0 开始的完整事件流回放”，不是最终形态的 checkpoint/restore。也就是说：
- 已完成：配置、周期输入、场景状态、运行期配置、输出对比、失败标记。
- 未完成：从中间 checkpoint 恢复内部状态、跨编译器/跨 CPU 的 bit-exact 保证、完整内部私有状态快照。

### 17.2 相关入口和文件

AR replay 的公共入口：
- `include/1q/airborne_radar/session/RadarReplaySession.h`
- `session::ReplayRadarTrace(const std::string& trace_dir)`

AR replay 的实现：
- `src/airborne_radar/session/RadarReplaySession.cpp`

AR trace 写入端：
- `src/airborne_radar/session/RadarTraceSession.cpp`
- `src/airborne_radar/session/RadarTraceSession.windows.cpp`

通用 replay 基础设施：
- `include/1q/replay/ReplayTrace.h`
- `src/common/replay/ReplayTrace.cpp`

覆盖测试：
- `tests/unit/ar_trace_session_adapter_test.cpp`
- `tests/unit/replay_trace_writer_test.cpp`

### 17.3 A 电脑写入的数据流

AR 模块通过 `RadarTraceSession` 包装真实 `RadarSession`。业务侧仍然调用 `RadarTraceSession::Step`、`StepWithResult`、`ApplyRuntimeConfig`，wrapper 在调用真实模型前后写入 replay events。

写入顺序如下：

```text
construct RadarTraceSession
  -> write session_config

ApplyRuntimeConfig(patch)
  -> write runtime_config_patch
  -> apply patch to RadarSession

StepWithResult(input)
  -> write cycle_input
  -> if provided, write scene_state
  -> execute RadarSession::StepWithResult
  -> write cycle_output

failure happens
  -> write failure_marker
```

注意：`cycle_input` 和 `scene_state` 必须在模型执行前写入。这样即使 A 电脑在本周期内部崩溃，trace 中也已经有触发问题的输入。

### 17.4 B 电脑回放的数据流

B 电脑调用：

```cpp
const session::RadarReplaySessionResult result =
    session::ReplayRadarTrace(trace_dir);
```

内部流程：

```text
BuildReplayTraceReport(trace_dir)
  -> 校验 manifest/schema/module/hash/event chain
  -> 统计 session_config/cycle_input/scene_state/runtime_config_patch/cycle_output/failure_marker

PlaybackReplayTrace(trace_dir)
  -> session_config: 创建 RadarSession
  -> runtime_config_patch: 调用 RadarSession::ApplyRuntimeConfig
  -> cycle_input: 暂存输入
  -> scene_state: 暂存场景
  -> cycle_output: 执行暂存的 input/scene，然后和 A 电脑输出比较
  -> failure_marker: 停止回放并返回故障 payload
```

这里有一个容易踩坑的细节：`RadarTraceSession::Step(input, scene_state)` 写事件的顺序是先 `cycle_input`，再 `scene_state`，最后才执行模型。因此 B 电脑不能在收到 `cycle_input` 时立刻执行模型，而是必须暂存输入，等收到对应 `cycle_output` 之前再执行。当前 `RadarReplaySession.cpp` 已按这个规则实现。

### 17.5 已记录的 AR payload

`session_config` 记录用于重建 `RadarSession` 的初始化配置，当前覆盖：
- `hardware.detection`
- `mission.orientation`
- `policy.beam_control`
- `policy.association`
- `policy.tracking`
- `policy.lifecycle`
- `policy.imm`
- `jamming_sensitivity_profile`

`cycle_input` 记录每个周期的驱动输入：
- `dt_sec`
- `platform_pose`
- `target_features`
- 目标速度、RCS、距离、笛卡尔位置、Swerling 类型等字段

`scene_state` 记录 Step 级别显式传入的环境状态：
- `atmospheric_physics`
- `atmospheric_context`
- `vegetation_scatter_physics`
- `jammer_emitters`

`runtime_config_patch` 记录运行期调参：
- full-domain `mission`
- full-domain `policy`
- `environment_runtime_config`
- `work_sub_mode`
- `scan_center_deg`
- `dwell_center_deg`
- `commanded_beamwidth_deg`
- `commanded_beamwidth_enabled`

`cycle_output` 记录用于对比的输出摘要：
- `RadarCycleResult`: validation issue 数量、是否执行本周期
- `TrackOutputFrame`: cycle index、发布轨迹数量

`failure_marker` 记录原始故障点：
- error code
- message
- cycle index
- diagnostics payload

### 17.6 回放结果结构

`RadarReplaySessionResult` 聚合三类信息：

```text
report
  -> trace 是否 ready、事件数量、是否有 failure marker、首个故障 payload

playback
  -> 实际处理了多少事件、应用了多少 input/scene/runtime patch、比较了多少 output、是否 divergence

AR 扩展字段
  -> reached_failure_marker
  -> failure_marker_payload_json
  -> first_error
```

使用侧判断建议：

```cpp
if (!result.ok) {
  // 回放过程失败，优先看 result.first_error
}

if (result.playback.divergence_found) {
  // B 电脑输出和 A 电脑记录输出不一致
}

if (result.reached_failure_marker) {
  // 已经走到 A 电脑记录的故障点
}
```

### 17.7 重要实现细节

1. replay trace 和 telemetry trace 是两套语义。telemetry 可以是摘要，replay 必须保存可驱动模型的 typed payload。

2. `cycle_output` 只用于校验，不用于驱动 B 电脑模型。B 电脑必须自己执行模型得到 actual output，然后和 A 电脑 expected output 比较。

3. Windows 侧当前 payload 标记仍保留 `"serializer":"flatbuffers"` 和 `"platform":"windows"`，但 Phase 1 durable replay payload 实际采用可检查 JSON 字段。后续如切到真正二进制 payload，需要同步实现稳定的 reader。

4. 当前 AR parser 是轻量字符串解析，目的是避免给 VS2015 `airborne_core` 目标引入额外 JSON include 依赖。它适合读取本模块 serializer 生成的稳定 JSON，不适合作为任意 JSON 通用解析器。

5. 旧 trace 兼容：如果旧 `session_config` 只有 type 壳而没有完整字段，回放会落到默认配置。这能兼容早期 trace，但不建议用于严肃复现。

6. 运行期 patch 的语义是“先记录，再应用”。回放时必须按事件流原始顺序调用 `ApplyRuntimeConfig`，否则后续周期行为会偏移。

7. `scene_state` 是 per-step 显式场景状态；`environment_runtime_config` 是运行期默认环境配置 patch。两者不是同一个东西，都需要记录。

8. `failure_marker` 不是 divergence。failure marker 表示 A 电脑记录的原始故障点；divergence 表示 B 电脑回放输出已经和 A 电脑记录输出不一致。

### 17.8 当前限制和后续工作

当前 Phase 1 仍有这些限制：
- 不支持从 checkpoint 恢复内部状态，只能从 cycle 0 或已有事件起点完整回放。
- `cycle_output` 目前仍是摘要级对比，不是完整 `RadarCycleResult` 深度结构对比。
- 没有记录 RNG engine 内部状态；如果未来 AR 引入随机噪声，至少需要记录 seed，最好记录可恢复 RNG state。
- 没有记录所有内部私有状态，例如 track pool、filter covariance、controller internal snapshot；这些应进入 Phase 3 checkpoint/restore。
- 轻量 JSON parser 依赖当前 serializer 的字段命名和格式，新增字段时要同步补 serializer、parser 和测试。

建议下一阶段优先级：
- Phase 2: 增加更完整的 output payload，用结构化字段定位 divergence。
- Phase 3: 设计 AR checkpoint，覆盖 track lifecycle、filter state、environment runtime state、controller runtime state。
- Phase 4: 再考虑压缩、二进制 payload、跨平台容差策略和大规模长时间仿真 trace 管理。
