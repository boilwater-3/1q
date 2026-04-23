# Replay Trace 复现采集设计

## 1. 背景

业务目标不是记录摘要日志，而是支持下面这条链路：

1. A 电脑创建场景并长时间仿真。
2. A 电脑在某个时刻出现问题。
3. 将采集数据带到 B 电脑。
4. B 电脑重建同一 session，按同一事件顺序回放，复现问题或定位首个分歧点。

因此采集器必须记录"可重新驱动模型"的完整输入数据，而不是仅记录统计摘要。

## 2. 设计目标

- 支持在 B 电脑重建 A 电脑仿真过程。
- 支持按事件顺序回放配置、输入、场景、运行时补丁。
- 支持用 A 电脑输出作为回放校验基准（**深度结构对比**，精确到每条轨迹每个字段）。
- 支持完整性校验（payload hash + event chain）。
- 支持故障时自动落盘 failure marker 与最近窗口。
- 支持分片存储与索引，便于大规模 trace 管理。
- 支持 **chunk 级 gzip 压缩**，Reader 透明解压。
- 模块只负责 typed payload 编解码，通用层负责 envelope 与存储。
- **跨平台**：macOS、Linux、Windows 均可运行。

## 3. 非目标

- 第一阶段不承诺跨编译器/跨 CPU 的 bit-exact 复现。
- 不把 replay trace 设计为通用业务日志系统。
- 不在第一阶段暴露全部内部私有状态（由后续 checkpoint/restore 完成）。

## 4. 产物目录结构

```text
trace-<trace_id>/
  manifest.json
  events/
    000000.events.jsonl          # 当前写入 chunk（明文）
    000001.events.jsonl.gz       # 已封存 chunk（压缩，Reader 透明解压）
    000002.events.jsonl.gz
  checkpoints/
    checkpoint-000000.fbs
    checkpoint-001000.fbs
  crash/
    failure.json
    last-window.events.jsonl
  indexes/
    cycles.idx
    checkpoints.idx
```

说明：

- `manifest.json`：运行身份与兼容性信息，含 `compress_closed_chunks` 字段。
- `events/*.jsonl`：JSONL 格式顺序事件流，当前 chunk 为明文，历史 chunk 压缩为 `.gz`。
- `checkpoints/*`：当前由 writer 预创建目录，checkpoint 事件/文件仍在 Phase 3 规划中。
- `crash/*`：故障上下文。
- `indexes/*`：当前写入 `cycles.idx`；`checkpoints.idx` 预留。

## 5. 事件 Envelope

```text
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
  "payload_encoding": "flatbuffers",
  "payload": null,
  "payload_base64": "BASE64_PAYLOAD_BYTES",
  "payload_hash": "fnv1a64:...",
  "previous_event_hash": "fnv1a64:..."
}
```

字段要点：

- 当前 AR replay 事件使用 `payload_encoding: "flatbuffers"`，写盘形态为 `payload:null` + `payload_base64`。
- `payload_hash`：对实际 payload 内容哈希。
- `previous_event_hash`：链式完整性校验。

## 6. 必须记录的事件类型

- `session_config`：完整 session 配置（含环境域 `environment_default_config`）。
- `cycle_input`：每个周期驱动输入。
- `scene_state`：每周期场景状态（如 AR 环境状态）。
- `runtime_config_patch`：运行时配置变更。
- `cycle_output`：每周期输出，含**完整 per-track 结构化字段**（用于深度比对）。
- `failure_marker`：故障事件与诊断信息（`has_validation_error` 时自动写入）。

说明：`checkpoint` 事件类型仍在规划中，当前代码路径未写入该事件。

## 7. Manifest 关键字段

- `trace_id`
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
- `failure_window_event_count`
- `compress_closed_chunks`（新增）：true 时已封存 chunk 自动压缩为 `.gz`

## 8. 三模块记录范围

### 8.1 AR

- `RadarSessionConfig`（含 `environment_default_config` 域）
- `RadarCycleInput`
- `EnvironmentSceneState`（含大气、植被散射、干扰源完整字段）
- `RadarRuntimeConfigPatch`
- `RadarCycleResult` / `TrackOutputFrame`（含 per-track 完整状态快照用于输出比对）

### 8.2 ESR

- `EsrSessionConfig`
- `EsrCycleInput`
- `EsrRuntimeConfigPatch`
- `EsrCycleResult` / `EsrOutputFrame`

### 8.3 EOS

- `EosSessionConfig`
- `EosCycleInput`
- `EosRuntimeConfigPatch`
- `EosCycleResult` / `EosOutputFrame`

补充状态（当前实现）：

- AR 提供了库内专用回放入口 `ReplayRadarTrace(trace_dir)`。
- EOS/ESR 当前提供 replay 事件采集与 codec，回放验证主要体现在示例与测试路径，尚无与 AR 对等的专用 `Replay*Trace` API。

## 9. 采集顺序（原则）

```text
construct session
  -> write manifest
  -> write session_config

ApplyRuntimeConfig(patch)
  -> write runtime_config_patch
  -> apply patch

Step(input[, scene_state])
  -> write cycle_input           (pending_input_written_ = true)
  -> write scene_state (if any)
  -> execute model
  -> write cycle_output          (pending_input_written_ = false)
  -> write failure_marker if has_validation_error (自动)
```

关键原则：输入在执行前写，输出在执行后写。

P2-B 保护：若连续两个 `cycle_input` 无中间 `cycle_output`，`RadarReplaySession` 报告错误，`RadarTraceSession` 通过 `pending_input_written_` 追踪状态。

## 10. Checkpoint 策略（Phase 3 规划中）

- 周期触发：每 N 个 cycle。
- 关键变更触发：runtime patch 前后。
- 异常触发：validation/error/failure 时。
- 手工触发：测试或调试显式请求。

checkpoint 至少应包含：

- `checkpoint_id`
- `cycle_index`
- `sim_time_sec`
- `last_event_sequence`
- `session_config_hash`
- `rng_state`（若存在）
- module-specific internal state

## 11. 故障窗口

采集器维护最近 N 条事件环形缓冲。故障发生时写入：

- `failure_marker`（事件流中）
- `crash/failure.json`
- `crash/last-window.events.jsonl`

**AR 自动写入**：`StepWithResult` 检测到 `has_validation_error=true` 时，由 `RadarTraceSession` 自动调用 `WriteFailureMarker`，无需调用方手动处理。

## 12. 序列化策略

- 当前主路径：FlatBuffers 作为 replay payload 编码。
- 当前 writer 统一写 `payload:null` + `payload_base64`（来源 `payload_bytes`）；reader 从 `payload_base64` 还原字节并做 hash/chain 校验。
- `payload_encoding` 当前主要作为语义标记（默认 `flatbuffers`），通用层不按编码类型切换写盘格式。
- FlatBuffers 生成头文件提交至仓库（`src/airborne_radar/session/generated/`），不在构建时自动生成。

## 13. B 电脑回放流程

```text
open trace
read manifest
check compatibility + scan hash/chain
build replay report
playback ordered events (按事件顺序调用回调)
apply input/scene/patch in order
AR 回放回调内做结构化输出比对（不在通用层自动比对）
stop on callback error or failure_marker (if enabled)
write replay report
```

## 14. 压缩与大规模 Trace 管理（Phase 4a 已实现）

### 14.1 体积估算

| 参数 | 值 |
|------|-----|
| 20 轨迹 × ~25 个 float/int 字段 | ≈ 2.5 KB FlatBuffers payload |
| Base64 + JSONL 包装 + hash | ≈ 3.5 KB/事件 |
| 10 Hz，每周期 2 个事件 | ≈ 70 KB/sec |
| 1 小时仿真 | ≈ 250 MB |
| 100 Hz / 50 轨迹 | ≈ 25 GB/hour |

### 14.2 Phase 4a：Chunk 级 gzip 压缩（已完成）

**设计**：在 `RotateEventChunkIfNeeded` 封存 chunk 时，若 `compress_closed_chunks=true`，则将 `.jsonl` 压缩为 `.jsonl.gz` 并删除原文件。Reader 在 `OpenEventChunk` 时优先查找 `.gz`，透明解压，接口不变。

**实现要点**：

- zlib 通过 Conan 供给：`zlib/1.3.1`（modern），`zlib/1.2.11`（VS2015），全平台一致。
- 压缩/解压实现在 `#if ONEQ_HAVE_ZLIB` 保护下，即使 zlib 不可用也不影响其他功能。
- 当前写入中的 chunk 保持明文，只有已封存的历史 chunk 被压缩，Crash 时不丢失事件。
- `ScanReplayTrace` / `PlaybackReplayTrace` 完全透明，不感知压缩。

**开启方式**：

```cpp
ReplayTraceManifest manifest;
manifest.compress_closed_chunks = true;
manifest.event_chunk_size = 10000U;
auto writer = std::make_shared<ReplayTraceWriter>(trace_dir, manifest);
```

**压缩比**：JSONL + Base64 文本负载在 zlib 下典型压缩比 **10×–20×**，250 MB → 15–25 MB。

### 14.3 Phase 4b–4c（规划中）

- **Phase 4b**：采样模式（`kFull` / `kEveryN` / `kOnChange` / `kSummaryOnly`）+ 存储约束（`max_trace_size_bytes` / `max_trace_age_hours`）。
- **Phase 4c**：Delta 编码（`cycle_output_delta` 事件类型），仅编码相邻帧间变化的轨迹，体积收益 2×–10×。

## 15. 实施阶段

| 阶段 | 内容 | 状态 |
|:---:|------|:---:|
| Phase 1 | 从 cycle 0 完整事件流回放 | ✅ 已完成 |
| Phase 2 | 完整 per-track 输出比对，codec round-trip 测试，failure_marker 自动写入 | ✅ 已完成 |
| Phase 3 | checkpoint/restore 内部状态恢复 | 🔲 规划中 |
| Phase 4a | Chunk 级 gzip 压缩，跨平台（Windows/macOS/Linux） | ✅ 已完成 |
| Phase 4b | 采样模式 + 存储约束 | 🔲 规划中 |
| Phase 4c | Delta 编码 `cycle_output_delta` | 🔲 规划中 |

## 16. 关键决策

- telemetry trace 与 replay trace 分离。
- replay 必须记录 typed payload，不能只记录摘要。
- output 用于深度结构对比（per-track），不用于驱动 replay。
- 兼容性显式写入 manifest。
- 压缩为 opt-in（`compress_closed_chunks=false` 默认），存量 trace 无影响。
- zlib 通过 Conan 统一供给，Windows 无需系统 zlib。

## 17. 待确认问题

- 是否需要对 trace 加密。
- 单次仿真允许的 trace 体积上限（Phase 4b 存储约束配置）。
- 是否需要跨平台严格复现或仅行为一致。
- 各模块 checkpoint 边界与状态最小集。

## 18. AR 模块实现说明

### 18.1 AR 数据收集器入口与主数据流

- 入口是 `RadarTraceSession`，通过 `RadarTraceSessionOptions.replay_writer` 注入 replay writer。
- `session_config` 在构造时写入（`trace_config_on_construct` 分支）。
- `cycle_input` 在每次 `Step` / `StepWithResult` 执行**前**写入，并设置 `pending_input_written_ = true`。
- `scene_state` 仅在带 scene 的 `Step` 路径执行前写入。
- `runtime_config_patch` 先写再 apply：`WriteRuntimeConfigPatchReplay -> session_.ApplyRuntimeConfig`。
- `cycle_output` 在模型执行后写入，并复位 `pending_input_written_ = false`。
- `failure_marker` 由 `StepWithResult` 在 `has_validation_error=true` 时自动写入（P1-A）。

### 18.2 Replay Writer/Reader 的 Payload 编码策略与完整性校验

- AR 事件设置 `payload_encoding="flatbuffers"`，writer 写 `payload:null` + `payload_base64`。
- `payload_hash` 对 `payload_bytes` 计算；`previous_event_hash` 链式完整性校验。
- reader 从 `payload_base64` 还原 `payload_bytes`，透明处理压缩 chunk。
- AR 各核心 payload 已走 FlatBuffers codec（encode/decode），decode 时做 verifier 校验。
- schema file id：`ARCI`（cycle_input）、`ARSS`（scene_state）、`ARSC`（session_config）。

### 18.3 AR 回放执行中 event_type 处理顺序与约束

- 回放入口：`ReplayRadarTrace(trace_dir)`，先 `BuildReplayTraceReport` 再 `PlaybackReplayTrace`。
- 分发顺序按 trace 文件事件顺序，不重排。
- `session_config` 先决：其他事件前必须已建 session，否则失败。
- 连续两个 `cycle_input` 无中间 `cycle_output`：`OnCycleInput` 返回 P2-B 错误（不再静默覆盖）。
- `cycle_output.payload_type` 仅支持 `RadarCycleResult` 或 `TrackOutputFrame`，其他类型失败。
- 选项：`require_output_callback=true`、`stop_on_first_divergence=true`、`stop_on_failure_marker=true`。

### 18.4 AR 输出比较与注意事项

- AR 输出比较基于 FlatBuffers 解码后的结构化字段，**深度 per-track 比对**（P0-B）。
- 比较函数：`TrackOutputFrameEqual` / `CycleResultEqual`。
- 当前 `TrackOutputFrameEqual` 实际比较字段包括：
  - frame 级：`cycle_index`、`published_track_count`、`batch_id`、`confirmed_track_count`、`contains_lost_tracks`、`tracks.size()`
  - track state 级：`association_key`、`external_target_id`、`status`、`position_{x,y,z}`、`velocity_{x,y,z}`、`speed`、`rcs`、`jamming_detected`、`hit_count`、`miss_count`
- 当前 AR 回放比对**未覆盖** `acceleration_*` 与 `acceleration` 字段；这些字段目前依赖 codec round-trip 测试保障编解码一致性。
- 新增字段须三联动：schema + codec encode/decode + 单测（round-trip + replay 流）。
- AR 回放链路已收紧为二进制 payload 优先；payload 为空时 decode 直接失败。
- `failure_marker` 在 AR 回放中也按 FlatBuffers 解码；生产侧写 failure marker 时应传 `payload_bytes`。

### 18.5 AR EventType 语义表

| event_type | 时机 | payload_type |
|------|------|------|
| `session_config` | 构造时 | `RadarSessionConfig` |
| `cycle_input` | Step 前 | `RadarCycleInput` |
| `scene_state` | Step 前（有场景） | `EnvironmentSceneState` |
| `runtime_config_patch` | ApplyRuntimeConfig 前 | `RadarRuntimeConfigPatch` |
| `cycle_output` | Step 后 | `RadarCycleResult` 或 `TrackOutputFrame` |
| `failure_marker` | 自动（has_validation_error）或手动 | `ReplayTraceFailure` |

### 18.6 AR Schema 与 Codec

- `schemas/replay/airborne_radar_replay.fbs`（`ARCI` — cycle_input）
- `schemas/replay/airborne_radar_scene_replay.fbs`（`ARSS` — scene_state）
- `schemas/replay/airborne_radar_session_replay.fbs`（`ARSC` — session_config，含 `EnvironmentDefaultConfig`）
- `src/airborne_radar/session/RadarReplayFlatbufferCodec.h/.cpp`（encode/decode 含 per-track DecisionTrackSnapshot）
- `src/airborne_radar/session/generated/`（预生成头文件，提交至仓库）

### 18.7 字段演进规则（强约束）

新增字段必须同步修改：

1. Schema（`.fbs`）
2. Codec（encode/decode）
3. 测试（至少 round-trip + replay 流）

否则会导致回放失败或产生 silent drift。

### 18.8 最小回归测试集

- `ReplayTraceWriterTest.*`（通用 writer/reader/scan/playback）
- `ReplayTraceCompressionTest.*`（gzip 压缩 round-trip，新增）
- `TraceSessionAdapterTest.Radar*`（文件：`tests/unit/ar_trace_session_adapter_test.cpp`，AR trace 写入/回放 integration）
- `ArReplayCodecRoundtripTest.*`（AR 全 payload 类型 codec round-trip，新增）

### 18.9 当前限制与后续工作（按代码现状）

- `PlaybackReplayTrace` 已支持通用分歧检测：当 `on_cycle_output` 回调返回非空 `actual_output_payload` 且与事件 `payload` 不一致时，填充 `divergence_*` 并置 `ok=false`；`stop_on_first_divergence=true` 时立即停止，`false` 时继续处理后续事件。
- `BuildReplayTraceReport` 已使用二进制语义输出首个 failure marker 负载：`first_failure_payload_base64`、`first_failure_payload_encoding`、`first_failure_payload_type`。
- 通用报告已将 `warning` 归类为已知非阻断事件（计入 `warning_event_count`，不计入 `unsupported_event_count`），因此不会单独导致 `replay_ready=false`。
- checkpoint 目录与索引目录已创建，但 checkpoint 事件/文件与 `checkpoints.idx` 仍待 Phase 3 落地。

文档编码要求：本文件应始终保持 UTF-8 与 LF 行尾。
