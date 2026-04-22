# Replay Trace 复现采集设计

## 1. 背景

业务目标不是记录摘要日志，而是支持下面这条链路：

1. A 电脑创建场景并长时间仿真。
2. A 电脑在某个时刻出现问题。
3. 将采集数据带到 B 电脑。
4. B 电脑重建同一 session，按同一事件顺序回放，复现问题或定位首个分歧点。

因此采集器必须记录“可重新驱动模型”的完整输入数据，而不是仅记录统计摘要。

## 2. 设计目标

- 支持在 B 电脑重建 A 电脑仿真过程。
- 支持按事件顺序回放配置、输入、场景、运行时补丁。
- 支持用 A 电脑输出作为回放校验基准。
- 支持完整性校验（payload hash + event chain）。
- 支持故障时落盘 failure marker 与最近窗口。
- 支持分片存储与索引，便于大规模 trace 管理。
- 模块只负责 typed payload 编解码，通用层负责 envelope 与存储。

## 3. 非目标

- 第一阶段不承诺跨编译器/跨 CPU 的 bit-exact 复现。
- 不把 replay trace 设计为通用业务日志系统。
- 不在第一阶段暴露全部内部私有状态（由后续 checkpoint/restore 完成）。

## 4. 产物目录结构

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

说明：

- `manifest.json`：运行身份与兼容性信息。
- `events/*.jsonl`：顺序事件流。
- `checkpoints/*`：快照数据。
- `crash/*`：故障上下文。
- `indexes/*`：定位 cycle/checkpoint 的加速索引。

## 5. 事件 Envelope

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
  "payload_encoding": "flatbuffers",
  "payload": null,
  "payload_base64": "BASE64_PAYLOAD_BYTES",
  "payload_hash": "fnv1a64:...",
  "previous_event_hash": "fnv1a64:..."
}
```

字段要点：

- `payload_encoding == "json"` 时，`payload` 为 JSON，通常不含 `payload_base64`。
- `payload_encoding != "json"`（如 flatbuffers）时，`payload` 为 `null`，`payload_base64` 存放二进制。
- `payload_hash`：对实际 payload 内容哈希。
- `previous_event_hash`：链式完整性校验。

## 6. 必须记录的事件类型

- `session_config`：完整 session 配置。
- `cycle_input`：每个周期驱动输入。
- `scene_state`：每周期场景状态（如 AR 环境状态）。
- `runtime_config_patch`：运行时配置变更。
- `cycle_output`：每周期输出（用于比对）。
- `checkpoint`：快照引用事件。
- `failure_marker`：故障事件与诊断信息。

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
- `dependency_versions_json`
- `float_policy`
- `default_tolerances_json`
- `checkpoint_interval_cycles`
- `event_chunk_size`
- `failure_window_event_count`

## 8. 三模块记录范围

### 8.1 AR

- `RadarSessionConfig`
- `RadarCycleInput`
- `EnvironmentSceneState`
- `RadarRuntimeConfigPatch`
- `RadarCycleResult`/`TrackOutputFrame`（用于输出比对）

### 8.2 ESR

- `EsrSessionConfig`
- `EsrCycleInput`
- `EsrRuntimeConfigPatch`
- `EsrCycleResult`/`EsrOutputFrame`

### 8.3 EOS

- `EosSessionConfig`
- `EosCycleInput`
- `EosRuntimeConfigPatch`
- `EosCycleResult`/`EosOutputFrame`

## 9. 采集顺序（原则）

```text
construct session
  -> write manifest
  -> write session_config

ApplyRuntimeConfig(patch)
  -> write runtime_config_patch
  -> apply patch

Step(input[, scene_state])
  -> write cycle_input
  -> write scene_state (if any)
  -> execute model
  -> write cycle_output
  -> write checkpoint (if triggered)
```

关键原则：输入在执行前写，输出在执行后写。

## 10. Checkpoint 策略

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

- `failure_marker`
- `crash/failure.json`
- `crash/last-window.events.jsonl`

## 12. 序列化策略

- 当前主路径：FlatBuffers 作为 replay payload 编码。
- JSON 仅作为通用 envelope 的一种显式编码类型，不作为 AR 回放主链路 fallback。
- 通用 writer/read 层支持 `payload_encoding` 分流，业务回放层按模块策略解码。

## 13. B 电脑回放流程

```text
open trace
read manifest
check compatibility
build replay report
playback ordered events
apply input/scene/patch in order
compare outputs
stop on divergence or failure_marker
write replay report
```

## 14. 实施阶段

- Phase 1：从 cycle 0 完整事件流回放（已完成主链路）。
- Phase 2：完善故障包与工程化工具链。
- Phase 3：完善 checkpoint/restore 内部状态恢复。
- Phase 4：压缩与大规模 trace 管理优化。

## 15. 关键决策

- telemetry trace 与 replay trace 分离。
- replay 必须记录 typed payload，不能只记录摘要。
- output 用于比对，不用于驱动 replay。
- 兼容性显式写入 manifest。

## 16. 待确认问题

- 是否需要对 trace 加密。
- 单次仿真允许的 trace 体积上限。
- 是否需要跨平台严格复现或仅行为一致。
- 各模块 checkpoint 边界与状态最小集。

## 17. AR 模块实现说明（历史章节）

本章保留历史背景。当前实现请以第 18 章“2026-04 更新”为准。

## 18. AR 模块实现（2026-04 更新）

AR 数据收集器入口与主数据流（代码依据）
入口是 RadarTraceSession，通过 RadarTraceSessionOptions.replay_writer 注入 replay writer。见 RadarTraceSession.h (line 23)、RadarTraceSession.cpp (line 474)。
配置（session_config）在构造时写入：WriteSessionConfigReplay，触发点在构造函数 trace_config_on_construct 分支。见 RadarTraceSession.cpp (line 333)、RadarTraceSession.cpp (line 479)。
输入（cycle_input）在每次 Step/StepWithResult 执行前写入：WriteCycleInputReplay。见 RadarTraceSession.cpp (line 321)、RadarTraceSession.cpp (line 487)、RadarTraceSession.cpp (line 523)。
场景（scene_state）仅在带 scene 的 Step 路径执行前写入：WriteSceneStateReplay。见 RadarTraceSession.cpp (line 345)、RadarTraceSession.cpp (line 505)。
运行时补丁（runtime_config_patch）在 ApplyRuntimeConfig 里“先写再 apply”：WriteRuntimeConfigPatchReplay -> session_.ApplyRuntimeConfig。见 RadarTraceSession.cpp (line 357)、RadarTraceSession.cpp (line 559)。
输出（cycle_output）在模型执行后写入：WriteTrackOutputReplay / WriteCycleResultReplay。见 RadarTraceSession.cpp (line 369)、RadarTraceSession.cpp (line 384)。
故障标记（failure_marker）当前不是 RadarTraceSession 自动写；由通用 ReplayTraceWriter::WriteFailureMarker 写入。见 ReplayTrace.h (line 189)、ReplayTrace.cpp (line 735)。AR 回放端会处理该事件，见 RadarReplaySession.cpp (line 210)。
replay writer/read path 的 payload 编码策略与完整性校验
当前 envelope 语义：payload_encoding=="json" 时写 payload；否则（flatbuffers）写 payload:null + payload_base64。见 ReplayTrace.cpp (line 698)、ReplayTrace.cpp (line 701)。
哈希语义：payload_hash 对 json 用 payload_json，对非 json 用 payload_bytes；没有 json fallback。见 ReplayTrace.cpp (line 177)、ReplayTrace.cpp (line 184)、ReplayTrace.cpp (line 675)。
事件链完整性：每条记录存 previous_event_hash，reader 校验 previous_event_hash_matches。见 ReplayTrace.cpp (line 705)、ReplayTrace.cpp (line 862)。
reader 还原：从 payload_base64 解出 payload_bytes；flatbuffers 事件 payload_json 常为 "null"。见 ReplayTrace.cpp (line 852)、ReplayTrace.cpp (line 853)。
AR 各核心 payload 已走 FlatBuffers codec（encode/decode），并在 decode 时做 verifier 校验。见 RadarReplayFlatbufferCodec.h (line 16)、RadarReplayFlatbufferCodec.cpp (line 658)、RadarReplayFlatbufferCodec.cpp (line 791)。
schema/file id 依据：ARCI(cycle_input)、ARSS(scene_state)、ARSC(session_config)。见 airborne_radar_replay.fbs (line 63)、airborne_radar_scene_replay.fbs (line 42)、airborne_radar_session_replay.fbs (line 221)。
AR 回放执行中 event_type 处理顺序与约束
回放入口：ReplayRadarTrace(trace_dir)，先 BuildReplayTraceReport 再 PlaybackReplayTrace。见 RadarReplaySession.cpp (line 225)。
分发顺序按 trace 文件事件顺序，不重排。见 ReplayTrace.cpp (line 1040)。
AR 回调绑定：on_session_config/on_cycle_input/on_scene_state/on_runtime_config_patch/on_cycle_output/on_failure_marker。见 RadarReplaySession.cpp (line 241)。
关键硬约束：
session_config 先决：cycle_input、runtime_config_patch、cycle_output 前必须已建 session，否则失败。见 RadarReplaySession.cpp (line 113)、RadarReplaySession.cpp (line 154)、RadarReplaySession.cpp (line 65)。
cycle_output 必须有待执行 cycle_input，否则失败。见 RadarReplaySession.cpp (line 69)。
cycle_output.payload_type 仅支持 RadarCycleResult 或 TrackOutputFrame，其他类型失败。见 RadarReplaySession.cpp (line 174)、RadarReplaySession.cpp (line 185)。
ReplayRadarTrace 选项是 require_output_callback=true、stop_on_first_divergence=true、stop_on_failure_marker=true。见 RadarReplaySession.cpp (line 249)。
细节约束（文档建议强调）：
scene_state 是“挂起到下一次 execute”的语义；推荐顺序 cycle_input -> scene_state -> cycle_output。见 RadarReplaySession.cpp (line 41)、RadarReplaySession.cpp (line 75)。
连续两个 cycle_input 会覆盖 pending_input（无显式报错），属于 trace 质量风险。见 RadarReplaySession.cpp (line 123)。
AR 输出比较当前是“摘要 JSON 比较”（payload_json 字符串比较触发 divergence），不是二进制 payload 逐字段深比较。见 RadarReplaySession.cpp (line 197)、ReplayTrace.cpp (line 1085)。
文档里应强调的注意事项
新增字段三联动：schemas/*.fbs + RadarReplayFlatbufferCodec encode/decode + 单测（至少 round-trip 与 replay 流）。依据文件 airborne_radar_replay.fbs、airborne_radar_scene_replay.fbs、airborne_radar_session_replay.fbs、RadarReplayFlatbufferCodec.cpp、ar_trace_session_adapter_test.cpp。
当前 AR 回放已按 flatbuffers bytes 解码，不应再依赖 json fallback；若事件写成 json-only，AR decode 会直接失败。见 RadarReplaySession.cpp (line 98)、RadarReplaySession.cpp (line 119)。
通用 replay 层仍支持 payload_encoding=json（为基础设施通用语义），但 AR 执行链路应统一写 flatbuffers。见 ReplayTrace.cpp (line 698)。
failure_marker 在 AR 回放中也按 FlatBuffers 解码；生产侧写 failure marker 时应传 payload_bytes。见 RadarReplaySession.cpp (line 216)、ReplayTrace.cpp (line 749)。
平台/实现差异：AR 当前只有 RadarTraceSession.cpp，文档若还引用 RadarTraceSession.windows.cpp 已过时；ESR/EOS 仍有 windows 分文件。见 src/airborne_radar/session/RadarTraceSession.cpp、src/electronic_surveillance_radar/session/EsrTraceSession.windows.cpp。
建议在 replay_trace_design.md 新增的小节标题（AR）
AR Replay 入口与事件写入链路（A 机）
AR Replay 回放调度与事件依赖约束（B 机）
FlatBuffers Payload 规范与完整性校验（Hash/Chain）
AR EventType 语义表（session_config/cycle_input/scene_state/runtime_config_patch/cycle_output/failure_marker）
字段演进与兼容性准则（Schema/Codec/Test 同步）

### 18.2 A 机事件写入链路

- 构造时写 `session_config`。
- `ApplyRuntimeConfig`：先写 `runtime_config_patch`，再 apply。
- `Step/StepWithResult`：执行前写 `cycle_input`，有场景则写 `scene_state`，执行后写 `cycle_output`。
- `failure_marker`：由通用 writer 写入。

推荐时序：

```text
session_config
(runtime_config_patch)*
for each cycle:
  cycle_input
  (scene_state)?
  cycle_output
...
failure_marker(optional)
```

### 18.3 B 机回放约束

- `session_config` 必须先于 `cycle_input/runtime_config_patch/cycle_output`。
- `cycle_output` 到达时必须已有 pending `cycle_input`。
- `cycle_output.payload_type` 仅支持 `RadarCycleResult` 或 `TrackOutputFrame`。
- 默认策略：`require_output_callback=true`、`stop_on_first_divergence=true`、`stop_on_failure_marker=true`。
- `scene_state` 为“挂起到下一次执行”的语义。

### 18.4 编码与完整性校验

- AR 主链路 payload：`flatbuffers`。
- envelope：
  - `json` -> 写 `payload`。
  - 非 `json` -> 写 `payload:null` + `payload_base64`。
- 哈希：
  - `json` 事件对 `payload_json` 哈希。
  - 非 `json` 事件对 `payload_bytes` 哈希。
  - 无 JSON fallback 哈希逻辑。
- event chain：每条事件记录并校验 `previous_event_hash`。

### 18.5 AR Schema 与 Codec

- `schemas/replay/airborne_radar_replay.fbs`（`ARCI`）
- `schemas/replay/airborne_radar_scene_replay.fbs`（`ARSS`）
- `schemas/replay/airborne_radar_session_replay.fbs`（`ARSC`）
- `src/airborne_radar/session/RadarReplayFlatbufferCodec.h/.cpp`

### 18.6 字段演进规则（强约束）

新增字段必须同步修改：

1. Schema（`.fbs`）
2. Codec（encode/decode）
3. 测试（至少 round-trip + replay 流）

否则会导致回放失败或产生 silent drift。

### 18.7 最小回归测试集

- `ReplayTraceWriterTest.*`
- `TraceSessionAdapterTest.Radar*`

---

文档编码要求：本文件应始终保持 UTF-8 与 LF 行尾。
