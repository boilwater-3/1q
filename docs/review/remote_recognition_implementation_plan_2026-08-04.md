# 远程识别实现会话规划（继续规划）

Status: draft
Date: 2026-08-04
Subject: 从 `remote_recognition_conflict_and_plan_2026-08-04.md`（Status: final）继续规划：证据复核 → Stage A 证据矩阵 → 冻结契约 → 阶段 1 实现级任务分解。
Codebase: HEAD `4678ec51`（branch `main`，与本会话开始一致，已全部复核）

> 本会话不写生产代码。产出物：实现会话可直接执行的阶段 1 规划与全阶段门禁条件。
> 沿用证据优先模式（`docs/common/contract.md` §证据优先开发模式 + `.zcode/skills/evidence-first-freeze-contract`）。

---

## 0. 证据复核（继续规划前核验，全部通过）

前文档（`remote_recognition_conflict_and_plan_2026-08-04.md`）的关键 `file:line` 证据在 HEAD `4678ec51` 下逐条复核：

| 引用 | 核验结果 |
|---|---|
| C1 `ScanScheduleResolver.cpp:68-79` default 静默返回 1.0 | ✅ 成立（`ResolveScanStepScale`） |
| C1 `:194` kStt passthrough | ✅ 成立（`ResolveScheduledBeamPointing`）；`:186` kStby、`:206` kTas 特判亦在 |
| C1 调度器签名无航迹参数（`ScanScheduleResolver.h:57-73,92-93`） | ✅ 成立 |
| C2 `TrackStateSnapshot.h:30-61` 零协方差字段 | ✅ 成立（struct 内无 covariance/uncertainty 字段） |
| C2 `TrackStateSnapshotEmitter.cpp:138-173` 丢弃 P | ✅ 成立（`BuildTrackStateSnapshots` 只填运动/rcs/status，不读 `gaussian_state.covariance`） |
| C3 `ArController.cpp:282-318` 时序 | ✅ 成立（`:290` 深拷贝 → `:298-305` 分类只回填 output_frame → `:308` decision_frame 写回未含分类） |
| C4 codec 手写编解码（`ArReplayFlatbufferCodec.cpp:30-38,46-74`） | ✅ 成立；fbs `DecisionTrackStateSnapshot`（`schemas/replay/airborne_radar_replay.fbs:14-35`）逐字段 |
| C5 schema_version 6（`ArController.cpp:418,445`） | ✅ 成立；`ArControllerRuntimeState` 为内部类型（`src/airborne_radar/runtime/ArController.h:37-39`），schema 6→7 纯内部改动 |
| DI 先例 `ThreatAssessmentEvaluator.h`（构造注入 IFeatureRepository） | ✅ 成立 |
| `ArRuntimeConfigPatch::has_work_mode/has_scan_center_deg/has_policy` | ✅ 成立（`ArRuntimeConfigPatch.h:55,61,64`） |
| kStt 外部 patch 先例（`RuntimePatchMapper.cpp:110-117`、`integration_demo.cpp:120-122`） | ✅ 成立 |
| 需求文档 §11/§12 冻结契约 | ✅ 成立；测试锚点文件全部存在（`ar_session_config_builder_test`、`ar_runtime_patch_mapper_test`、`ar_signal_scan_schedule_test`、`ar_replay_codec_roundtrip_test`、`ar_core_controller_test`、`ar_public_api_convenience_test`） |

### 新增发现（前文档未覆盖，必须并入实现规划）

**V1（replay 比对语义）**：`ArReplaySession.cpp:151-153` 的"字节比对容忍两帧分叉"实为**整包重编码后字节精确相等**（`event.payload_bytes == actual_payload`）。含义：录制侧已含回填后分叉（decision_frame 无 recognition、output_frame 有），回放侧复现同一分叉即可通过；前提是 codec 对 recognition 字段**两帧都显式编解码**。这强化 C4：漏写 codec 会直接 divergence，且 roundtrip 测试因读默认值误判通过。

**V2（D1 注入通道，阶段 4 前置问题）**：当前架构下**没有任何"每周期向 pipeline 设 scan_center"的控制器内通道**：
- runtime patch 全部经 session 级 `ArSession::TryApplyRuntimeConfig`（`ArSession.cpp:833-855`）写入 `pending_runtime_state`，由 `CommitPendingRuntimeConfig`（`:215-268`，仅 ArSession 调用）在周期边界提交给具体 `SignalPipeline::UpdateExecutionConfig`；
- `ISignalPipeline`（`src/airborne_radar/signal/pipeline/ISignalPipeline.h:39-118`）无 scan_center setter；
- `ArController` 只持有 `signal::ISignalPipeline&`。
- kStt 先例是**外部调用方**逐周期 `TryApplyRuntimeConfig`（demo 即此），不是 controller 内部路径。

→ 阶段 4 的"controller 内新增内部航迹选择器，经 `has_scan_center_deg` patch 设 scan_center"需要先确定注入机制（见 §5 开放项 O1）。**阶段 1-3 不受影响**。

**V3（prior-cycle 航迹来源）**：controller 无法触达 pipeline 内部的 `auto_lifecycle_manager`。controller 自己的 `impl_->cycle_state.latest_output.tracks`（含分类）与 `impl_->latest_decision_observation.input_frame.tracks`（含识别前快照）即是 prior-cycle 航迹来源，含 `status`/`association_key`/位置。阶段 4 选择器取数应从这里取，而非 pipeline 内部对象。

**V4（边界文档佐证 D3）**：`docs/airborne_radar/boundaries.md:104`「TrackOutputFrame 不扩展的决策依据」已确立"轨道级字段放 `TrackStateSnapshot`、帧级上下文字段放 `ArCycleResult`"的分层。D3（recognition 只回填 `TrackStateSnapshot`、摘要进 `ArCycleResult`）与该边界一致，实现时引用此先例。

**V5（枚举穷尽性预扫）**：全仓 `ArWorkMode` 引用点已预扫（见 §4.2），switch/分支落点仅 `ScanScheduleResolver.cpp` 三处；codec 仅 `static_cast` 往返（`:655,:1595`），无需分支处理。

---

## 1. Stage A 证据矩阵

按证据优先模式，冻结项为"进入 Stage B 前必须证明的需求"，不是实现想法。

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|---|---|---|---|---|---|---|
| E1. `kLrr=4` 枚举穷尽性 | 加枚举值后 `ScanScheduleResolver` 存在静默光栅 fallback，kLrr 行为未定义 | `ScanScheduleResolver.cpp:68-79,186-229,238-252`；全仓 grep（见 §4.2 预扫表） | 编译期枚举审计 + 新增 `ar_signal_scan_schedule_test` 用例断言 kLrr 三函数行为 | kLrr 在 `ResolveScanStepScale`/`ResolveScheduledBeamPointing`/`ResolveScheduledDwellCenter` 均有显式 case，测试断言 passthrough/step_scale/零偏；无其它未处理 switch | kLrr 语义本应是光栅（与需求冲突，需求 §3.1 为驻留指向）→ reject 需求方修改 | **pass** |
| E2. 导出 P 的 position 分块迹 | emitter 可无损访问 `gaussian_state.covariance`（6×6）并填标量 | `TrackStateSnapshotEmitter.cpp:138-173`；`TrackState` 内部 gaussian_state 先例（`BuildAssociationSeeds` `:175-189` 已直接搬 gaussian_state） | 阶段 2 单测：确认航迹 `estimation_uncertainty_trace > 0` 且与 P 左上 3×3 迹一致 | emitter 在快照路径能取到非零 P 迹；数值与手算一致 | P 恒零或 emitter 不可达 → narrow（仅运动质量用 hit_count 代理） | **pass**（字段先于阶段 2 落 DTO） |
| E3. 识别回填只进 `TrackOutputFrame::tracks`（D3） | 威胁分类先例可复制；replay 字节比对容忍分叉（V1） | `ArController.cpp:290-305`；`ArReplaySession.cpp:151-153` 字节精确比对；boundaries.md:104 分层先例 | 阶段 4 集成测试：kLrr 周期后 decision_frame 无 recognition、output_frame 有；replay 往返一致 | 两帧分叉与威胁分类模式一致且回放通过 | replay 比对要求两帧一致 → 必须同步回填 decision_frame（改需求 §11.4） | **pass** |
| E4. replay codec 显式编解码新字段 | 手写 codec 不加字段时静默丢数据、roundtrip 误判通过 | `ArReplayFlatbufferCodec.cpp:30-38,46-74`；fbs `DecisionTrackStateSnapshot` | roundtrip 测试填非默认 recognition/uncertainty 值断言往返相等 | encode→decode 逐字段相等（含嵌套 null 语义） | —（C4 已定，无 reject 情形） | **pass** |
| E5. `ArControllerRuntimeState` schema 6→7 | 进程内 capture/restore 无热迁移负担，同版本内一致 | `ArController.h:37-39`；`ArController.cpp:418,445` | restore 单测（schema 7 往返、错 owner/schema 拒绝） | 同版本 roundtrip 通过；拒绝路径不变 | —（C5 已澄清，风险低） | **pass** |
| E6. F1/F2 物理保真度定性 | 识别双通道极化与距离像相干叠加需在 boundaries 登记为识别专用例外 | `src/airborne_radar/environment/`（RfScene 极化仅干扰链）；全模块效能级（无复数 IQ） | 阶段 2 门禁前完成 boundaries.md 登记（文档动作） | boundaries.md 出现"识别专用更高保真观测/准信号级例外"表述并有证据引用 | —（文档登记，无代码风险） | **pass**（登记为阶段 2 门禁） |
| E7. D1 注入通道机制（阶段 4） | controller 有每周期 scan_center 注入通道 | 反证已成立（V2）：`ISignalPipeline` 无 setter、patch 路径 session-owned | 阶段 4 门禁前最小探针：对比三条候选通道（见 §5 O1）的实现面 | 选定通道实现面 ≤ 计划承诺（不改调度器签名/不扩 public API） | 三条候选都需扩内部接口 → 回 Stage A 重新定义 Path A 边界 | **defer**（阶段 4 门禁前定） |

## 2. 判定汇总

- **pass（进入 Stage B 边界）**：E1-E5（E2 的 DTO 字段属阶段 1，填充属阶段 2）；E6 为文档动作。
- **defer**：E7 —— 缺失证据是"注入通道的最小实现面"，下一探针在阶段 4 门禁前做，用最小改动原则对比三条候选。
- **无 reject**。三处真冲突经前文档深研已降级，本矩阵未发现新硬阻塞。

---

## 3. 冻结契约（阶段 1：DTO 与配置）

实现会话阶段 1 的允许/禁止范围。超出即停止并回到 Stage A。

### Proven requirement

- `kLrr=4` 成为合法工作模式且调度器对 kLrr 显式定义行为（驻留指向语义，D1 Path A）；
- 识别 DTO 与配置按需求 `(1)(1)` §11.1-11.5 冻结契约落地（含 §2.5 措辞修正：协方差用 P 迹、回填仅 output_frame）；
- replay schema/codec 对新增字段显式编解码（C4），进程内快照 schema 6→7（C5）；
- 识别未激活时所有新增字段保持默认值，基础探测/关联/跟踪路径零行为变化。

### Allowed scope（阶段 1）

- **Modules/directories**：`include/1q/airborne_radar/config/`、`include/1q/airborne_radar/session/`、`src/airborne_radar/signal/pipeline/`（仅 `ScanScheduleResolver.{h,cpp}`）、`src/airborne_radar/session/`（仅 `ArReplayFlatbufferCodec.{h,cpp}`）、`src/airborne_radar/runtime/`（仅 `ArController.{h,cpp}` 的 schema 版本）、`schemas/replay/airborne_radar_replay.fbs`、`src/airborne_radar/signal/tracking/`（仅 `TrackStateSnapshotEmitter` 的 DTO 引用，填充留阶段 2）。
- **Classes/functions**：`ArWorkMode`（kLrr）、`ArRecognitionConfig`/`ArRecognitionFeatureWeights`（新文件）、`ArPolicyConfig`（第七子域）、`AspectRcsSample`/`PolarizationRcsSample`/`RangeRcsScatterer`（新）、`ArSceneTarget`（三 vector 字段）、`ArRecognitionState`/`ArRecognitionCategory`/`ArRecognitionFeatureDimension`/`ArRecognitionFeatureScores`/`ArRecognitionResult`/`ArRecognitionCycleSummary`（新文件）、`TrackStateSnapshot`（recognition + estimation_uncertainty_trace）、`ArCycleResult`（has_recognition_summary + recognition_summary）、`ScanScheduleResolver` 三函数 kLrr 显式 case、codec `EncodeTrackStateSnapshot`/`DecodeTrackStateSnapshot`、`ArControllerRuntimeState` schema 7、`ArSessionConfigValidation`（识别配置规则）。
- **Tests/docs**：`ar_session_config_builder_test.cpp`、`ar_runtime_patch_mapper_test.cpp`、`ar_signal_scan_schedule_test.cpp`、`ar_replay_codec_roundtrip_test.cpp`（扩展）；需求文档 §2.5 措辞修正（两版）；`docs/review/` 本规划文件随实现更新。

### Explicitly out of scope（阶段 1）

- 识别运行时：观测构造、四类提取器、`RecognitionTrackState`、匹配器、JSON 数据库（阶段 2/3）；
- controller 识别回填与 kLrr 航迹选择器、scan_center 注入通道（阶段 4，含 E7）；
- `IFeatureRepository`/`ThreatAssessmentEvaluator`/`FeatureRepository` 任何改动；
- `ISignalPipeline`/`ArSession`/`RuntimePatchMapper` 签名改动；
- boundaries.md 的 F1/F2 登记（阶段 2 门禁前完成，不在阶段 1）；
- 任何 public API 之外的抽象、兼容层、跨模块泛化。

### Behavior boundary

- **Inputs**：kLrr 可经 `has_work_mode` 切换（机制现成，不改 patch 结构）；识别 DTO 默认值即"未激活"。
- **Outputs**：`TrackStateSnapshot::recognition.state == kDisabled` 默认；`ArCycleResult::has_recognition_summary == false` 默认；`estimation_uncertainty_trace == 0.0f` 默认（阶段 1 不填充）。
- **Errors/fallback**：kLrr 在 `ResolveScanStepScale` 返回 1.0、`ResolveScheduledBeamPointing` passthrough scan_center、`ResolveScheduledDwellCenter` 返回零偏——不得落入默认光栅路径；codec 对缺失字段回退默认值（fbs 语义），显式编解码新字段。
- **Lifecycle/debug/trace**：识别字段随快照参与两帧序列化；`ArSessionReplayStateV3` 加 `active_database_version`（默认空串）；`ArControllerRuntimeState` schema 6→7 仅内部，capture/restore 同版本校验。

### Acceptance gates（阶段 1）

- Build：`llvm-ninja-debug-local` 与 `llvm-ninja-release-local` 均构建通过（release 优先）。
- Focused tests：`ar_signal_scan_schedule_test`（kLrr 三函数行为）、`ar_session_config_builder_test`（识别配置构造/校验拒绝）、`ar_runtime_patch_mapper_test`（has_policy 含 recognition 整域提交 + 非法 patch 保持状态）、`ar_replay_codec_roundtrip_test`（识别字段非默认值往返）。
- Contract tests：`ar_public_api_convenience_test`、`check_public_api_boundary.cmake`、`check_cross_domain_naming.cmake` 不回归。
- 全量 ctest：本 preset 全部通过（`-j 4`）。

### Non-goals

- 阶段 1 不建立任何识别计算路径（无 `src/airborne_radar/recognition/` 目录）；
- 不改变 kTws/kTas/kStt 现有调度、探测、关联行为（回归测试锁定）；
- 不新增第五快照域（识别状态归 `ArControllerRuntimeState`，schema 7 承载）；
- 不在阶段 1 承诺 kLrr 目标选择行为（阶段 4）。

---

## 4. 阶段 1 实现级任务分解（file-by-file）

### 4.1 DTO 与配置

| # | 文件 | 动作 | 依据 |
|---|---|---|---|
| 1.1 | `include/1q/airborne_radar/config/ArOrientationConfig.h:65-70` | 枚举末端加 `kLrr = 4`（注释：远程识别，驻留指向） | 需求 §11.1 |
| 1.2 | `include/1q/airborne_radar/config/ArRecognitionConfig.h`（新） | `ArRecognitionFeatureWeights`（默认 0.25×4）+ `ArRecognitionConfig`（§11.2 全字段默认值） | 需求 §11.2 |
| 1.3 | `include/1q/airborne_radar/config/ArPolicyConfig.h:148-155` | 第七子域 `recognition`（末端） | 需求 §11.2 |
| 1.4 | `include/1q/airborne_radar/session/ArSceneTypes.h:23-48` | `AspectRcsSample`/`PolarizationRcsSample`/`RangeRcsScatterer` + 三 vector 字段（默认空） | 需求 §11.3 |
| 1.5 | `include/1q/airborne_radar/session/ArRecognitionResult.h`（新） | 状态/大类/维度枚举 + `ArRecognitionFeatureScores` + `ArRecognitionResult` + `ArRecognitionCycleSummary` | 需求 §11.4-11.5 |
| 1.6 | `include/1q/airborne_radar/session/TrackStateSnapshot.h:30-61` | 末端加 `recognition` + `estimation_uncertainty_trace{0.0f}`（D2，注释指向 §2.2 决策） | D2/C2 |
| 1.7 | `include/1q/airborne_radar/session/ArCycleResult.h:42-68` | 末端加 `has_recognition_summary{false}` + `recognition_summary` | 需求 §11.5 |
| 1.8 | 配置校验（`ArSessionConfigValidation` 所在实现） | 需求 §11.2 验证规则 11 条：权重域/和、enabled⇒路径非空、窗口≥dt、各项边界；error 消息含被拒字段名 | 需求 §11.2 |

### 4.2 枚举穷尽性审计（预扫完成，落点三处）

全仓 `ArWorkMode` 引用 21 文件，**src 内分支落点仅 `ScanScheduleResolver.cpp`**；codec `:655,:1595` 为 `static_cast` 往返（无 switch，无需改）；`check_cross_domain_naming.cmake` 与 `ar_primary_naming_contract_test` 只引用具体值（kStt/kTas），无枚举穷尽断言，但**实现后必须重跑**。

| # | 位置 | 现状 | 阶段 1 动作 |
|---|---|---|---|
| 2.1 | `ScanScheduleResolver.cpp:68-79` `ResolveScanStepScale` | kStt 走 default=1.0 | kLrr 显式 case 返回 1.0（与 kStt 并列，去 default 依赖） |
| 2.2 | `ScanScheduleResolver.cpp:194-196` `ResolveScheduledBeamPointing` | kStt passthrough | kStt/kLrr 并列 passthrough（D1：kLrr 指向=scan_center） |
| 2.3 | `ScanScheduleResolver.cpp:238-252` `ResolveScheduledDwellCenter` | kStt 返回零偏 | kStt/kLrr 并列零偏（纯驻留语义） |
| 2.4 | `:186`（kStby）、`:206`（kTas） | 不受影响 | 不改；注释确认 kLrr 不落入 `:198-229` 光栅路径 |
| 2.5 | 头注释同步 | `ScanScheduleResolver.h:55,80` Doxygen | 补 kLrr 语义（驻留指向/零偏） |

**测试（2.6）**：`tests/unit/airborne_radar/ar_signal_scan_schedule_test.cpp` 新增 kLrr 三用例：beam pointing == 限幅后 scan_center、step_scale == 1.0、dwell center == 零偏；并断言 kStt 既有行为不回归。

### 4.3 replay schema 与 codec（C4/C5 落地）

| # | 文件 | 动作 | 依据 |
|---|---|---|---|
| 3.1 | `schemas/replay/airborne_radar_replay.fbs:14-35` | `DecisionTrackStateSnapshot` 末端加 `estimation_uncertainty_trace:float`；新增嵌套表 `ArRecognitionResultV1`（§11.4 全字段）+ 引用字段 | C4 |
| 3.2 | `schemas/replay/airborne_radar_replay.fbs:297-308` | `ArSessionReplayStateV3` 末端加 `active_database_version:string` | 需求 §11.9 |
| 3.3 | `src/airborne_radar/session/ArReplayFlatbufferCodec.cpp:30-38` | encode 显式写两字段（嵌套表 null → 不创建/默认） | C4 |
| 3.4 | `ArReplayFlatbufferCodec.cpp:46-74` | decode 显式读两字段（null → 默认 `kDisabled`/空） | C4 |
| 3.5 | `src/airborne_radar/runtime/ArController.h:39` + `ArController.cpp:418,445` | `schema_version` 6→7（字段注释同步） | C5 |
| 3.6 | fbs 生成 | 经 `cmake/project/codegen/FlatBuffers.cmake` 现有流程重生成，generated 头随构建更新 | — |

**测试（3.7）**：`tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp` 扩展：填非默认 `recognition`（state=kModelConfirmed、score 非零）与 `estimation_uncertainty_trace` 往返断言；空默认值往返断言（防 C4 误判通过）；`active_database_version` 非空串往返。

### 4.4 文档同步（随阶段 1 提交）

- 需求文档两版（`remote_recognition_design(1)(1).md` 及简版）：§2.5 修正清单 4 条（§3.1 调度、§3.2 执行点、§8 协方差、§11.4 回填）落字。
- 阶段 1 完成后更新本规划文档 Stage C 记录。

### 4.5 阶段 1 完成定义

1. 详版 §12 阶段 1 七条 + 冲突计划文档阶段 1 三条（kLrr 无未处理分支；uncertainty 默认 0；codec 显式处理）；
2. `ar_signal_scan_schedule_test` 锁定 kLrr 调度行为；
3. schema 6→7 与 codec 扩展随同一次提交闭环（不得拆散造成中间态不兼容）。

---

## 5. 阶段 2-5 就绪度与开放项

### 开放项 O1（阶段 4 门禁前必须定）：D1 Path A 注入通道（E7 defer）

三候选（按最小实现面排序）：

| 候选 | 机制 | 实现面 | 评注 |
|---|---|---|---|
| **O1a（推荐）** | `ISignalPipeline` 新增窄内部虚方法（如 `SetCycleScanCenterDeg(az,el)`，仅 kLrr 消费，周期内生效、随 capture/restore 回滚） | 接口 +1 虚方法、SignalPipeline +1 实现、controller 调用 | 内部接口，非 public；与"不改调度器签名"相容（调度器仍 passthrough） |
| O1b | ArSession 侧接线：session 在 `RunExecutionCycle` 中询问 controller 的 kLrr 指向并构造 `ArRuntimeConfigPatch` 走既有 pending 通道 | 改 ArSession 循环 + controller 暴露内部查询 | 复用 patch 机制，但每周期 pending→commit 往返较重，且与"controller 内选择器"表述略有偏移 |
| O1c | 外部调用方逐周期 `TryApplyRuntimeConfig`（kStt 先例原样复制） | 无库内改动，示例/调用方负责 | 与计划"controller 内新增内部航迹选择器"不符，降级为外部驱动 |

**门禁探针**（阶段 4 启动时做，5 分钟内）：对比 O1a/O1b 触达的私有字段（`runtime_.config.base_config` vs `pending_runtime_state`），以改动行数最少且不经 public API 者为定。

### 阶段 2 门禁（F1/F2 登记先行）

- boundaries.md 登记两条（F1 双通道极化、F2 距离像相干叠加），位置建议 `## 非目标` 或新增"识别子模型边界"小节；引用本规划文档与需求 §5.3/§5.4 作证据。
- `TrackStateSnapshotEmitter::BuildTrackStateSnapshots` 填 `estimation_uncertainty_trace`（E2 探针即其单测）。

### 阶段 3/5

- 无重大审查修正（前文档已确认）；阶段 5 的模式切换从零积累依赖阶段 4 的 Path A 行为，验收项 9 保留。

### 风险矩阵增量（相对前文档）

| 风险 | 阶段 | 缓解 |
|---|---|---|
| O1 注入通道未定导致阶段 4 范围漂移 | 4 | 门禁探针先行；O1a 为回退基准 |
| V1 字节精确比对放大 codec 遗漏代价 | 1 | roundtrip 测试填非默认值断言（3.7）；codec 与 schema 同次提交 |
| kLrr 显式 case 与 kStt 并列后未来 Path B 迁移 | 1→4 | 并列书写；Path B 仅需拆开并加调度器参数，注释留痕 |

---

## 6. 状态与下一个动作

- 前文档三处真冲突降级结论复核通过；新增发现 V1-V4 已并入规划，其中 **V2 产生开放项 O1（阶段 4 门禁）**，不阻塞阶段 1。
- 实现会话启动方式：`feature/ar-recognition-stage1` 分支，按 §4 任务分解执行；**阶段 1 必须一次性闭环 DTO+枚举审计+replay schema/codec+schema 7**（§4.5-3），随后过 `ar_public_api_convenience_test` 等契约测试门禁。
- 文档同步动作随实现批次走：§2.5 措辞修正、boundaries.md F1/F2（阶段 2 门禁）、本规划文档 Stage C 回写。


---

## 7. Stage C 回写（五阶段实现完成，2026-08-04）

### 实现范围

| 阶段 | 提交 | 内容 |
|---|---|---|
| 1 DTO 与配置 | `b8f3fa64` | kLrr 枚举 + 识别 DTO/配置/校验 + ScanScheduleResolver kLrr case + replay schema/codec + schema 7 + RuntimeConfigState 保留识别配置 + session 配置 codec 识别子域 |
| 2 观测与特征 | `ba707268` | 四提取器 + ObservationBuilder + emitter P 迹 + boundaries F1/F2 登记 |
| 3 数据库与匹配 | `cfbdc663` | 库内 JSON 解析器（深度/EOF/转义守卫）+ 原子加载校验 + 动态加权匹配 |
| 4 链路集成 | `0aa2c0e6` | RecognitionTracker 状态机 + O1a 周期扫描中心覆盖 + 回填/摘要/回滚 + replay 溯源 |
| 5 效能验证 | `a674ddde` | SNR/带宽/驻留门控 + 七类场景 + 混合 + 模式切换 + replay 字节往返 |

### 审查修正（最终完整性审查后）

1. **BLOCKER 修复**：session 配置 codec `co_site_paths` 向量在 table builder 打开期间创建
   （flatbuffers NotNested 约束；release 下静默损坏）——向量创建前置。
2. **`\u` 转义 off-by-one**：`\uXXXX` 后紧跟字符被吞——索引修正 + 回归测试。
3. **回滚边界补全**：`ArControllerRuntimeState` 快照补 work_mode/recognition_config/
   recognition_database_path；Restore 时路径不一致释放数据库、下次提交重载。
4. **首次确认时间语义**：新增 `first_conclusion_time_sec`（仅首次确认记录），
   `mean_first_confirmation_sec` 改用它。
5. **数据库 profile 适用条件强制**：`minimum_aspect_coverage_deg`/`minimum_bandwidth_hz`
   在匹配器 Applicable 中生效（原为解析后未消费）。
6. **摘要真值准确率落地**：`has_ground_truth`/`category_accuracy`/`model_accuracy`
   由 target_name 命中数据库 model_id 时统计（仅统计，不参与识别）。
7. **峰间距合并**：`kMinimumPeakSeparationM=0.5m`（原 0 导致同距散射中心重复计数）。
8. **识别禁用语义**：`enabled=false` 时输出帧识别字段复位 kDisabled（原残留旧结论）。

### 验收结果

- 构建：`llvm-ninja-release-local` 通过；全量 ctest 通过（含 contract/replay/integration）。
- 详版 §12 五阶段验收：阶段 1 七条 + 阶段 2 九条 + 阶段 3 十条 + 阶段 4 十条 + 阶段 5 十条
  全部以测试落地（锚点：ar_session_config_builder/ar_signal_scan_schedule/
  ar_replay_codec_roundtrip/ar_recognition_feature/ar_recognition_database/
  ar_recognition_integration/ar_recognition_scenario）。
- 遗留 NIT（有意保留，已记录）：kLrr 指向不钳制扫描限位（与 kStt 先例一致）；
  覆盖标志不入 pipeline 快照（下次周期必被重设/清除，无泄漏路径）；
  外部输出适配/调试视图不承载识别字段（子集设计，非回归）。
