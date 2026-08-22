---
Status: draft
Date: 2026-08-21
Review-Baseline: `feature/docs-two-channel-output-model` @ `6776523e`（docs freeze；实现未开）
Authority: RIR 三类观测投影（DebugView / LifecycleRecorder / ExclusionCauseRecorder）
  Stage A 证据矩阵与冻结契约。非规范性记录；字段冻结供 Stage B 实施。
  若与 `docs/common/session_contract.md` 冲突，以 session_contract 为准；
  若与库实现冲突，以库为准（实现未齐前以本文 + session_contract 为应建口径）。
Stage B 落地（2026-08-22）：三类投影 + Attach 契约 + 规则 13b 排除码子集已实现
  （§5 残留表记录子集口径与延后项）；本文转为历史依据，后续以库与 session_contract 为准。
---

# RIR 观测投影：Stage A 证据矩阵与冻结契约

## 0. 结论速览

- **用户裁定（2026-08-21）**：RIR 需要三类观测投影——`DebugView`、生命周期 recorder、
  排除原因差分 recorder；与 AR/EOS/SBIRS 目标列表型观测完备对齐。
- **与 common 冻结联动**：`session_contract.md` 已将输出模型正文改为「两通道 + 可选投影」；
  目标列表型模块三类投影为观测完备必选项；RIR 实现未齐前规则 10/11/13b/13e 仍空洞。
- **本文件冻结 Stage B 边界**；**不写代码**。AR 产品航迹上的真值 ID 搬迁不在本文范围。
- Stage A 判定：F1–F6 **pass**（需求与先例充分）；F7 **narrow**（排除诊断 code 清单以
  「本周期已存在的门控语义」为上限，禁止借投影扩探测物理）。

## 1. 现状证据（2026-08-21）

| 事实 | 内容 | 证据 |
|---|---|---|
| 产品双通道已有 | `RirOutputFrame`（识别结论 + 特征量测）+ `RirCycleResult`（含 `track_attributions`） | `RirOutputTypes.h`；`RirCycleResult.h` |
| 信封已有归属对照 | `RirTrackAttributionRecord`：association_key ↔ 真值 + 最小航迹诊断；不进产品帧 | `RirOutputTypes.h`；dual_product Stage B |
| 库内无三类投影 | 无 `RirOutputDebugView*` / `Rir*LifecycleRecorder` / `RirExclusionCauseRecorder` | public header 检索；session_contract 旧「暂无 L3」 |
| 示例自行拼视图 | `rir_sensor_component` 读 `track_attributions` 写 `CA_LOG_VIEW`，无标准状态枚举 | `examples/component_attachment/components/rir_sensor_component.cpp` |
| 无 13b 逐目标排除 issue | `RirIssueCodes` 仅 validation/config；无 `rir.target_*` 门控排除码；无 `RirIssueCause` | `RirIssueCodes.h` |
| AR 航迹级先例 | DebugView 按输入目标；Lifecycle：`kFirstConfirmed`/`kUpdated`/`kLost`/`kNotTracked`；Exclusion 差分 `(code,cause)` | `ArTrackOutputDebugView.h`；`ArTrackLifecycleRecorder.h`；`ArExclusionCauseRecorder.h` |
| EOS 检测级先例 | DebugView 含未检测目标；Lifecycle 检测事件；Exclusion 依赖 13b location | `EosOutputDebugView.h`；`EosDetectionLifecycleRecorder.h` |

## 2. Stage A 证据矩阵

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|---|---|---|---|---|---|---|
| F1 需求真实性 | 集成需要标准化「每输入目标一行」与跨周期事件，现有归属表不够 | 用户裁定 + 示例自拼视图 | 对照 EOS/AR 组件落盘 | 用户明确三类都要 | 用户改口为仅信封对照 | pass |
| F2 产品形态对齐 | RIR 是目标列表型，应与 AR/EOS/SBIRS 同为投影必选 | session_contract 产品形态表 | 契约条文 | 列入目标列表型必选 | 改判为 ESR 类豁免 | pass |
| F3 粒度选航迹级 | 生命周期/视图应对齐 AR 航迹语义，而非 EOS 检测语义 | RIR 产品=航迹识别；`track_attributions` | 对照 AR/EOS 事件枚举 | 事件键为 external_target_id + association_key | 强制检测级归属 | pass |
| F4 信封对照复用 | DebugView/Lifecycle **消费**既有 `track_attributions`，不另造第二套真值表 | dual_product 修订 2 | 字段核对 | 投影只读信封归属 + 输入表 | 在产品帧再嵌真值 | pass |
| F5 Attach 零行为 | Session `Attach*` 与规则 10/11 同构，不改变 Step 语义 | session_contract 规则 10/11 | 对照 EOS Session | 裸指针、自动 Update、非执行空事件 | 强制调用方手动 Update | pass |
| F6 不进产品通道 | 投影 DTO 不得混入 `RirOutputFrame` | 去真值化 + 两通道纪律 | 头文件边界 | public 产品帧无 debug/lifecycle 字段 | 产品帧加状态枚举 | pass |
| F7 排除码上限 | 排除差分依赖 13b；RIR 今日无逐目标排除 issue，须**先补门控诊断**再挂 recorder | `RirIssueCodes.h` 空缺 | 列出现有门控语义 | 只为已有门控语义新增 `rir.*` info code + cause | 借机新增物理门 | narrow |

## 3. 冻结契约（Stage B 实施边界）

### 3.1 公共挂载（Session）

- `RirSession::AttachTrackLifecycleRecorder(RirTrackLifecycleRecorder*)`
- `RirSession::AttachExclusionCauseRecorder(RirExclusionCauseRecorder*)`
- 语义对齐规则 10/11：非拥有裸指针、`nullptr` 解绑、Step 内自动 `Update`、
  注册与否不影响 `Step`/`StepWithResult` 返回值。
- DebugView **无 Attach**：无状态 `RirOutputDebugViewBuilder::Build(input, result)`，
  与 EOS/AR 同形。

### 3.2 DebugView（`RirOutputDebugView`）

按**输入场景目标表顺序**逐行合成（含本周期无航迹的目标）。

| 字段/概念 | 冻结规格 |
|---|---|
| 周期头 | `input_cycle_index`；`executed_this_cycle`（`status == kCompleted`）；`abort_reason`；`issues` 转写 |
| 指定任务镜像 | 自信封：`designated_target_id` / `designation_active` / `designation_reverted_to_scan` / `designation_revert_reason` / `dwell_center_deg` |
| 逐目标键 | `external_target_id` + `target_name`（自输入；有归属时与 `track_attributions` 一致） |
| 状态枚举 `RirDebugTargetStatus` | `kConfirmed` / `kTentative` / `kLost` / `kNotInOutput` / `kCycleNotCompleted`（对齐 AR 航迹调试态；识别结论有无**不**另开状态位，见诊断字段） |
| 航迹诊断 | `has_track`；`association_key`；`hit_count`；滤波 ENU 位置/速度（有航迹时自归属或快照；无航迹时用输入几何回填斜距/方位——具体回填量 Stage B 与输入字段对齐） |
| 识别诊断 | `has_recognition_output`；类别/型号/置信度（有出口②时填充，否则默认）；**不得**把识别结论当生命周期状态 |

**非目标**：DebugView 不替代信封 `track_attributions`；不进 replay 为强制字段（若 Stage B
加性进 replay，须另开 schema 冻结；默认可仅 public API + 单测）。

### 3.3 生命周期（`RirTrackLifecycleRecorder`）

对齐 `ArTrackLifecycleRecorder` 航迹语义：

| 事件 kind | 语义 |
|---|---|
| `kFirstConfirmed` | 目标对应航迹首次进入 confirmed |
| `kUpdated` | 已确认航迹本周期仍确认 |
| `kLost` | 航迹进入 lost |
| `kNotTracked` | 输入目标无对应航迹（默认关闭，配置 `emit_not_tracked_events`） |
| `kDesignationDropped` | 指定识别任务作废/回扫终态（镜像信封 `designation_revert_reason`） |

- 事件携带：`cycle_index`、`external_target_id`、`target_name`、`association_key`、
  航迹状态、可选 `designation_revert_reason`。
- 非 `kCompleted` 周期：空事件、不推进状态（规则 10 / contract 失败语义 e）。

### 3.4 排除诊断 + 排除差分（规则 13b/13e）

**前置**：引入 `RirIssueCause`（至少 `kNone` + 聚合门主因枚举，形态对齐 AR/EOS），
并在 `RirIssue` 上具备与五模块同构的 `cause` + `location`（`kSceneEntity`）。

**允许新增的 info 级排除 code（上限，Stage B 可子集落地，不得超集发明物理）**：

| code（草案字面） | 含义（须对应现有执行路径） |
|---|---|
| `rir.target_beyond_recognition_range` | 超识别最大作用距离 |
| `rir.target_mode_not_identify` | 本周期非识别工作模式，不建识别观测 |
| `rir.target_no_feature_database` | 无特征库 / HoldCycle，特征链空 |
| `rir.target_outside_dwell` | 不在本周期驻留波束内（扫描/指定几何） |
| `rir.target_detection_gate` | 检测门未过（若有逐目标可定位路径） |
| `rir.target_observation_invalid` | 观测全维无效，不产出口① |

`RirExclusionCauseRecorder`：差分键 `(code, cause)`；事件 kind `kEntered` /
`kChanged` / `kExited`；原料仅 `result.issues` 中带 scene-entity location 的排除条目；
纯观测、零行为改变。

**非目标**：不为「尚无执行路径」的假设门写 code；不把校验拒绝（整周期 abort）当成逐目标排除。

### 3.5 replay / 测试 / 示例

| 项 | 冻结 |
|---|---|
| replay | 默认：三类投影**不强制**进 RIR2 schema；若加性扩展须 Stage B 末单独验收 roundtrip |
| 单测 | Builder 状态表；Lifecycle 转换；Exclusion A2/A3/A4；非执行周期空事件 |
| 示例 | `rir_sensor_component` 改为优先 `LastDebugView()` + recorder 事件写 `CA_LOG_*`（可保留归属摘要过渡） |
| public API | 新头列入 RIR session 白名单 + consumer 可链接冒烟 |

### 3.6 明确非目标

1. 不搬迁 AR 航迹产品上的 `external_target_id`（独立工作）。
2. 不改变出口①/②字段与去真值化纪律。
3. 不把投影状态机塞进 `RirController` 探测主链（只读组装）。
4. 不做逐检测级归属（产品仍是航迹级）。

## 4. Stage B 验收门

1. `unit::remote_identification_radar` 覆盖 DebugView / Lifecycle / Exclusion 新用例全绿。
2. Session Attach 契约单测：解绑、非执行空事件、注册不影响 status。
3. 产品帧守卫：`RirOutputFrame` 无真值名、无 debug 状态枚举泄漏。
4. 规则 13b：至少落地 §3.4 code 子集 + `cause`/`location`；Exclusion recorder 可差分。
5. 示例组件可经标准投影写视图行（不再是唯一依赖手写归属拼接）。
6. `/completeness-review` 或同等聚焦集：public_api / install_manifest 含新头。

## 5. 残留与后续

| 项 | 状态 |
|---|---|
| AR 产品航迹真值 ID 收回 | 用户另行处理；本文不阻塞 |
| RIR 排除 code 全集 vs 子集 | Stage B 已落 4/6 子集：detection_gate（含 AR 同构主因分类）、beyond_recognition_range、mode_not_identify、no_feature_database；`outside_dwell`（无逐目标执行分支）与 `observation_invalid`（逐航迹键、非场景实体键）延后，落地需再冻结 |
| 投影进 replay | 投影本体未进 schema；归属记录 `track_status` 以加性字段进 `RirTrackAttributionRecordV2`（roundtrip 已验） |
| session_contract 空洞条款关闭 | 已回写（2026-08-22）：目标列表型表/13b 对齐状态/13e 适用范围均含 RIR |

## 6. 与「两通道 + 投影」的关系

```text
产品通道   RirOutputFrame          （双产品，去真值化）
信封通道   RirCycleResult          （含 track_attributions）
投影       DebugView / Lifecycle / Exclusion   ← 本文冻结；消费信封+输入，不回流产品
```
