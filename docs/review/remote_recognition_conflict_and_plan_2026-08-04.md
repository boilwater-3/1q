# 远程识别需求冲突分析与实现计划

Status: draft
Date: 2026-08-04
Subject: 针对 `remote_recognition_design(1).md`（简版）/`remote_recognition_design(1)(1).md`（详版）远程识别需求的冲突审查、深研决策与修正实现计划。
Codebase: HEAD `4678ec51`（branch `main`）

> 本文档整合三阶段工作成果：冲突分级（§1）→ 代码级深研与工程决策（§2）→ 五阶段实现计划（§3）。
> 所有结论附 `file:line` 证据。代码暂不动；进入实现会话时以此为据。

---

## 1. 冲突审查

### 1.1 审查范围与方法

逐条将需求文档对代码库的**静态引用**（字段、枚举、结构）与**动态集成点**（调度、回填、导出、回放）分开核查。判定分级：
- 🔴 真冲突：现有设计承载不了，必须破坏既有契约或新建机制。
- 🟡 半冲突：可扩展，但有必须同步改的副作用，遗漏会静默失败。
- ✅ 无冲突：现有机制直接接纳。

### 1.2 总结

| # | 冲突点 | 审查判定 | 深研后（见 §2） |
|---|---|---|---|
| C1 | `kLrr` 扫描调度无落点——`ScanScheduleResolver` 对航迹零感知 | 🔴 真冲突 | 🟠 复杂但可行（D1 Path A） |
| C2 | `innovation_covariance` 不在快照导出路径 | 🔴 真冲突 | 🟡 需导出，但有更优代理（D2） |
| C3 | 识别结果回填时序窗口比需求描述窄 | 🔴 真冲突 | 🟢 实际非冲突（D3） |
| C4 | replay 编解码手写，加字段不改 codec 静默丢数据 | 🟡 半冲突 | 🟡 保持（必改 codec） |
| C5 | `schema_version` 硬相等，风险被需求夸大 | 🟡 半冲突 | 🟡 澄清（进程内，风险低） |
| F1 | 双通道极化——探测链单极化，识别双极化是平行未对账路径 | 物理保真度 | 需在 boundaries 定性 |
| F2 | 距离像相干叠加——效能级定位与信号级实现自相矛盾 | 物理保真度 | 需在 boundaries 定性 |

**核心结论：三处真冲突经深研均降级，无硬阻塞。** 现有设计能承载该需求。

### 1.3 真冲突详证

**C1. `kLrr` 扫描调度无落点**
`ScanScheduleResolver`（`src/airborne_radar/signal/pipeline/ScanScheduleResolver.cpp`）是唯一波束调度入口，设计契约为**模式驱动 + 静态 scan_center**：
```cpp
// :194  kStt 只返回静态配置字段，非航迹
if (orientation_config.work_mode == config::ArWorkMode::kStt) { return normalized_scan_center; }
// :76   default 分支对未处理枚举值静默返回 1.0
```
`ApplyScanScheduleToRuntimeConfig`（`ScanScheduleResolver.h:92`）与 `ResolveScheduledBeamPointing`（`:57-73`）签名里**没有航迹列表参数**。全仓 grep 无任何"指向航迹"机制（designate/illuminate/track_follow/beam_to_track 均零命中）；`ControlDirectiveType` 11 值无波束指向；`ArControlProfile` 无 az/el 字段。

**C2. `innovation_covariance` 不在导出路径**
- `TrackStateSnapshot`（`TrackStateSnapshot.h:30-61`）**零协方差字段**。
- 内部 `TrackState::gaussian_state`（`TrackState.h:57`）持 6×6 协方差 P，但 `TrackStateSnapshotEmitter::BuildTrackStateSnapshots`（`:138-173`）**丢弃 P**。
- `innovation_covariance` 是单周期瞬态（`KalmanUpdater.h:69`），IMM 的 `CombineEstimates`（`ImmFilter.h:366-383`）**只组合 mean/covariance，无组合 innovation covariance**；AR **从不调用** `GetModelUpdateResults()`（唯一调用方是 SBIRS `SbirsTrackingCoordinator.cpp:129`）。

**C3. 回填时序**
`ArController::RunOnce`（`src/airborne_radar/runtime/ArController.cpp`）：
```cpp
:282  decision_frame = signal_result.decision_frame;          // 拷贝1
:290  track_output_frame.tracks = decision_frame.tracks;      // 拷贝2 → 此后两份独立
:293  Evaluate(decision_frame, ...)                           // ThreatAssessment 在此内部
:298-305  威胁分类回填 → 仅改 track_output_frame.tracks
:308  signal_result.decision_frame = decision_frame;          // 写回（未被分类改）
:314  latest_decision_observation.input_frame = decision_frame; // 观测帧（无分类结果）
:318  cycle_state.latest_output = track_output_frame;         // 输出帧（含分类）
```
`:290` 是深拷贝，之后两 vector 独立。威胁分类先例只改 output_frame。

### 1.4 半冲突与物理保真度

**C4. replay codec 手写**：replay 用 FlatBuffers（`schemas/replay/airborne_radar_replay.fbs`），table 字段标签制，**末端加字段线路兼容**。但 `ArReplayFlatbufferCodec.cpp` 的 `EncodeTrackStateSnapshot`（`:30-38`）、`DecodeTrackStateSnapshot`（`:46-74`）、`EncodeSessionReplayStateV3`（`:1323`）、`TryDecodeSessionReplayStateV3`（`:1333`）**逐字段手写**。不加 encode/decode：新字段序列化静默丢弃、反序列化零初始化，roundtrip 测试因读到默认值而**误判通过**。

**C5. schema_version 硬相等**：`ArController::RestoreRuntimeState`（`ArController.cpp:445`）`if (... || state.schema_version != 6U) return false;`，无迁移。但该 schema **只用于进程内 capture/restore**（发射准备/回滚），**不在 replay 文件里**。同版本代码内 capture 写 N、restore 校 N 正常；本库无热重载，实际风险低。真正要处理的是 C4 的 replay 侧 schema。

**F1. 双通道极化**：探测链严格单极化——`ArSceneTarget::rcs` 单标量 m²；`signal/detection/` 下 grep "polariz" 零命中；`RfScenePolarization` 仅用于干扰链极化失配损耗（`RfScene.cpp:120-138`）。需求 §5.3 的通道级 SNR 物理路径探测侧不存在。需在 boundaries.md 登记为"识别专用更高保真观测，不与探测链逐项对账"。

**F2. 距离像相干叠加**：全模块效能级（`SignalDetector` Swerling+MarcumQ，`SignalDetector.cpp:54-66`；`RfScene.h` 明确"不生成复数 IQ"）。需求 §5.4 第 3 步 `sqrt(rcs_m2)·exp(j·phase)` 相干叠加是全模块唯一信号级计算。需在 boundaries.md 定性为"识别专用准信号级子模型"，或降级为纯统计模板。

### 1.5 无冲突项（现有机制直接接纳）

| 需求项 | 证据 |
|---|---|
| `ArWorkMode` 加 `kLrr=4` | `ArOrientationConfig.h:65-70`；除 ScanScheduleResolver 外仅 codec `static_cast` |
| `ArPolicyConfig` 加 `recognition` 第七子域 | `ArPolicyConfig.h:148-155`；`has_policy` 整域覆盖现成 |
| `ArSceneTarget` 加三 vector 字段 | `ArSceneTypes.h:23-48`，默认空向量 ABI 兼容 |
| `TrackStateSnapshot` 加 `recognition` | `TrackStateSnapshot.h:30-61`；speed/acc/vel/pos_z/rcs m²/association_key u64 齐全 |
| `ArCycleResult` 加 `recognition_summary` | `ArCycleResult.h:42-68`；`applied_decision_*` provenance 先例现成 |
| `has_work_mode` 切换 kLrr | `ArRuntimeConfigPatch.h:61-62` |
| `IFeatureRepository` 旁挂识别库 | DI 注入现成（`ThreatAssessmentEvaluator.h:31,99`）；⚠️ `ComputeDistance` private，识别库自实现匹配 |
| `ArSessionReplayState` 加 `active_database_version` | FlatBuffers 末端加字段（配合 C4 改 codec） |
| 四域快照回滚 | 归入 `ArControllerRuntimeState`（schema 6→7），不新增第五域 |
| `SignalCycleAbortReason` 六值 | `ArOutputTypes.h:45-52`，与需求完全一致 |
| `bandwidth_hz` 默认 4.5 MHz | `ArHardwareConfig.h:118`（值对，需求行号 171 漂移） |
| `ArInputValidation` 三级严重度 | `validation_types.h:19-23`；kWarning 不阻断、kError 阻断周期 |
| `BuildTrackMapByAssociationKey` | `TrackOutputQueries.cpp:40`（自由函数，非成员——需求措辞偏差） |

### 1.6 字段引用准确度

需求对代码库的静态引用准确度高：`ArWorkMode` 四值、`ArPolicyConfig` 六子域、`TrackStateSnapshot` 各字段、`FeatureRepository` 三键 0.45/0.35/0.20 + 三类默认记录、`SignalCycleAbortReason` 六值——**全部属实**。仅 `bandwidth_hz` 行号漂移（171 实为 118）、`BuildTrackMapByAssociationKey` 措辞偏差（成员 vs 自由函数）。

---

## 2. 深研与工程决策

### 2.1 C1 深研：扫描调度——prior-cycle 航迹可达，无 chicken-and-egg

**时序**（`SignalPipeline::Impl::RunCycle`，`SignalPipeline.cpp:118-172`）：
```
:136 SampleEnvironment        无航迹
:148 ResolveRuntimePipelineConfig (LPI/ECCM)
:151 ApplyScanScheduleToRuntimeConfig  ← 调度在此；本周期待确认航迹未形成
:156 ExecuteCycle             探测/关联/滤波/生命周期 → 本周期航迹才形成
```
但在 `:151` 时点，`runtime_.owned.auto_lifecycle_manager` 是**跨周期持久对象**，持有 prior-cycle 确认航迹（`BuildTrackStateSnapshots()` 可取，含 `position_x/y/z`/`status`/`association_key`）。**kLrr 用 prior-cycle 航迹驱动波束指向，时序可行。**

**kStt 先例（决定性）**：kStt 并非调度器级航迹感知，而是外部 controller 经 RuntimePatch 设 scan_center、调度器 passthrough：
```cpp
// RuntimePatchMapper.cpp:110-117  外部 patch（kStt 真实驱动方式）
if (patch.has_scan_center_deg) { next_execution_config...scan_center_deg = patch.scan_center_deg; }
// ArRuntimeConfigBuilder.h:70  WithScanCenterDeg(...) 现成 builder
// integration_demo.cpp:120-122  已示范外部按周期 patch scan_center 跟踪目标
```
**LPI/ECCM**（`ControlProfileEffects.cpp:55-113`）只改功率/PRF/频率/波束宽度/旁瓣/增益，**从不写 scan_center_deg**，不强制扫描图样约束。kLrr 指向航迹不与 LPI/ECCM 冲突；唯一交互是 kLrr 长驻留与 LPI `dwell_scale` 叠加——设计取舍，非冲突。

**scan_center 下游消费链**（确认指向生效）：
```
ScanScheduleResolver.cpp:260 写 scan_center
  → DetectionExecution.cpp:214-218 → ArOrientationUtils.h:177 → 天线增益 → SNR → Pd
```

### 2.2 C2 深研：P 是更优代理，association 已用它

association 路径**不取 ImmFilter 的 innovation_covariance，而从 P 重算 HPHᵀ+R**：
```cpp
// DataAssociation.cpp:563-567
ComputeProjectedMeasurementCovariance(predicted): return H * predicted.covariance * H.transpose();
// Hypothesiser.cpp:112-158  运行时 S = projected_cov + measurement_cov 重建
```
即 **P（预测协方差）是模块内已确立的"航迹估计不确定性"本源信号**，innovation_covariance 是派生量。用 P 的 position 分块迹作运动质量代理，比瞬态的 innovation_covariance 更本源、更稳定、与 association 口径一致。

### 2.3 C3 深研：照威胁分类先例即可

- `ThreatAssessmentEvaluator::Evaluate`（`ThreatAssessmentEvaluator.cpp:29-81`）只读 `speed`/`rcs`/`status`（`:115-138`），FeatureRepository 查询只传 `speed`/`rcs`（`:88-89`）。LpiEvaluator 读 `LpiSourceInfo`，EccmEvaluator 读干扰列表——**决策三件套均无识别字段读取机制**。
- 回放字节比对（`ArReplaySession.cpp:151-153`）已容忍两帧分叉（威胁分类先例：target_type 在 output_frame 有、decision_frame 无）。
- **识别在当前架构下是纯并行输出**。照威胁分类先例在 `:290` 之后回填 track_output_frame 即可。

### 2.4 工程决策（已确认）

**D1. C1 kLrr 波束指向 → Path A（复用 scan_center patch）**
- controller 从 prior-cycle 确认航迹选优先目标，经现有 `ArRuntimeConfigPatch::has_scan_center_deg` 设波束指向；`ScanScheduleResolver` 对 kLrr 像 kStt 一样 passthrough。
- `ArWorkMode` 加 `kLrr=4`；`ResolveScheduledBeamPointing`/`ResolveScanStepScale`/`ResolveScheduledDwellCenter` 新增 kLrr 显式 case（passthrough + step_scale + zero dwell），避免命中 default 静默光栅扫描。
- **不改**：调度器签名、`ExecutionConfig`、`ArControlProfile`、探测链、LPI/ECCM、`ArRuntimeConfigPatch`（机制现成）。
- **边界标注**：Path A 下 kLrr 是**纯驻留模式**，无搜索+识别交织；若未来需并发搜索，升级 Path B（调度器签名未改，无迁移负担）。

**D2. C2 运动质量信号 → 导出 P 的 position 分块迹（标量）**
- `TrackStateSnapshot` 新增标量字段（建议 `estimation_uncertainty_trace`）。
- `TrackStateSnapshotEmitter::BuildTrackStateSnapshots`（`:138-173`）从 `track.gaussian_state.covariance`（6×6）取左上 3×3 求迹填入。**无需改 TrackLifecycleManager**——P 已在 gaussian_state。
- 同步 replay codec encode/decode + fbs schema。
- 需求 §8 表格"innovation_covariance（内部字段，识别只读）"修正为"预测协方差 P 的 position 分块迹"。

**D3. C3 回填时序 → 照威胁分类先例**
- 识别结果回填 `TrackOutputFrame::tracks`（`ArController.cpp:290` 之后）；**不进** `decision_frame`/`DecisionObservation`。
- replay 用同一 TrackStateSnapshot schema 序列化两帧，识别字段在两帧均序列化；录制 ground-truth 反映回填后分叉，回放字节比对通过。
- 需求 §11.4"同时进入两帧"修正为"仅 TrackOutputFrame::tracks"。**理由**：当前架构下 ThreatAssessment 只读 speed/rcs/status，识别是纯并行输出；若未来需识别影响 ThreatAssessment，再改 Evaluate 签名并在 `:285→:293` 回填 decision_frame。

### 2.5 需求文档措辞修正清单（实现前必须同步）

两版需求文档均需按 D1/D2/D3 修正：

| 需求章节 | 原表述 | 修正为 |
|---|---|---|
| §3.1 kLrr 扫描 | "对重点航迹分配识别驻留""复用现有波束调度" | controller 从 prior-cycle 航迹选优先目标、经 `has_scan_center_deg` patch 设指向；调度器对 kLrr passthrough（D1） |
| §3.2 执行点 | "DecisionInputFrame 后、战术决策前""可作威胁评估附加输入" | 照威胁分类先例回填 track_output_frame；当前为纯并行输出，不进 decision_frame/ThreatAssessment（D3） |
| §8 协方差 | "innovation_covariance（内部字段，识别只读）" | 预测协方差 P 的 position 分块迹；快照新增标量字段（D2） |
| §11.4 回填（详版） | "同时进入两帧" | 仅 TrackOutputFrame::tracks（D3） |

---

## 3. 修正实现计划

以 `(1)` §10 五阶段为骨架（与 `(1)(1)` 详版结构一致），逐阶段注入审查与决策修正。字段/契约以 `(1)(1)` §11/§12 冻结契约为准，但按 §2.5 修正措辞。

### 阶段 1：DTO 与配置

| 项 | 动作 | 依据 |
|---|---|---|
| `ArWorkMode` 加 `kLrr=4` | 枚举末端加值 | ✅ |
| **枚举穷尽性审计** | grep 全仓 `ArWorkMode` switch；`ScanScheduleResolver.cpp:68-79,194,241` 为 kLrr 加显式 case，避免 default 静默光栅 | C1 |
| `ArPolicyConfig` 加 `recognition` 第七子域 | 末端加字段 | ✅ |
| `ArSceneTarget` 加三 vector 字段 | 默认空向量 | ✅ |
| `TrackStateSnapshot` 加 `recognition` | 末端加字段 | ✅ |
| **`TrackStateSnapshot` 加 `estimation_uncertainty_trace`** | 末端加 float，阶段 2 由 emitter 填充 | **C2/D2** |
| `ArCycleResult` 加 `recognition_summary` | 末端加字段 | ✅ |
| replay schema bump | `ArSessionReplayStateV3` 加 `active_database_version`；`DecisionTrackStateSnapshot` 加 recognition + uncertainty；`ArControllerRuntimeState` schema 6→7 | C4/C5 |

**完成定义**（除详版 §12 阶段 1 七条外）：kLrr 无未处理分支；uncertainty 字段默认 0；codec 显式处理新字段（不静默丢）。

### 阶段 2：观测与特征提取

| 项 | 动作 | 依据 |
|---|---|---|
| 四类 FeatureExtractor + RecognitionObservationBuilder | 按 §5，严守 dBsm vs m² | ✅ |
| **emitter 导出 P 迹** | `TrackStateSnapshotEmitter:138-173` 从 gaussian_state.covariance 取左上 3×3 迹填入 | **C2/D2** |
| 运动质量 q(motion) 用 uncertainty_trace 归一化 | | D2 |
| **F1 极化定性** | boundaries.md 登记为"识别专用更高保真观测，不与探测链对账" | F1 |
| **F2 距离像定性** | boundaries.md 登记：相干叠加为准信号级例外，或降级统计模板 | F2 |

**完成定义**：uncertainty_trace 由 emitter 正确填充（单测断言非零）；F1/F2 已在 boundaries.md 定性。

### 阶段 3：数据库与匹配

| 项 | 动作 | 依据 |
|---|---|---|
| RecognitionFeatureDatabase JSON 加载 + 全量原子替换 | 按 §7 | ✅ |
| **匹配逻辑自实现** | `FeatureRepository::ComputeDistance` private；识别库自实现加权匹配（首期 YAGNI） | ✅ |
| 动态质量加权 `score = Σw·q·s / Σw·q` | 按 §6；q(motion) 用阶段 2 uncertainty_trace | D2 |

阶段 3 无重大审查修正。

### 阶段 4：链路集成（C1/C3 落地核心）

| 项 | 动作 | 依据 |
|---|---|---|
| **kLrr 波束指向（Path A）** | controller 内新增内部航迹选择器（威胁/距离/确认状态/上周期不确定度排序），从 prior-cycle `auto_lifecycle_manager->BuildTrackStateSnapshots()` 取确认航迹，算视角，经 `has_scan_center_deg` patch 设 scan_center；调度器 passthrough | **C1/D1** |
| **识别回填（照先例）** | `ArController.cpp:290` 之后回填 `track_output_frame.tracks[i].recognition`；不进 decision_frame | **C3/D3** |
| 四域快照回滚 | 归入 `ArControllerRuntimeState`（schema 6→7），不新增第五域 | ✅ |
| abort 边界 | 按 §9；`kSensorPoweredOff` 保持结论、`kRuntimePreparationFailed` 回滚、`kValidationRejected` 不推进积累 | ✅ |
| trace/replay | database_version 进 ArSessionReplayState；recognition 经 codec 序列化两帧；回放字节比对 | C4 |
| 运行期 patch | `has_policy` 整域覆盖识别配置；`has_work_mode` 切 kLrr；可同 patch | ✅ |

**完成定义**：kLrr 下波束指向与选择器输出一致且不绕过 LPI/ECCM（断言测试）；kLrr 不改基础探测链 SNR/Pd/协方差（回归测试，对比 kTws 基线）；识别结果在 output_frame 可见、decision_frame 不可见（同威胁分类模式）；回放往返一致。

**边界标注**：Path A 下 kLrr 纯驻留，无搜索+识别交织。

### 阶段 5：效能验证

| 项 | 动作 | 依据 |
|---|---|---|
| 七类标注场景 + 混合 + 模式切换序列 | 按详版 §12 阶段 5 十条 | ✅ |
| **模式切换从零积累** | TWS→LRR→TWS→LRR：第二次进 LRR 从零积累（Path A 下 controller 随 kLrr 退出停止积累） | D1 |
| trace/replay 往返一致 | 全场景覆盖 | C4 |

阶段 5 无重大审查修正。

### 风险矩阵

| 风险 | 阶段 | 缓解 |
|---|---|---|
| C4 replay codec 静默丢字段 | 1 | 显式改 encode/decode；roundtrip 测试断言非默认值 |
| F1 极化保真度未定性 | 2 | boundaries.md 先登记再写极化提取器 |
| F2 距离像相干叠加与效能级矛盾 | 2 | boundaries.md 定性为准信号级例外，或降级统计模板 |
| C1 controller 航迹选择器是新逻辑 | 4 | 单测选择策略；集成测试验证波束指向 |
| 双版需求文档措辞分叉 | 全程 | 以 §2.5 修正清单统一 |

---

## 4. 状态

- 冲突审查 + 深研 + 五阶段计划修正完成，三处真冲突均降级，无硬阻塞。
- 两个工程选型已确认：D1 Path A、D2 P 迹导出。
- 代码暂不动。进入实现会话时，以阶段 1（DTO + 枚举穷尽性审计 + replay schema）启动，并同步落实 §2.5 措辞修正到需求文档。
