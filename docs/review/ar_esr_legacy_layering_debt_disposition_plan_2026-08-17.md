---
Status: draft
Date: 2026-08-17
Review-Baseline: `main` @ `b9da39d9`（docs: finalize p0-p5 delivery write-back and module registry）
Authority: AR/ESR 历史越层债务处置立项——本次处置范围锁定 TARGET-OQ-1（airborne_radar）
  与 TARGET-OQ-2（electronic_surveillance_radar）两项存量偏离的偿还，含 fusion AR 通道、
  ECM 调度输入、threat_assessment 接线的联动改造。
  本文是本处置的唯一状态与阶段计划文档；进度、范围变更、阶段验收只在此登记。
  规定性规则见 docs/common/contract.md §目标处理分层契约（规则 2/3/4/7）与 §证据优先开发模式；
  存量偏离登记见 docs/common/open_questions.md（TARGET-OQ-1/2）；
  上游交付见 target_domain_development_plan_2026-08-17.md（估计层/推演层已落地，
  满足本处置的再进入条件）。
  本文为非规范草案；若与库实现冲突，以库为准。
---

# AR/ESR 历史越层债务处置计划（TARGET-OQ-1/2，分阶段）

## 0. 结论速览

- **处置范围**：偿还 TARGET-OQ-1（AR 估计/推演职责前置：航迹产品外发 + 识别结论回填）与
  TARGET-OQ-2（ESR 威胁等级进 public 输出）。上一交付（SBIRS + RIR 范围）已明确"不偿还
  AR/ESR 债务"并登记排除（target_domain_development_plan_2026-08-17.md §1.2 第 1 条）；
  本计划即该债务的处置立项，与上一交付边界互补、不重叠。
- **再进入条件核查（均满足）**：
  - OQ-1 条件一"估计层轨迹滤波（fusion 演进）立项时"：P2 已落地（`51c87d70` 边界冻结 +
    `0b1a1d6a` 逐航迹无迹滤波与航迹管理）。
  - OQ-1 当前边界的等待条件"直至推演层识别面立项"：target_inference 已建立（`9d402196`，
    含类型概率融合；`InferenceTrackState.type_evidence` 为外部类型证据预留输入位）。
  - OQ-2 条件"ESR 公共 API 修订立项时"：本文立项即构成该立项。
- **终态裁定方向（Stage A 判定，正式裁定随 D1 迁移契约签认冻结）**：
  1. **AR 走航迹+量测双输出改造**：航迹帧保留为 AR 核心公开输出（机载雷达的主职是探测与
     跟踪，航迹是传感器本位产品）；新增公开量测/检测输出通道作为补充（fusion 可消费量测级
     数据做特征融合）；识别结论（`target_type`/`target_probability`）退出航迹公开面，类型
     证据改由推演层识别面（target_inference）供给链承载。
  2. **ESR `threat_level` 退出 public DTO**，威胁启发式整体退出传感器；ECM 调度威胁分改由
     决策层（threat_assessment）供给。
- **阶段总览**：

| 阶段 | 名称 | 性质 | 关键前置 |
|---|---|---|---|
| D0 | 立项与 Stage A 证据矩阵 | Stage A，零生产代码 | 无（本文档） |
| D1 | 迁移契约冻结 | 契约，零生产代码 | D0 判定方向签认 |
| D2 | ESR 轨道实施 | Stage B | D1 的 ESR 契约 + threat_assessment 子裁定 |
| D3 | AR 轨道实施 | Stage B | D1 的 AR 契约（与 D2 无依赖，可并行） |
| D4 | 回写关闭 | Stage C | D2/D3 |

## 1. 处置范围与边界

### 1.1 In scope

| 面 | 内容 |
|---|---|
| AR 公开输出 | 航迹帧保留为 AR 核心公开输出（航迹是机载雷达本位产品，下游火控/武器分配依赖）；新增量测/检测输出通道作为补充（public DTO + 接入 `ArCycleResult`）；识别结论字段（`target_type`/`target_probability`）退出航迹公开面——识别不是 AR 本职，改由推演层供给 |
| AR 内部 | 关联（LAPJV）/滤波（KF/IMM）/生命周期为传感器本位信号处理链，航迹是其对外产品；滤波原语单源合规登记；内部威胁分维持"仅驱动 LPI/ECCM"既有边界 |
| fusion | AR 适配器继续消费航迹帧（核心路径不变）；新增量测消费适配器（`AdaptArDetectionsToDetectionRecords`）供 fusion 特征融合路径使用；航迹帧中识别字段退出后 `DetectionRecord.quality` 来源调整 |
| ESR | `threat_level` 退出 `EmitterHypothesis` public DTO；`InferThreatFromCluster` 启发式整体退出；ESM 假设管理（关联/生命周期/平滑）与模式推断确认为合法 ESM 产品形态（仅文档冻结，无代码语义变更） |
| ECM | 调度威胁分（`threat_score`）改由 threat_assessment 供给；接线设计（含决策层输入面子裁定）在 D1 冻结 |
| replay | `airborne_radar_replay.fbs` / `esr_replay.fbs` 的兼容性裁定与 codec 处置 |
| 治理 | TARGET-OQ-1/2 关闭迁出；contract.md 两处过时表述真化（见 D4） |

### 1.2 Out of scope（本次处置明确不做）

1. TARGET-OQ-3（SBIRS Estimated 后验外发）：P0 已裁"维持装备语义"，随指标签认冻结，
   与本处置无关。
2. TARGET-OQ-4（RIR 识别产品形态）：已由 `rir_dual_product_stage_a_2026-08-18.md` 修订为
   双产品形态（识别结论 + 特征量测帧）；方案 a 降级为兼容选项。本处置不涉及 RIR 模块。
3. threat_assessment 评分算法/权重/输出语义改动：本处置只涉及其**输入面接线**（ESM 特征
   入口子裁定），评分模型本身不动。
4. ESM 假设管理与模式推断的代码语义变更：FI-E3 为确认性裁定（合法 ESM 产品形态），只
   冻结文档。
5. kLrr 识别子系统：已按 2026-08-15 耦合审计全量迁出（`1ac346ca`），与 OQ-1 登记的保留
   决策层启发式无关。
6. SAR（不接入 fusion 的既定边界）、EOS 通道、SBIRS/RIR 适配器。

### 1.3 处置内必须完成的裁定（D1 门）

| 裁定 | 阻塞 | 内容 |
|---|---|---|
| AR 量测输出契约 | D3 | 量测记录字段集（单位后缀、量测质量、协方差、键位语义）、坐标系对齐、`ArCycleResult` 接入方式、内部检测链字段充分性核验 |
| AR 识别字段退出路径 | D3 | `target_type`/`target_probability` 从航迹公开面退出的时序与消费方迁移（推演层 target_inference 已有 `type_evidence` 预留位） |
| fusion AR 量测适配器 | D3 | 新增 `AdaptArDetectionsToDetectionRecords`（key=0 走空间/方位门 vs 调用方键承载）、质量归一化口径 |
| threat_assessment ESM 输入面 | D2 | `ThreatEvaluationInput`（运动学特征）与 ESM 辐射源特征（RF/PRI/脉宽）失配：输入面扩展（决策层公共 API 变更，独立冻结）vs 调用方特征映射 |
| ESR replay 兼容裁定 | D2 | `threat_level` 字段弃用语义（保留可读、写默认值）vs schema 版本升级 |
| AR replay 版本化裁定 | D3 | 量测记录新增 vs 既有航迹记录的共存；识别字段（`target_type`/`target_probability`）从航迹记录退出 |

## 2. Stage A 证据矩阵

判定依据 docs/common/contract.md §证据优先开发模式与 evidence-first-freeze-contract 流程。
矩阵结论为**方向判定**；每项的正式冻结随 D1 迁移契约签认。

### 2.1 AR 轨道（TARGET-OQ-1）

| 冻结项 | 假设 | 证据来源 | 探查/测试 | 通过判据 | 拒绝判据（回退路径） | 判定 |
|---|---|---|---|---|---|---|
| FI-A1 识别结论出口迁移（`target_type`/`target_probability` 退出公开面） | 识别结论可退出 AR public 输出且不丢失必需能力——推演层识别面已能承载供给 | contract.md:165-167 规则 2 明文禁止；TrackStateSnapshot.h:57-60 字段定义；ArController.cpp:307-315 回填点；InferenceTrackState.h:46-47 `type_evidence` 预留位；SensorAdapters.cpp:21-23/47-49 fusion 已有状态基准回退（`ArBaseQualityForStatus`）；threat_component.cpp:86-87 威胁示例显式传 NaN 不消费 | 全库消费点盘点（已完成）：库内唯一跨模块消费 = fusion SensorAdapters；其余为 ArExternalOutputAdapter.cpp:91、ArTrackOutputDebugViewBuilder.cpp:60、replay codec/fbs、roundtrip 测试 | 消费链闭合可迁移：fusion 回退存在 + 推演层输入位存在 + 威胁链不依赖 | 若发现库外/关键消费方硬依赖该字段且无替代输入位 → 降级为"字段标注弃用 + 供给迁移过渡期" | pass |
| FI-A2 AR 新增量测输出通道 | AR 可在保留航迹帧的同时新增公开量测通道，内部 `RawTrackMeasurement` 字段足以支撑公开量测 DTO | 内部检测链已存在量测形态雏形（SignalDetector.h:18-23 `DetectionResult` → TrackMeasurementProcessing.cpp:49-60 `RawTrackMeasurement`：位置 + 3×3 量测协方差 R + detection margin）；EOS/SBIRS 适配范式（SensorAdapters.cpp:82-131）可复制；航迹帧保留为 fusion 核心消费路径不变，量测通道为可选补充 | 内部 `RawTrackMeasurement` 字段对公开量测 DTO 的充分性核验 | 内部量测链字段足以支撑公开量测 DTO（单位/协方差/质量），量测通道与航迹帧可并行不悖 | 若内部量测字段不足以支撑公开 DTO → 降级为仅内部使用，不新增公开通道 | pass（方向：航迹+量测双输出） |
| FI-A3 AR 双输出共存 | 航迹帧（含 `association_key`）与量测帧（key=0 无身份）可共存于同一周期输出，下游按需消费 | FusionEngine.cpp:277 身份键直挂现状（AR `association_key` 直挂）；docs/fusion/boundaries.md:13-21 关联键边界（键 0 = 无身份走空间/方位/特征门）；航迹帧继续走键直挂路径，量测帧走空间/方位门路径——两条路径独立、不互相干扰 | D1 前补双输出场景下 fusion 消费路径 characterization（航迹键直挂 + 量测门并行） | 两条消费路径可并行运行，关联质量不互相退化 | 若双输出导致下游混淆（航迹键 vs 量测键冲突）→ 契约明确消费路径选择规则 | pass |
| FI-A4 滤波原语单源化核验 | AR 滤波原语已单源合规，无需代码变更 | src/airborne_radar/signal/tracking/ 六个头（IKalmanPredictor/IKalmanUpdater/KalmanPredictor/KalmanUpdater/ImmFilter/GaussianTrackState）均为 common/estimation 模板的 documented 重导出外观（头注释自证"向后兼容外观"）；common/estimation 目录含全部原语（含 P1 新增 Unscented*）；AR 与 fusion 消费不同原语（KF/IMM vs UKF）但同源单源，合规 | grep 证实 src/airborne_radar 无第二处滤波原语实现（已核验） | 合规结论登记即可 | 若 D1 发现外观头被库外以 public 依赖消费 → 外观弃用计划进迁移契约 | pass（已合规，零代码；D3 可选外观弃用登记） |

### 2.2 ESR 轨道（TARGET-OQ-2）

| 冻结项 | 假设 | 证据来源 | 探查/测试 | 通过判据 | 拒绝判据（回退路径） | 判定 |
|---|---|---|---|---|---|---|
| FI-E1 `threat_level` 退出 public DTO | 决策层产品可退出传感器公开面，机器消费链唯一且可迁移 | contract.md:165-167 规则 2；EmitterHypothesis.h:31-37/47 字段；HypothesisAssociator.cpp:58-67（模式+SNR 启发式）、:288/:337（内部写入）、:394（导出公开假设）；机器消费唯一链 = EcmEsrAdapter.cpp:10-20/72-73（threat_level×confidence 二次计分）→ EcmSession.cpp:137（调度排序）；fusion 适配器不读该字段（SensorAdapters.cpp:55-80）；测试锁定：ecm_session_test.cpp:316-336、ecm_session_consumer.cpp:13-28；replay：esr_replay.fbs:128 + EsrReplayFlatbufferCodec.cpp:339/:413/:420 | 消费链闭合性盘点（已完成，见上行证据） | ECM 改供方案成立（FI-E2），消费链可整体迁移 | 若 replay 字节级兼容为硬约束且不允许弃用默认值写入 → 走 schema 版本升级裁定（FI-E4 拒绝路径） | pass |
| FI-E2 ECM 威胁分改由 threat_assessment 供给 | 威胁评分归决策层后 ECM 调度输入可由决策层产品供给 | contract.md:157 决策层职责表（威胁评分与等级 → `ThreatResult`）；`SchedulingThreat` 对 score 来源无偏好（EcmTypes.h:110 `EcmSensorObservation::threat_score` 为普通注入字段）；ECM 已有平行注入通道先例（truth-driven `EcmTruthThreat`，EcmSession.cpp:144-154）证明 score 注入路径形态；**失配证据**：ThreatEvaluationInput.h:21-29 现为运动学特征（speed/rcs 类），ESM 辐射源特征（RF/PRI/脉宽/模式）无输入位 | D1 子裁定：输入面扩展（决策层公共 API 变更，独立冻结）vs 调用方特征映射 | D1 完成子裁定并冻结接线契约 | 若威胁评估语义上无法承载 ESM 特征（评分模型不适用）→ 回退 ECM 内部计分方案（镜像 AR"内部威胁分仅驱动资源管理"既有边界） | pass（方向：threat_assessment 供给；子裁定挂 D1） |
| FI-E3 ESM 假设管理 + 模式推断合法性 | ESM 假设管理（最小费用流指派 + 指数混合平滑）与模式推断（PRI/脉宽→工作模式）可确认为合法 ESM 产品形态 | open_questions.md OQ-2 自述"可辩护为 ESM 量测语义标注 / ESM 产品形态"；`mode` 为公开量测语义标注字段（EmitterHypothesis.h:22-28）；fusion 特征门消费 RF 四元特征、不消费威胁语义；RIR OQ-4 的"装备使命形态"先例 | 无（确认性裁定，证据为既有契约文本与消费面盘点） | 冻结进 ESR design.md（ESM 产品形态声明），不动代码 | 无（若未来推演层需要 ESM 识别语义，另行立项） | pass（确认保留） |
| FI-E4 replay 兼容性裁定方向 | replay 兼容可用"字段保留弃用语义 + codec 版本化"方向处置，旧回放文件保持可读 | 两个 schema 的争议字段均为追加式标量（airborne_radar_replay.fbs:32-33/35、esr_replay.fbs:128），删除字段不破坏旧文件读取；codec 已有字段白名单与合法性检查结构（EsrReplayFlatbufferCodec.cpp:40-42）；contract.md:144 "删除或重命名已公开工具仍属于 public API 变更，必须先冻结兼容迁移契约" | roundtrip 测试对弃用默认值的适配方案核验 | D1 冻结正式裁定（弃用默认值写入 vs 版本升级）并同步 roundtrip/consumer 测试口径 | 若 roundtrip 字节级比对约束不允许默认值写入 → schema 版本升级 | pass（方向裁定） |

## 3. 阶段定义

### D0：立项与 Stage A 证据矩阵（本次交付，已完成）

**目标**：把两项债务的触发条件、终态方向、证据矩阵与阶段计划登记成文，作为后续一切
处置的验收基线。

工作项：
1. 本文档（Stage A 证据矩阵 + 阶段定义 + 风险登记）。
2. open_questions.md 的 TARGET-OQ-1/2 条目追加证据与建议裁定段（镜像 OQ-3/4 的 P0 先例）；
   索引表状态保持 open——条目只在 D4 处置完成时迁出删除。

退出门：本文档评审通过 + 判定方向签认（正式裁定随 D1 迁移契约签认冻结）。
**零生产代码**（证据优先模式规则 2：Stage A 未判定不进实现）。

### D1：迁移契约冻结（零生产代码）

**目标**：按 OQ-1/2 再进入条件的要求，一次性冻结全部迁移契约（不得零碎单独修改）。

工作项：
1. **AR 航迹+量测双输出迁移契约**：
   - 航迹帧保留，识别字段退出：`target_type`/`target_probability` 从 `TrackStateSnapshot`
     退出（推演层 target_inference `type_evidence` 预留位已就绪）；航迹帧其余字段
     （运动学/协方差迹/生命周期/关联键）保持不变；
   - 新增量测输出 DTO 字段冻结：物理量单位后缀契约（contract.md §物理量单位命名）、规则 5
     去真值化（量测帧只允许调用方关联键，不带场景真值）、量测质量字段（SNR/detection margin
     类，规则 2 豁免范围）、量测协方差（规则 6 误差预算精神在量测侧的对应）、坐标系与
     既有 ECEF/ENU 契约对齐；
   - `ArCycleResult` 接入方式：航迹帧 + 量测帧并行接入；
   - fusion 新增量测适配器（`AdaptArDetectionsToDetectionRecords`）语义与 quality 来源
     调整；航迹适配器保持不变；
   - 消费方迁移清单：识别字段退出影响的消费方（ArController 回填点、DebugView、replay
     codec、tests）；新增量测通道的消费方（examples/tests/白名单按需新增）；
   - replay 版本化裁定与验收门（unit/replay/contract/consumer 四层）。
2. **ESR `threat_level` 迁移契约**：字段退出时序；`InferThreatFromCluster` 与内部
   TrackState 威胁字段整体退出（ESR 无内部威胁分消费闭环，区别于 AR 的 LPI/ECCM 边界）；
   ECM 接线设计（含 threat_assessment ESM 输入面子裁定）；replay 弃用语义裁定；消费方
   迁移清单（ecm_session_test、ecm_session_consumer、multi_model_scenario、esr replay、
   esr_sensor_component 事件字段）。
3. 判定正式冻结登记（open_questions 对应条目状态推进）。

退出门：两份契约 + 三个子裁定（§1.3）全部冻结并签认。

### D2：ESR 轨道实施（Stage B）

1. `EmitterHypothesis.threat_level` 退出（DTO 字段、启发式、内部 TrackState 字段、导出点）。
2. ECM 威胁分来源改造（按 D1 子裁定：接受 ThreatResult 注入或降为量测字段装配 + 调用方
   补 `threat_score`）。
3. replay codec 处置 + 测试更新（ecm_session_test、ecm_session_consumer、
   multi_model_scenario、esr replay roundtrip、esr_batch_validation 按需）。
4. ESM 产品形态声明冻结进 ESR design.md（FI-E3 落地）。

退出门：`unit::electronic_surveillance_radar` + `unit::electronic_countermeasure` + 相关
replay/contract 测试全绿。

### D3：AR 轨道实施（Stage B）

1. 识别字段退出（`target_type`/`target_probability` 从 TrackStateSnapshot、DebugView、
   ArController 回填点删除）；`ThreatAssessmentEvaluator::IdentifyTarget` 按契约处置
   （内部威胁分/资源管理边界维持，识别结论不再回填公开面）。
2. 公开量测输出通道落地（新增 `ArDetectionOutput` DTO + CycleExecutor 量测装配 +
   `ArCycleResult` 接入航迹帧+量测帧 + 契约测试）。
3. fusion 新增 AR 量测适配器（`AdaptArDetectionsToDetectionRecords`）+ quality 语义
   调整；航迹适配器保持不变。
4. replay 版本化 + codec + roundtrip 测试（航迹记录保留，量测记录新增，识别字段删除）。
5. 消费方迁移（识别字段退出影响的消费方 + 新增量测通道消费方）。
6. 双输出场景下 fusion 消费路径 characterization 证据归档。

退出门：`unit::airborne_radar` + `unit::fusion` + 相关 replay/contract/consumer 测试全绿。

### D4：回写关闭（Stage C）

1. TARGET-OQ-1/2 关闭：结论迁出 open_questions（条目删除），规则性结论回写
   contract.md（AR 航迹+量测双输出边界、ESM 产品形态条款按需）与受影响模块 design.md
   （airborne_radar、electronic_surveillance_radar、electronic_countermeasure、fusion、
   threat_assessment 按需）。
2. contract.md 两处过时表述真化：分层表推演层"尚未建立"（contract.md:156，
   target_inference 已建立）；规则 8 括号"当前 tests/contract/ 无该守护"（contract.md:184，
   守护已于 `3a798c29` 落地）。
3. 评估纯净度守护扩展可行性（公开 DTO 字段语义级守护——现有 include 方向守护不覆盖
   产品语义偏离；仅评估登记，不强制立项）。
4. 本计划文档标记完成态。

## 4. D1 冻结契约（2026-08-18 冻结；需求方指令确认全程执行）

**契约修订（2026-08-18）**：需求方裁定项目未上线、**不考虑兼容性**——下述 replay 条款按
"schema 字段直接删除/自由重构，旧 trace 文件一律作废"执行，废弃原 deprecated 槽位保留与
旧文件可读性设计。其余条款不变。

三个子裁定结论（§1.3）：

| 裁定 | 结论 |
|---|---|
| threat_assessment ESM 输入面 | **输入面扩展（泛型证据属性槽）**：`ThreatEvaluationInput` 新增 `emitter_threat_evidence`（float [0,1]，调用方组装），MADM 六属性扩为七属性；模式→证据映射表归调用方（镜像"距离由调用方计算"既有模式；决策层不得引用传感器类型，规则 1） |
| ESR replay | **字段直接删除**：`esr_replay.fbs` 删除 `threat_level` 字段；codec 不再编解码；旧 trace 作废（无兼容负担） |
| AR replay | **航迹表族保留 + 量测帧新增**：航迹表族（TrackOutputFrame/TrackStateSnapshot 包装）保留在 `airborne_radar_replay.fbs`；新增量测帧表（`ArDetectionOutputFrame`）；`ArCycleResultV3` 同时记录航迹帧和量测帧；识别字段（`target_type`/`target_probability`）从航迹记录删除；旧 V3 trace 一律作废（项目未上线，无兼容负担） |

### 4.1 契约 A：ESR threat_level 迁移契约

**已证实需求**（FI-E1/E2/E4 = pass）：规则 2 禁止威胁评分作为传感器 public 输出；机器消费链唯一（ECM）；威胁评分归决策层（contract.md 分层表）。

**迁移设计**：
1. `EmitterHypothesis` 删除 `EsrThreatLevel` 枚举与 `threat_level` 字段（公开 DTO 破坏性变更，本契约即冻结迁移契约）。
2. `HypothesisAssociator` 删除 `InferThreatFromCluster`、`TrackState::threat_level` 与全部写点/导出点；SNR 项随启发式整体退出（`mean_snr_db` 为簇内部统计，非公开量测，不得再由传感器侧供给）。
3. threat_assessment 新增第七属性 `emitter_threat_evidence`：输入（调用方组装，建议口径 = 模式基准 [制导/连续波照射 1.0、跟踪 0.66、搜索 0.33、未知 0] × 假设置信度，映射表归调用方）、权重 `weight_emitter_threat_evidence`（默认 0.0——默认配置下行为位恒等，ESM 场景显式启用）、`AttributeContribution` 增槽。既有六属性语义/断点/阈值不动。
4. ECM `TryBuildEcmSensorObservationFrame` 增第四参 `threat_scores`（按假设索引对齐；逐元素必须有限且 ∈ [0,1]，违规整帧拒绝 fail-closed）；删除 `ThreatLevelScore`。ECM 不得引用 threat_assessment 类型（规则 1 传感器层禁引决策层；分数以值级传入）。
5. replay：`esr_replay.fbs` 直接删除 `threat_level` 字段（无兼容负担）；codec 删 `add_threat_level`/`IsValidThreatLevel`/解码回填；`EmitterHypothesisEqual` 删该字段比对。
6. ESM 产品形态冻结（FI-E3 落地）：假设关联/生命周期/平滑与模式推断为合法 ESM 量测语义标注/产品形态，写入 `docs/electronic_surveillance_radar/design.md`。

**允许范围**：`include/1q/electronic_surveillance_radar/session/EmitterHypothesis.h`；`src/electronic_surveillance_radar/pipeline/HypothesisAssociator.{h,cpp}`、`session/EsrReplayFlatbufferCodec.cpp`、`session/EsrReplaySession.cpp`；`include/1q/threat_assessment/{ThreatEvaluationInput.h,ThreatEvaluatorConfig.h,ThreatResult.h}`、`src/threat_assessment/ThreatEvaluator.cpp`；`include/1q/electronic_countermeasure/EcmEsrAdapter.h`、`EcmTypes.h`（注释）、`src/electronic_countermeasure/EcmEsrAdapter.cpp`；`schemas/replay/esr_replay.fbs`（注释标记弃用）；测试 `ecm_session_test.cpp`、`ecm_session_consumer.cpp`、`multi_model_scenario_test.cpp`（3 处调用点）、`threat_evaluator_test.cpp`（第七属性用例）；示例 `events.h`、`esr_sensor_component.cpp`；文档 ESR algorithms/design。

**明确禁止**：不动 fusion ESR 适配器（不读 threat_level，无影响即无改动）；不动 ECM 调度排序/资源账本/欺骗状态机语义与 `EcmTruthThreat` 通道；不动 threat_assessment 既有六属性；不新增任何威胁语义字段进传感器 DTO。

**行为边界**：`threat_scores` 与 `hypotheses` 尺寸不一致 → false（不部分写回）；`emitter_threat_evidence` 缺省 0 且默认权重 0 → 既有调用输出位恒等。

**验收门**：`unit::electronic_surveillance_radar`、`unit::electronic_countermeasure`、`unit::threat_assessment`、ESR replay 测试、`integration::cross_domain`、`check_public_api_boundary`、docs 守护全绿。

**非目标**：ESR 公开 API 其他修订；威胁评分算法演进（权重寻优/新归一化）；ESM 假设管理重构。

### 4.2 契约 B：AR 航迹+量测双输出迁移契约

**已证实需求**（FI-A1/A2/A3 = pass）：AR 航迹是机载雷达本位产品（探测与跟踪），下游火控/武器分配依赖；内部 `RawTrackMeasurement`（位置 + 3×3 量测协方差 R + detection margin）已是量测形态雏形，可新增公开量测通道供 fusion 特征融合路径使用；识别结论（`target_type`/`target_probability`）不是 AR 本职，应退出航迹公开面由推演层供给。

**终态架构**：
1. **航迹帧保留**（核心输出）：`ArTrackOutput.h`（`TrackOutputFrame` + 查询函数）保留；`ArExternalOutputAdapter`/`ArCycleOutputAdapter`（ECEF 航迹帧）保留；`TrackStateSnapshot` 继续作为航迹公开产品。删除 `target_type`/`target_probability`（识别结论退出航迹公开面；输入方向恒为默认值，仅输出方向被回填——纯输出侧产物）；`estimation_uncertainty_trace` 保留。识别结论不再回填任何公开面；内部启发式识别继续驱动 LPI/ECCM（既有边界）。
2. **新公开量测帧**（补充输出）`ArDetectionOutput.h`：`ArDetectionRecord{position_x/y/z_m（雷达局部 ENU，与量测协方差同帧）、measurement_covariance 3×3（行主序 9 元素，雷达方程基于 SNR 推算的 R）、detection_margin_db}` + `ArDetectionOutputFrame{cycle_index, batch_id, detections}`。**不携带**场景真值标识/目标名（对齐 SBIRS boundaries 非目标 7 先例）、航迹/生命周期/识别语义、协方差以外的估计量。
3. **ArCycleResult 双输出接入**：`ArCycleResult` 同时携带航迹帧（`output_frame`，既有字段不变）和量测帧（新增 `detection_frame` 字段）；`ArSession::Step` 返回类型同步；`ONEQ_SENSOR_SESSION_CONTRACT` 锚定双输出；`ArTraceSession` 同步。
4. **fusion 双路径适配**：航迹适配器 `AdaptArTracksToDetectionRecords` 保持不变（核心消费路径，key=关联键直挂）；新增量测适配器 `AdaptArDetectionsToDetectionRecords(source_id, platform, frame)`——局部 ENU 位置 + 平台位姿 → LLA（复用既有坐标换算原语），key=0（无身份，走空间/方位门），quality 口径对齐 EOS/SBIRS 适配器。
5. **replay**：航迹表族保留（`TrackOutputFrame`/`TrackStateSnapshot` 包装表不变）；新增量测帧表（`ArDetectionOutputFrame`）；`ArCycleResultV3` 同时记录航迹帧和量测帧；识别字段（`target_type`/`target_probability`）从航迹记录中删除。
6. **内部量测出线**：`SignalCycleResult` 增量测帧字段；`CycleExecutor::CollectCycleOutputs` 从 scratch 量测组装；`ArController` cycle_state 与 `ArSession`/`ArRfCycleState` 装配链同步。
7. **ArTrackOutputDebugView 保留**：观测工具面不变（航迹帧仍为公开产品，DebugView 继续有意义）；删除识别字段相关视图列。

**允许范围**：公开头（新增 ArDetectionOutput.h；改 ArCycleResult/ArSession/ArTraceSession/ArOutputTypes/TrackStateSnapshot（删识别字段）/airborne_radar.hpp）；src/airborne_radar（Controller/Session/TraceSession/ReplayCodec/CycleExecutor/ArController 回填点删除）；`include/1q/fusion/SensorAdapters.h` + `src/fusion/SensorAdapters.cpp`（新增量测适配器）；`schemas/replay/airborne_radar_replay.fbs`；`tests/contract/check_public_api_boundary.cmake` 白名单；消费方迁移清单：识别字段退出影响的消费方（ArController 回填点、DebugView 识别列、replay codec 识别字段、tests 断言）；新增量测通道消费方（`tests/unit/fusion/sensor_adapters_test.cpp` 新用例、`tests/unit/airborne_radar/` 量测输出测试、replay roundtrip 新字段）。

**明确禁止**：不动 AR 内部信号链算法（检测/关联/滤波/生命周期语义原样，仅新增量测导出与删除识别字段回填）；不动航迹公开面结构（`ArTrackOutput`/`ArExternalOutputAdapter`/`ArCycleOutputAdapter` 保留）；不动 SBIRS/RIR/EOS/ESR 适配器；不动 fusion 引擎与关联键边界（航迹键直挂路径不变，量测 key=0 属既有"无身份探测"语义）；不扩展 `DetectionRecord` 结构。

**行为边界**：拒绝周期航迹帧/量测帧各自独立——航迹帧为空不阻塞量测帧输出，反之亦然；量测帧仅含本周期检测成功目标（检测失败目标不产生记录）；STT/designation 派生字段语义不变。

**验收门**：`unit::airborne_radar`、`unit::fusion`、`integration::airborne_radar`、`integration::cross_domain`、AR replay 测试、AR contract/consumer 测试、`check_public_api_boundary`、docs 守护全绿；examples 经 v141 语法级验证（Windows 无 spdlog，沿既有口径）。

**非目标**：AR 检测链物理算法演进；量测帧字段扩展（RCS 等——待证据需求再立项）；fusion 引擎改造；航迹公开面重构。

## 5. 风险与未决项

| 风险 | 影响 | 缓解 |
|---|---|---|
| 识别字段退出影响库外消费方（AR `target_type`/`target_probability`） | 编译断链或逻辑缺失 | D1 契约给消费方迁移清单；推演层 target_inference 已有 `type_evidence` 预留位；FI-A1 登记回退路径 |
| threat_assessment ESM 输入面失配需决策层公共 API 扩展 | 变更面扩大到决策层 | D1 子裁定独立冻结；回退方案 ECM 内部计分（FI-E2 拒绝路径） |
| replay 旧文件兼容 | 历史回放不可读 | 项目未上线无兼容负担；roundtrip 测试锁定 |
| 双输出（航迹+量测）增加维护面 | 公开 API 复杂度上升 | 航迹帧为核心路径、量测帧为可选补充；契约明确消费路径选择规则；不强制消费方同时消费两者 |
| 判定方向未签认即实施 | 违反证据优先门禁 | D0 退出门强制签认；D1 契约零生产代码 |

## 6. 进度登记

| 阶段 | 状态 | 说明 |
|---|---|---|
| D0 | 完成（本文档即回写） | Stage A 证据矩阵 8 项判定登记（6 pass + 1 pass-已合规 + 1 pass-确认保留）；open_questions OQ-1/2 证据与建议裁定段追加；零生产代码 |
| D1 | 完成 | 两份迁移契约冻结（§4.1/§4.2）+ 三个子裁定（§4 表）；需求方 2026-08-18 指令确认全程执行（视同签认） |
| D2 | pending | ESR 轨道实施 |
| D3 | pending（方向修正：纯量测→航迹+量测双输出） | AR 轨道实施 |
| D4 | pending | 回写关闭（OQ-1/2 迁出 + contract.md 真化） |
