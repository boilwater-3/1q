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
  1. **AR 走量测级输出改造**：新增公开量测/检测输出通道（现仅存在于模块内部），fusion 改
     消费量测；航迹帧退出核心运行面；AR 内部关联/滤波/生命周期降格为分层契约规则 3 行为
     建模，不再外发。
  2. **识别结论出口迁移**：`target_type`/`target_probability` 退出 AR public 输出，类型证据
     改由推演层识别面（target_inference）供给链承载。
  3. **ESR `threat_level` 退出 public DTO**，威胁启发式整体退出传感器；ECM 调度威胁分改由
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
| AR 公开输出 | 新增量测/检测输出通道（public DTO + 接入 `ArCycleResult`）；识别结论字段退出；航迹帧（`TrackOutputFrame`/`ArExternalTrackOutputFrame`/`TrackStateSnapshot` 公开面）退出核心运行面——退出路径与时序由 D1 契约冻结 |
| AR 内部 | 关联（LAPJV）/滤波（KF/IMM）/生命周期保留为规则 3 行为建模（驱动 STT 指向、驻留、丢锁判定），不再作为对外产品；滤波原语单源合规登记；内部威胁分维持"仅驱动 LPI/ECCM"既有边界 |
| fusion | AR 适配器从航迹帧消费切换为量测消费；`DetectionRecord.quality` 语义来源调整（识别置信度 → 量测质量） |
| ESR | `threat_level` 退出 `EmitterHypothesis` public DTO；`InferThreatFromCluster` 启发式整体退出；ESM 假设管理（关联/生命周期/平滑）与模式推断确认为合法 ESM 产品形态（仅文档冻结，无代码语义变更） |
| ECM | 调度威胁分（`threat_score`）改由 threat_assessment 供给；接线设计（含决策层输入面子裁定）在 D1 冻结 |
| replay | `airborne_radar_replay.fbs` / `esr_replay.fbs` 的兼容性裁定与 codec 处置 |
| 治理 | TARGET-OQ-1/2 关闭迁出；contract.md 两处过时表述真化（见 D4） |

### 1.2 Out of scope（本次处置明确不做）

1. TARGET-OQ-3（SBIRS Estimated 后验外发）：P0 已裁"维持装备语义"，随指标签认冻结，
   与本处置无关。
2. TARGET-OQ-4（RIR 识别产品形态）：方案 a 已裁（调用方键映射，零库内改动）；RIR 豁免
   形态与推演层复用边界不在本处置重开。
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
| AR 航迹通道退出路径 | D3 | 一步切换 vs 弃用过渡窗口；观测工具面（DebugView/replay 旧记录）保留形态 |
| fusion AR 通道切换语义 | D3 | 量测键语义（key=0 走空间/方位门 vs 调用方键承载）、切换前后关联质量 characterization |
| threat_assessment ESM 输入面 | D2 | `ThreatEvaluationInput`（运动学特征）与 ESM 辐射源特征（RF/PRI/脉宽）失配：输入面扩展（决策层公共 API 变更，独立冻结）vs 调用方特征映射 |
| ESR replay 兼容裁定 | D2 | `threat_level` 字段弃用语义（保留可读、写默认值）vs schema 版本升级 |
| AR replay 版本化裁定 | D3 | 量测记录新版本 vs 旧 V3 航迹记录（含 `target_type`/`target_probability`/`estimation_uncertainty_trace`）的共存与弃用 |

## 2. Stage A 证据矩阵

判定依据 docs/common/contract.md §证据优先开发模式与 evidence-first-freeze-contract 流程。
矩阵结论为**方向判定**；每项的正式冻结随 D1 迁移契约签认。

### 2.1 AR 轨道（TARGET-OQ-1）

| 冻结项 | 假设 | 证据来源 | 探查/测试 | 通过判据 | 拒绝判据（回退路径） | 判定 |
|---|---|---|---|---|---|---|
| FI-A1 识别结论出口迁移（`target_type`/`target_probability` 退出公开面） | 识别结论可退出 AR public 输出且不丢失必需能力——推演层识别面已能承载供给 | contract.md:165-167 规则 2 明文禁止；TrackStateSnapshot.h:57-60 字段定义；ArController.cpp:307-315 回填点；InferenceTrackState.h:46-47 `type_evidence` 预留位；SensorAdapters.cpp:21-23/47-49 fusion 已有状态基准回退（`ArBaseQualityForStatus`）；threat_component.cpp:86-87 威胁示例显式传 NaN 不消费 | 全库消费点盘点（已完成）：库内唯一跨模块消费 = fusion SensorAdapters；其余为 ArExternalOutputAdapter.cpp:91、ArTrackOutputDebugViewBuilder.cpp:60、replay codec/fbs、roundtrip 测试 | 消费链闭合可迁移：fusion 回退存在 + 推演层输入位存在 + 威胁链不依赖 | 若发现库外/关键消费方硬依赖该字段且无替代输入位 → 降级为"字段标注弃用 + 供给迁移过渡期" | pass |
| FI-A2 AR 公开输出量测级重构 | AR 公开输出可重构为量测形态，规则 3 的"raw output 记录保持量测形态"可达 | contract.md:168-171 规则 3 明文"不外发航迹/状态族估计产品……raw output 记录保持量测形态"；`TrackStateSnapshot` 现状 100% 航迹产品（运动学/协方差迹/生命周期状态/关联键，无单周期量测语义字段）；内部检测链止步模块内（SignalDetector.h:18-23 `DetectionResult` → TrackMeasurementProcessing.cpp:49-60 `RawTrackMeasurement`：位置 + 3×3 量测协方差 R + detection margin，量测形态雏形已存在）；fusion 消费滤波后运动学存在 R 矩阵噪声语义失配（与 TARGET-OQ-3 P0 实测同类：平滑估计 vs 量测的噪声语义差异不可忽略） | D1 前补 fusion AR 通道切换前后关联/滤波质量 characterization（量测门 vs 键直挂）；内部 `RawTrackMeasurement` 字段对公开量测 DTO 的充分性核验 | 内部量测链字段足以支撑公开量测 DTO（单位/协方差/质量），EOS/SBIRS 适配范式（SensorAdapters.cpp:82-131）可复制 | 若库外消费方硬依赖航迹帧不可迁移 → 降级装备语义正名方案（镜像 OQ-3 裁定：适配层标注来源 + boundaries 冻结装备语义为契约豁免） | pass（方向：量测级改造） |
| FI-A3 AR 关联单源化 | 传感器内关联降格为纯行为建模后，对外关联可唯一化到估计层 | FusionEngine.cpp:277 身份键直挂现状（AR `association_key` 现即直挂，不做空间再关联——"关联两次"的实际形态是两套并行生命周期/滤波）；docs/fusion/boundaries.md:13-21 关联键边界（键 0 = 无身份走空间/方位/特征门，跨源一致归调用方）；EOS/SBIRS key=0 门关联既有范式 | 切换后 AR 量测（key=0 或调用方键）在 fusion 关联四层的行为 characterization | 量测输出不带航迹键后，空间/方位门关联范式已存在且可验收 | 若密集场景 characterization 显示误关联率超 fusion 既有验收 → 契约改用键承载备选（量测携带调用方键——规则 5 允许调用方关联键，非传感器航迹键） | pass |
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
1. **AR 公共 API 量测化迁移契约**：
   - 量测输出 DTO 字段冻结：物理量单位后缀契约（contract.md §物理量单位命名）、规则 5
     去真值化（只允许调用方关联键，不带场景真值）、量测质量字段（SNR/detection margin
     类，规则 2 豁免范围）、量测协方差（规则 6 误差预算精神在量测侧的对应）、坐标系与
     既有 ECEF/ENU 契约对齐；
   - `ArCycleResult` 接入方式与内部 `RawTrackMeasurement` 字段充分性核验结论；
   - 航迹通道退出路径（一步切换 vs 弃用过渡窗口）与观测工具面保留形态（DebugView 为
     观测工具面，不得反向改变核心运行面）；
   - fusion 适配器切换语义（`AdaptArTracksToDetectionRecords` → 量测适配）与 quality
     来源调整；
   - 消费方迁移清单：examples（ar_sensor_component 等 4 组件）、tests（sensor_adapters、
     ar replay roundtrip、ar_decision_layer、ar_track_output_debug_view、
     ar_public_api_convenience、ar_session、multi_model_scenario、batch_validation、
     ar_session_consumer）、check_public_api_boundary.cmake 白名单；
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

1. 公开量测输出通道落地（DTO + 输出装配 + `ArCycleResult` 接入 + 契约测试）。
2. fusion AR 适配器切换 + quality 语义调整。
3. 识别字段退出（TrackStateSnapshot、ArExternalTrackKinematics、DebugView、
   ArController 回填点删除）；`ThreatAssessmentEvaluator::IdentifyTarget` 按契约处置
   （内部威胁分/资源管理边界维持，识别结论不再回填公开面）。
4. 航迹通道退出（按 D1 裁定路径与时序）。
5. replay 版本化 + codec + roundtrip 测试。
6. 消费方迁移（examples/tests/白名单）。
7. fusion AR 通道切换 characterization 证据归档（关联质量不回退超出契约阈值）。

退出门：`unit::airborne_radar` + `unit::fusion` + 相关 replay/contract/consumer 测试全绿。

### D4：回写关闭（Stage C）

1. TARGET-OQ-1/2 关闭：结论迁出 open_questions（条目删除），规则性结论回写
   contract.md（AR 量测输出边界、ESM 产品形态条款按需）与受影响模块 design.md
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
| AR replay | **schema 自由重构**：航迹表族（TrackOutputFrame/DecisionTrackStateSnapshot/TrackStateSnapshot 包装）从 `airborne_radar_replay.fbs` 删除；`ArCycleResultV3.output_frame` 字段类型替换为量测帧表；旧 V3 trace 一律作废 |

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

### 4.2 契约 B：AR 公开输出量测化迁移契约

**已证实需求**（FI-A1/A2/A3 = pass）：规则 3"raw output 记录保持量测形态、不外发航迹/状态族估计产品"；`TrackStateSnapshot` 现状 100% 航迹产品；fusion 消费滤波运动学存在 R 矩阵噪声语义失配；内部 `RawTrackMeasurement`（位置 + 3×3 量测协方差 R + detection margin）已是量测形态雏形。

**终态架构**：
1. **新公开量测帧** `ArDetectionOutput.h`：`ArDetectionRecord{position_x/y/z_m（雷达局部 ENU，与量测协方差同帧）、measurement_covariance 3×3（行主序 9 元素，雷达方程基于 SNR 推算的 R）、snr_db、detection_margin_db、echo_power_dbw}` + `ArDetectionOutputFrame{cycle_index, batch_id, detections}`。**不携带**场景真值标识/目标名（对齐 SBIRS boundaries 非目标 7 先例）、航迹/生命周期/识别语义、协方差以外的估计量。
2. **Step 返回类型切换**：`ArSession::Step` 返回 `ArDetectionOutputFrame`；`ArCycleResult.output_frame` 类型切换为量测帧（字段名保留）；`ONEQ_SENSOR_SESSION_CONTRACT` 锚定切换；`ArTraceSession` 同步。
3. **航迹公开面退出**（一步切换，无过渡窗口）：`ArTrackOutput.h`（帧 + 7 查询函数）删除、`output/TrackOutputQueries.cpp` 删除；`ArExternalOutputAdapter`/`ArCycleOutputAdapter`（ECEF 航迹帧）删除；`ArTrackOutputDebugView` 与其 Builder 退役（观测源为航迹帧；内部行为调试由 trace/replay 观测面承载）；STT 指定派生（`ArSession.cpp` BuildSttDesignationCycleState）改走内部航迹快照链（不再经公开帧查询）。
4. **TrackStateSnapshot 保留为决策 SPI 输入形状**：`DecisionInputFrame`（公开，决策引擎是唯一许可 SPI）继续持有 `TrackStateSnapshotList`；该结构语义收窄为"传感器行为建模状态向决策扩展点的输入"，不再是发布产品。删除 `target_type`/`target_probability`（回填产物；输入方向恒为默认值，仅输出方向被回填——纯输出侧产物）；`estimation_uncertainty_trace` 保留（SPI 输入方向的内部识别运动质量因子，规则 3 合法）。识别结论不再回填任何公开面；内部启发式识别继续驱动 LPI/ECCM（既有边界）。
5. **fusion 适配切换**：删除 `AdaptArTracksToDetectionRecords`；新增 `AdaptArDetectionsToDetectionRecords(source_id, platform, frame)`——局部 ENU 位置 + 平台位姿 → LLA（复用既有坐标换算原语），key=0（无身份，走空间/方位门），quality 口径对齐 EOS/SBIRS 适配器。
6. **replay**：`airborne_radar_replay.fbs` 自由重构——删除航迹表族（TrackOutputFrame/DecisionTrackStateSnapshot/TrackStateSnapshot 包装表）；`ArCycleResultV3.output_frame` 字段类型替换为量测帧表（`ArDetectionOutputFrame`）；旧 V3 trace 一律作废（项目未上线，无兼容负担）。
7. **内部量测出线**：`SignalCycleResult` 增量测帧字段；`CycleExecutor::CollectCycleOutputs` 从 scratch 量测组装；`ArController` cycle_state 与 `ArSession`/`ArRfCycleState` 装配链同步。

**允许范围**：公开头（新增 ArDetectionOutput.h；改 ArCycleResult/ArSession/ArTraceSession/ArOutputTypes/DecisionInputFrame/TrackStateSnapshot/airborne_radar.hpp；删 ArTrackOutput/ArExternalOutputAdapter/ArCycleOutputAdapter/ArTrackOutputDebugView）；src/airborne_radar（Controller/Session/TraceSession/ReplayCodec/查询与视图删除/STT 派生/CycleExecutor）；`include/1q/fusion/SensorAdapters.h` + `src/fusion/SensorAdapters.cpp`；`schemas/replay/airborne_radar_replay.fbs`；`tests/contract/check_public_api_boundary.cmake` 白名单；消费方迁移清单：`tests/unit/fusion/sensor_adapters_test.cpp`、`tests/unit/airborne_radar/`（ar_track_output_debug_view/ar_output_boundary/ar_core_controller/ar_cycle_output_builder 等）、`tests/integration/airborne_radar/ar_session_test.cpp`、`tests/integration/cross_domain/multi_model_scenario_test.cpp`、`tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp`、`tests/contract/airborne_radar/ar_public_api_convenience_test.cpp`、`tests/contract/public_api/public_headers_smoke_test.cpp`、`tests/consumer/ar_session_consumer.cpp`、`tests/consumer/batch_validation/ar_batch_validation.cpp`、examples（ar_sensor_component/threat_component/fusion_component 及 events/core）。

**明确禁止**：不动 AR 内部信号链算法（检测/关联/滤波/生命周期语义原样，仅新增量测导出与删除公开航迹封装）；不动 SBIRS/RIR/EOS/ESR 适配器；不动 fusion 引擎与关联键边界（AR 量测 key=0 属既有"无身份探测"语义，无键空间变更）；不扩展 `DetectionRecord` 结构。

**行为边界**：拒绝周期 `output_frame` 为空量测帧（与既有"拒绝周期不复用上一帧"一致）；量测帧仅含本周期检测成功目标（检测失败目标不产生记录）；STT/designation 派生字段语义不变（改内部数据源，不改冻结的派生规则）。

**验收门**：`unit::airborne_radar`、`unit::fusion`、`integration::airborne_radar`、`integration::cross_domain`、AR replay 测试、AR contract/consumer 测试、`check_public_api_boundary`、docs 守护全绿；examples 经 v141 语法级验证（Windows 无 spdlog，沿既有口径）。

**非目标**：AR 检测链物理算法演进；量测帧字段扩展（RCS 等——待证据需求再立项）；fusion 引擎改造；内部航迹栈重构。

## 5. 风险与未决项

| 风险 | 影响 | 缓解 |
|---|---|---|
| 破坏性公共 API 变更影响库外使用方（AR 航迹帧/识别字段、ESR threat_level） | 编译断链 | D1 契约给弃用窗口/版本化路径；consumer 测试先迁；FI-A1/FI-A2/FI-E1 均登记回退路径 |
| fusion AR 通道由键直挂改量测门后关联质量回退 | 融合产品质量下降 | D3 强制 characterization 前后对比；不达标走 FI-A3 拒绝路径（调用方键承载） |
| threat_assessment ESM 输入面失配需决策层公共 API 扩展 | 变更面扩大到决策层 | D1 子裁定独立冻结；回退方案 ECM 内部计分（FI-E2 拒绝路径） |
| replay 旧文件兼容 | 历史回放不可读 | FI-E4 方向：字段保留弃用语义；roundtrip 测试锁定；必要时版本升级 |
| AR 内部航迹栈保留但不再外发，公开面测试语义漂移 | 行为回归不可见 | 内部栈测试（unit 树内含内部头）不受影响；公开面断言随迁移清单同步改写，不得删除覆盖 |
| 航迹帧退出后 examples 展示/调试价值损失 | 演示能力下降 | DebugView 观测工具面按契约裁定保留形态；示例可改消费 fusion 产品 |
| 判定方向未签认即实施 | 违反证据优先门禁 | D0 退出门强制签认；D1 契约零生产代码 |

## 6. 进度登记

| 阶段 | 状态 | 说明 |
|---|---|---|
| D0 | 完成（本文档即回写） | Stage A 证据矩阵 8 项判定登记（6 pass + 1 pass-已合规 + 1 pass-确认保留）；open_questions OQ-1/2 证据与建议裁定段追加；零生产代码 |
| D1 | 完成 | 两份迁移契约冻结（§4.1/§4.2）+ 三个子裁定（§4 表）；需求方 2026-08-18 指令确认全程执行（视同签认） |
| D2 | pending | ESR 轨道实施 |
| D3 | pending | AR 轨道实施 |
| D4 | pending | 回写关闭（OQ-1/2 迁出 + contract.md 真化） |
