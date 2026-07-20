# `docs/` 文档可靠性审查报告

Status: draft
Review-Date: 2026-07-20
Review-Scope: `docs/` 当前待审文档，排除已迁入权威设计的 AR 和 `docs/flight_dynamic/design.md`
Approval-State: SBIRS class A applied; SBIRS class B and remaining modules pending owner approval

## 1. 审查目的与边界

本报告用于审批当前 `docs/` 中已确认的过时、错误、夸大或证据断裂描述。它是非权威评审草案，
不替代 `docs/common/contract.md` 或各模块 `design.md`。审批通过后，稳定结论应迁入相应权威文档，
本草案随后删除。

本轮边界：

- 当前纳入 10 份文档：ESR、SAR、EOS、SBIRS 四份待审模块设计，`common/` 三份，`practice/` 三份。
- AR 审查结论已经批准并迁入 `docs/airborne_radar/design.md`，本草案不再保留 AR 副本。
- 按审批人指示排除 `docs/flight_dynamic/design.md`，不对其可靠性作任何判断。
- 初始审查以只读方式核验 live `include/`、`src/`、`tests/`、`examples/`、`schemas/` 和 CMake。
- 各模块由不同 subagent 独立复核；`common/` 与 `practice/` 由主审查线程交叉核验。
- 结构守卫通过不等于语义可靠。本报告只把可由当前执行路径、DTO/schema、测试或构建配置直接支持的
  项目列为“已确认”。

严重度定义：

- **P1**：会误导调用方理解 replay、输出、校验、构建/安装或运行时状态等外部契约。
- **P2**：架构职责、算法顺序、验证强度或证据归属与 live 实现不符。
- **P3**：名称、路径、数量、默认值或局部措辞过时，通常不直接改变行为理解。

处置类型：

- **D（docs-only）**：以 live 行为为事实来源，可直接修正文档。
- **A（approval required）**：文档可能表达预期契约，而代码未实现；需审批“记录当前限制”还是另开代码批次。
- **Q（question/defer）**：证据不足或需要 fresh build/characterization，本轮不得写成确定结论。

## 2. 总体结论

| 文档 | 结论 | 已确认问题概况 | 建议处置 |
|---|---|---:|---|
| `docs/electronic_surveillance_radar/design.md` | 需修订 | 3 P1、2 P2、1 P3 | D；另保留 2 个 Q |
| `docs/sar/design.md` | 需修订 | 3 P1、6 P2、若干 P3 | D + replay A；另保留 1 个 Q |
| `docs/electro_optical_sensor/design.md` | 需修订 | 3 P1、3 P2、1 P3 | D + replay A |
| `docs/space_based_infrared_sensor/design.md` | 第一类已处理、第二类待讨论 | 7 个事实/依据项已修或复核；9 个设计-实现分歧/术语项冻结 | 本轮仅 docs/comment 修订；B 类不裁决 |
| `docs/common/contract.md` | 需修订/需裁决 | 异常、命名空间、模块数量、文档结构和关系图漂移 | D + 规范性 A |
| `docs/common/usage.md` | 需修订 | 安装、Conan 消费、shared/static 和 C++ 标准描述错误 | D；安装承诺 A |
| `docs/common/open_questions.md` | 未发现确认问题 | 未找到与“当前无开放问题”直接冲突的证据 | 保持 |
| `docs/practice/batch_validation.md` | 需修订 | 模块/场景数量、sequence、退出码、测试路径和硬门描述过时 | D |
| `docs/practice/ci.md` | 需修订 | Windows preset 和 contract 数量过时 | D + Windows 策略 A |
| `docs/practice/coverage.md` | 未发现确认问题 | 当前 script 与 branch-first 口径一致 | 保持 |

结论不是“文档整体不可用”：多数物理链、DTO 分层、失败复用和 snapshot/replay 基础语义仍有 live
证据。问题集中在近期新增能力没有同步到旧架构图、batch 文案把场景名称扩大成不存在的硬断言，以及
replay/config 字段在 public DTO、schema、codec 和测试之间没有形成完整闭环。

## 3. 审批用证据矩阵

| Freeze item | 假设 | 主要证据 | 允许修订条件 | 否决/暂停条件 | 当前决策 |
|---|---|---|---|---|---|
| F1 模块架构图和执行顺序 | 文档图应对应当前真实调用链 | Session/Controller/Pipeline/Adapter live callers | 唯一调用链与图不一致 | 仅为明确标注的概念图 | pass：D |
| F2 replay 能力边界 | 文档声称可回放的结果影响字段均进入 schema/codec/comparator | EOS/SAR schema、codec、ReplaySession | 结果影响字段缺失或输入被拒绝 | 文档已明确写出限制 | pass：A |
| F3 初始化与 runtime 校验 | 文档必须区分 trusted create、report-only validation 和 atomic patch reject | `*Session::Create*`、resolver、contract tests | live 行为可直接判定 | 预期契约尚未确定 | pass：D |
| F4 batch 硬契约 | “硬检查”必须存在对应 `checks.Add`/exit gate | 五个 batch executables | 有明确 check id 并影响退出码 | 仅由场景名称或 warning 推断 | pass：D |
| F5 common 构建/异常契约 | active contract 应与构建标志和 live source 一致 | CMake flags、try/catch、presets | 事实性描述可核验 | 文档是尚未实现的强制规范 | narrow：A |
| F6 安装/下游消费 | usage 示例必须能由当前 package/install 产物支持 | `conanfile.py`、ProjectInstall、consumer CI | package/export/target 闭环存在 | 仅源码依赖可获取 | pass：A |
| F7 evidence 锚点 | evidence 应指向存在且能证明该陈述的测试 | `rg`、CTest inventory、test sources | 文件与具体测试均存在且语义相关 | 只有历史注释/旧行号 | pass：D |
| F8 未确认算法疑点 | 不把未经 fresh build/characterization 的推断写成事实 | pending probes | 补齐可复现测试 | 只有静态推断或旧二进制 | defer：Q |

## 4. 模块文档详细发现

### 4.1 Electronic Surveillance Radar

#### ESR-01（P1，D）truth evaluation 数据流顺序错误

- 文档位置：`docs/electronic_surveillance_radar/design.md:180,195`，图示 Hypothesis/Association → Truth。
- live 证据：`src/electronic_surveillance_radar/pipeline/InterceptPostProcessingExecutor.cpp:251-270`
  直接从 raw records 做 truth association；`:303-306` 才独立生成 hypothesis；`:311-324` 从 scene
  计算 missed truth。
- 建议：把 truth evaluation 画成 raw records/scene 的旁路消费，不依赖 hypothesis chain。

#### ESR-02（P1，D）`EsrOutputManager` 被误写成 live 输出组件

- 文档位置：`docs/electronic_surveillance_radar/design.md:67,87,95,214`。
- live 证据：Controller 只构造成员（`EsrController.cpp:22`），实际在 `:73-85` 直接 stamp/move 输出；
  `EsrOutputManager.cpp:8-17` 的方法仅在测试中调用，没有生产复用链。
- 建议：从生产图移除，或明确标注为未接线 helper/test surface。

#### ESR-03（P2，D）“纯三通道结果”遗漏状态字段

- 文档位置：`docs/electronic_surveillance_radar/design.md:124`。
- live 证据：`src/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h:178-183` 已有
  `sensor_powered_off`，Controller 在 `EsrController.cpp:65-71` 消费；文档后文 `:350-365` 反而准确。
- 建议：统一前后描述，写成三输出通道 + execution/status metadata。

#### ESR-04（P1，D）batch “硬契约”范围夸大

- 文档位置：`docs/electronic_surveillance_radar/design.md:393-397`。
- live 证据：`examples/batch_validation/esr_batch_validation.cpp:406-424,457-484` 只硬检查 replay、
  nonexecuted、marker、invalid recovery 和 identity continuity；没有 lifecycle recorder，也没有直接
  Lost/residue/boundary 全套断言。
- 建议：逐项引用当前 check id；其余降为未覆盖或另补测试。

#### ESR-05（P2，D）验证路径仍是旧目录布局

- 文档位置：`:238,239,266,281,369-371,386-389`。
- 当前路径分别位于：
  - `tests/unit/electronic_surveillance_radar/`
  - `tests/integration/electronic_surveillance_radar/`
  - `tests/replay/electronic_surveillance_radar/`
- 建议：全部改为 repo-relative 当前路径和具体测试名。

#### ESR-06（P3，D）recycle 阈值措辞存在 off-by-one

- 文档把触发条件写成“超过阈值”；`HypothesisAssociator.cpp:337-340` 实际为 `>=`。
- `tests/unit/electronic_surveillance_radar/esr_hypothesis_associator_test.cpp:189-212` 证明阈值 2
  在第二次 miss 即 recycle。
- 建议：改为“达到阈值”。

#### ESR-Q1（Q）batch id replay 宽度

- public `EsrCycleResult::batch_id` 是 `uint64`，schema 为 `uint32`，codec 存在窄化。
- 需要增加 `> UINT32_MAX` roundtrip/trace characterization 后才能决定是 schema defect 还是范围契约。

#### ESR-Q2（Q）resolver “验证 patch”措辞范围

- live resolver 只直接约束 scan rate/bounds/center；需先确认文档意图是“所有域语义校验”还是“本 resolver
  所有支持字段”，本轮不写成确定缺陷。

### 4.2 Synthetic Aperture Radar

#### SAR-01（P1，D）五文件治理模型与当前 contract 自相矛盾

- 文档位置：`docs/sar/design.md:265,625`。
- live 证据：`docs/common/contract.md:346` 与
  `tests/contract/check_sar_doc_governance.cmake:22,55` 均要求 SAR 只有一个 active `design.md`。
- 建议：删除五文件模型，明确本文是唯一模块设计权威。

#### SAR-02（P1，D）RDA phase reference 的顺序和语义错误

- 文档位置：`docs/sar/design.md:127,312,435`，写成方位压缩后/global phase。
- live 证据：`src/sar/imaging/SarRda.cpp:185` range compression 后，`:213` 应用 reference，`:220`
  才 azimuth FFT；`SarPhaseReference.cpp:61` 是空间变化参考；真正全局常相位对齐在
  `SarImageQuality.cpp:172`。
- 建议：区分成像链中的 spatially varying phase reference 与质量比较中的 global constant alignment。

#### SAR-03（P1，A）外部 raw IQ 不可 replay 的限制被遗漏

- 文档位置：`docs/sar/design.md:13,104,191,206`，整体暗示统一 trace/replay。
- live 证据：`include/1q/sar/session/SarCycleInput.h:51` 已写限制；
  `SarReplayFlatbufferCodec.cpp:217` 不支持外部 raw IQ；`SarTraceSession.cpp:135` 在带 replay writer 时拒绝。
- 建议：审批选择记录当前限制，或另开完整 schema/codec/trace/replay 扩展；本轮不修改 schema。

#### SAR-04（P2，D）配置图暗示所有构造均经过 validation

- 文档位置：`docs/sar/design.md:151,175`。
- live 证据：`SarSession.h:57`、`SarSession.cpp:32` 的 `Create` 是 trusted path；
  `CreateWithValidation` 在 `SarSession.cpp:37` 报告问题但仍构造。
- 建议：明确 trusted create、reporting validation 和 runtime atomic reject 的边界。

#### SAR-05（P2，D）composition root 与 runtime scheduling 混画

- 文档位置：`docs/sar/design.md:89,105`。
- live 证据：`SarSessionCompositionRoot.cpp:9` 只构造依赖；运行时由 `SarSession.cpp:50` 委托，
  `SarController.cpp:68` 执行 scheduling。
- 建议：拆分 construction view 与 cycle execution view。

#### SAR-06（P2，D）public 构造边界错误

- 文档位置：`docs/sar/design.md:625`，声称 private ctor + factory friend。
- live 证据：`include/1q/sar/session/SarSession.h:27` 有 public 默认构造且无 friend。
- 建议：按 live public API 修订。

#### SAR-07（P2，D）batch 硬契约夸大

- 文档位置：`docs/sar/design.md:611`。
- live 证据：`examples/batch_validation/sar_batch_validation.cpp:489,512` 主要检查 completed stage、
  replay 和 image-quality warning；没有直接观察 lifecycle/ring buffer 全契约。
- 建议：只保留现有 check ids 和 warning/hard gate 分类。

#### SAR-08（P2，D）“6 m passes，9 m fails”过度概括

- 文档位置：`docs/sar/design.md:473,595`。
- live 证据：`sar_second_order_motion_compensation_evidence_test.cpp:277,304` 的多目标矩阵中，
  6 m、delay 12 已出现 NRMS 0.329653 failure；0.177/0.273 只对应 reference target。
- 建议：写成“reference target 在 6 m 条件通过；完整矩阵在 6 m 已存在失败点”。

#### SAR-09（P2/P3，D）大量 evidence 路径和计数过时

- 文档位置：`:342,419,460,486,510,522,574,603` 等路径缺少当前 `/sar/` domain 目录或仅写 basename。
- 其他事实漂移：
  - `:45` generated path 应是 build/generated，不是 `src/sar/session/generated`。
  - `:454` image-quality 测试数写 9，当前为 8。
  - `:585` calibration 约 340 行，当前文件约 402 行。
  - `:637` 不存在 `compatibility::sar` CTest；live target 为 `sar_cxx11_compat`，labels 为
    `compatibility;sar`。
  - `:447` single-pulse fallback 已不成立，当前 RDA 拒绝 single pulse。
- 建议：优先用文件+测试名，避免易漂移的源码行数统计。

#### SAR-Q1（Q）`kFullPipelineL3` 与 validation 共存规则

- builder 会同时启用 L2+L3，但 validation 似乎拒绝二者共存。
- 需 fresh builder→validation→step characterization 后再决定文档或代码哪一侧错误。

### 4.3 Electro-Optical Sensor

#### EOS-01（P1，D）初始化配置不会原子拒绝非法值

- 文档位置：`docs/electro_optical_sensor/design.md:302,305`。
- live 证据：`EosSession.h:72` 的 `Create()` 是 trusted path；`:75` 与 `EosSession.cpp:63` 表明
  `CreateWithValidation()` 只报告 issues 且仍构造；
  `eos_public_api_convenience_test.cpp:520` 明确锁定非法配置仍构造。只有
  `EosRuntimeConfigResolver.cpp:117-202` 对 runtime patch 原子拒绝。
- 建议：改成“初始化校验显式、报告式、非阻断；runtime patch 原子拒绝”。

#### EOS-02（P1，A）replay 漏掉初始 `mission.power_on`

- 文档位置：`docs/electro_optical_sensor/design.md:253,266`，并引用
  `SessionConfigPreservesAllDomains`。
- live 证据：
  - `EosMissionConfig.h:26` 的 `power_on` 影响结果，mapper 在 `EosPipelineConfigMapper.cpp:68`
    映射成 `sensor_enabled`，pipeline 在 `EosPipeline.cpp:387` 产生 powered-off abort。
  - `schemas/replay/eos_session_replay.fbs:10` 的 mission table 没有该字段。
  - `EosReplayFlatbufferCodec.cpp:313,363` 编解码也遗漏；roundtrip test 未设置/断言该字段。
  - `EosReplaySession.cpp:115` 使用解码配置重建 session，因此初始关机 trace 可能按默认开机回放。
- 建议：优先审批为代码/schema bundle；若暂不实现，文档必须明确限制，且不得继续用“AllDomains”证明完整性。

#### EOS-03（P1，D）batch 硬检查和 sector retask 夸大

- 文档位置：`docs/electro_optical_sensor/design.md:427-431`。
- live 证据：`eos_batch_validation.cpp:499-518` 只硬检查 replay complete、nonexecuted、marker、
  attribution 数量和帧内 detection id unique；未使用 lifecycle recorder，未提交非法 runtime patch。
  `:402` 的所谓 sector retask 只改 scan rate，未改 sector bounds/center。
- 建议：按现有 check id 收窄；FOV/lifecycle、atomic patch 和真正 sector retask 标为未覆盖。

#### EOS-04（P2，D）range gate 不是 SNR 前过滤器

- 文档位置：`docs/electro_optical_sensor/design.md:205,276,326`。
- live 证据：`EosPipeline.cpp:401` 只有 FOV 触发前置 `continue`；`:518-544` 对 range 外目标仍构造
  record 并计算 IR/visible/fused SNR，最后只把 range 合入 `detected`；
  `eos_pipeline_test.cpp:104` 明确验证该行为。
- 建议：写明 FOV 是 record membership gate，range 是最终 detection eligibility gate。

#### EOS-05（P2，D）runtime resolver 错挂到 Controller

- 文档位置：`docs/electro_optical_sensor/design.md:96,160-162`。
- live 证据：`EosSession.cpp:87-102` 由 Session 直接 resolve、更新 internal config 并调用 pipeline；
  `EosController.cpp:26` 没有 resolver 调用。
- 建议：图改成 Session → Resolver → Session config → Pipeline。

#### EOS-06（P2，D）`EosCycleOutputAdapter` 职责错误

- 文档位置：`docs/electro_optical_sensor/design.md:137,155,277`。
- live 证据：Controller 在 `EosController.cpp:125,159` 直接组装 frame/result；
  `EosCycleOutputAdapter.cpp:8` 仅做外部坐标转换；debug/lifecycle 分别由
  `EosOutputDebugViewBuilder.cpp:62`、`EosDetectionLifecycleRecorder.cpp:72` 作为 caller-side helper 消费。
- 建议：重新划分 Controller、coordinate adapter、debug/lifecycle consumer。

#### EOS-07（P3，D）preset 基线名称过时

- 文档位置：`docs/electro_optical_sensor/design.md:296` 写 `default`。
- live 证据：public enum 为 `EosEnvironmentPreset::kStandard`，example JSON 和 loader 也只使用
  `kStandard` 名称。
- 建议：写成 `standard (kStandard)`；表中数值仍正确。

#### EOS-Q1（Q）未消费的 public 配置

- `focal_length_m` 被 mapper 复制但当前 pipeline 未消费；`pressure_hpa` 被校验/replay，但环境模型只消费
  湿度和温度。当前文档未明确声称二者影响结果，因此暂不判定为文档错误；可作为后续 config-effect
  characterization 项。

### 4.4 Space-Based Infrared Sensor

SBIRS 采用比全局 D/A/Q 更严格的二分类：

- **第一类（A）过时事实/过时依据**：无需选择设计方案；只要修正文档或注释，就能与已由当前代码、
  配置、枚举、测试或 batch check 直接证明的事实一致。
- **第二类（B）设计-实现分歧**：把文档改成代码现状会隐含否决现有设计，或把代码改成文档方案会改变
  架构/算法/状态连续性；没有对照实验、失败场景和验收门时，本轮不裁决。

| 分类 | 原 finding | 本轮处置 |
|---|---|---|
| A1 | SBIRS-01 权威头仍称“新模块目标设计” | 已改为现有模块的 current design authority，并标出 B 类过渡边界 |
| A2 | SBIRS-07 cue latency 保留旧物理解说 | 已改为 command/truth 同在 latency horizon 评估 |
| A3 | SBIRS-08 默认状态转移遗漏 tracking gate | 已区分默认 tracking gate 与默认关闭的 NIS loss |
| A4 | SBIRS-11 batch hard-contract 覆盖夸大 | 已按当前 `checks.Add`/退出门收窄 |
| A5 | SBIRS-12 失效 evidence | 已删裸行号、修正 Joseph 位置、撤掉不存在的 AR test 作为依据 |
| A6 | SBIRS-13 observation stage/失败原因不完整 | 已修设计文档和 public enum 注释 |
| A7 | 原 SBIRS-Q2 的 CA evidence 只来自旧二进制 | fresh build 后 suite 已注册且 5/5 通过，补具体测试锚点 |
| B1 | SBIRS-02 runtime patch 是否保留滤波状态 | 冻结，待连续性收益与 stale-state 风险证据 |
| B2 | SBIRS-03 大气边界、Beer–Lambert、optics/GSD 链 | 冻结，待物理误差预算和场景影响证据 |
| B3 | SBIRS-04 resolver/output/component 所有权 | 冻结，待架构职责与可测试性比较 |
| B4 | SBIRS-05 WFOV 回绕与 gate/SNR 顺序 | 冻结，待边界场景与输出影响测试 |
| B5 | SBIRS-06 NFOV 独立辐射/噪声链 | 冻结，待双链与共享 SNR 模型的误差比较 |
| B6 | SBIRS-09 debug view 是否直接暴露状态机 | 冻结，涉及 public observability 边界 |
| B7 | SBIRS-10 TruthAssisted 是否叠加误差 | 冻结，待该模式定义和统计语义 |
| B8 | SBIRS-12 的 SRIF/CKF/UDKF veto 依据及原 Q1 | 冻结；旧 AR 注释不能证明 SBIRS 否决，需 focused characterization |
| B9 | “融合输出”是算法融合还是 result aggregation | 冻结，先定义术语和实际 consumer |

#### SBIRS-A1（P1，第一类，已处理）权威头仍称新模块目标设计

- 文档位置：`docs/space_based_infrared_sensor/design.md:1,5`。
- live 证据：`src/sbirs_sensor/CMakeLists.txt:3` 已列完整生产源，文档 `:7` 自己也承认已实现。
- 处理：标题改为当前设计，Authority 改为 existing module 的 current design authority；同时规定未验证
  候选不能混写成当前事实，并明确 B 类段落在裁决前不能作为 live 行为依据。

#### SBIRS-B1（P1，第二类，冻结）runtime patch 保留滤波状态的陈述与实际相反

- 文档位置：`docs/space_based_infrared_sensor/design.md` §2.5.2 的 runtime patch 适用边界。
- live 证据：`SbirsSession.cpp:37` 每次有效 patch 都调用 `SbirsPipeline::ApplyConfig`；
  `SbirsPipeline.cpp:183` 重建 scheduler/pointing，清除 cue/target state，并通过
  `tracking_coordinator_.ClearForStandby()` 清除 EKF/NIS/IMM。
- 暂不处理：文档方案强调跨周期连续性，代码方案强调配置切换后的状态一致性；需要构造 patch 前后
  tracking continuity、旧状态污染和 replay 确定性的对照场景后再裁决。

#### SBIRS-B2（P1，第二类，冻结）设计中的物理链尚未进入生产路径

- 文档位置：`docs/space_based_infrared_sensor/design.md` §1.2～§1.4、§2.7～§2.9。
- live 证据：`SbirsEnvironmentModel.cpp:37`、`SbirsPipeline.cpp:155,266` 只有
  `base_atmospheric_transmittance × (1-A_total)` 标量链；没有 `H_atm`/目标高度门、
  `EvaluateRadiativeTransfer`、波长/距离 Beer–Lambert 求解，也没有独立 optics/diffraction/GSD 组件。
- 暂不处理：不能只因当前代码较简化就否决设计链；需先量化大气边界、波长/距离传播与 optics/GSD
  对当前场景 SNR/门控的影响。

#### SBIRS-B3（P1，第二类，冻结）组件图与 runtime/output 所有权存在分歧

- 文档位置：`docs/space_based_infrared_sensor/design.md` §1.4～§1.6。
- live 证据：`SbirsSession.cpp:37` 直接调用 resolver；`SbirsController.cpp:15` 内联组装 result；
  `SbirsCycleOutputAdapter.cpp:6` 仅提供 native-field guard；没有图中具体 `FrameContext` 或
  `SbirsTargetStateMachine` 类型。
- 暂不处理：先比较 Session-owned resolver 与 Controller-owned resolver 的事务边界、测试隔离和
  runtime patch 原子性，再决定改图还是改实现；概念节点也需单独决定是否保留。

#### SBIRS-B4（P1，第二类，冻结）WFOV 回绕与 gate/SNR 顺序存在分歧

- 文档位置：`docs/space_based_infrared_sensor/design.md` §2.3、§2.7。
- live 证据：`SbirsPipeline.cpp:20,229,266` 显示扫描先归一化到 `[-180,180)`，只在超过 end 时跳 start；
  不是在 `[start,end]` 宽度上取模。目标处理顺序是 Earth occultation → range → SNR → WFOV FOV。
- 暂不处理：扫描区间跨 ±180°、边界命中以及 FOV 外目标是否应计算 SNR 都会改变行为；需先补边界
  场景和结果差异证据。

#### SBIRS-B5（P1，第二类，冻结）NFOV 独立链与共享 SNR 实现存在分歧

- 文档位置：`docs/space_based_infrared_sensor/design.md` §1.4～§1.5、§2.4。
- live 证据：`SbirsPipeline.cpp:288,337,542` 每目标只计算一次标量 `snr`，WFOV/NFOV 共用，
  仅阈值不同；NFOV 不再次运行 radiometry/noise。
- 暂不处理：独立 NFOV 链可能更符合双视场硬件抽象；在没有参数、误差预算和验收场景前，不把共享
  SNR 直接提升为设计结论。

#### SBIRS-A2（P2，第一类，已处理）cue latency 仍保留旧物理解说

- 文档位置：`docs/space_based_infrared_sensor/design.md` §2.4“窗口判定”。
- live 证据：`SbirsPipeline.cpp:425,430` 中命令由两点角度 CV 前推到 latency horizon，delayed truth
  也按速度前推；并非“命令指向测量瞬间位置”。
- 处理：已写明命令和 eligibility truth 均评估 latency horizon，残差来自 WFOV 误差、CV 失配、
  ATP 和 NFOV 窗口；不改变算法。

#### SBIRS-A3（P2，第一类，已处理）默认状态转移不只受存在/电源影响

- 文档位置：`docs/space_based_infrared_sensor/design.md` §2.5.2“状态转移”。
- live 证据：`SbirsPolicyConfig.h:75` 默认 tracking gate loss 为 2；`SbirsPipeline.cpp:339` 达阈值会
  回到 `WideCandidate`。NIS loss 默认关闭不代表其他丢锁条件关闭。
- 处理：已分别描述默认启用、阈值为 2 的 tracking gate loss，以及默认关闭的 NIS gate loss，并补
  两个具体 pipeline 测试证据。

#### SBIRS-B6（P2，第二类，冻结）debug view 不直接暴露目标状态机枚举

- 文档位置：`docs/space_based_infrared_sensor/design.md` §2.2、§2.11。
- live 证据：`SbirsOutputDebugView.h:21`、`SbirsOutputDebugViewBuilder.cpp:28` 只提供调试 status 和
  推断 observation stage，没有 `SbirsTargetState`。
- 暂不处理：直接暴露状态机可能增强诊断，但会扩大 public DTO/replay 边界；需先确定 observability
  contract，再决定改文档或实现。

#### SBIRS-B7（P2，第二类，冻结）TruthAssisted 输出误差语义存在分歧

- 文档位置：`docs/space_based_infrared_sensor/design.md` §2.5.1。
- live 证据：`SbirsPipeline.cpp:307,345` 直接用真值 az/el 做命令和成功记录角度；测量误差只在
  EstimatedTracking correction、WFOV/capture 路径使用。
- 暂不处理：需先冻结 TruthAssisted 是“严格真值基线”还是“真值状态 + 可配置传感器误差”的模式定义。

#### SBIRS-A4（P2，第一类，已处理）batch hard-contract 覆盖夸大

- 文档位置：`docs/space_based_infrared_sensor/design.md` §2.12。
- live 证据：`sbirs_batch_validation.cpp:332-394` 的硬检查包括执行计数、通道唯一、目标覆盖、NIS 事件、
  恢复后有检测、replay 数量和 marker；没有直接证明状态隔离、稳定通道映射、reject 后与 clean session
  等价或 filter continuity。
- 处理：已按当前 check id 收窄；明确 batch 不直接证明跨周期通道稳定映射、零 mutation、filter/channel
  continuity 或 clean-session 等价。

#### SBIRS-A5/B8（P2，拆分处理）evidence 锚点断裂

- 文档位置：`docs/space_based_infrared_sensor/design.md` §2.5.3、§3。
- `EkfFilter.h:252-255` 现在是 LLT failure 分支，Joseph covariance 已移到 `:265-268`。
- “AR 500 周期病态测试”当前只剩 `ar_backend_evaluation_test.cpp:9` 历史注释，不是注册测试。
- 多个 `:268-294`、`:1174-1186` 锚点没有文件名，无法解析。
- 第一类处理：已删除 TBD/NCC/Otsu/DBSCAN 条目中无文件名、不可解析的裸行号；Joseph 实现位置修正为
  `EkfFilter.h:265-268`；明确旧 AR 文件头注释不算 live test evidence。
- 第二类冻结：以上事实不足以决定是否维持 SRIF/CKF/UDKF veto；设计只标注“依据待复核”，结论强度
  留待第二类讨论。

#### SBIRS-A6（P2，第一类，已处理）observation stage/失败原因说明不完整

- 文档位置：`docs/space_based_infrared_sensor/design.md` §2.11、§3。
- live 证据：`SbirsOutputTypes.h:18,49`；`kNarrowFieldTrack` 同时承载 Estimated 和 TruthAssisted，
  失败原因清单还漏掉 `kNfovPointingTimeout`。
- 处理：设计文档和 `SbirsOutputTypes.h` 注释统一为“NFOV 持续跟踪（估计或真值辅助）”；失败原因
  清单补入 `kNfovPointingTimeout`。

#### SBIRS-A7/B8/B9（拆分处理）原未确认项

- CKF veto 的近天底/极地几何理由缺少当前 SBIRS 场景矩阵。
- CA 证据已刷新：fresh release build 后 `SbirsCueCaCharacterizationTest` 注册 5 项且 5/5 通过；
  设计文档已补持续加速收益和标称噪声零回退失败的具体测试锚点。该结果支持“当前不接线 CA”，不代表
  CA 永久否决。
- “融合输出”未找到独立融合算法消费者，可能只是 result aggregation 用语；归入 B9，待定义后处理。

## 5. Common 文档详细发现

### 5.1 `docs/common/contract.md`

#### COMMON-01（P1，A）异常契约与 live source/build 不一致

- 文档位置：`docs/common/contract.md:125`，同时声称禁止异常并保证 `-fno-exceptions` 构建。
- live 证据：`src/flight_dynamic/adapter/JsbsimAdapter.cpp:124-129,255-258` 和
  `src/sar/output/ImageFormatter.cpp:163-200` 存在 try/catch；当前 CMake 未设置全项目 `-fno-exceptions`。
- 风险：这是规范性 contract 与实现不一致，不能只把 contract 改成“允许异常”而不经审批。
- 审批选择：C1 以 live 现状收窄规则；或 C2 保留禁止异常为目标并另开代码/构建整改。

#### COMMON-02（P2，D）`src/common` 命名空间映射过度绝对

- 文档位置：`:106-119`。
- live 证据：`src/common/coordinate/PositionTransform.cpp:7-8` 使用 `oneq::coordinate`；
  `src/common/replay/ReplayTrace.cpp:27-28` 使用 `oneq::replay`；
  `src/common/trace/TraceSink.cpp:15-16` 使用 `oneq::trace`。
- 建议：为 public-domain implementation carve-out，不能笼统规定全部 `oneq::common`。

#### COMMON-03（P2，D）SBIRS 多处遗漏或计数仍为四模块

- 文档位置：`:107,181,199,219`。
- live 证据：contract 表格本身已列 AR/ESR/EOS/SAR/SBIRS；
  `src/sbirs_sensor/session/SbirsSession.cpp:10-13` 也使用 session-owned Impl。
- 建议：统一模块数量，补齐 SBIRS composition/runtime/config 叙述。

#### COMMON-04（P2，D）`InterceptPipelineResult` 三通道陈述过时

- 文档位置：`:210`。
- live 证据：ESR result 已含 `sensor_powered_off`；同文 AR/ESR 源码行号锚点也已移动。
- 建议：写三业务通道 + execution metadata，并换成测试名锚点。

#### COMMON-05（P2，A）Windows bootstrap 规范与当前构建入口分叉

- 文档位置：`:291-294`，规范要求 shell/GitHub bootstrap，不使用 Windows Conan。
- live 证据：`CMakePresets.json:79-170` 已存在 Windows Conan 和 no-Conan presets；
  `scripts/fetch_third_party.bat` 是 no-Conan 路径，但尚不能单凭 preset 存在证明完整支持。
- 审批选择：保留规范并把 presets 标成未验收；或更新规范为双路径。需要真实 Windows configure/build/
  install/consumer 证据后才能宣布支持。

#### COMMON-06（P3，D）文档结构计数过时

- 文档位置：`:348-351` 写 common 只有两份。
- live 证据：`docs/common/usage.md` 是第三份；`check_docs_structure.cmake:49-52` 明确允许三份。
- 建议：改成 contract/open_questions/usage 三份。

#### COMMON-07（P2，D）模块关系图把概念输入画成直接依赖

- 文档位置：`:359-426`，把 flight_dynamic 画成唯一平台状态生产者并直接连各传感器。
- live 证据：传感器模块没有 include/dependency 到 flight_dynamic，而是各自消费 public `CycleInput` DTO。
- 建议：明确图是 external orchestration concept，不能标成仓库内直接调用/依赖。

### 5.2 `docs/common/usage.md`

#### USAGE-01（P1，A）“零依赖安装”表述误导

- 文档位置：`:5,9,11`，与同文 `:35` 要求 Conan cache 前置自相矛盾。
- live 证据：consumer CI 使用 Conan toolchain；ProjectInstall 只携带有限 dependency metadata，
  active spdlog/HighFive/JSBSim 等并未随 1q install tree 捆绑。
- 建议：区分“无需手工管理 include/link target”与“无需准备第三方依赖”。后者当前不成立。

#### USAGE-02（P1，A）Conan `requires = "1q/0.1"` 当前不能提供文档所称包

- 文档位置：`:53-90`。
- live 证据：`conanfile.py` 只有 name/version/dependency bootstrap，没有 `build()`、`package()`、
  `package_info()`，因此不能导出库和 CMake target 给下游。
- 建议：删除/标记为规划中的消费方式，或另开 Conan packaging 实现与 consumer proof。

#### USAGE-03（P2，D）target 类型和语言标准默认值错误

- 文档位置：`:100-104,119`。
- live 证据：`ProjectOptions.cmake:6` 默认 `BUILD_SHARED_LIBS=ON`；`ProjectSetup.cmake:19-24`
  默认 C++17；static define 在 `ProjectTargets.cmake:56-59` 条件启用。Local presets 强制 static 不等于
  项目默认 static。
- 建议：分别描述项目默认、preset override 和 consumer-visible define。

#### USAGE-04（P3，D）依赖配置文件名大小写/拼写不精确

- 文档位置：`:32`；live 文件包含 `nanoflann-config`、`flatbuffers-config` 等形式。
- 建议：使用 install tree 实际文件名或避免承诺内部 config 文件清单。

### 5.3 `docs/common/open_questions.md`

未发现能够直接推翻当前“无开放问题”状态的证据。本文可以保持不变；但若审批把 replay/schema、
Windows contract 或 Conan packaging 选为后续未决事项，应在是否进入 `open_questions.md` 之间另行裁决，
不能由本审查草案自动提升。

## 6. Practice 文档详细发现

### 6.1 `docs/practice/batch_validation.md`

#### BATCH-01（P1，D）模块和场景总数过时

- 文档位置：`:13-17,46-51,91-105,154,174-180,192-200`，仍写四模块、92 场景并未描述 sequence。
- live 证据：`examples/batch_validation/README.md:3-4,23-24` 和 CMake 当前注册五模块；
  `--list-scenarios` 结果为 199 sweep + 31 sequence = 230：
  - AR 52 + 6
  - EOS 36 + 6
  - ESR 48 + 6
  - SAR 36 + 6
  - SBIRS 27 + 7
- 建议：以 README/可执行列举为 source of truth，并单列 sweep/sequence。

#### BATCH-02（P1，D）replay divergence 不是 warning

- 文档位置：`:167-170,208`，写 divergence 不影响退出码。
- live 证据：五个 batch executable 均把 replay failure/divergence 记录为 Error，并在 error/check failure
  时返回 2，例如 AR `ar_batch_validation.cpp:489-500,736-737`、EOS `:486-497,698`、
  ESR `:444-455,672`、SAR `:467-477,688`、SBIRS `:374-385,528`。
- 建议：明确 replay divergence 是 blocking error；物理趋势才是 warning。

#### BATCH-03（P2/P3，D）测试路径和 matrix-test 概括过时

- 文档位置：`:158` 的 EOS replay 路径应为
  `tests/replay/electro_optical_sensor/eos_replay_session_test.cpp`。
- `:205` 声称通用 `tests/unit/*_matrix_test` 硬门；当前 matrix tests 只在 SAR domain 找到，不能泛化五模块。
- 建议：改成当前具体目标/文件，不使用不存在的通用 wildcard 契约。

### 6.2 `docs/practice/ci.md`

#### CI-01（P2，A）“没有 Windows preset”事实已过时，但支持状态未证明

- 文档位置：`:25,81`。
- live 证据：`CMakePresets.json:79-170` 已有 Windows Conan/no-Conan presets。
- 建议：删除“preset 不存在”；是否写成“已支持 Windows”必须等待真实 Windows 全链证明。

#### CI-02（P3，D）contract 数量过时

- 文档位置：`:13` 写 17 个 guard。
- live `ctest -N -L contract` 列出 18 个 script guards，连同 6 个 compiled contract tests 共 24。
- 建议：避免硬编码易漂移总数，或说明统计口径并由 CTest inventory 生成。

### 6.3 `docs/practice/coverage.md`

未发现确认的过时内容。`llvm-ninja-coverage`、`tools/coverage_report.sh` 对 profraw placement 的所有权、
`--label/--clean/--no-test` 等入口以及 branch-first 诊断口径与 live script 基本一致。

## 7. 已确认仍准确的关键边界

为避免审批时把文档误判为整体失效，以下内容已由 live 路径核验：

- 各模块 public/internal 分层和默认 composition root 基本成立；没有新增 public pipeline/controller SPI。
- ESR raw/canonical/hypothesis 三类业务输出和 powered-off 状态路径存在。
- SAR phase-reference/image-quality 基础实现、raw/result/debug 分层和大部分 replay DTO 路径存在。
- EOS IR/visible、Planck/传播/NEP、Lambertian、空间频谱、昼夜融合权重和 failure-marker continuation 有实现。
- SBIRS 六状态集合、默认 EKF、可选 IMM、ATP/cue、snapshot 原子恢复和七个 sequence 场景存在。
- `docs/common/open_questions.md` 和 `docs/practice/coverage.md` 本轮未找到确认的语义漂移。

## 8. 待审批决策

请逐项选择；推荐项已标注：

### AP-1：事实性文档修订范围

- [ ] **批准（推荐）**：一次性修订所有标记 D 的已确认项，不改代码/API/schema/tests。
- [ ] 只修 P1，P2/P3 延后。
- [ ] 指定需要排除的 finding ID：__________。

### AP-2：replay 能力缺口

- [ ] **批准（推荐）**：本批先在 EOS/SAR 文档明确当前限制；分别另开 evidence-first schema/code 项。
- [ ] 要求本批同时实现 EOS `power_on`、SAR raw IQ replay（这会显著扩大范围，需重新冻结合同）。
- [ ] 维持原文，不记录限制（不推荐，现有证据已证明描述过宽）。

### AP-3：全项目异常规则

- [ ] **推荐短期方案**：contract 写清当前存在第三方边界 try/catch、未保证 `-fno-exceptions`；另开规范收敛项。
- [ ] 保留“禁止异常/必须 `-fno-exceptions`”为强制目标，并安排代码与构建整改。
- [ ] 允许现状且删除禁止异常规范。

### AP-4：Windows 支持口径

- [ ] **批准（推荐）**：只陈述 Windows presets 已存在，但支持状态待真实 Windows 全链验证。
- [ ] contract 固定 shell/GitHub bootstrap 为唯一认可路径，Conan presets 标为实验性。
- [ ] 正式支持 Conan 与 no-Conan 双路径，并补两套 CI 证据。

### AP-5：安装与 Conan 消费承诺

- [ ] **批准（推荐）**：usage 删除“零依赖”和当前可 `requires=1q/0.1` 的承诺，改写为已验证 install/consumer 路径。
- [ ] 保留承诺并另开 Conan package/export/consumer 实现批次。

### AP-6：batch 文档强度

- [ ] **批准（推荐）**：所有模块只列当前存在且影响退出码的 check ids；warning 与 hard error 明确分离。
- [ ] 维持现有硬契约文字，并补齐缺失的 lifecycle/FOV/state continuity/atomic patch 检查。

### AP-7：未确认项

- [ ] **批准（推荐）**：ESR batch-id 宽度、SAR FullPipelineL3、EOS 未消费配置继续 defer；SBIRS
  CKF/SRIF/UDKF 冻结为 B8，“融合输出”冻结为 B9，本批不新增或强化结论；CA evidence 已刷新，
  生产接线仍维持当前拒绝，待新的 characterization 触发条件后再讨论。
- [ ] 指定需要立即验证的 Q 项：__________。

## 9. 审批后的建议冻结范围

若 AP-1、AP-2 推荐项获批，下一批建议冻结为：

### Frozen Contract（候选，尚未生效）

Proven requirement:

- active 文档必须与当前 live 调用链、字段、schema、验证入口和构建事实一致。

Allowed scope:

- `docs/common/*.md`
- `docs/practice/*.md`
- 五个纳入模块的 `docs/*/design.md`
- 本报告仅用于跟踪；结论迁移完成后删除。

Explicitly out of scope:

- `docs/flight_dynamic/design.md`
- `include/1q/`、`src/`、`schemas/`、生成头、tests、examples 和 CMake 行为
- replay/schema 缺口的实现
- Q 类未确认项
- 放宽测试阈值、skip、known-limit 或 compatibility policy

Acceptance gates:

- `cmake -D SOURCE_DIR=/Users/aurora/Code/1q -P tests/contract/check_docs_structure.cmake`
- `cmake -D SOURCE_DIR=/Users/aurora/Code/1q -P tests/contract/check_doc_legacy_term_guard.cmake`
- `cmake -D SOURCE_DIR=/Users/aurora/Code/1q -P tests/contract/check_sar_doc_governance.cmake`
- `cmake -D SOURCE_DIR=/Users/aurora/Code/1q -P tests/contract/check_install_manifest.cmake`
- `cmake -D SOURCE_DIR=/Users/aurora/Code/1q -P tests/contract/check_public_api_boundary.cmake`
- `git diff --check`
- 对所有新增/替换的 `[evidence: ...]` 路径和测试名执行精确 `rg` 校验。

## 10. 本报告的只读验证记录

- 文档结构基线：common=3、modules=6、module_docs=6、review 在创建本草案前为 0、practice=3、violations=0。
- `check_doc_legacy_term_guard`、`check_sar_doc_governance`、`check_install_manifest`、
  `check_public_api_boundary` 均通过。
- `ctest -N -L contract`：当前 24 个 contract entries（18 script + 6 compiled）。
- 五个 batch executable 的 `--list-scenarios`：共 199 sweep + 31 sequence。
- ESR/SAR 使用现有 release 测试二进制做聚焦验证；EOS/SBIRS 以 test listing、源码和不会生成输出的
  scenario listing 为主，避免把旧构建产物误当成 current-source 唯一证据。
- 审查结论以 live source/schema/test evidence 为主；结构守卫只作为形状证明。

报告中的模块与 common/practice 项仍等待审批；在审批前，不应修改相应 active contract/design
来提前固化尚未批准的 A/Q 类结论。
