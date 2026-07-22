Status: draft

# 工程级 RF、ECM 与 AR/ESR 实现回审

**Review-Date:** 2026-07-22

**Review-Baseline:** `b6acda1aecf895803fe2f658add4bc6a3b1da97c`

**Architecture-Reclassification:** 2026-07-22 authority freeze; implementation pending

**Authority:** 非规范性审查记录；不得替代 `docs/common/contract.md`、
`docs/airborne_radar/design.md`、`docs/electronic_surveillance_radar/design.md` 或
`docs/electronic_countermeasure/design.md`。

## 1. 范围与结论

本轮只审查基线提交中的公共 RF 基元、ECM、AR/ESR 工程压制干扰路径、trace/replay、
snapshot、跨模块测试和迁移边界。SAR、EOS、SBIRS、复数 IQ 以及新的欺骗/转发算法不在范围内。

基线已经建立以下可继续演进的骨架：

- 公共 `RfEmission` / `RfReceiverSite` / `TryEvaluateRfLink()` 及 W 域聚合；
- AR/ESR 的 tagged `none / legacy / engineering` 输入与新旧载荷互斥校验；
- ECM 点频、阻塞、扫频的基本资源分配、发射事实、snapshot、trace/replay；
- ESR 目标信号和工程干扰使用同一公共单程链路预算；
- AR 默认物理探测并将工程干扰功率加入探测噪声账本。

但该提交应被视为**工程路径集成基线**，不能视为原计划已经收口。基线仍有 8 个 P1 闭环阻塞项和
7 个 P2 完整性或证明缺口。原列出的 4 个架构选择已经在本轮写入 authority；它们不再是开放问题，
而是后续实现必须满足的验收合同。最主要的断点是：AR 工程干扰尚未形成
接收机侧 interference observation/ECCM 反馈闭环；ECM 的来源周期、滑行和发射事实边界不够严格；
ESR 的接收天线方向与扫描事实不一致，并会静默吞掉工程链路失败。

## 2. 判定方法

结论以 baseline 的 live header、source、schema、codec、测试和构建注册为证据，不以设计文档的声明
代替执行路径。分类如下：

- **A：文档/完成度声明偏差**，实现并未支持相同强度的结论；
- **B：待冻结的架构选择**，不同语义都可能成立，但必须只有一个 authority；
- **C：已冻结合同的实现缺陷**，需要先修复再宣称工程闭环完成；
- **D：证明缺口**，实现可能部分成立，但现有测试不能证明验收目标。

优先级含义：P1 必须在工程路径对外宣称完成前关闭；P2 可分批处理，但不得在迁移收口时遗留。

## 3. 架构冻结后的重新判定

本次冻结把“接入干扰机制 → 接收机影响机制 → 探测/观测映射”定义为完整链条，而不是在现有
`RfEmissionSegment` 和 `jamming_detected` 周围继续补字段。权威边界如下：

| 实施域 | 已冻结的核心机制 | 原 review 条目映射 |
|---|---|---|
| RF-WORLD | world cycle 分成 prepare/emit 与 receive/complete；AR/ECM 发布实际发射后由 orchestrator 原子冻结统一 `RfSceneFrame`；已发布发射不因接收失败回滚 | RF-ECM-01/02、RF-SNAP-01、RF-TEST-01 |
| RF-COMMON | 检测单元/接收通道级统计精度；platform/equipment/emission 分离；设备级有向 co-site；公共单程链路只到接收设备输入；参数化 waveform 合同在 characterization 后冻结 | RF-COMMON-01、RF-ECM-03、RF-PERF-01 |
| RF-AR-RX | 目标 echo 保持模块内双程雷达方程；外部 RF 走单程链路；宽带前端与 range/Doppler/beam/time-frequency detection-cell 两级账本；processed SINR/Pfa/Pd 决定量测；独立 interference observation 驱动下一发射准备的 ECCM | RF-AR-01/02 |
| RF-ESR-RX | 意图中立的统一场景入口；固定 receiver operating state；宽带前端、channel/resolution cell、可分辨性、pulse/energy observation、结构化 impairment；hypothesis 必须由观测统计产生 | RF-ESR-01/02/03 |
| RF-ECM-TX | ECM 只拥有 prepare/emit；严格 ESR provenance、模式切换缓存失效、逐威胁/滑行/热/随机状态；点频/阻塞/扫频发布参数化实际发射；首期固定或外部已解析天线，不拥有定向控制 | RF-ECM-01/02/03/04、RF-RP-01、RF-SNAP-01 |
| RF-MIGRATION | legacy 欺骗/转发 adapter 与 engineering scene 严格隔离；统一场景迁移完成前不删 legacy；schema/replay/example/consumer/batch/install/C++11 guards 同步收口 | RF-MIG-01、RF-TEST-01、RF-PERF-01 |

由此，原证据矩阵中的 `B` 项均已完成裁决：RF-ESR-03 转为 **C/D**，因为“观测估计而非真值复制”
已经冻结、实现和证明均未完成；RF-ECM-04 转为 **C/D**，因为首期不实现 ECM 定向波束，bearing 只可
用于排序/attribution，而逐威胁状态、channel 与实际发射映射仍待实现。其余 P1/P2 优先级不因文档冻结
自动下降；冻结只消除了设计歧义，没有修复 baseline 代码。

## 4. 证据矩阵

| 编号 | 生产者 → 消费者 | live evidence | 分类 | 优先级 | 判定 |
|---|---|---|---|---|---|
| RF-AR-01 | engineering emission → AR detection/ECCM | `JammingEffects.cpp`、`EnvironmentService.cpp::RefreshFrozenSnapshotFromActiveScene`、`CycleExecutor.cpp::BuildEccmSourceInfo` | C/A | P1 | 工程功率进入 SNR，但 `jamming_detected`、jammer source 和 ECCM 原始观测仍只来自 legacy 摘要 |
| RF-AR-02 | ECCM profile → AR lifecycle/timing | `CycleExecutor.cpp::ResolveLifecycleExtraMissTolerance`、`ControlProfileEffects.cpp` | C | P1 | 频率捷变、旁瓣对消、重频抖动和烧穿仍直接增加失配容忍；重频抖动只改 PRF，未形成分段脉冲时序重叠 |
| RF-ECM-01 | ESR(N-1) → ECM(N) | `EcmSession.cpp::IsValidInput`、`EcmSession::StepWithResult` | C | P1 | 新观测只要求来源周期小于 ECM 周期，旧帧可被当作新鲜帧；TruthAssisted 仍可残留非零 sensor 来源周期 |
| RF-ECM-02 | cached observation → glide/safe stop | `EcmSession.cpp::StepWithResult` | C | P1 | TruthAssisted 成功周期不老化缓存的 sensor frame，模式切回后可能复活超过两周期的旧观测 |
| RF-ECM-03 | ECM input/config → emission frame | `EcmSession.cpp::IsValidConfig`、`IsValidInput`、`BuildEmission` | C | P1 | 发射天线/极化未校验；只按威胁中心频率筛选，阻塞和扫频分段可越过硬件频率上下界；生成后未做 frame 级原子校验 |
| RF-ESR-01 | active scan beam → receive gain/gate | `InterceptDetectionExecutor.cpp::BuildReceiverSite`、`ProcessSingleEmitter` | C | P1 | gate 使用 active beam，但公共链路中的接收天线波束始终指向当前目标，导致接收增益事实与扫描事实分裂 |
| RF-ESR-02 | RF link failure → ESR cycle result | `InterceptDetectionExecutor.cpp::ProcessSingleEmitter` | C | P1 | 目标链路失败直接跳过；其他源和工程发射链路失败被忽略，缺失 co-site isolation 等错误不能形成结构化整周期拒绝 |
| RF-RP-01 | runtime patch trace → replay | `EcmTraceSession.cpp::ApplyRuntimeConfig`、`EcmReplaySession.cpp::OnRuntimeConfigPatch` | C | P1 | trace 在应用前记录 patch 且不记录 apply result；replay 强制要求 `applied=true`，空补丁和拒绝补丁不可忠实回放 |
| RF-ESR-03 | emitter truth → hypothesis → ECM adapter | `InterceptDetectionExecutor.cpp`、`EmitterHypothesis.h`、`EcmEsrAdapter.cpp` | C/D | P2 | 已冻结为观测统计估计，但 RF、带宽、PRI、脉宽仍直接复制真值，只附加不确定度；adapter 又丢弃 PRI/脉宽不确定度 |
| RF-ECM-04 | observation list → threat state/scheduler | `EcmSession.cpp::SchedulingThreat`、`BuildEmission` | C/D | P2 | 已冻结首期固定/外部已解析天线；当前仍只有整帧缓存和排序，没有逐威胁老化/状态，`channel_index` 不影响实际发射事实 |
| RF-SNAP-01 | ECM session → snapshot/replay continuation | `EcmTypes.h::EcmRuntimeState`、`EcmSession.cpp::RestoreRuntimeState`、`ecm_replay.fbs` | C/D | P2 | 内存 snapshot 含 RNG/ID/缓存，但恢复未完整校验帧内容及状态组合，FlatBuffers replay schema 也未记录完整累积调度态 |
| RF-COMMON-01 | public emission → direct link evaluation | `RfLinkBudget.cpp::IsValidSegment`、`TryEvaluateRfLink` | C | P2 | frame validator 拒绝负起始时间，直接 `TryEvaluateRfLink()` 却会接受负 `start_time_s`，公共 API 合同不一致 |
| RF-TEST-01 | ESR→ECM→AR/ESR integration → acceptance | `multi_model_scenario_test.cpp` | D | P2 | 现有闭环测试主要证明载荷接线和周期执行，未证明 SNR 恶化、interference observation、下一成功周期 ECCM、频率适配和资源重新分配 |
| RF-PERF-01 | 64/1000/1000 workload → performance gate | `rf_interference_performance_test.cpp` | D | P2 | 已测 100 周期 P95，但只断言无 validation error，未断言实际执行，也没有预热后持续内存增长证明 |
| RF-MIG-01 | legacy/public/example → engineering-only closure | public AR/ESR headers、replay schemas、`examples/` | A/D | P2 | legacy public jammer 字段仍广泛存在；没有 ECM umbrella header、独立示例或 batch 场景，迁移删除门尚未满足 |

## 5. P1 阻塞项详述

### 5.1 AR 工程干扰的可观测闭环未接通

`TryResolveEngineeringInterferencePowerW()` 已把工程发射转换为接收功率并写入
`resolved_engineering_jam_noise_w`，因此工程压制会降低物理探测 SNR。这一部分成立。

断点位于观测和决策侧：`EnvironmentService::RefreshFrozenSnapshotFromActiveScene()` 只从
`active_scene.jammer_emitters` 生成 `jammer_sources`，并用 legacy `power_db` 阈值生成
`jamming_detected`。后续 `BuildEccmSourceInfo()`、track 输出和外部决策输入都依赖该摘要。
结果是工程干扰可以压低检测率，却不能按 J/N 门控产生 interference observation，也不能驱动计划要求的
“观测 → 下一成功周期 ECCM → 重算链路”反馈。

关闭条件：在接收机链路完成后生成独立的工程 interference observation；只由接收机计算 J/N、AoA、频率及
不确定度；source ID 仅用于 attribution/replay；legacy 欺骗/转发适配层不得重复施加压制效果。

### 5.2 AR 仍存在对跟踪生命周期的直接经验作用

`ResolveLifecycleExtraMissTolerance()` 根据旁瓣对消、频率捷变、重频抖动和烧穿直接增加
`TrackLifecycleManager` 的 `max_miss_before_lost`。这违反“压制干扰只能通过 SNR、检测概率和量测误差
影响航迹”的冻结规则。该逻辑也使 ECCM 即使没有改善当前 RF 链路，仍能延长航迹寿命。

重频抖动目前改变 `effective_prf_hz`，但 RF 接收窗口仍以整周期持续时间求时间重叠，无法证明它改变了
实际脉冲与干扰分段的重叠。关闭时应删除生命周期直连，只保留链路/时序事实造成的间接收益。

### 5.3 ECM 来源周期与滑行语义不严格

fresh sensor frame 目前只要求 `source_esr_success_cycle_index < input.cycle_index`，因此首次输入一个很旧的
帧会被重置为 age 0。TruthAssisted 输入只检查 sensor observations 为空，没有要求残余
`source_esr_success_cycle_index == 0`。此外，TruthAssisted 成功周期不会推进已缓存 sensor frame 的年龄；
切回 SensorDriven 后，该帧只增加一次年龄，可能在实际超过两个成功 ECM 周期后重新发射。

关闭条件：明确并校验 fresh frame 的“上一成功 ESR 周期”来源；两种模式的所有载荷字段完全互斥；
所有成功 ECM 周期都必须以一致规则推进或失效 sensor 缓存；拒绝和关机保持状态且不生成新发射。

### 5.4 ECM 生成的 RF 事实没有完整封口

ECM 配置校验没有证明 spot/barrage/sweep 的完整占用频带落在硬件范围内，周期输入也没有校验
`transmit_antenna` 和 `transmit_polarization`。`BuildEmission()` 后没有调用公共 frame validator，导致坏的
方向图、未知极化或越界扫频可以由 ECM 自己生成，再由 AR/ESR 在消费阶段失败。

关闭条件：调度前做可行频带裁剪/拒绝，构造后以公共 frame validator 原子验证；ECM 失败必须返回明确
status 且不发布部分发射、不推进 RNG/ID/热状态。

### 5.5 ESR 接收方向图与扫描波束不一致

ESR 先解析 `active_beam` 并用于 `InterceptGate`，但 `BuildReceiverSite()` 又把接收天线 boresight 指向
正在处理的 emitter。于是目标和每个干扰源的链路都可能获得与真实扫描指向无关的接收增益，gate 与
link budget 使用了两套方向事实。

关闭条件：当前调谐窗口与 active receive beam 一起成为单一、可回放的接收机状态；目标和所有干扰
都使用该状态计算方向增益，不能逐目标重指向。

### 5.6 ESR 静默吞掉公共 RF 链路错误

目标链路失败时 `ProcessSingleEmitter()` 直接返回；其他普通辐射源和工程发射的
`TryEvaluateRfLink()` 失败则被当作零干扰跳过。尤其在同平台发射缺少 co-site isolation 时，这会把必须
fail closed 的配置错误伪装成“没有贡献”，并允许周期继续生成其他观测。

关闭条件：区分合法的零时频重叠与非法链路；非法链路向上传播结构化 abort/status，整周期原子拒绝，
不得伪造观测或提交部分 hypothesis 状态。

### 5.7 ECM runtime patch trace/replay 不对称

trace 在调用 `ApplyRuntimeConfig()` 前记录 patch，未记录 `has_requested_update/applied`。replay 收到 patch
后却要求 `applied=true`。因此 live session 可以合法接受事件记录但不应用空补丁或非法补丁，随后 replay
必然失败，不能忠实复现原执行。

关闭条件：trace 同时记录 patch 和 apply result，或只记录已经成功提交的 patch；replay 必须复现原
apply 结果并比较，而不是假设所有记录都成功。

## 6. P2 完整性与证明缺口

1. **去真值化不足。** hypothesis 不含 truth emitter ID，但 RF、带宽、PRI、脉宽中心值仍直接等于场景
   真值。实现必须从 pulse/energy observation 统计估计，并用 characterization 标定估计误差，使标称误差
   与所发布的不确定度、SNR 和分辨率一致。
   `EcmSensorObservation` 还需保留 PRI/脉宽不确定度，或者 authority 明确 ECM 首期不消费这两项。
2. **威胁状态过薄。** 当前 ECM 对观测整帧缓存并按 score 排序，不维护逐威胁年龄、置信度演化、占用状态
   或通道驻留，channel index 也不改变发射事实。首期已经冻结为固定/外部已解析发射天线；bearing 只可
   用于威胁排序或 attribution，不得在 ECM 内隐式变成指向控制。
3. **snapshot 校验和 schema 归属不完整。** `RestoreRuntimeState()` 还应验证缓存 observation、重复 ID、
   source cycle、age、`has_successful_cycle` 组合及末尾垃圾 RNG 文本。continuation snapshot 由 session
   runtime state 拥有，event replay 由 trace/replay schema 拥有；两者都必须覆盖各自恢复所需的完整累积态。
4. **公共链路入口不一致。** `TryValidateRfEmissionFrame()` 拒绝负分段起始时间，但直接链路求解只验证
   finite。冻结合同要求公共 `Try` evaluator 对非法活动区间自足、无异常、原子拒绝；实现尚未满足。
5. **跨模块测试只证明接线。** 需要固定至少一个多周期闭环，分别断言 ESR(N-1) 来源、ECM(N) 频带、
   AR/ESR(N) 的接收功率/SNR 损失、AR interference observation、下一成功周期 ECCM、重叠变化与 ECM 重新分配。
6. **性能验收不完整。** P95 场景应断言 AR/ESR 每周期实际执行；预热后内存增长需要稳定、可移植的测量
   方式和阈值，不能只凭进程最终退出推断。
7. **迁移尚未收口。** legacy jammer 字段仍保留是阶段兼容所需，但删除门应由 first-party example、consumer、
   batch、旧 trace 回放和 public/install/C++11 guard 一起驱动。当前 ECM 缺少聚合公共头和独立示例/batch
   场景，因此不能删除 legacy，也不能宣称迁移完成。

## 7. 已冻结的架构选择

以下裁决已写入 common 与模块 design；本 review 只记录映射，不拥有这些合同：

| 编号 | 已冻结裁决 | Authority |
|---|---|---|
| RF-OQ-1 | tuning/channel plan 按成功 ESR receive/complete 周期推进；拒绝、缺失冻结 scene、关机不推进 | ESR design §2.6 |
| RF-OQ-2 | 首期 ECM 只使用固定或平台/硬件层已解析天线；ECM scheduler 不拥有 pointing actuator | ECM design §3 |
| RF-OQ-3 | invalid emission、缺失设备级 co-site、unsupported near-field 是未执行失败；receiver saturation 是已执行 impairment | common contract RF 章节；AR design §2.5；ESR design §2.2/§2.6 |
| RF-OQ-4 | legacy 删除门是 producer/consumer/example/batch/旧 trace/install/C++11 全部迁移，不以新 DTO 存在为准 | common contract 跨模块输出；ECM design §4 |

此外，本次新增并冻结两项上位选择：精度层级是 detection-cell/receiver-channel 统计模型，不生成复数 IQ；
世界周期采用 prepare/emit → frozen scene → receive/complete，两阶段间的发射提交与接收回滚相互分离。

## 8. 实施关闭顺序

继续遵守每批最多 5 个文件、Release 构建和聚焦测试通过后再进入下一批：

1. **公共模型与世界编排。** 先 characterization platform/equipment/emission 身份、参数化 pulse/sweep、
   严格单程链路和设备级 co-site；再冻结 prepare/emit、scene validation/freeze、receive/complete 状态机。
2. **AR 发射与接收物理链。** 发布实际 AR emission；建立前端/检测单元账本和独立 interference
   observation；删除压制/ECCM 对 lifecycle 的直连；证明实际 ECCM 状态改变下一发射准备和检测裕度。
3. **ESR 统一接收链。** 用固定 receiver state 消费统一 scene；实现前端/channel ledger、可分辨性、
   pulse/energy observation、结构化失败/impairment 和由观测产生的 hypothesis。
4. **ECM 发射状态机。** 严格来源和模式缓存失效，补逐威胁/滑行/资源/热/随机状态，发布通过公共
   validation 的参数化实际发射，并使 runtime patch、snapshot、trace/replay 对称。
5. **迁移与验收。** 先增加两阶段闭环、replay continuation、性能执行/内存和 batch 场景，再迁移
   producer/consumer/example/install/C++11；最后删除旧 public jammer 摘要，legacy 欺骗/转发 adapter 暂留。

每一批仍最多修改 5 个文件，并在 Release 构建、聚焦测试和 `git diff --check` 通过后进入下一批。
authority 已先于实现更新，因此后续每批必须把“当前行为”与“冻结目标”分别标注，不能提前引用新目标
作为实现完成证据。

## 9. 最终验收门

- 公共 RF characterization 覆盖负时间、同平台缺隔离、方向图/极化、重复 ID、顺序无关聚合；
- AR 工程路径可以只凭接收机链路产生 interference observation，并在下一成功周期证明 ECCM 改变实际链路；
- 压制干扰与 ECCM 不再直接修改 association、Kalman、IMM 或 lifecycle 参数；
- ESR 的 active beam、调谐窗口、目标和所有干扰共享同一接收机事实，非法链路整周期 fail closed；
- ECM fresh/glide/safe-stop、双模式互斥、功率/通道/热守恒、运行补丁和 snapshot/replay 全部确定性；
- 跨模块测试验证物理量和状态转移，而不只验证 `executed_this_cycle`/无 validation error；
- Release 64/1000/1000 × 100 周期 P95 小于 100 ms，所有周期真实执行且预热后无持续内存增长；
- public/install/C++11、docs、replay、batch-validation guards 通过后，才启动 legacy public 字段删除。

在上述门全部满足前，本 review 保持 `draft`，工程 RF 状态保持“architecture frozen, implementation
pending”。完成实现和证据迁移后，权威文档改为 current behavior，再删除本 review；不得反向把 review
提升为长期 authority。
