Status: draft

# 工程级 RF、ECM 与 AR/ESR 实现回审

**Review-Date:** 2026-07-22

**Review-Baseline:** `b6acda1aecf895803fe2f658add4bc6a3b1da97c`

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

但该提交应被视为**工程路径集成基线**，不能视为原计划已经收口。当前有 8 个 P1 闭环阻塞项、
7 个 P2 完整性或证明缺口，以及 4 个需要冻结的架构选择。最主要的断点是：AR 工程干扰尚未形成
接收机侧 jammer observation/ECCM 反馈闭环；ECM 的来源周期、滑行和发射事实边界不够严格；
ESR 的接收天线方向与扫描事实不一致，并会静默吞掉工程链路失败。

## 2. 判定方法

结论以 baseline 的 live header、source、schema、codec、测试和构建注册为证据，不以设计文档的声明
代替执行路径。分类如下：

- **A：文档/完成度声明偏差**，实现并未支持相同强度的结论；
- **B：待冻结的架构选择**，不同语义都可能成立，但必须只有一个 authority；
- **C：已冻结合同的实现缺陷**，需要先修复再宣称工程闭环完成；
- **D：证明缺口**，实现可能部分成立，但现有测试不能证明验收目标。

优先级含义：P1 必须在工程路径对外宣称完成前关闭；P2 可分批处理，但不得在迁移收口时遗留。

## 3. 证据矩阵

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
| RF-ESR-03 | emitter truth → hypothesis → ECM adapter | `InterceptDetectionExecutor.cpp`、`EmitterHypothesis.h`、`EcmEsrAdapter.cpp` | B/C | P2 | RF、带宽、PRI、脉宽估计值直接复制真值，只附加不确定度；adapter 又丢弃 PRI/脉宽不确定度，sensor-driven 仍不够传感器化 |
| RF-ECM-04 | observation list → threat state/scheduler | `EcmSession.cpp::SchedulingThreat`、`BuildEmission` | B/C | P2 | 仅有整帧缓存和排序，没有逐威胁老化/状态；bearing 不参与指向，`channel_index` 不影响发射事实 |
| RF-SNAP-01 | ECM session → snapshot/replay continuation | `EcmTypes.h::EcmRuntimeState`、`EcmSession.cpp::RestoreRuntimeState`、`ecm_replay.fbs` | C/D | P2 | 内存 snapshot 含 RNG/ID/缓存，但恢复未完整校验帧内容及状态组合，FlatBuffers replay schema 也未记录完整累积调度态 |
| RF-COMMON-01 | public emission → direct link evaluation | `RfLinkBudget.cpp::IsValidSegment`、`TryEvaluateRfLink` | C | P2 | frame validator 拒绝负起始时间，直接 `TryEvaluateRfLink()` 却会接受负 `start_time_s`，公共 API 合同不一致 |
| RF-TEST-01 | ESR→ECM→AR/ESR integration → acceptance | `multi_model_scenario_test.cpp` | D | P2 | 现有闭环测试主要证明载荷接线和周期执行，未证明 SNR 恶化、jammer observation、下一成功周期 ECCM、频率适配和资源重新分配 |
| RF-PERF-01 | 64/1000/1000 workload → performance gate | `rf_interference_performance_test.cpp` | D | P2 | 已测 100 周期 P95，但只断言无 validation error，未断言实际执行，也没有预热后持续内存增长证明 |
| RF-MIG-01 | legacy/public/example → engineering-only closure | public AR/ESR headers、replay schemas、`examples/` | A/D | P2 | legacy public jammer 字段仍广泛存在；没有 ECM umbrella header、独立示例或 batch 场景，迁移删除门尚未满足 |

## 4. P1 阻塞项详述

### 4.1 AR 工程干扰的可观测闭环未接通

`TryResolveEngineeringInterferencePowerW()` 已把工程发射转换为接收功率并写入
`resolved_engineering_jam_noise_w`，因此工程压制会降低物理探测 SNR。这一部分成立。

断点位于观测和决策侧：`EnvironmentService::RefreshFrozenSnapshotFromActiveScene()` 只从
`active_scene.jammer_emitters` 生成 `jammer_sources`，并用 legacy `power_db` 阈值生成
`jamming_detected`。后续 `BuildEccmSourceInfo()`、track 输出和外部决策输入都依赖该摘要。
结果是工程干扰可以压低检测率，却不能按 J/N 门控产生 jammer observation，也不能驱动计划要求的
“观测 → 下一成功周期 ECCM → 重算链路”反馈。

关闭条件：在接收机链路完成后生成独立的工程 jammer observation；只由接收机计算 J/N、AoA、频率及
不确定度；source ID 仅用于 attribution/replay；legacy 欺骗/转发适配层不得重复施加压制效果。

### 4.2 AR 仍存在对跟踪生命周期的直接经验作用

`ResolveLifecycleExtraMissTolerance()` 根据旁瓣对消、频率捷变、重频抖动和烧穿直接增加
`TrackLifecycleManager` 的 `max_miss_before_lost`。这违反“压制干扰只能通过 SNR、检测概率和量测误差
影响航迹”的冻结规则。该逻辑也使 ECCM 即使没有改善当前 RF 链路，仍能延长航迹寿命。

重频抖动目前改变 `effective_prf_hz`，但 RF 接收窗口仍以整周期持续时间求时间重叠，无法证明它改变了
实际脉冲与干扰分段的重叠。关闭时应删除生命周期直连，只保留链路/时序事实造成的间接收益。

### 4.3 ECM 来源周期与滑行语义不严格

fresh sensor frame 目前只要求 `source_esr_success_cycle_index < input.cycle_index`，因此首次输入一个很旧的
帧会被重置为 age 0。TruthAssisted 输入只检查 sensor observations 为空，没有要求残余
`source_esr_success_cycle_index == 0`。此外，TruthAssisted 成功周期不会推进已缓存 sensor frame 的年龄；
切回 SensorDriven 后，该帧只增加一次年龄，可能在实际超过两个成功 ECM 周期后重新发射。

关闭条件：明确并校验 fresh frame 的“上一成功 ESR 周期”来源；两种模式的所有载荷字段完全互斥；
所有成功 ECM 周期都必须以一致规则推进或失效 sensor 缓存；拒绝和关机保持状态且不生成新发射。

### 4.4 ECM 生成的 RF 事实没有完整封口

ECM 配置校验没有证明 spot/barrage/sweep 的完整占用频带落在硬件范围内，周期输入也没有校验
`transmit_antenna` 和 `transmit_polarization`。`BuildEmission()` 后没有调用公共 frame validator，导致坏的
方向图、未知极化或越界扫频可以由 ECM 自己生成，再由 AR/ESR 在消费阶段失败。

关闭条件：调度前做可行频带裁剪/拒绝，构造后以公共 frame validator 原子验证；ECM 失败必须返回明确
status 且不发布部分发射、不推进 RNG/ID/热状态。

### 4.5 ESR 接收方向图与扫描波束不一致

ESR 先解析 `active_beam` 并用于 `InterceptGate`，但 `BuildReceiverSite()` 又把接收天线 boresight 指向
正在处理的 emitter。于是目标和每个干扰源的链路都可能获得与真实扫描指向无关的接收增益，gate 与
link budget 使用了两套方向事实。

关闭条件：当前调谐窗口与 active receive beam 一起成为单一、可回放的接收机状态；目标和所有干扰
都使用该状态计算方向增益，不能逐目标重指向。

### 4.6 ESR 静默吞掉公共 RF 链路错误

目标链路失败时 `ProcessSingleEmitter()` 直接返回；其他普通辐射源和工程发射的
`TryEvaluateRfLink()` 失败则被当作零干扰跳过。尤其在同平台发射缺少 co-site isolation 时，这会把必须
fail closed 的配置错误伪装成“没有贡献”，并允许周期继续生成其他观测。

关闭条件：区分合法的零时频重叠与非法链路；非法链路向上传播结构化 abort/status，整周期原子拒绝，
不得伪造观测或提交部分 hypothesis 状态。

### 4.7 ECM runtime patch trace/replay 不对称

trace 在调用 `ApplyRuntimeConfig()` 前记录 patch，未记录 `has_requested_update/applied`。replay 收到 patch
后却要求 `applied=true`。因此 live session 可以合法接受事件记录但不应用空补丁或非法补丁，随后 replay
必然失败，不能忠实复现原执行。

关闭条件：trace 同时记录 patch 和 apply result，或只记录已经成功提交的 patch；replay 必须复现原
apply 结果并比较，而不是假设所有记录都成功。

## 5. P2 完整性与证明缺口

1. **去真值化不足。** hypothesis 不含 truth emitter ID，但 RF、带宽、PRI、脉宽中心值仍直接等于场景
   真值。应冻结“估计值如何由观测生成”的模型；至少要让标称误差与所发布的不确定度、SNR 和分辨率一致。
   `EcmSensorObservation` 还需保留 PRI/脉宽不确定度，或者 authority 明确 ECM 首期不消费这两项。
2. **威胁状态过薄。** 当前 ECM 对观测整帧缓存并按 score 排序，不维护逐威胁年龄、置信度演化、占用状态
   或通道驻留；bearing 被复制但没有进入发射指向，channel index 也不改变发射事实。需要先裁决首期是
   全向发射还是 ECM 拥有定向波束控制。
3. **snapshot 校验和 schema 归属不完整。** `RestoreRuntimeState()` 还应验证缓存 observation、重复 ID、
   source cycle、age、`has_successful_cycle` 组合及末尾垃圾 RNG 文本；若 replay 依赖事件重演而非快照，
   authority 应明确区分 continuation snapshot 与 event replay，并分别给出 schema 所有权。
4. **公共链路入口不一致。** `TryValidateRfEmissionFrame()` 拒绝负分段起始时间，但直接链路求解只验证
   finite。应冻结公共 link evaluator 是自足的严格入口，还是要求调用方先验证 frame；当前“Try”接口更适合
   自足原子拒绝。
5. **跨模块测试只证明接线。** 需要固定至少一个多周期闭环，分别断言 ESR(N-1) 来源、ECM(N) 频带、
   AR/ESR(N) 的接收功率/SNR 损失、AR jammer observation、下一成功周期 ECCM、重叠变化与 ECM 重新分配。
6. **性能验收不完整。** P95 场景应断言 AR/ESR 每周期实际执行；预热后内存增长需要稳定、可移植的测量
   方式和阈值，不能只凭进程最终退出推断。
7. **迁移尚未收口。** legacy jammer 字段仍保留是阶段兼容所需，但删除门应由 first-party example、consumer、
   batch、旧 trace 回放和 public/install/C++11 guard 一起驱动。当前 ECM 缺少聚合公共头和独立示例/batch
   场景，因此不能删除 legacy，也不能宣称迁移完成。

## 6. 需要冻结的架构选择

以下问题不应在实现中继续隐式演化，裁决后只写入各自唯一 authority：

| 编号 | 问题 | 建议冻结方向 |
|---|---|---|
| RF-OQ-1 | tuning plan 按 world cycle 还是成功 ESR cycle 推进 | 建议按成功 ESR cycle；拒绝/关机不推进，snapshot/replay 保存当前位置 |
| RF-OQ-2 | ECM 发射波束由调用者提供还是 ECM planner 拥有 | 首期若不建定向调度，明确冻结为全向/固定硬件波束并删除无效 bearing 暗示；否则由 ECM 输出实际 pointing |
| RF-OQ-3 | AR/ESR 公共链路失败如何上卷 | 建议共享 fail-closed 分类：invalid emission、missing co-site isolation、near-field unsupported、receiver saturated |
| RF-OQ-4 | legacy public 字段删除门 | 建议以所有一方 example/consumer/batch 和旧 trace 分流完成为门，不以新 API 已存在为门 |

## 7. 建议的关闭顺序

继续遵守每批最多 5 个文件、Release 构建和聚焦测试通过后再进入下一批：

1. 公共 RF 严格入口、ECM 输入来源/滑行、发射 frame 原子校验；
2. ECM runtime patch trace/replay 对称和 snapshot 恢复校验；
3. AR 工程 jammer observation 与下一成功周期 ECCM，删除生命周期直连；
4. ESR active receive beam 单一事实和结构化链路失败上卷；
5. ESR 估计误差/不确定度到 ECM 的完整适配与逐威胁状态；
6. 多周期语义验收、性能执行/内存门和 migration guards；
7. 证据通过后更新 authority，最后删除本 draft review。

## 8. 最终验收门

- 公共 RF characterization 覆盖负时间、同平台缺隔离、方向图/极化、重复 ID、顺序无关聚合；
- AR 工程路径可以只凭接收机链路产生 jammer observation，并在下一成功周期证明 ECCM 改变实际链路；
- 压制干扰与 ECCM 不再直接修改 association、Kalman、IMM 或 lifecycle 参数；
- ESR 的 active beam、调谐窗口、目标和所有干扰共享同一接收机事实，非法链路整周期 fail closed；
- ECM fresh/glide/safe-stop、双模式互斥、功率/通道/热守恒、运行补丁和 snapshot/replay 全部确定性；
- 跨模块测试验证物理量和状态转移，而不只验证 `executed_this_cycle`/无 validation error；
- Release 64/1000/1000 × 100 周期 P95 小于 100 ms，所有周期真实执行且预热后无持续内存增长；
- public/install/C++11、docs、replay、batch-validation guards 通过后，才启动 legacy public 字段删除。
