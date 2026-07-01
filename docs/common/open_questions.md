# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

---

## OQ-1 四模块 runtime 配置提交策略不一致（已结案：A 维度闭环 + B/C 维度契约化）

各传感器模块对"runtime 配置修改如何安全生效"采用了四种不同策略，没有统一范式。这不是 bug——每个策略单独看都自洽——但是跨模块阅读时的主要认知负担来源，且新模块/新维护者缺少明确的基准。

| 模块 | patch 校验入口 | 配置提交时机 | 提交失败处理 | 周期执行失败处理 |
| --- | --- | --- | --- | --- |
| airborne_radar | resolver（`has_requested_update`+`is_valid`） | 延迟到下个周期边界（`StepWithResult` 开头） | 4 子系统快照回滚 | 4 子系统快照回滚 |
| electronic_surveillance_radar | resolver | 立即（调用即生效） | **无回滚**（commit 与 rollback 状态空间不重合，见 OQ-1a） | pipeline + controller 快照回滚 |
| electro_optical_sensor | resolver | 立即（调用即生效） | 无回滚 | pipeline 快照回滚（封装在 `EosController::RunOnce` 内，非 session 层） |
| sar | **无 resolver**（`ApplyPatchToConfig` 盲写 `has_*`） | 立即 patch 到 `runtime_config` | 无回滚 | 执行前 gate（`ValidateRuntimeConfigForStep`） |

证据：
- AR 延迟提交 + 回滚：`src/airborne_radar/session/RadarSession.cpp:117`（`CommitPendingRuntimeConfig`，在 `controller.RunOnce` 之前）、`:185-197`（快照 capture + 失败 restore）。patch 经 `ApplyRuntimePatch` resolver 校验（`:298-317`）。
- ESR 立即提交 + 执行失败回滚：`src/electronic_surveillance_radar/session/EsrSession.cpp:98`（`ApplyRuntimeConfigWithResult` 立即调 `UpdateConfig`）、`:48-58`（`RunCycle` 内 capture/restore）。patch 经 `ResolveEsrRuntimeConfigPatch` resolver 校验（`:101`）。
- EOS 立即提交（session 层无事务，但执行回滚在 controller 内）：`src/electro_optical_sensor/session/EosSession.cpp:86`（`TryApplyRuntimeConfig` 立即 `ApplyInternalConfig`，无 capture/restore）；`src/electro_optical_sensor/runtime/EosController.cpp:68-69,88,90-111`（`RunOnce` 内 capture → execute → 失败 `RestoreRuntimeState` + 复原 `latest_output`）。patch 经 `ResolveEosRuntimeConfigPatch` resolver 校验（`:88`）。
- SAR 立即 patch + 执行前 gate：`src/sar/session/SarSession.cpp:194`（`TryApplyRuntimeConfig` 直接 `ApplyPatchToConfig` 盲写 `has_*`，**无 resolver、无 `is_valid` 校验，只要 `HasRequestedUpdate` 即恒返回 true**）、`:127`（`ValidateRuntimeConfigForStep`，校验整体推迟到 step）。

为何未决：真正的分叉在两个正交维度——(A) patch 校验入口（SAR 是唯一离群点，无 resolver）；(B)/(C) 提交时机与事务性。B/C 维度各有合理性（AR 谨慎因信号 pipeline 状态多；EOS 把执行回滚下沉到 controller 内；SAR 无累积运行期状态，gate + 无回滚结构上正确），没有失败/竞态证据指向某一策略错误。但 A 维度是收敛且低风险的：让 SAR 的 `TryApplyRuntimeConfig` 也过 resolver 校验，不触碰任何模块的事务模型。

推进需要：
- 一个真实的失败案例（如某模块立即提交导致周期内配置中途变化、产生不可预料结果），或
- 明确的架构方向选择（如统一为 AR 式延迟提交 + 回滚，或统一为 EOS 式立即提交）。

**已推进（A 维度，SAR resolver）**：见下"推进记录"。

**已契约化（B/C 维度，提交时机与事务性）**：经论证，行为统一（把四模块拉成同一 commit/rollback 实现）有害——SAR 无累积状态会得到空操作事务，ESR 的 restore 当前是不可达死分支（见 OQ-1a）。真正的认知负担病因是"实现暗示了不存在的语义"，而非"四实现不同"。故采用**契约分类统一**（零行为风险）：在 `docs/common/contract.md` 新增「运行期配置提交策略」节，按 pipeline 状态空间复杂度分两类——**事务性提交**（仅 `airborne_radar`：有累积状态且 commit/执行存在真实失败路径）与**立即提交**（`electronic_surveillance_radar`/`electro_optical_sensor`/`sar`：调用即生效、单向不回滚）。各模块 `TryApplyRuntimeConfig`/capture-restore 处补 doc 声明所属类别与边界；ESR 的不可达 restore 分支显式注明现状。详见下"推进记录"。本条随契约化结案，仅保留作背景溯源。

### 推进记录

- **A 维度已闭环（SAR resolver，commit `774eaafc`）**：新增 `src/sar/session/SarRuntimeConfigResolver.{h,cpp}`，对齐 ESR/EOS 的"写之前校验"范式（boolean-only reject）。`SarSession::TryApplyRuntimeConfig` 改走 `ResolveSarRuntimeConfigPatch`：对有效补丁行为零变化（仍写 `policy` 字段、返回 true）；对无效补丁（`minimum_snr_db` 非有限、`enable_l1_rda_imaging` 无 `enable_raw_echo_generation`）从原先的"静默接受→step 时 abort"前置为"apply 时拒绝并返回 false"。后者把原 step-time gate（`SarRuntimeConfigValidation.cpp:54-58`）的静态可判定子集前置，消除"SAR `TryApplyRuntimeConfig` 恒真"的隐式契约。测试：`tests/unit/sar_runtime_config_resolver_test.cpp`（6 用例）；既有 `SarSessionPipelineTest`/`SarReplaySessionTest`/`SarPublicApiConvenienceTest` 全量通过，零回归。状态：SAR 的 patch 校验入口现已与 AR/ESR/EOS 对齐。

- **B/C 维度已契约化（提交时机与事务性）**：未改变任何模块行为。在 `docs/common/contract.md` 新增「运行期配置提交策略」契约节，确立两类分类与四模块固定归属，规则要求归属由状态空间决定（非风格偏好）、所有模块须经 resolver 校验、立即提交类不得声称 session 层回滚、事务性提交类不得在执行成功前落定配置。代码侧：`InterceptPipeline::CaptureRuntimeState`/`RestoreRuntimeState` 补 doc 注明快照不含 config 及 ESR 立即提交归属；`EsrSession::RunCycle` restore 分支注明当前不可达（待 `kOutputContractViolation` 接线）；`RadarSession`/`EosSession`/`SarSession` 的 `TryApplyRuntimeConfig` 各注明所属提交类别。测试：`sar_session_pipeline_test.cpp` 新增 3 用例（有效 patch 立即生效、无效 patch 拒绝不污染、L1-RDA 依赖违反拒绝），与既有 ESR/EOS integration apply 测试形成跨模块对等覆盖。


注：P2.3（commit `a83928b3`）只收拢了 ESR 写路径绕 extension 的往返，未触碰任何模块的提交时机/安全策略。

## OQ-1a ESR commit 与 rollback 的状态空间不重合（已结案：原命题证伪）

OQ-1 的细化：ESR 的 runtime config 提交（`ApplyRuntimeConfigWithResult`）与周期执行回滚（`RunCycle` 内 capture/restore）作用于**不同的状态机部件**。

**已结案（经代码证伪，原命题不成立）**：原担心"commit 后执行失败回滚不恢复 config"。但代码事实证明 ESR **当前没有任何 pipeline 执行失败的 abort 路径**：

- `EsrPipelineAbortReason` 枚举（`include/1q/electronic_surveillance_radar/session/EsrOutputTypes.h:60-64`）有 4 值，但全代码赋值点（`rg EsrPipelineAbortReason::` src/）只用到 3 个：`kNone`（成功，`EsrController.cpp:83`）、`kValidationRejected`（校验失败，`:48`，在 `pipeline.RunCycle` **之前** return）、`kRuntimeStateRestoreRejected`（restore 自身失败，`:135`）。`kOutputContractViolation` **从未被赋值**——pipeline 结果契约校验在 ESR controller 里没有接线（对比 EOS `EosController.cpp:90-111` 的 `IsEosExecuteResultContractValid` 是接了的）。
- 因此 `EsrSession::RunCycle:53-55` 的 restore 条件 `!ExecutedLatestCycle() && abort != kValidationRejected` 在当前实现下**永假**——`controller.RunOnce` 拿到 `pipeline_result` 后无条件 move 进 output 并标记 executed（`EsrController.cpp:79-84`），不存在"执行了但失败"的结局。restore 分支是死代码。

**降级结论**：ESR 的 config 实际是**单向立即生效、永不回滚**的设计。`InterceptPipeline::UpdateConfig`（`InterceptPipeline.cpp:52-57`）走 `associator_.UpdateConfig`（换 config 留 tracks），而非重建 associator——证明 config（无累积，每次 RunCycle 从 `config_` 重新派生 `pipeline_config`/`runtime_config`）与累积状态（`rng_`/`next_*_id`/`tracks`）是**有意分离**的两个状态空间，不是疏漏。

对比 AR：AR 的 `CommitPendingRuntimeConfig`（`RadarSession.cpp:117`）失败时 restore 含 pipeline 在内的 4 个子系统，且只在执行成功的 `FinalizePendingRuntimeConfig`（`:167-176`）才落定——AR 有真实的 commit 失败路径（`UpdateExecutionConfig`/`UpdateConfig` 可返回 false），故需要事务对齐。ESR 没有这种失败路径，故不需要。

**残留可清理项（非 bug，认知负担来源）**：`EsrSession::RunCycle` 的 capture/restore 机制 + `kOutputContractViolation` 枚举值给人"存在执行失败回滚"的错觉，实则当前不可达。是否接线 pipeline 结果契约校验（激活该路径）或删除死分支，留给后续按需决定——但当前 OQ-1a 的"config 回滚裂缝"命题已不成立。

## OQ-2 飞行动力学局部 NE 投影 cos-lat 约定分叉

`WaypointManager` 与 `Maneuver` 各自实现了 lat/lon → 局部北/东米投影，但两者使用的 cos-纬度约定不同，对大 cross-track 偏移会产生不同几何结果。

- `WaypointManager` 用平均纬度 cos：`src/flight_dynamic/guidance/WaypointManager.cpp:24`（`mean_latitude_rad = 0.5 * (lat + origin_lat)`）、`:28`（`east_m = ... * cos(mean_latitude_rad)`）。
- `Maneuver` 用参考中心纬度 cos：`src/flight_dynamic/guidance/Maneuver.cpp:160`、`:203`（`cos_lat = std::cos(center.latitude_rad)`），并在文件内 6 处内联重复该投影。

为何未决：无法从代码判断哪个约定是有意为之。两者在小偏移下数值接近，分叉只在远场才显现；现有测试未覆盖"两套约定应一致"或"应不同"的断言。合并到单一投影需要先决定以哪个 cos-lat 约定为准。

推进需要：
- 领域知识确认：orbit / figure-8 / racetrack 几何中，cos(mean lat) 与 cos(center/reference lat) 哪个是正确意图；
- 决定后，要么统一为单一约定（带参数的 helper），要么显式记录"两者有意不同"并保留；
- 重测 orbit / figure-8 / racetrack 几何，确认无回归。

注：P2.5a（commit `ff0c9a2c`）只收拢了 `NormalizeRad`/`RadToDeg360`（角度归一化），明确未触碰此 NE 投影分叉。

## OQ-3 EOS replay 派生环境字段非对称 codec

`EosSessionConfig` 的 replay codec 对派生环境字段是非对称的：encode 写入、decode 丢弃。schema 注释自称这些字段"冗余记录以支持精确比对"，但 decode 端从不读取它们。

- encode 写入派生字段：`src/electro_optical_sensor/session/EosReplayFlatbufferCodec.cpp:338`（`BuildModelConfigFromScenario` 后写入 `radiative_transfer_model_derived` 等）。
- decode 不读派生字段：`DecodeEosSessionConfig` 只还原 `scenario_config`，从不读 `*_derived`（依赖 consumer 重新 `BuildModelConfigFromScenario`）。
- schema 注释自述冗余：`schemas/replay/eos_session_replay.fbs` 的 `EosEnvironmentConfig`。

为何未决：当前行为正确（consumer 会重新 derive），不是 bug。但这是 source-of-truth / drift 隐患——若 `BuildModelConfigFromScenario` 的 preset→factor 映射改变，replay buffer 的"冗余比对"字段会与 fresh decode 静默分叉，且现有 roundtrip 测试无法捕获（decode 不读这些字段）。

推进需要：先补一个断言"派生字段 encode/decode 一致"的 roundtrip 测试，再决定是让 decode 读回派生字段、还是从 schema 删除它们。

注：P2.2（commit `6ad98476`）只收拢了 detection-record 的 encode/decode，明确未触碰此派生字段非对称。

## OQ-4 飞行动力学失速速度 ρ 来源漂移

失速速度公式 `V_stall = sqrt(2W / (ρ·S·CLmax))` 的 ρ 在三个调用点来源不一致，是已知 bug 但尚无失败测试证据。

- Autopilot：硬编码 `kRhoSeaLevel = 0.002377`：`src/flight_dynamic/autopilot/Autopilot.cpp:339`。
- EngineManager `GetRotationSpeedKts`：读 property tree `atmosphere/rho-slugs_ft3`：`src/flight_dynamic/propulsion/EngineManager.cpp:184`。
- EngineManager `GetDefaultApproachSpeedMps`：读了 property tree ρ 做校验（`:290`），但 V_stall 计算又用硬编码 `kRhoSeaLevel`（`:311`）——同一函数内 ρ 来源自相矛盾。

为何未决：narrow 重构契约要求零行为变化，且无测试因 ρ 漂移而失败。修它等于改变至少一个调用点的数值输出，必须有测试兜底。

推进需要：
- 确认正确的 ρ 来源应是哪个（property tree 的实时大气密度，还是固定海平面常数）；
- 补一个针对 ρ 来源的失败/边界测试（如高海拔场景下 V_stall 应随 ρ 变化）；
- 在测试兜底下统一 ρ 来源。

注：P2.4（commit `65cc7fc4`）把 CLmax + V_stall 公式收拢为单一 `AircraftPerformanceDerivation` helper，但 ρ 作为入参透传，严格保留了三处现状——漂移本身未修。

## OQ-5 "extension point" 抽象基类去留审查（四 seam cluster）

取消用户高度自定义后，"为扩展点而存在"的抽象需逐个审判。保留抽象基类只应满足至少一个条件：(1) 明确 public SPI；(2) 跨模块稳定能力接口（sink/provider）；(3) 内部算法族策略点（当前有多实现或运行时选择）；(4) 内部测试/故障注入 seam（验证回滚、失败路径、复杂状态机，且无更低成本替代）；(5) 解耦大型内部层的稳定 internal port。据此审查四个 `I*` 接口，结论：仅 `ISignalPipeline` 通过（标准 4），其余三个不过——但当前最危险的不是虚函数本身，而是 namespace、导出宏、文档仍暗示"可扩展"，与新 public boundary（`docs/electro_optical_sensor/design.md:400,404`）冲突。

| seam | 生产实现 | 测试 double | namespace | 导出宏 | 公开头 | 落点 |
| --- | --- | --- | --- | --- | --- | --- |
| AR `ISignalPipeline` | 1 (`SignalPipeline`) | 2（rollback+abort） | `extension::` | 无 | 否 | 标准 4：保留 |
| ESR `IEsrContext` | 1 (`MutableEsrContext`,`final`) | 0 | `extension::` | **`ONEQ_API`** | 否 | 全不满足 |
| EOS `IEosEnvironmentService` | 1（匿名 ns 一行 forwarder） | 0 | `environment` | 无 | 否 | 全不满足 |
| AR `IAssignmentSolver`/`IHypothesiser` | 各 1 | 0 | `signal::association` | 无 | 否 | 全不满足 |
| AR `IDistanceMetric` | 1（+1 prod 死 + 子接口） | 0（1 null 契约） | `signal::association` | 无 | 否 | 半满足，单独审 |

证据（逐个）：

(a) **AR `ISignalPipeline` — 保留，仅清 namespace/doc**：声明于 `src/airborne_radar/signal/pipeline/ISignalPipeline.h:38`，namespace `extension::`（`:23`），无导出宏，文件 doc（`:3`）自称"内部实现细节，不对外暴露"。唯一生产实现 `SignalPipeline`（`SignalPipeline.h:23`，`final`），在 `RadarSessionCompositionRoot.cpp:69-70` 与 `:91-92` 两处硬编码 `new`，无 factory/无运行时选择。被 `RadarController` 以 `ISignalPipeline&` 持有（`RadarController.h:64`）。**两个 test double 专测 rollback/abort**：`RecordingSignalPipeline`（`tests/contract/ar_public_api_convenience_test.cpp:324`，覆盖 `CaptureRuntimeState`/`RestoreRuntimeState` 回滚 `:367-396`、config 拒收 `:356-360`、`kRuntimePreparationFailed` 注入 `:333-337`）；`AbortingSignalPipeline`（`tests/unit/ar_core_controller_test.cpp:123`，rollback round-trip `:160-186`）。误导点：`CaptureRuntimeState`/`RestoreRuntimeState` 的 `@note`（`:105-107`、`:113-115`）描述"when injected into RadarSession/RadarController"，与文件 doc 的"内部"自相矛盾。

(b) **ESR `IEsrContext` — 误导信号最全，去 `ONEQ_API` + 移出 `extension::`**：声明于 `src/electronic_surveillance_radar/pipeline/IEsrContext.h:26`，`class ONEQ_API IEsrContext`（`ONEQ_API` 见 `include/1q/api.hpp:36`，即 dllexport），namespace `electronic_surveillance_radar::extension`。唯一实现 `MutableEsrContext`（`MutableEsrContext.h:17`，`final`，无导出宏——基类导出、实现不导出，自相矛盾）。栈上单点构造（`InterceptPipeline.cpp:72`），`const&` 透传（`:80`、`:82`），无 factory、无多态派发。**纯只读 context bag**：7 个 const getter（`IEsrContext.h:28-49`），写方法在接口外（`MutableEsrContext::BeginCycle`）。零 test double。无任何 `PUBLIC_HEADERS_*` 引用（`src/electronic_surveillance_radar/CMakeLists.txt:41-42` 的 `PUBLIC_HEADERS_ESR_EXTENSION` 为空），外部 consumer 测试 `tests/consumer/esr_extension_consumer.cpp` 不引用它。doc 自称"依赖倒置"（`:20-25`）。**三重信号**（dllexport + `extension::` + 依赖倒置）叠加撞 `design.md:400/404`。

(c) **EOS `IEosEnvironmentService` — 可 devirtualize**：声明于 `src/electro_optical_sensor/environment/IEosEnvironmentService.h:21`，单方法 `ResolveFactors`（`:30-31`）。唯一实现 `DefaultEosEnvironmentService` 在 `EosPipeline.cpp:43` 的匿名 namespace 内，是 `return ResolveEnvironmentFactors(inputs)` 的一行 forwarder。构造于 `EosPipeline.cpp:364-367`（无条件 `new`，DI 注入点已删），`shared_ptr<IEosEnvironmentService>` 成员（`EosPipeline.h:59`）只在 `EosPipeline.cpp:527` 解引用一次。零 test double（`tests/unit/eos_environment_model_test.cpp` 测底层 free function，不测接口）。commit `babd75bc` 已删"外部接管"doc 与 DI 注入参数，commit 自述保留接口理由是"合理的内部多态"——但该多态从不发生。

(d) **AR association 三联 — 拆两批**：均在 `src/airborne_radar/signal/association/`，namespace `airborne_radar::signal::association`（**非** `extension::`），无导出宏、无 public 头、无 factory/无 config 选择，唯 `DataAssociationEngine` 消费，`DataAssociation.cpp:121-130` 同一构造块硬连。
- `IAssignmentSolver`（`AssignmentSolver.h:19`）/`IHypothesiser`（`Hypothesiser.h:39`）：各 1 实现（`LapjvSolver.h:18`、`DenseCostHypothesiser.h:54`，均 `final`），consumer 以**具体类型**持有（`DataAssociation.h:228`、`:227`），接口类型在 consumer 处**从不出现**——零成本折叠。
- `IDistanceMetric`（`DistanceMetric.h:18`）：通过 `ICovarianceAwareDistanceMetric*` 派发（`DataAssociation.cpp:125`、`:137` → `Hypothesiser.cpp:9-11,38,75,122`）；prod 死实现 `MahalanobisDistanceMetric`（`DistanceMetric.h:33`，仅测试构造）；唯一活实现 `FullMahalanobisDistanceMetric`（`:75`）；子接口 `ICovarianceAwareDistanceMetric`（`:60`，doc `:54-59` 自述"消除 dynamic_cast"）；`ar_signal_association_test.cpp:537` 有 `static_cast<IDistanceMetric*>(nullptr)` null 契约测试。**真有算法族影子但无运行时选择**，单独审。

为何未决：审查为静态代码分析，未执行任何重构。判断标准本身尚未进 contract.md，是提案级框架（已对照代码验证）。`IDistanceMetric` 的 `ICovarianceAwareDistanceMetric` 子接口去留是真未决项（它只为消除一次 dynamic_cast 而存）。对四个"保留/降级"的结论是推荐，需各模块 owner 确认无计划内第二实现。

推进需要（两阶段，对齐"先删语义、再审单实现"）：
- **Phase 1（语义/导出清理，并行低风险）**：① ESR `IEsrContext` 去 `ONEQ_API` + 移出 `extension::` + doc 去依赖倒置措辞——优先级最高，因唯一物理导出且撞 public boundary；② AR `ISignalPipeline` 移出 `extension::` → internal port 语义，重写 `@note` 为 rollback 单测 seam，**保留接口**；③ association 三联 doc 去暗示（namespace 已正确，仅措辞）。
- **Phase 2（去虚化审查，逐个定，不打包）**：① EOS `IEosEnvironmentService` → devirtualize（`EosPipeline` 直接持值成员或调 free function，`babd75bc` 已铺路）；② ESR `IEsrContext` → devirtualize（塌缩为具体只读 struct）；③ `IAssignmentSolver`+`IHypothesiser` → 折叠（consumer 已用具体类型）；④ `IDistanceMetric` → 单独审，倾向连 prod 死 `MahalanobisDistanceMetric` 一起清，`ICovarianceAwareDistanceMetric` 改 concrete 持有；⑤ `ISignalPipeline` 不动。

注：本审查基于代码静态分析，未触碰任何代码；`ISignalPipeline` 的 rollback/abort test double 是其保留的核心证据，`IEsrContext` 的 `ONEQ_API` 是当前唯一编译进 ABI 的误导信号。
