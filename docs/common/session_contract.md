---
Status: active
Last-reviewed: 2026-08-03
Authority: 有 Session 的传感器模块的统一会话契约
Answers: SessionConfigBuilder、Session 组合所有权、运行期配置提交策略、电源单源、三层输出模型、Replay/trace 语义
---

# 会话相关模块契约

本文承载"有 `*Session` 会话模型的传感器模块"（AR/ESR/EOS/SAR/SBIRS）的统一契约规则。这些规则不是
"所有模块必须遵守"（`flight_dynamic` 无会话模型，不适用），而是"有会话的模块必须对齐"。
所有模块都必须遵守的跨模块契约见 `docs/common/contract.md`。

## SessionConfigBuilder

所有 `*SessionConfigBuilder` 都是**薄封装**：

1. 内部只持有 `*SessionConfig` 副本；`Build()` 直接返回该副本，不做任何翻译、合并或覆写。
2. 语义档位（profile）是各模块 `XxxProfileConstants.h` 中的预定义结构体常量
   （如 `profiles::kLongRangeHighPowerHardware`、`profiles::kThreatWarningMission`），
   用户直接整域赋值；常量只含该档位管理的字段，其余字段保持 struct 默认值。
3. 配置中不存在隐式优先级："对 config 的任何赋值即最终决定"，档位在前、微调在后时微调胜出。
   不得以任何形式复活 dirty flag / Profile 枚举 / 隐式覆写机制。
4. 细粒度工程参数由调用方直接编辑 `*SessionConfig` 四域字段。
5. 运行期变更走 `*RuntimeConfigPatch` / `*RuntimeConfigBuilder`。
6. 配置合法性由独立 validator 检查最终 config。

不得重新引入 leaf setter，例如 frame rate、scene center、minimum SNR、atmospheric loss 这类直接字段编辑器。
struct 默认值即语义默认（no-op 档位不提供常量）；整域赋值会重置子域内未被常量管理的字段，调用方应先赋档位再设场景特定数据。

## Session composition ownership

AR/EOS/ESR/SAR/SBIRS 的 `Session::Impl` 是 session 依赖图的所有权边界。组合根可以在装配过程中使用
raw pointer 回填组件关系，但 `Impl` 长期持有状态不得同时保存 `std::unique_ptr<X>` 与同一对象的
`X&` 成员。`Impl` 应只保存 owning member，并在使用点通过 accessor 或局部引用派生依赖引用。

规则：

1. `Session` public move 语义由外层 `std::unique_ptr<Impl>` 承担；不得让 `Impl` 内部的冗余引用成为移动/所有权重构的隐藏前提。
2. 组合根创建的默认 controller、pipeline、context、environment service 由对应 session 唯一拥有。
3. AR 唯一 public 决策 seam 是 `ArCycleResult::decision_observation` 与
   `ArSession::SubmitExternalDecision()` 组成的步间 observation/response seam。外部决策模块
   与 session 同进程运行，但不注入或替换内部对象；内部 baseline 每个成功周期仍持续计算。
   Public seam 只公开 profile 覆盖值、observation、提交状态和控制来源；默认决策器的
   分类结果、模式与状态存储不得进入 `include/1q`。
   [evidence: tests/contract/airborne_radar/ar_public_api_convenience_test]
4. 若未来新增非 owned 依赖，必须先在模块 design 或本契约声明其生命周期边界，不能通过 `Impl` 冗余引用隐式表达。

## 运行期配置提交策略

`*RuntimeConfigPatch` 的提交（commit）与周期内失败回滚（rollback）按 pipeline 的状态空间复杂度分两类。每个 `*Session` 必须显式归属其中一类，且实际行为不得与所属类的承诺冲突。五个传感器模块当前归属固定如下——这是对已实现行为的契约化，不是行为变更要求。

| 类别 | 承诺 | 归属模块 |
|---|---|---|
| **事务性提交** | patch 经 resolver 校验并延迟到下个发射边界；实际发射发布前的 commit/执行失败完整恢复，发布后则提交配置与发射事实，只恢复接收、检测和跟踪候选状态。 | `airborne_radar` |
| **立即提交** | patch 经 resolver 校验；`TryApplyRuntimeConfig` 调用即生效，配置单向落定、不在 session 层回滚。若 pipeline 持有累积状态且执行可能失败，回滚边界由该模块在内部层（如 controller）声明，不上升为 session 层契约。 | `electronic_surveillance_radar`、`electro_optical_sensor`、`sar`、`sbirs_sensor` |

AR 仍以单周期 `StepWithResult()` 作为公共接口。其内部先提交实际发射事实，再完成接收、检测和跟踪；
发射提交后，后续接收侧失败不得撤销 waveform/phase/ID 状态。AR replay 与 runtime patch 必须保存该
内部提交边界；调用方不管理 prepare/complete 阶段。

归属判定规则与各模块细节：

1. **归属由状态空间决定，不由风格偏好决定。** 仅当 pipeline 同时满足"有跨周期累积状态"且"commit/执行存在真实失败路径"时，才采用事务性提交。两者缺一即为立即提交。
   - `airborne_radar`：4 个子系统各有独立 runtime state，`UpdateConfig`/`UpdateExecutionConfig` 可返回 false，故发射前需要事务对齐；`PrepareRfCycle` 成功后以 post-emission 快照恢复接收候选，并保留 runtime config、emission ID、时间线、跳频/PRI 相位和已消费控制。
   - `electronic_surveillance_radar`：config 无累积（每 RunCycle 重新派生），`UpdateConfig` 走换 config 留 tracks；`InterceptPipelineResult` 只承载去真值化 observation、emitter 两个业务输出及执行 metadata，当前没有其它 pipeline 执行失败 commit 路径。
   - `electro_optical_sensor`：执行回滚封装在 `EosController::RunOnce`，不上升为 session 层事务。
   - `sar`：runtime config 立即提交，但 pipeline 持有 pulse ring、轨迹缓冲、pulse ID 与 PRF 分数余量。`SarController::RunOnce` 在 pipeline 执行前捕获这些状态，任一执行 abort 后完整恢复；配置本身不随执行失败回滚，故仍归属立即提交类。
   - `sbirs_sensor`：pipeline 持有跨周期目标状态机，但 `RunCycle` 返回 record/attribution 原子元素，controller 输出装配后没有可能失败的 commit 步骤，因此不激活周期回滚。pipeline 的 capture/restore 是经 mutation 前完整校验的 internal checkpoint，用于确定性 continuation 与状态恢复测试，不在 session 层暴露事务语义。归属立即提交类。
2. **所有五个传感器模块的 patch 必须经 resolver 校验**（`is_valid`/`has_requested_update`），不得盲写。
3. **立即提交类不得声称 session 层回滚。** 若其内部存在 capture/restore 能力（如 ESR 的累积状态快照），必须在代码 doc 注明该机制的实际边界，避免阅读者误以为 session 层提供配置回滚或已激活的执行失败回滚。
4. **事务性提交类不得在配置边界被接受前落定配置语义状态。** 配置的"逻辑当前值"
   （如 AR 的 `runtime_state`）与"已推送到子系统的物理状态"必须在对齐点之后才一致。
   AR 设备关机是结构化的非执行边界（`kSensorPoweredOff`），不是 pipeline 故障：session
   必须撤销该周期对控制/环境状态的消费，保留待应用的外部决策，同时完成已验证关机配置的
   物理对齐和 pending finalize；其余执行 abort 仍保持 staged + rollback。

## 电源状态单源契约

AR/ESR/EOS/SBIRS 四模块的电源状态必须遵守单源原则：

1. `*SessionConfig` 顶层 `sensor_enabled` 是会话初始电源状态的**唯一来源**；
   `*MissionConfig` 不含电源字段（`mission.power_on` 已整体移除）。
2. `*RuntimeConfigPatch::has_sensor_enabled` / `sensor_enabled` 是运行时电源变更的**唯一入口**
   （SBIRS 已从 `has_power_on`/`WithPowerOn` 统一对齐）；`has_mission` 整块域不影响电源。
3. 运行时补丁解析顺序（整块先、叶子后）仅约束几何/模式字段（scan_center、work_mode 等），
   不产生电源状态的二重路径。违反本契约的字段名/映射（`power_on`、`has_power_on`、
   `WithPowerOn` 回流）由 `tests/contract/check_cross_domain_naming.cmake` 阻断 7 硬性守护。
4. SAR 例外保持：其补丁仅含处理开关，无电源域，不受本契约约束。

## 三层输出模型

所有传感器/产品模块遵守三层输出模型：

| 层级 | 入口 | 责任 |
|---|---|---|
| 原始系统输出层 | `Step()` 返回的 `*OutputFrame` | 真实传感器或产品输出 |
| 结构化执行结果层 | `StepWithResult()` 返回的 `*CycleResult` | 输出帧、执行状态、校验、abort reason 和诊断摘要 |
| 开发调试视图层 | `*OutputDebugViewBuilder` / `*LifecycleRecorder` | 人读状态、生命周期事件、输入实体回填 |

开发调试视图层中，`*LifecycleRecorder` 是转换检测状态机（非数据存储）：累积状态刻意最小化为每实体
1-bit 存在标志（已确认/已检测/已有产品），事件富信息在每次 `Update()` 时从 L2 实时转发，不在内部累积。
`*OutputDebugViewBuilder` 与 `*LifecycleRecorder` 在 L3 内并列，互不消费——前者是无状态快照构造器，
后者是有状态跨周期状态机。

规则：

1. `Step()` 只返回主系统输出帧。
2. `StepWithResult()` 是状态判断入口。
3. 日志只用于人读运行信息，状态判断不得依赖解析日志文本。
4. 调用方主动注入且被结构化结果正确拒绝的无效输入属于可预期边界，不记 error；批量验证中只有
   "未按预期拒绝"或拒绝后状态发生污染才记 error 并使验证失败。
5. 数值 ID 是稳定关联键，名称只用于人读、trace/replay、报告和调试视图。
6. `external_target_id` 与模块实体 ID 当前都允许 `0` 作为合法值；`0` 不得触发
   validation error。若未来引入可表达负数的外部输入入口，负数 ID 必须在转换为
   public `std::uint64_t` DTO 前被拒绝。
7. 仿真真值不得混入面向外部系统的真实输出通道。
8. `*CycleResult` 的输出帧、指标和诊断产品仅在 `status == kCompleted`（或等价的
   `executed_this_cycle=true`）时代表本周期的有效计算结果；非执行周期返回默认空帧
   （`cycle_index=0`、空载荷），不复用上一有效输出，不得按真实零值参与统计。
   `reused_previous_output` 概念已废除。
9. 所有中止路径（`abort_reason` 非 `kNone`）必须执行三写：
   a. **结构化信号**：设置 `abort_reason`（粗粒度枚举，~6 值，与模块对齐）。
   b. **结构化诊断**：写入 `*DiagnosticIssueList`（`severity` + `code` + `message`），
      细粒度 code 带模块前缀（如 `"sar.snr_below_minimum"`）。
   c. **人读日志**：`PROJECT_LOG_ERROR` 或 `PROJECT_LOG_WARN`。
   三写缺一不可。SAR 为参考实现（`SarDiagnosticUtils::WriteAbort`）。
   `ValidationIssueList` 承载输入校验的结构化问题（severity/code/location/field/message），
   与 `*DiagnosticIssueList` 职责不同，不得混用。
10. `*LifecycleRecorder` 有两种驱动方式，二选一，不得混用：
    a. **手动驱动**：调用方在每个执行周期自行调用 `Update()`；漏调会导致状态机失步
       （例如错过产品/目标消失的周期后，内部 1-bit 标志仍为"存在"，后续不再发出 `Lost` 事件）。
    b. **自动驱动**：调用方通过 `*Session::Attach*LifecycleRecorder()` 注册记录器，
       Session 在 `StepWithResult()`/`Step()` 内部自动调用 `Update()`，调用方无需手动调用。
    无论哪种方式，非执行周期都返回空事件列表且不推进内部状态。
11. `*Session::Attach*LifecycleRecorder()` 是可选注册契约：
    a. Session 持有 recorder 的**非拥有裸指针**，调用方须保证 recorder 生命周期长于注册期；
       传入 `nullptr` 解除注册，解除后 Session 不再驱动。
    b. Session 不缓存事件；本周期产生的生命周期事件通过 `recorder->GetLastEvents()` 获取
       （recorder 记住最近一次 `Update()` 的结果）。
    c. 注册与否不影响 `Step()`/`StepWithResult()` 的返回值和执行语义——纯观测工具，零行为改变。

### 执行状态信号统一

五个传感器模块的 `*CycleResult` 统一包含强类型 `*CycleStatus` 枚举，表达单周期高层执行状态：

| 模块 | 枚举类型 | 典型值 |
|---|---|---|
| AR | `ArCycleStatus` | `kCompleted`, `kPoweredOff`, `kRejectedInvalidInput`, `kRejectedInvalidConfig`, `kRejectedExecution` |
| ESR | `EsrCycleExecutionStatus` | `kCompleted`, `kRejected`, `kPoweredOff` |
| EOS | `EosCycleStatus` | `kCompleted`, `kPoweredOff`, `kRejectedInvalidInput`, `kRejectedExecution` |
| SAR | `SarCycleStatus` | `kCompleted`, `kRejectedInvalidInput`, `kRejectedExecution`（细粒度失败信息由 `SarDiagnosticIssue::code` + 日志双写） |
| SBIRS | `SbirsCycleStatus` | `kCompleted`, `kPoweredOff`, `kRejectedInvalidInput`, `kRejectedExecution` |

`executed_this_cycle` 保留为 `status == kCompleted` 的便捷访问器（向后兼容）。
`abort_reason` 是强类型枚举（SAR 为 `SarPipelineAbortReason`，其余模块类似），提供更细粒度的终止原因。
状态判断应优先使用 `status` 枚举，`executed_this_cycle` 仅用于简单 bool 门控。

### Attribution（仿真真值归属）层级

仿真真值归属（detection → simulation target 映射）在各模块的层级位置不同，这是有意设计：

| 模块 | 归属位置 | 理由 |
|---|---|---|
| AR | L1（`TrackStateSnapshot` 内嵌 `external_target_id`/`target_name`） | track 是系统级估计，关联键是 track 语义的一部分 |
| EOS | L2（`EosCycleResult.detection_attributions`） | detection 是原始传感器输出，归属是仿真附加信息 |
| SBIRS | L2（`SbirsCycleResult.detection_attributions`） | 同 EOS |
| ESR | 无归属数据 | ESR 输出是观测+假设，不含检测到目标的映射 |
| SAR | 无归属数据 | SAR 输出是图像产品，不含检测到目标的映射 |

规则：
1. **L1 不含仿真真值归属**（EOS/SBIRS 已遵守；AR 的 track 关联键是特例，不适用于检测型传感器）。
2. **L2 允许承载归属**——归属是结构化数据（非人读），replay/trace 需要消费。
3. **L3（DebugView/LifecycleRecorder）通过 L2 访问归属**，不是归属的唯一载体。
4. EOS/SBIRS 的 `*CycleOutputAdapter` 守卫（如 `SbirsOutputFrameContainsOnlyNativeFields()`）
   确保 L1 不含归属泄漏。

## Replay 与 trace 语义

1. 模块级 cycle-output 回调必须用 `ReplayTraceOutputStatus` 结构化表达比较结果。
   `kDivergence` 是输出分叉；`kOtherFailure` 是解码、类型或执行失败，不能被标记为
   分叉；不得通过解析人读 error 文本推断状态。
2. `ReplayTracePlaybackResult.divergence_found` 是分叉的权威结构化信号。
   `kHandledByModule` 和 `kDivergence` 均计入 `compared_output_count`，
   `kOtherFailure` 不计入且不改变分叉状态；两条路径必须遵守同一
   `stop_on_first_divergence` 语义。
3. `TraceSink` 是调试/观测记录格式，不能作为 `ReplayXxxTrace()` 输入。
   需要可回放目录时必须使用 `ReplayTraceWriter` 并经对应
   `*TraceSessionOptions::replay_writer` 传入；两者可同时配置，但不能相互替代。
4. `ReplayTraceWriter(overwrite=false)` 必须对既有 replay 工件 fail closed：不得重写
   manifest、追加事件或重置 sequence/hash chain。续写已有 trace 需要独立、显式且能恢复
   sequence、hash、chunk 与 index 状态的 resume 契约，不能复用普通创建入口隐式实现。
5. Replay I/O 不得把错误伪装成正常结束。Reader 调用方必须可通过
   `ReplayTraceReadStatus` 区分事件、正常 trace 末尾与读取错误；Writer 调用方必须可通过
   `ReplayTraceWriteStatus` 和 `first_error()` 观察初始化、写入及刷新失败。扫描和回放遇到
   缺失 manifest、缺失首 chunk 或底层读取错误时必须 fail closed。
6. 模块 FlatBuffers codec 只共享无 schema 知识的机械基元：已完成 builder 的字节复制，以及
   字段布局一致的 `FailureMarker` 空值、空 payload、verifier 和共有字段解码保护。schema、DTO
   映射、payload identifier、模块错误文本、外部数据资格与 divergence 行为继续由模块拥有；
   不得把公共 helper 扩张为万能 codec 或跨模块对象图。
7. runtime patch trace 必须记录实际应用结果，不能只记录请求。replay 应重新应用 patch 并比较结构化
   status、是否包含请求以及是否提交；合法拒绝和空补丁是可回放事件，不得被强制解释为成功。输入配置、
   patch 和输出中以整数存储的 enum 必须逐值校验；未知值应原子拒绝且不得部分修改解码目标。

[evidence: tests/replay/sbirs_sensor/sbirs_replay_codec_roundtrip_test]
[evidence: tests/replay/sar/sar_replay_codec_roundtrip_test]
[evidence: tests/replay/airborne_radar/ar_replay_codec_roundtrip_test]
[evidence: tests/replay/electronic_surveillance_radar/esr_replay_codec_roundtrip_test]
[evidence: tests/replay/electronic_surveillance_radar/esr_replay_session_test]
