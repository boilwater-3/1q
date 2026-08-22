---
Status: active
Last-reviewed: 2026-08-23
Authority: 有 Session 的传感器模块的统一会话契约
Answers: 会话配置直接赋值、Session 组合所有权、运行期配置提交策略、电源单源、分层周期记录+可选投影、可观测性单源、Replay 持久化语义、条件五域 orientation
---

# 会话相关模块契约

本文承载"有 `*Session` 会话模型的传感器模块"（AR/ESR/EOS/SAR/SBIRS/RIR）的统一契约规则。这些规则不是
"所有模块必须遵守"（`flight_dynamic` 无会话模型，不适用），而是"有会话的模块必须对齐"。
RIR 于 2026-08 并入本契约范围（会话门面/条件五域配置/电源单源/统一问题列表/执行状态信号已对齐，
会话校验入口与 AR 同为 session 层）。配置域形状见 `docs/common/contract.md`「条件五域配置所有权」
（SBIRS/AR/ESR/RIR 五域；EOS/SAR 四域）。所有模块都必须遵守的跨模块契约见
`docs/common/contract.md`。

**输出模型口径（2026-08-23）**：正文使用「分层周期记录 + 可选投影」（规则 15）。
「两通道 / 信封嵌产品副本 / 三层 / L1 / L2 / L3」为历史别名，嵌副本口径已废止。
齐套标准是产品形态表（见「可观测性单源」），不是「有没有某条并行通道」。
目标列表型模块（AR/EOS/SBIRS/RIR）的三类观测投影为观测完备必选项；RIR 字段与挂载见
`docs/review/rir_observability_projections_freeze_2026-08-21.md`。
库内周期落盘只允许 Replay；禁止再引入不能被 `ReplayXxxTrace()` 消费的记录文件。
规则 15 于 2026-08-23 冻结，**代码与 schema 尚未落地**——当前扁平 `*CycleResult` 不得解读为已实现。

## 会话配置直接赋值

`*SessionConfig` / `*RuntimeConfigPatch` 由调用方直接构造与字段赋值；**不得**再提供
`*SessionConfigBuilder` / `*RuntimeConfigBuilder` 公开 API。

1. 语义档位（profile）是各模块 `XxxProfileConstants.h` 中的预定义结构体常量
   （如 `profiles::kLongRangeHighPowerHardware`、`profiles::kThreatWarningMission`），
   用户直接整域赋值；常量只含该档位管理的字段，其余字段保持 struct 默认值。
2. 配置中不存在隐式优先级："对 config 的任何赋值即最终决定"，档位在前、微调在后时微调胜出。
   不得以任何形式复活 dirty flag / Profile 枚举 / 隐式覆写机制 / leaf setter Builder。
3. 细粒度工程参数由调用方直接编辑 `*SessionConfig` 配置域字段（条件五域或四域，见
   `contract.md`）。
4. 运行期变更只写 `*RuntimeConfigPatch`：每个生效字段必须显式设置对应 `has_*`
   （嵌套域再设子级 `has_*`，如 `environment.has_scenario_config`）。
   有 orientation 域的模块：该域为初始化静态配置，**不得**进入 RuntimeConfigPatch
   （不得新增 `has_orientation`；`has_mission` 不得覆写静态安装角/限位/稳定/失准）。
5. 配置合法性由独立 validator 检查最终 config。

不得重新引入 leaf setter 建造者，例如 frame rate、scene center、minimum SNR、atmospheric loss 这类
链式字段编辑器。struct 默认值即语义默认（no-op 档位不提供常量）；整域赋值会重置子域内未被常量管理的字段，
调用方应先赋档位再设场景特定数据。

## Session composition ownership

AR/EOS/ESR/SAR/SBIRS/RIR 的 `Session::Impl` 是 session 依赖图的所有权边界。组合根可以在装配过程中使用
raw pointer 回填组件关系，但 `Impl` 长期持有状态不得同时保存 `std::unique_ptr<X>` 与同一对象的
`X&` 成员。`Impl` 应只保存 owning member，并在使用点通过 accessor 或局部引用派生依赖引用。

规则：

1. `Session` public move 语义由外层 `std::unique_ptr<Impl>` 承担；不得让 `Impl` 内部的冗余引用成为移动/所有权重构的隐藏前提。
2. 组合根创建的默认 controller、pipeline、context、environment service 由对应 session 唯一拥有。
3. AR 内部闭环仍是「分析本拍特征 → 改下一拍发射控制」。
   - 每成功周期：`TacticalCoordinator` 做威胁分类与 LPI/ECCM 提案，`ControlReducer` 收成
     `ArControlProfile`，下一成功发射时消费。
   - 唯一 public 决策缝是 `ArSession::SubmitExternalDecision()`（整包替换下一拍 profile）。
   - 外部模块不注入或替换内部对象；分类与内部 baseline 每个成功周期仍持续计算。
   - `DecisionObservation` / `DecisionInputFrame` 不得作为周期记录字段（规则 15f）。
   - Public 只公开 profile 覆盖值、提交状态和控制来源 provenance；默认决策器的分类结果、
     模式与状态存储不得进入 `include/1q`。
   [evidence: src/airborne_radar/decision/TacticalCoordinator.cpp]
   [evidence: src/airborne_radar/runtime/ArController.cpp]
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

AR/ESR/EOS/SBIRS/SAR/RIR 六模块的电源状态必须遵守单源原则：

1. `*SessionConfig` 顶层 `sensor_enabled` 是会话初始电源状态的**唯一来源**；
   `*MissionConfig` 不含电源字段（`mission.power_on` 已整体移除）。
2. `*RuntimeConfigPatch::has_sensor_enabled` / `sensor_enabled` 是运行时电源变更的**唯一入口**
   （SBIRS 已从 `has_power_on`/`WithPowerOn` 统一对齐；SAR 于 COMMON-OQ-4 收敛补齐；RIR 建模即按
   本契约，无历史双轨）；
   `has_mission` 整块域不影响电源。
3. 运行时补丁解析顺序（整块先、叶子后）仅约束几何/模式字段（scan_center、work_mode 等），
   不产生电源状态的二重路径。违反本契约的字段名/映射（`power_on`、`has_power_on`、
   `WithPowerOn` 回流）由 `tests/contract/check_cross_domain_naming.cmake` 阻断 7 硬性守护。

## 分层周期记录 + 可选投影

所有有 Session 的传感器/产品模块遵守本模型。齐套标准是**一次 `StepWithResult()` 生产一份分层周期记录**；
观测投影按产品形态选装。不得用「有没有旧称 L3」单独判定模块是否完整。
「信封嵌有产品帧副本」口径已废止，详见规则 15。

**实现状态（2026-08-23）**：规则已冻结，代码与 Replay schema **未落地**。当前头文件仍为扁平
`*CycleResult`（内嵌 `output_frame`、SAR 图像与 IQ 同袋、AR `decision_observation`）。
不得把当前字段布局解读为本节已实现。

### 生产者与分层

| 入口 | 责任 |
|---|---|
| `StepWithResult()` | 唯一真正的 Step：生产一份分层周期记录 |
| `Step()` | 糖：只返回产品层（移动，不是第二份副本） |

| 层 | 问的问题 | 放什么 | 非执行周期 |
|---|---|---|---|
| 执行 | 这拍跑没跑成、为什么 | `cycle_index` + `status` + `abort_reason` + `issues` | 照样填 |
| 产品 | 真实下游会拿到什么 | 航迹 / 探测 / 成像结果（SAR 含聚焦图像） | 空载荷 |
| 副作用 | 即使没产品，物理上已发生什么 | AR/RIR `emission_frame` | 可以有（发射已提交、接收失败） |
| 仿真附件 | 库外系统看不到的对照 | 归属表 | 空 |
| 模块闭环 | 传感器自己的下一拍控制 | AR/RIR 指定回退、控制 provenance；不含决策观测袋 | 按模块 |

EOS / SBIRS / ESR 没有的层放空结构，不得为齐套假造。公开头保持 C++11：空对象表示缺席。

### 可选投影（旧称 L3 / 开发调试视图层）

| 投影 | 入口 | 责任 |
|---|---|---|
| DebugView | `*OutputDebugViewBuilder` | 帧作用域人读快照、输入实体回填 |
| 生命周期 | `*LifecycleRecorder` | 跨周期确认/更新/丢失等事件 |
| 排除差分 | `*ExclusionCauseRecorder` | 跨周期排除原因 `(code,cause)` 差分事件 |

投影按产品形态选装，完整齐套面（分层周期记录 / 三写 / 投影 / Replay）以「可观测性单源」产品形态表为准，
不得在该表之外点菜增减通道。

| 产品形态 | 模块 | 三类投影要求 |
|---|---|---|
| 目标列表型 | AR / EOS / SBIRS / RIR | **必选**（观测完备）；RIR 已按冻结文档落地（2026-08-22，见 `docs/review/rir_observability_projections_freeze_2026-08-21.md`） |
| 非目标列表传感器 | ESR | 仅 ExclusionCause（发射源身份键）；不提供目标 DebugView / Lifecycle |
| 成像传感器 | SAR | ProductDebugView + ProductLifecycle；ExclusionCause 与规则 13b/13e 为空洞条款 |

观测投影中，`*LifecycleRecorder` 是转换检测状态机（非数据存储）：累积状态刻意最小化为每实体
1-bit 存在标志（已确认/已检测/已有产品），事件富信息在每次 `Update()` 时从周期记录实时转发，
不在内部累积。`*OutputDebugViewBuilder` 与 `*LifecycleRecorder` / `*ExclusionCauseRecorder`
并列、互不消费——前者是无状态快照构造器，后两者是有状态跨周期状态机。
`*OutputDebugViewBuilder` 对无探测/无轨迹目标回填**输入实体量值**（EOS 的方位/俯仰/距离、
SBIRS 的方位/俯仰（卫星→目标视线 ECI 极坐标，弧度）、AR 的 RCS；RIR 为滤波 ENU
位置/速度，无航迹时回填输入斜距与视线角几何，见 RIR 观测投影冻结文档），与检测/航迹记录同参考系；有产品记录时
以记录观测值覆盖——未检测也可见目标量值，供调用方人读/结构化落盘（规则 12）。

规则：

1. `StepWithResult()` 是唯一真正的 Step，生产一份分层周期记录（规则 15）。
2. `Step()` 只返回产品层；不得再声称周期记录嵌有产品副本。
3. 日志只用于人读运行信息，状态判断不得依赖解析日志文本。
4. 调用方主动注入且被结构化结果正确拒绝的无效输入属于可预期边界，不记 error；批量验证中只有
   "未按预期拒绝"或拒绝后状态发生污染才记 error 并使验证失败。
5. 数值 ID 是稳定关联键，名称只用于人读、replay、报告和调试视图。
6. `external_target_id` 与模块实体 ID 当前都允许 `0` 作为合法值；`0` 不得触发
   validation error。若未来引入可表达负数的外部输入入口，负数 ID 必须在转换为
   public `std::uint64_t` DTO 前被拒绝。
7. 仿真真值不得混入面向外部系统的真实输出通道。
8. 周期号只活在执行层，永远等于本次输入号（成功与非执行相同）。产品层仅在
   `status == kCompleted` 时代表本周期有效计算结果；非执行周期产品为空载荷，
   不复用上一有效输出，不得按真实零值参与统计，也不得用产品帧 `cycle_index=0`
   表示「没发生」。`reused_previous_output` 概念已废除。
   （落地前现状：多数模块仍同时带 `input_cycle_index` 与内嵌 `output_frame.cycle_index`，
   失败路径后者为 0——这是待迁移实现，不是本规则的目标形态。）
9. 所有中止路径（`abort_reason` 非 `kNone`）必须执行三写：
   a. **结构化信号**：设置 `abort_reason`（粗粒度枚举，~6 值，与模块对齐）。
   b. **结构化诊断**：写入唯一问题列表 `*IssueList`（统一问题列表模型，规则 14），
      细粒度 code 带模块前缀（如 `"sar.snr_below_minimum"`、
      `"esr.validation.invalid_emitter_frequency"`）。
   c. **人读日志**：`PROJECT_LOG_ERROR` 或 `PROJECT_LOG_WARN`。
   三写缺一不可。SAR 为参考实现（`SarDiagnosticUtils::WriteAbort`）。
10. 跨周期观测记录器（`*LifecycleRecorder` 轨迹/探测生命周期、`*ExclusionCauseRecorder`
    排除原因差分；统称 recorder）由各 `*Session::Attach*()` 注册后自动驱动：
    Session 在 `StepWithResult()`/`Step()` 内部自动调用 `Update()`，调用方无需（也不应）手动调用
    `Update()`——漏调会导致状态机失步（例如错过产品/目标消失的周期后，内部 1-bit 标志仍为"存在"，
    后续不再发出 `Lost` 事件）。`Update()` 保留为 public 仅为单元测试与内部驱动可达性，
    不属于调用方接口。非执行周期返回空事件列表且不推进内部状态。
    一个 Session 可同时注册多个 recorder（各自独立 `Attach*`/驱动/`GetLastEvents()` 通道），
    注册与否互不影响、不影响执行语义。
11. `*Session::Attach*()` recorder 注册契约（`Attach*LifecycleRecorder`/`AttachExclusionCauseRecorder`）：
    a. Session 持有 recorder 的**非拥有裸指针**，调用方须保证 recorder 生命周期长于注册期；
       传入 `nullptr` 解除注册，解除后 Session 不再驱动。
    b. Session 不缓存事件；本周期产生的生命周期事件通过 `recorder->GetLastEvents()` 获取
       （recorder 记住最近一次 `Update()` 的结果）。
    c. 注册与否不影响 `Step()`/`StepWithResult()` 的返回值和执行语义——纯观测工具，零行为改变。
12. 跨周期观测由调用方负责：`*OutputDebugViewBuilder` 是帧作用域快照构造器，本库不提供跨周期
    状态查询接口；"到目前为止"的累积信息由调用方将每周期 DebugView 以结构化格式（如 JSON/
    FlatBuffers）写入自己的日志/事件系统获得。
    - 规则 3 的"状态判断不得依赖日志文本"约束对象是模块内部代码，不限制调用方对其日志系统的
      使用，但调用方应结构化落盘，避免文本解析。
    - 结构化格式与字段布局由调用方自定（可参考 `*OutputDebugView` 的字段集合直接转写）；
      组件化集成示范见 `examples/component_attachment`（各传感器组件每周期取 `LastDebugView()`
      直写中文人读行到集成端日志（事件行 / 视图行分两个文件）——日志给人读，落盘密度三模式（非标称行/跨周期增量/每周期
      摘要）由 `logger/logger_modes.h` 模式选择区宏门控；结构化持久化由外部集成方接入自己的
      日志/事件系统，示例不再内置 JSON 序列化器）。
13. 正常执行周期（`status == kCompleted`）的可观测性：
    a. **周期级执行摘要日志**：正常执行周期应输出周期级 `PROJECT_LOG_INFO` 摘要，格式基线
       `[XxxPipeline] cycle_index={} …`（模块自定附加字段，如扫描方位、检测数/目标数、排除计数），
       仅用于人读运行信息（规则 3）。
       对齐参考实现：ESR（`[InterceptPipeline]`）与 EOS（`[EosPipeline]`）为既有参考；
       SBIRS（`[SbirsPipeline]`）为首个按本规则对齐实现；AR（`[SignalPipeline]`）与 SAR
       （`[SarPipeline]`）于 2026-08 对齐。
    b. **按目标门控排除诊断**：正常执行周期中目标被门控排除（视场/SNR/距离/遮挡/几何等）应写
       `kInfo` 级 `*IssueList` 条目，code 带模块前缀（如 `"sbirs.target_out_of_wfov"`），
       message 携带目标标识（`target_id`；ESR 无目标概念，以发射源标识
       platform/equipment/emission id 为载体）与关键量值。
       - **不属于三写**（三写仅约束中止路径，规则 9），仅承载排查信息；调用方按规则 12 落盘
         DebugView 时自然携带。
       - **参考实现**：SAR（`SarDiagnosticUtils::MakeInfoDiagnostic`/`MakeWarningDiagnostic`）
         + SBIRS/AR/ESR/EOS（2026-08 对齐；ESR 以发射源标识为载体）。
       - **message 稳定性**：message 为人类可读文本，内容与格式**不承诺解析稳定性**——
         机器消费只认 code；量值（如 `range_m`/`snr`/方位角）如需程序化消费，应另行定义
         结构化字段，不得解析 message。
       - **门内归因（cause 字段）**：聚合门排除时必须携带机器可读主因——`cause` 结构化字段
         （各模块 `<Module>IssueCause` 枚举，默认 `kNone`），标识导致门失败的物理链路主因。
         "聚合门"指单一门限折入多种物理因素（如 AR 的 SNR 检测门：距离/方向图偏轴/噪声底/
         RCS 全部折入 `snr < min_snr_db`）；主因 = 对该项取理想值后门余量增益最大者（如
         `ArIssueCause::kDistanceLimited`）。具体门（遮挡/距离带/视场外等本身可定位的门）
         不强制细分，`cause` 保持 `kNone`，关键量值进 message 或结构化字段。
         `cause` 不替代 `code`（code 仍是唯一机器键，规则 13b message 稳定性），不改变状态
         语义（规则 13c）。本条目为**所有模块共同约束**：五模块 `*Issue` 结构同构携带该字段，
         SAR 无排除诊断（13b 空洞条款），恒 `kNone`（见 `docs/sar/boundaries.md`）。
       - **实体机器可读关联（location）**：排除诊断须结构化携带场景实体索引
         （`location.kind = kSceneEntity + entity_index`；ESR 以发射源标识为键，见各模块
         boundaries），作为机器可读实体关联——供跨周期差分记录器（规则 13e）
         按"该排除属于哪个实体"判定，不依赖解析 message 人读文本中的 target_id。类型层
         （`ValidationLocation`/`kSceneEntity`/`entity_index`）与 replay 编解码（含 -1 哨兵
         还原）为共享基础设施。
    c. **状态语义边界**：kInfo 排除诊断不得改变 `*CycleStatus`、生命周期事件或 DebugView 状态
       语义（如 `kNotInOutput`）；排除原因只经 diagnostics 承载，不新增状态位。
    d. **适用范围边界（例外）**：13b 的"门控排除"仅指视场/SNR/距离/遮挡等**门限判定**；
       目标失效（`active=false`、输入中消失等 → `kLost`）属生命周期语义，**不产生**排除诊断，
       由生命周期事件与 DebugView（如 `present_in_input`）承载；SBIRS/AR/EOS 参考实现同此边界。
    e. **排除原因跨周期差分记录器**：规则 13b 的 `cause`/`code` 为每周期瞬时快照（无跨周期
       记忆）；库内提供 `*ExclusionCauseRecorder` 对持续被排除实体做跨周期差分观测，产出
       结构化变化事件（A2 进入/A3 原因变化/A4 退出；A1 原因稳定不产事件），与既有
       `*LifecycleRecorder` 并列（独立 recorder、独立 `GetLastEvents()` 通道、独立 Attach 注册）。
       - **差分键**为 `(code, cause)` 组合对（非纯 cause）：避免同为 `kNone` 的具体门切换盲区
         （如 SBIRS 遮挡↔距离带，cause 均 kNone 但 code 不同）。
       - **原料依赖**：13b 的"实体机器可读关联"（location）条款——记录器按 `entity_index`
         关联"该排除属于哪个实体"，无 location 则无法差分。
       - **状态最小化**：每实体仅记忆上一执行周期的 `(code,cause)` 对（无条目 = 上周未被排除），
         非执行周期不推进状态（与 `*LifecycleRecorder` 语义一致）。
       - **纯观测**：只读 `result.issues`（按 `location.kind == kSceneEntity` 过滤），不改变
         `*CycleStatus`、排除诊断、DebugView 状态语义（规则 13c 边界延续）。
       - **适用范围**：具有"按实体门控排除"语义的模块（AR/SBIRS/EOS/ESR/RIR）；SAR 无排除诊断
         （13b 空洞条款），不适用。五模块全部落地（RIR 于 2026-08-22 补齐）：AR/SBIRS/RIR 以
         `target_id` 为内部状态键；ESR 无 target_id 概念，以发射源标识（platform/equipment/emission id 三元组）
         为内部状态键——`location.entity_index` 为 identity 排序后下标（与
         `InterceptDetectionExecutor` 排序序一致），记录器 Update 时按同一序重排 emissions
         把 entity_index 解析回 identity 三元组，**内部以 identity 为键**免疫跨周期发射源
         集合变化时的下标移位。EOS 单一视场门，与 SBIRS 同构（target_id 键）。
   **对齐状态（2026-08）**：六传感器模块已全部按本规则对齐（SBIRS/AR/ESR/EOS/SAR/RIR——
   RIR 于 2026-08-22 补齐 cause/location 与排除码子集）。SAR 无
   逐目标门控排除（集体成像模型，几何/SNR 门均为整周期中止 → 三写），13b 对其为空洞条款，
   以 kInfo/kWarning 正常路径诊断承载（见 `docs/sar/boundaries.md`）。
14. **统一问题列表模型**：`*CycleResult` 只承载**单一问题列表** `*IssueList`（字段名 `issues`）；
    输入校验问题与执行诊断问题不得以平行字段（如 `validation_issues`、`diagnostics`）分开承载。
    每条问题条目（各模块 `*Issue` 结构）：
    a. `severity`：`kInfo` / `kWarning` / `kError`。
    b. `phase`（来源标签）：`kInputValidation`（输入校验阶段，调用方输入问题）/
       `kExecution`（管线执行阶段，含关机等运行态条件）/
       `kOutputContract`（输出违反内部契约）。phase 是结构化来源判别字段；状态判断仍以
       `status`/`abort_reason` 为准（规则 9a），phase 不改变状态语义。
    c. `code`：字符串，带模块前缀。输入校验问题（周期输入校验 `Validate*CycleInput` 与
       创建时配置校验 `Validate*SessionConfig` 共用）编码为 `"<module>.validation.<snake_case>"`
       （如 `"esr.validation.invalid_emitter_frequency"`）；执行诊断保持既有 code 字符串
       （如 `"sar.snr_below_minimum"`）。机器消费只认 code（规则 13b）；既有执行诊断 code
       字符串是 replay/trace 稳定语义，不得重命名。
       **code 全集单一事实来源**：各模块公开头文件 `include/1q/<module>/session/<Module>IssueCodes.h`
       （如 `EosIssueCodes.h`）定义本模块全部 code 常量，库内调用点一律引用其常量、集成方以该
       头文件为 code 目录；新增/改名 code 必须同步该头文件（常量值即 code 值，调用点随引用
       自动更新）。各模块 boundaries 均登记该指针。
    d. `message`：人读文本，内容与格式不承诺解析稳定性（规则 13b）。
    e. `location` / `field`（可选定位）：`location.kind == kGlobal` 或 `field` 为空表示无定位；
       定位服务人读、replay 保真与机器消费（排除诊断的 `kSceneEntity + entity_index` 供差分
       记录器按实体关联，见规则 13b 实体机器可读关联条款与规则 13e），不用于状态判断。
    f. `cause`（可选归因）：各模块 `<Module>IssueCause` 枚举，默认 `kNone`；仅排除诊断
       （规则 13b 门内归因条款）使用，标识聚合门失败的主因物理链路；`cause` 不用于状态判断，
       不替代 `code`（机器键仍只认 code）。
    输入校验入口（`Validate*CycleInput`）返回同一问题条目列表（`phase = kInputValidation`）；
    `HasValidationError` 按 `phase == kInputValidation && severity == kError` 判定。
    **周期输入校验层归属与 issues 流向（COMMON-OQ-9 收敛，2026-08）**：
    - 周期输入校验以控制器 `RunOnce` 为权威点（SAR/SBIRS/ESR/EOS）；校验 issues 直通周期
      结果——`RunOnce` 内装配 `*CycleResult` 并缓存（`BuildCycleResult` 仅返回缓存，SAR/
      ESR/EOS 同构；SBIRS 由 `RunOnce` 直接返回）。**禁止校验缓存与查询 API**
      （`last_validation_issues` 缓存字段与 `GetLastValidationIssues` 查询 API 已全部删除，
      残余为零）。
    - 校验拒绝不附加粗粒度 abort 条目：校验问题本身就是 error 级诊断（规则 9 写二由它们
      承载），拒绝路径显式补齐 `abort_reason` 与日志。
    - **AR 特例**：公共路径入口校验保留在 session（`ValidateArCycleInput` 含外部运动学
      坐标系转换，控制器输入面不含 platform/targets 原始数据，无法下移）；运行期校验
      唯一化在控制器 `RunOnce`（会话层对同一输入的二次校验已删除），拒绝明细经出参
      直通并装配进最终周期结果（运行期拒绝 `abort_reason` 保持真实值，不写死替换）。
    - **RIR 同形**：周期输入校验在 session 层（`ValidateRirCycleInput` 含平台 ECEF→LLA
      转换与 rf_scene 窗口对齐），拒绝明细直通 `RirCycleResult.issues`。
    创建时配置校验入口（`Validate*SessionConfig`）返回同一 `*IssueList`：
    - `phase = kInputValidation`、`severity = kError`，code 按 c 条
      `<module>.validation.<snake_case>` 规则（同条件在创建时与运行期路径 code 逐字一致），
      `field` 定位配置字段路径。
    - `CreateWithDiagnostics` 出参类型为 `*IssueList`（非阻断语义见
      `docs/common/contract.md` §会话创建入口）。
    - 各模块 config 域 `ConfigValidationCode` 枚举、`ConfigValidationIssue` /
      `ValidationIssue` 结构与 `ValidationIssueList` 别名已删除。
    旧符号移除声明：
    - 各模块 `ValidationCode` 枚举、`ValidationIssue` 类型与平行列表字段不再作为输出通道。
    - `*CycleResult` 不得保留可推导的 error 布尔缓存字段（`has_validation_error` /
      `has_error` 已删除，调用方以 `HasValidationError(issues)` 或遍历判定）。
    - SAR 为参考实现（无平行字段）；ESR/EOS/SBIRS/AR 于 2026-08 按模块收敛完成
      （迁移状态见下表）。
    [evidence: tests/contract/sar/sar_three_write_guard_test.cpp —— 参考实现 issues 唯一列表 + phase 断言]
    [evidence: tests/contract/electronic_surveillance_radar/esr_three_write_guard_test.cpp —— 迁移后 phase 断言]
    [evidence: tests/unit/sar/sar_input_validation_test.cpp —— 校验问题 code "sar.validation.<snake>" + phase 断言]
    [evidence: tests/unit/sar/sar_session_config_builder_test.cpp —— config 域 code "sar.validation.<snake>" 断言]
    [evidence: tests/unit/sbirs_sensor/sbirs_session_config_builder_test.cpp —— 无枚举 config 域 code 断言]

    **对齐状态（2026-08，全部已对齐）**：

    | 模块 | `validation_issues` 平行字段 | `phase` 来源标签 | 可选定位 | config 域（`Validate*SessionConfig`） | 校验层归属与 issues 流向 |
    |---|---|---|---|---|---|
    | SAR | 无（参考实现） | 已对齐 | 已对齐 | 已统一 | 控制器 RunOnce + 直通（参考实现） |
    | ESR | 已迁移 | 已对齐 | 已对齐 | 已统一 | 控制器 RunOnce + 直通（2026-08 收敛） |
    | EOS | 已迁移 | 已对齐 | 已对齐 | 已统一 | 控制器 RunOnce + 直通（2026-08 收敛） |
    | SBIRS | 已迁移 | 已对齐 | 已对齐 | 已统一 | 控制器 RunOnce + 直通（2026-08 收敛） |
    | AR | 已迁移 | 已对齐 | 已对齐 | 已统一 | 公共入口 session + 运行期控制器（2026-08 收敛） |
    | RIR | 无（新模块直接实现） | 已对齐 | 已对齐 | 已统一（`ValidateRirSessionConfig`） | session 层校验 + 直通 |

### 传感器方位坐标系约定（SBIRS）

**SBIRS 输出的 `az`/`el`（检测记录 `SbirsDetectionRecord`、扫描相位
`SbirsOutputFrame::scan_azimuth_rad` / `scan_elevation_rad`（阶段 4 起））为 ECI 极坐标
（2026-08 正式变更），不是卫星局部地平系**：

- `az = atan2(los.y, los.x)`：相对 **ECI x 轴**（J2000 平赤道面，`SbirsVector3M`
  的 y/x 分量），单位 **rad**，取值范围 `[0, 2π)`；
- `el = asin(los.z / |los|)`：相对**赤道面**（ECI z 轴为天顶参考），单位 **rad**，
  取值范围 `[-π/2, π/2]`；星下点方向（目标在卫星正下方）`el ≈ −π/2`，
  北天极方向 `el ≈ +π/2`。

输入仍为 **ECEF**（`SbirsCycleInput::satellite_position_ecef_m`、`SbirsSceneTarget::position_ecef_m`），
且必须携带 **UTC 儒略日**（`utc_julian_day`，缺失即校验拒绝）：管线在周期入口按 GMST
把卫星/目标位置与速度旋转到 ECI 后统一计算（实现见
`include/1q/coordinate/inertial_transform.h` 与 `src/sbirs_sensor/foundation/SbirsGeometry.{h,cpp}`），
检测记录以 ECI 视线向量计算，无局部地平转换。内部角度量纲保持 deg、方位对称约定
`(-180°, 180°]`；输出边界统一换算为弧度并折入 `[0, 2π)`/`[-π/2, π/2]`。

**集成含义（反开发者直觉，易踩点）**：

1. 场景几何编排（卫星位置、目标分布、WFOV 扫描中心/覆盖）须按 ECI 极坐标参考设计：
   给定 UTC 时刻的 GMST 决定 ECEF 场景在 ECI 中的方位（az 平移 GMST、el 不变）。
   例如目标位于卫星正下方时扫描中心俯仰角应配置为 `−90°` 而非 `0°`
   （`0°` 指向赤道面水平方向，星下点目标将完全落在视场外）。
2. `scan_start_az_deg` 为 ECI 方位（deg），合法域 `[0, 360)`。
3. 该方位参考系与机载通道（AR/ESR/EOS 的平台局部系方位）**不同**；跨平台方位
   融合/相干关联需要调用方先做坐标系对齐（本库不提供转换，业务层职责）。
   参考实现见 `examples/component_attachment` 的 SBIRS 组件与 README 简化声明。

**扫描参数参考系（2026-08-17 阶段 2 起，安装指向链）**：

- 卫星姿态（`SbirsCycleInput::satellite_attitude_eci_body_deg`，**必填**，Z-Y-X 欧拉，
  Body→ECI；零欧拉合法 = 体轴对齐 ECI）与安装角（`SbirsOrientationConfig::mount_angles_deg`，
  Body→Sensor，静态配置）复合为指向合成链；**输出 az/el 仍为上述 ECI 极坐标参考**，安装矩阵
  只影响内部光轴几何（消费方兼容）。
- 扫描参数参考系由 `SbirsOrientationConfig::stabilization_mode` 决定：
  - `kBodyStabilized`（默认）：`scan_start_az_deg`/`scan_center_el_deg` 为**传感器系**角度；
    零姿态 + 零安装角下与 ECI 一致（既有配置零行为变化）。星下点编排注意同样适用：传感器系
    `el = −90°` 指向传感器 z 负向（星下点）。
  - `kInertialStabilized`：扫描参数保持上述 **ECI 参考定义**（扫描弧为惯性固定方向），经链路
    反解到传感器系实现。
- 传感器系扫描限位（`sensor_scan_limits_deg`，默认 az [-180,180]、el [-90,90] 全开）约束
  WFOV 扫描弧与 NFOV 命令；配置校验拒绝扫描弧超限位的配置。

### 执行状态信号统一

六个传感器模块的 `*CycleResult` 统一包含强类型 `*CycleStatus` 枚举，表达单周期高层执行状态：

| 模块 | 枚举类型 | 典型值 |
|---|---|---|
| AR | `ArCycleStatus` | `kCompleted`, `kPoweredOff`, `kRejectedInvalidInput`, `kRejectedInvalidConfig`, `kRejectedExecution` |
| ESR | `EsrCycleExecutionStatus` | `kCompleted`, `kRejected`, `kPoweredOff` |
| EOS | `EosCycleStatus` | `kCompleted`, `kPoweredOff`, `kRejectedInvalidInput`, `kRejectedExecution` |
| SAR | `SarCycleStatus` | `kCompleted`, `kRejectedInvalidInput`, `kRejectedExecution`, `kPoweredOff`（细粒度失败信息由 `SarIssue::code` + 日志双写） |
| SBIRS | `SbirsCycleStatus` | `kCompleted`, `kPoweredOff`, `kRejectedInvalidInput`, `kRejectedExecution` |
| RIR | `RirCycleStatus` | `kCompleted`, `kPoweredOff`, `kRejectedInvalidInput`, `kRejectedInvalidConfig`, `kRejectedExecution`（与 AR 同集） |

`abort_reason` 是强类型枚举（SAR 为 `SarPipelineAbortReason`，其余模块类似），提供更细粒度的终止原因。
状态判断以 `status` 枚举为准（`status == kCompleted` 即本周期的有效执行标志）。

### Attribution（仿真真值归属）挂载

仿真真值归属（detection/track → simulation target 映射）在各模块的挂载位置不同，这是有意设计：

| 模块 | 归属位置 | 理由 |
|---|---|---|
| AR | 双挂载：产品层（`TrackStateSnapshot` 内嵌 `external_target_id`/`target_name`，**deprecated 遗留**，sim-only）＋ 仿真附件层（`ArCycleResult.track_attributions`，**权威路径**） | **历史特例**：track 是系统级估计，关联键历史上写进产品航迹。对照表已补齐（2026-08-21，与 RIR 同构、产品导出航迹级），产品字段降级为 deprecated 遗留，回收（去真值化）由后续独立工作处理 |
| EOS | 仿真附件层（`EosCycleResult.detection_attributions`） | detection 是原始传感器输出，归属是仿真附加信息 |
| SBIRS | 仿真附件层（`SbirsCycleResult.detection_attributions`） | 同 EOS |
| RIR | 仿真附件层（`RirCycleResult.track_attributions`） | 同为对照表；产品粒度航迹级，归属航迹级（不进 `RirOutputFrame` 产品层） |
| ESR | 无归属数据 | ESR 输出是观测+假设，不含检测到目标的映射 |
| SAR | 无归属数据 | SAR 输出是图像产品，不含检测到目标的映射 |

规则：
1. **产品层默认不含仿真真值归属**（EOS/SBIRS/RIR 已遵守；AR 为上表历史特例——产品字段已标记
   deprecated/sim-only，权威关联路径是 `ArCycleResult.track_attributions`，新代码不得以产品
   字段为关联依据）。
2. **仿真附件层允许承载归属**——归属是结构化数据（非人读），Replay 落盘需要消费。
3. **观测投影通过周期记录访问归属**，不是归属的唯一载体。
4. EOS/SBIRS 的 `*CycleOutputAdapter` 守卫（如 `SbirsOutputFrameContainsOnlyNativeFields()`）
   确保产品层不含归属泄漏。
5. **新产品类型的真值归属只允许仿真附件层挂载**，由 CI 源码守卫 `attribution_mounting_guard`
   （`tests/contract/check_attribution_mounting.cmake`）强制：公共头中的真值标识符
   （`external_target_id`/`target_name`/`*Attribution*`）只允许出现在守卫注册表内；AR 产品遗留
   注册表冻结、不得新增条目，特例不再开第二次。

## 可观测性单源

调用方面只记四件事：分层周期记录、派生投影、Replay 落盘、`PROJECT_LOG` 人读旁路。
禁止再引入与 `StepWithResult()` 并行的第二套周期事实生产者，禁止再引入不能被
`ReplayXxxTrace()` 消费的记录文件。

本规则约束有 `*Session` 的传感器模块（AR/ESR/EOS/SAR/SBIRS/RIR）。ECM 是效应器，不进
IssueList / 三写 / 三类投影，只提供周期状态与 Replay 落盘。fusion / threat_assessment /
navigation / flight_dynamic 是算法面，不进本模型。

### 四件事

1. **生产者唯一。** 机器消费只认 `StepWithResult()` 返回的分层周期记录
   （执行层 `status` / `abort_reason` / `issues`，含规则 13b 的 kInfo 排除；产品层见规则 15）。
   单源指一次周期一份记录，不是一个扁平上帝结构体。
2. **投影是记录的视图，不是第二套事实。** DebugView / Lifecycle / ExclusionCause 只读
   同周期 `input` 与周期记录，禁止自行再写一份排除或中止原因。Attach 与否零行为改变（规则 11c）。
3. **落盘只有 Replay，且按层编码。** 需要复现实验时使用 `*RecordingSession` + `ReplayTraceWriter`；
   回放入口为 `ReplayXxxTrace()`。禁止再引入 FlexBuffers 观测帧、手搓 cycle JSON 或任何
   不能被 `ReplayXxxTrace()` 消费的记录文件。Recording 不得为落盘再拷一份产品。
4. **`PROJECT_LOG` 是人读旁路。** 默认关闭；状态判断禁止解析日志文本（规则 3）。
   示例层中文事件与验收文件（`*acceptance.log`）不是库可观测性 API。

中止路径仍执行规则 9 三写（`abort_reason` + `issues` + `PROJECT_LOG`）：粗信号、细码、人读日志
语义不同，不是三套事实。正常周期排除只写入 `issues`（规则 13b），不升格为三写。

### 产品形态齐套表

「有没有某通道」不得由模块自选。形态决定必选面；豁免只允许写在本表。

| 形态 | 模块 | 周期记录 + 三写 | 投影 | Replay 落盘 |
|---|---|---|---|---|
| 目标列表传感器 | AR / EOS / SBIRS / RIR | 必选 | DebugView + Lifecycle + ExclusionCause | `*RecordingSession` + `ReplayXxxTrace` |
| 非目标列表传感器 | ESR | 必选 | 仅 ExclusionCause（发射源身份键）；不提供目标 DebugView / Lifecycle | 同上 |
| 成像传感器 | SAR | 必选 | ProductDebugView + ProductLifecycle；ExclusionCause 空洞（规则 13b/13e） | 同上 |
| 效应器 | ECM | 不进 IssueList/三写；`EcmCycleStatus` 为周期状态 | 无三类投影 | `EcmRecordingSession` + `ReplayEcmTrace` |
| 算法面 | fusion / threat_assessment / navigation / flight_dynamic | 不适用 | 不适用 | 不适用 |

规则：

1. **单源**：一个周期事实只允许 `StepWithResult()` 生产；投影与 Replay 只能消费该周期记录（及同周期 input）。
2. **单落盘**：`ReplayTraceWriter` 是库内唯一周期持久化格式。禁止再引入不能被 `ReplayXxxTrace()`
   消费的记录文件。
3. **记录包装器**：上表承诺 Replay 的模块必须提供 `*RecordingSession` 门面，选项仅 `replay_writer`
   与是否在构造时记录配置；不得再提供第二套不能回放的记录选项。
4. **点菜式覆盖视为契约违规**：模块不得在齐套表外自行增减通道；新增形态必须先改本表。

**对齐状态（2026-08-23）**：产品形态表（投影 / Recording 门面 / 三写）六传感器 + ECM 已齐套。
规则 15 分层记录为**契约冻结、代码未落地**。

[evidence: include/1q/remote_identification_radar/session/RirRecordingSession.h]
[evidence: include/1q/remote_identification_radar/session/RirReplaySession.h]
[evidence: tests/replay/remote_identification_radar/rir_replay_session_test.cpp]
[evidence: tests/contract/remote_identification_radar/rir_three_write_guard_test.cpp]
[evidence: src/remote_identification_radar/session/RirSession.cpp]

## 规则 15：分层周期记录（2026-08-23 冻结）

有 Session 的传感器模块必须对齐。ECM 效应器与算法面不适用。
本规则是下一轮 DTO / schema 改造的权威；当前扁平 `*CycleResult` 是待迁移实现。

- **15a 一次生产。** 一个周期只允许 `Session::StepWithResult()` 生产一份周期记录。
  投影与 Replay 只消费该记录及同周期 input。
- **15b 按层组合。** 记录含执行 / 产品 / 副作用 / 仿真附件；AR/RIR 闭环（指定、控制 provenance）可另成块。
  禁止把 Lifecycle 事件、`DecisionObservation` 航迹复印塞进记录。
- **15c 产品不嵌套复制。** 废止「信封嵌有产品帧副本」。`Step()` 从同一记录取出产品层。
- **15d 单一周期号。** 执行层 `cycle_index` 永远等于本次输入号。产品在非执行周期为空载荷，
  不得用产品帧 `cycle_index=0` 表示未发生。（收敛原 COMMON-OQ-7、RIR-OQ-2。）
- **15e SAR 产品边界。** 聚焦图像是产品，`Step()` 必须能拿到图像。
  `SarOutputFrame` 标量元数据可作为产品内的 trivially_copyable 子结构。
  原始相位历史（IQ）默认不进周期记录；仅在调用方显式要求成像链回放时另表落盘。
  [evidence: include/1q/sar/session/SarCycleResult.h]
- **15f AR 决策缝。** 内部「特征 → 分类 → LPI/ECCM → 下一拍 `ArControlProfile`」闭环保留。
  `DecisionObservation` / `DecisionInputFrame` 不得作为周期记录字段。
  唯一 public 决策缝是 `SubmitExternalDecision()`；外部模块读产品航迹、干扰观测、本拍 `control_profile`。
  周期记录只留 `applied_decision_source` 与对应 cycle/batch。
  Replay 覆盖用独立 submit 事件，不落盘第二份航迹。
  [evidence: src/airborne_radar/decision/TacticalCoordinator.cpp]
  [evidence: include/1q/airborne_radar/session/ArCycleResult.h]

**对齐状态（2026-08-23）**：契约冻结，代码与 schema 未落地。

本周交付模块（RIR / SBIRS）**不改 public `*CycleResult` 字段布局**。规则 15 对它们是目标形态映射，不是本周改动清单：

| 模块 | 本周 | 现有字段如何对应分层 | 交付后再动 |
|---|---|---|---|
| SBIRS | 不动 public DTO / Replay schema | 执行 = `input_cycle_index`+`status`+`issues`+`abort_reason`；产品 = `output_frame`；仿真附件 = `detection_attributions`。无副作用、无闭环、无决策观测袋。 | 可选嵌套成层；唯一行为差是 15d（失败时产品帧 `cycle_index` 现为 0） |
| RIR | 不动 public DTO / Replay schema | 执行同上；产品 = `output_frame`；副作用 = `emission_frame`；仿真附件 = `track_attributions`；闭环 = 指定回退 / `dwell_center_deg` / 识别摘要。无 `DecisionObservation`。失败周期已把输入号写进输出帧（更接近 15d）。 | 可选嵌套成层，不改语义 |

行为向改造（SAR 图像进产品、AR 拆决策观测袋）不进 RIR/SBIRS 本周范围。

## Replay 持久化语义

1. 模块级 cycle-output 回调必须用 `ReplayTraceOutputStatus` 结构化表达比较结果。
   `kDivergence` 是输出分叉；`kOtherFailure` 是解码、类型或执行失败，不能被标记为
   分叉；不得通过解析人读 error 文本推断状态。
2. `ReplayTracePlaybackResult.divergence_found` 是分叉的权威结构化信号。
   `kHandledByModule` 和 `kDivergence` 均计入 `compared_output_count`，
   `kOtherFailure` 不计入且不改变分叉状态；两条路径必须遵守同一
   `stop_on_first_divergence` 语义。
3. `ReplayTraceWriter` 是库内唯一周期持久化入口，经对应 `*RecordingSessionOptions::replay_writer`
   传入。`*RecordingSession` 包装 `*Session`，拦截 Step / runtime patch / 外部决策并写出可被
   `ReplayXxxTrace()` 消费的目录。不得再提供第二套不能回放的记录格式。
4. `ReplayTraceWriter(overwrite=false)` 必须对既有 replay 工件 fail closed：不得重写
   manifest、追加事件或重置 sequence/hash chain。续写已有记录需要独立、显式且能恢复
   sequence、hash、chunk 与 index 状态的 resume 契约，不能复用普通创建入口隐式实现。
5. Replay I/O 不得把错误伪装成正常结束。Reader 调用方必须可通过
   `ReplayTraceReadStatus` 区分事件、正常记录末尾与读取错误；Writer 调用方必须可通过
   `ReplayTraceWriteStatus` 和 `first_error()` 观察初始化、写入及刷新失败。扫描和回放遇到
   缺失 manifest、缺失首 chunk 或底层读取错误时必须 fail closed。
6. 模块 FlatBuffers codec 只共享无 schema 知识的机械基元：已完成 builder 的字节复制，以及
   字段布局一致的 `FailureMarker` 空值、空 payload、verifier 和共有字段解码保护。schema、DTO
   映射、payload identifier、模块错误文本、外部数据资格与 divergence 行为继续由模块拥有；
   不得把公共 helper 扩张为万能 codec 或跨模块对象图。
7. runtime patch 记录必须写下实际应用结果，不能只记录请求。replay 应重新应用 patch 并比较结构化
   status、是否包含请求以及是否提交；合法拒绝和空补丁是可回放事件，不得被强制解释为成功。输入配置、
   patch 和输出中以整数存储的 enum 必须逐值校验；未知值应原子拒绝且不得部分修改解码目标。

[evidence: tests/replay/sbirs_sensor/sbirs_replay_codec_roundtrip_test]
[evidence: tests/replay/sar/sar_replay_codec_roundtrip_test]
[evidence: tests/replay/airborne_radar/ar_replay_codec_roundtrip_test]
[evidence: tests/replay/electronic_surveillance_radar/esr_replay_codec_roundtrip_test]
[evidence: tests/replay/electronic_surveillance_radar/esr_replay_session_test]
[evidence: tests/replay/remote_identification_radar/rir_replay_codec_roundtrip_test]
