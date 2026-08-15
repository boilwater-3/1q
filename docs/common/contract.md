# 跨模块契约

Status: active
Last-reviewed: 2026-08-07
Authority: common contract for all modules
RF-Interference-Architecture: frozen target; AR/ESR/ECM RF v2 implemented (per-module status in each design.md)

本文合并原顶层 public API customization、session config builder、三层输出可观测性和文档治理契约。模块级文档不得与本文冲突。

## 证据优先开发模式

对算法、架构、模块内部优化、输出语义、配置语义和 public API 相关改动，默认采用
`.claude/skills/evidence-first-freeze-contract` 定义的证据优先模式。

强制规则：

1. 先判定，再契约，再实现。
2. Stage A 未得到 `pass` 或 `narrow` 判定时，不进入生产代码实现。
3. Stage B 前必须冻结实现契约，明确允许范围、禁止范围、行为边界、验收条件和非目标。
4. 实现只能覆盖已被证据证明的最小边界；不得借机扩大 public API、跨模块抽象、schema、trace/replay 或兼容层。
5. 验收失败时回到证据矩阵重新拆分原因；不得通过放宽阈值、扩大 skip 或弱化测试制造通过。

具体 evidence matrix、契约模板、输出格式和回写要求由 repo skill 维护；公共契约只规定该流程是高风险开发的默认门禁。

## Public API 边界

默认 public API 只允许稳定门面和稳定 DTO：

- 模块聚合入口头。
- `*Session`，包括 `Create` / `CreateWithDiagnostics` 等静态创建入口。
- `*SessionConfig` 四域配置和运行期 patch。

### 会话创建入口的非阻断语义

`*Session::Create` 与 `*Session::CreateWithDiagnostics` 的校验/构造语义必须遵守以下非阻断契约（五模块已实现并经契约测试覆盖）：

1. `Create(config)` 是信任路径，不做配置校验。
2. `CreateWithDiagnostics(config, issues)` 是校验路径——**无论 @p issues 是否含有 error 都会构造并返回会话（非阻断）**；`issues` 为模块 `*IssueList`（统一问题列表模型，见 `docs/common/session_contract.md` 规则 14，config 域校验问题 code 为 `"<module>.validation.<snake_case>"`），仅为咨询性诊断输出，传入 `nullptr` 时仅构造会话、不写回。
3. 两入口均不会因校验失败而拒绝构造；当前不存在"校验失败即不构造"语义。
4. 调用方须据 `issues->empty()`（或 `HasValidationError`）自行决定后续处置。

不得在文档或实现中宣称校验失败会阻断会话创建，也不得让任一模块私自把 `CreateWithDiagnostics` 改为门禁语义（校验失败即不返回会话）；若未来确实需要门禁语义，应新增独立入口点承载，不得复用现有咨询性入口名。
[evidence: tests/contract/electronic_surveillance_radar/esr_public_api_convenience_test.cpp::CreateWithDiagnosticsReportsIssuesButStillConstructsSession]
[evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test.cpp::CreateWithDiagnosticsReportsIssuesButStillConstructsSession]
[evidence: tests/contract/sbirs_sensor/sbirs_public_api_convenience_test.cpp::CreateWithDiagnosticsReportsIssues]
- `*CycleInput`、scene target/emitter/point target 等单周期输入 DTO。
- `*OutputFrame`、`*CycleResult` 等输出和结构化执行结果 DTO。
- trace/replay、debug view、lifecycle recorder 等已经形成外部消费合同的工具。

业务模块 public 类型使用模块所有权前缀：`Ar*`、`Eos*`、`Esr*`、`Sar*`、`Sbirs*`。领域术语不受该规则机械约束，例如 `radar_cross_section`、`RadarEquations` 这类物理概念可保留领域名；但 session/config/cycle/result/adapter/trace/replay/debug/lifecycle 等 public DTO 和门面不得把通用领域词误用为模块前缀。

默认禁止公开：

- pipeline/controller/context/environment service 等内部装配 seam。
- algorithm executor、focusing selector、calibration/focusing/truth oracle 等内部阶段。
- generated replay headers、内部 execution config、测试专用 mock 接口。
- 仅有单一生产实现且没有外部替换需求的虚接口。

唯一允许的用户自定义 SPI 是 `airborne_radar` 的 decision engine。其它模块默认只提供稳定 session 门面。

### 四域配置所有权

`*SessionConfig` 的 hardware / mission / policy / environment 按“参数表达的用户意图”划分，而不是按
当前内部消费类所在目录划分：

- hardware：发射机、接收机、天线、探测器、光学和波形等物理能力。
- mission：开关机、工作模式、扫描几何、指向和任务目标。
- policy：最低 SNR、虚警概率、脉冲积累数、检测 margin、关联门限等可由任务策略调整的判决规则。
- environment：场景介质、传播、地表和大气观测等外部环境语义。

因此最低 SNR、Pfa 等探测门限不得因内部 signal detector 与 hardware 相邻而放入 hardware。跨模块的
同类判决参数应归 policy；模块可以保留不同物理算法和字段集合，但不能改变所有权含义。
[evidence: tests/unit/airborne_radar/ar_session_config_builder_test.cpp]
[evidence: tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp]

### 物理量单位命名

公开标量字段必须在名称后缀中表达调用方实际提交的单位。跨模块同物理量只有在公式输入量纲和消费语义
一致时才统一单位；不得为了表面一致，在 public 边界隐藏换算或让字段退化为无单位名。

EOS 的 `detector_area_cm2` 与 `detector_detectivity_cm_sqrt_hz_per_w` 共同进入厘米制 D* / NEP 公式，
因此必须保留显式单位后缀，禁止退化成无单位 `detector_area`。SBIRS 当前没有具备焦距、像元几何与
视场立体角映射的成像链，因此不公开未消费的 detector-area 参数。
[evidence: tests/contract/check_cross_domain_naming.cmake]

## 跨模块物理基元复用

跨模块函数不得因名称或量纲相似而合并。只有输入单位、数值精度、有效/非法输入策略、几何归一化、环境/效率因子和输出失败语义完全同义时，才允许复用无状态纯函数；复用前必须有跨模块 characterization 测试。

当前 EOS/SBIRS 的冻结结论：

| 函数族 | 结论 | 原因 |
|---|---|---|
| Planck 光谱辐亮度 | 仅 EOS 保留，SBIRS 已删除 | SBIRS 目标签名改为调用方提供辐射强度（W/sr），无温度输入、不做 Planck 换算 |
| 接收功率 | 不合并 | EOS 接收孔径面积并除以 `4πr²`；SBIRS 接收孔径直径、使用 `P = I_t·A_ap·τ/d²` 口径和量子效率 |
| 噪声/NEP | 不合并 | EOS 是背景抑制与 NEP 链；SBIRS 是 photon/thermal/readout RMS 与兼容 NEP 回退 |
| 输出方位参考系 | 不合并 | SBIRS 输出 ECI 极坐标弧度（2026-08 正式变更：输入 ECEF + UTC 儒略日，周期入口按 GMST 旋转到 ECI；az∈[0, 2π)、el∈[-π/2, π/2]）；EOS/AR/ESR 保持各自平台局部系约定 |
| 距离输出 | 不输出 | SBIRS 被动红外不测距：raw output 无距离字段；`estimated_range_m` 仅为内部诊断且仅对归属目标回填；示例层不展示距离 |

这不是未来共享 foundation 的禁止令：新的候选必须先证明上述语义完全相同，且不得以转换器、默认值或兼容层掩盖差异。

### 工程 RF 契约

以下为 AR/ESR/ECM 公共 RF 事实域（`oneq::electromagnetics`）的跨模块契约条款。设计描述（provenance
四级、单周期交换时序、接收机影响分层）见 [rf_architecture.md](rf_architecture.md)。

1. 发射事实不得再用单一 `entity_id` 同时承担设备身份、同平台判断和待处理信号排除。
2. 发射事实禁止携带 `received_power`、J/S、J/N、receiver impairment、`jamming_detected` 或成功概率。
3. AR 目标回波不得伪装成外部 `RfEmission` 后复用单程公式；外部雷达、ECM 和其他 RF 源才走公共单程链路。
4. 有效带外发射或零时频重叠产生零贡献，不是错误。非有限值、非法活动区间/波形、负功率、重复 emission
   ID、缺失设备级 co-site isolation 和不支持的近场路径必须 fail closed，且不得部分写回。
5. 超过已标定最大线性输入功率是合法物理结果：周期仍视为已执行并输出结构化 saturated impairment，
   但不得伪造观测；它与输入/配置非法导致的整周期拒绝严格分离。
6. 业务模块之间不直接调用；调用方只转交公共值类型，不需要创建 RF scene、回填 AR 自身发射、管理 token
   或调用 prepare/complete 状态机。
7. 非空 RF frame 必须与消费者周期窗口完全一致；空 frame 表示没有外部 RF 干扰，不要求虚构身份或 mode。
8. 输入拒绝不消费 emission ID、hop/PRI phase、随机流、待应用控制或跟踪状态；设备关机只推进世界 chronology。
9. 模块在返回成功结果时原子提交本周期发射、接收、检测和累积状态。

[evidence: tests/unit/common/common_rf_link_budget_test]
[evidence: tests/unit/common/common_rf_scene_test]

### 折射率温标输入迁移

公开折射率入口只提供 `RefractivityInputs` + `RefractivityTemperaturePair` +
`TryRefractivityIndex`。摄氏与开氏字段必须满足
`kelvin = celsius + 273.15`（允许 0.05 K 浮点容差）；温标错配、非有限/越界标量或空输出必须
fail closed，且失败不得修改调用方输出。

[evidence: tests/unit/airborne_radar/ar_atmosphere_physics_test.cpp::TypedPublicRefractivityMatchesPhysicalKernel]
[evidence: tests/unit/airborne_radar/ar_atmosphere_physics_test.cpp::TypedPublicRefractivityRejectsMismatchedTemperaturePairAtomically]

### 核心运行面与观测工具面

public API 分为两类，二者都受 public boundary、install manifest 和 consumer 测试保护：

1. **核心运行面**：模块聚合入口、config、input、session、raw output 与 cycle result。它定义调用方驱动模型和消费仿真结果的稳定语义。
2. **观测工具面**：trace/replay、debug view、lifecycle recorder 及其结果 DTO。它用于诊断、复现和人读归属，不能反向改变核心运行面、raw output 或控制行为。

观测工具面的新增字段或事件必须保持三层输出分离，并同步 schema/codec、对应 replay/trace 测试和 consumer 测试；删除或重命名已公开工具仍属于 public API 变更，必须先冻结兼容迁移契约。

## 内部共享命名空间

`src/common/` 同时容纳两类实现，目录位置本身不决定 C++ 命名空间：

- 已由 `include/1q/<domain>/` 公开的领域 API 实现使用对应 public 命名空间，例如
  `oneq::coordinate`、`oneq::replay`、`oneq::trace`。
- 只在库内部跨模块复用的设施使用 `oneq::common::<domain>`，不构成 public API。

规则：

1. `src/common/` 下的类型必须能追溯到 public 领域头或明确的 `oneq::common::<domain>` 所有权；
   不得仅因目录名把 public-domain implementation 改入 `oneq::common`。
2. 跨模块共享工具不得放在 `oneq::internal::*` 或
   `oneq::trace::internal` 这类模糊内部命名空间中。
3. 不得为 `oneq::common::*` 工具新增 `oneq::internal::*` dual-alias 或
   兼容 using 块；迁移期 alias 只能作为同一批次内的临时编译过渡，最终提交前必须删除。
4. `namespace internal` 只可用于测试或翻译单元局部辅助语义；跨文件、跨模块消费的
   `src/common/` 设施必须有明确的 `oneq::common::<domain>` 所属域。
5. 若某工具需要成为外部消费者合同，应通过 `include/1q/` 公开并补充 public API
   边界测试，而不是从 `src/common/` 泄漏。

## 实现安全与失败语义

下列规则源自 `src/` 架构与安全审查，是所有模块共享的规定性约束。

1. **项目失败语义不得依赖 C++ 异常。** `src/` 与 `include/` 不得新增 `throw`，也不得用
   `std::runtime_error`、`std::invalid_argument` 等异常作为项目 API 的失败通道。I/O、构造和解析失败
   必须转换为错误状态、空 reader 或诊断字段。对 HighFive、JSBSim 等可能抛异常的第三方边界，允许在
   最窄调用点使用既有 `try/catch`，但 catch 后必须转换为项目状态且不得让异常穿透 session/adapter
   边界。当前构建没有全局启用 `-fno-exceptions`，因此本文不承诺该编译模式已经成立；若要建立该门禁，
   必须先替换或隔离所有第三方异常边界并增加真实构建验证。

2. **存在性标志必须与数据一致，且由校验层断言。** `has_xxx` 若表达“调用方是否提供本周期可选
   数据”，当 `has_xxx=false` 但对应数据非默认值时，输入校验必须报 error 级问题并 abort，不得让
   数据静默跳过。典型反例是 AR 的 `has_environment`：环境快照已写入但因漏置 flag 不被消费，会让
   杂波/干扰/大气数据完全不进入信号链且无任何信号。若布尔量明确是配置选择器而非数据存在性标志，
   可以定义关闭时整组候选参数不生效且不校验，但必须在 public Doxygen 和模块 design 明确优先级，并以
   启用/关闭对照测试锁定。

3. **非执行周期必须产生准确的结构化 reason，不得静默或伪造故障。**
   a. **校验失败必须显式 reason**：校验权威层（控制器 `RunOnce`；AR 公共路径入口为 session，
      见 session_contract.md 规则 14 校验层归属条款）设置显式 abort reason（如
      `kValidationRejected`），不执行 pipeline，不合成空输出帧，不把非法输入记作新的有效
      batch/帧。
   b. **关机等合法非执行状态用独立 reason**（如 `kSensorPoweredOff`），不得映射成 output
      contract violation。
   c. **五模块统一不复用**：非执行周期（校验失败/关机/执行 abort）的 `Step()` 与
      `*CycleResult.output_frame` 一律返回默认空帧，永不复用上一有效输出。
      `reused_previous_output` 字段、以及支撑复用的 `latest_output`/`previous_output`/
      `has_latest_output`/`has_previous_output` cache 字段已全部删除。
   d. **reason 数值向后兼容**：新增 reason 以显式数值追加，保留已有 replay/trace 中既有数值语义。
   e. **Lifecycle recorder 边界**：不得把非执行周期（`status != kCompleted`）解释为目标丢失或未检测；
      非执行周期不产生 lifecycle 事件，也不推进其累积状态。
   [evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailure]
   [evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]
   [evidence: tests/contract/electronic_surveillance_radar/esr_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailure]
   [evidence: tests/contract/sar/sar_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]
   [evidence: tests/contract/sbirs_sensor/sbirs_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]

4. **外部输入解析与 trace 读取必须有上限与完整性校验。** 自研解析器（如 JSON）必须有最大嵌套深度限制、顶层 value 后的 EOF 校验与转义完整性校验。trace/replay 文件读取必须在读入前检查大小上限（与写入侧守卫对齐）。磁盘写失败必须检查流状态并记录，不得静默丢失。

   failure marker 是 trace 中一个可报告的失败边界，不是 replay 的终止符。回放必须记录 marker，
   继续应用并比较其后的有效输入/输出，使"有效 -> 拒绝 -> 恢复"整段都进入确定性比较；只有
   divergence、损坏记录或不兼容模块等真正无法继续解释 trace 的错误才终止回放。
   [evidence: tests/replay/airborne_radar/ar_rf_trace_session_test.cpp::ArRfTraceSessionTest.RejectedCycleAndSameCycleRetryReplayExactly]
   [evidence: tests/replay/electro_optical_sensor/eos_replay_session_test.cpp::EosReplaySessionTest.ReplayEosTraceContinuesAfterFailureMarker]
   [evidence: tests/replay/electronic_surveillance_radar/esr_replay_session_test.cpp::EsrReplaySessionTest.ReplayEsrTraceContinuesAfterFailureMarker]
   [evidence: tests/replay/sar/sar_replay_session_test.cpp::SarReplaySessionTest.ReplayContinuesAfterFailureMarker]
   [evidence: tests/replay/sbirs_sensor/sbirs_replay_session_test.cpp::SbirsReplaySessionTest.ReplayContinuesAfterFailureMarker]

5. **数值归一化必须是常数时间。** 角度/周期归一化等可能接受无界输入的工具函数必须用 `std::fmod` 等常数时间实现，不得用 `while` 循环减/加周期，避免极大输入近似死循环。

## 数值下限语义

数值下限常量不得只因命名相似而合并。当前允许三类边界：

1. **通用数值防护下限**：防除零、对数域、阈值归一化等纯数值保护使用
   `oneq::common::numerics::kNumericFloor` 或更专门的 common numerics helper。
2. **坐标/姿态退化阈值**：ECEF 原点、方向向量零范数、接近姿态奇异点等几何退化判断保留在
   `common/coordinate` 局部实现内，阈值应按坐标算法精度选择，不与功率/概率数值下限共享。
3. **模块局部几何阈值**：例如 EOS 外部输入适配中目标与平台几乎重合的 range gate，属于模块输入几何退化判断，应保留模块局部阈值和状态码。

新增 floor 常量前必须先归入上述语义桶；不能把物理/几何阈值机械改为 `kNumericFloor`，也不能把通用除零保护散落成模块私有常量。

## 会话相关模块契约（指针）

以下契约只对"有 `*Session` 会话模型的传感器模块"（AR/ESR/EOS/SAR/SBIRS）有效，不是所有模块的跨模块契约。完整内容见 [session_contract.md](session_contract.md)：

- SessionConfigBuilder 薄封装规则（无 dirty flag / 无隐式覆写）
- Session composition ownership（`Impl` 所有权边界、AR 决策 seam）
- 运行期配置提交策略（事务性提交 vs 立即提交的分类表 + 各模块归属判定规则）
- 电源状态单源契约（`sensor_enabled` 唯一来源、`has_sensor_enabled` 唯一入口，五模块统一）
- 三层输出模型（OutputFrame / CycleResult / DebugView 分离 + 失败语义）
- 统一问题列表模型（`*IssueList` 单一列表 + `phase` 来源标签 + 可选定位；输入校验不设平行字段，见 session_contract.md 规则 14）
- Replay 与 trace 语义（结构化比较状态、TraceSink vs ReplayTraceWriter、codec 边界、runtime patch trace）

## 工程治理规则（指针）

CMake 工程边界（target 作用域、Windows 验收）和测试架构（type×domain 组织、CTest label、partition 注册）是工程基础设施规则，见 [docs/practice/build_and_test_governance.md](../practice/build_and_test_governance.md)。

## 文档结构

`docs/` 只允许以下一级目录：

- `common`
- `review`
- `practice`
- `airborne_radar`
- `electro_optical_sensor`
- `electronic_surveillance_radar`
- `flight_dynamic`
- `sar`
- `sbirs_sensor`

`docs/` 顶层不保留散落的 Markdown 文件。所有文档必须落在上述某个一级目录内。

`review/` 是唯一允许的评审和迁移草案目录，只能存放扁平 Markdown 草案文件。每个草案必须在文件头声明 `Status: draft`，不得作为当前权威文档引用；结论落定后，应迁入 `common/contract.md`、`common/open_questions.md` 或对应模块 `design.md`，再删除草案。

`practice/` 存放工程实践与基础设施类设计文档（非业务模块设计）：构建、测试策略、覆盖率、示例程序、批量验证框架等跨模块工程产物。每份文档为扁平 Markdown，文件头声明 `Status: active` 与 `Authority:`（如 `build infrastructure`、`test infrastructure`、`examples`）。`practice/` 不存放业务模块设计——模块设计归各自 `design.md`；也不存放契约规则——规定性规则归 `common/contract.md`。

每个业务模块以 `design.md` 为设计权威**入口**，另允许 `boundaries.md`、`data-flow.md`、`algorithms.md` 三个设计文档。`design.md` 承载模块定位与文档导航；`boundaries.md` 承载模块级边界、非目标与设计变更规则；`data-flow.md` 承载数据流、输入输出与状态所有权；`algorithms.md` 承载算法登记表与每算法的实现边界、反直觉点。切分原则：模块级边界（主语是"模块/API/输出"）归 `boundaries.md`，算法级边界（主语是"某算法/某计算路径"）归 `algorithms.md`。文档写代码读不出来的内容（定位/边界/禁令/反直觉点/否决理由），算法逐步逻辑归代码。历史决策记录（旧版 `decisions.md`、`history.md`、`contract.md`）和模块入口（`README.md`）的内容已内聚到该文档集中。

`common/` 只允许保留五份文档：

- `contract.md` —— 公共契约（规定性：所有模块必须遵守的规则）。
- `session_contract.md` —— 有 Session 的传感器模块的统一会话契约（SessionConfigBuilder、Session 组合所有权、运行期配置提交、电源单源、三层输出、Replay/trace 语义）。
- `open_questions.md` —— 跨模块架构观察与待决项（非规定性：记录调查中发现但尚未定论的议题，不构成契约约束）。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从 open_questions.md 移除。
- `rf_architecture.md` —— AR/ESR/ECM 公共 RF 工程架构设计描述（provenance、单周期交换时序、接收机影响分层）。
- `usage.md` —— 当前已验证的构建、安装与外部消费指南；不得承诺尚未由 consumer 验证的打包方式。

模块目录内不保留 `archive/`、`audits/`、`contracts/`、`design/`、`decisions/`、`workflow/`、`migration/` 等展开式历史目录。历史细节需要追溯时从 git 历史读取。

各模块以 `design.md` 为设计权威入口，配合 `boundaries.md`、`data-flow.md`、`algorithms.md`。限制条件与否决方向的证据引用直接嵌入对应文档的 `[evidence: ...]` 标注，指向对应测试文件和 git 历史。
