# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

## 当前状态（2026-07-31 实时代码复核）

原 OQ-1、OQ-3、OQ-8、OQ-9、OQ-10a 至 OQ-10m 均已完成、拒绝或冻结；对应结论和测试证据已迁入
`docs/common/contract.md` 各模块 `design.md`。ESR runtime validation 边界也已完成并迁入 ESR design。
COMMON-OQ-2 已完成（2026-07-31）：AR/ESR/SAR/EOS 四模块 `SessionConfigBuilder` 已收敛为薄封装，
Profile 枚举与 dirty flag 机制整体移除，语义档位退化为 `XxxProfileConstants.h` 预定义结构体常量；
结论回写进 `docs/common/contract.md` §SessionConfigBuilder 与四模块 design.md。
COMMON-OQ-4 已完成（2026-07-31）：AR/ESR/EOS/SBIRS 四模块电源状态字段提升——
`*MissionConfig` 删除 `power_on`，唯一来源为 `*SessionConfig::sensor_enabled`（运行时补丁
`has_sensor_enabled` 唯一入口，SBIRS 命名对齐），二重状态在类型层面消除；
结论回写进 `docs/common/contract.md` §电源状态单源契约 与 `check_cross_domain_naming.cmake` 阻断 7。
当前保留一项 Common/Practice 构建边界、五项 Common 跨模块设计边界（COMMON-OQ-3,5..8）、三项 ESR
非阻塞设计边界和四项 SBIRS 非阻塞仿真边界，均不构成已批准实现要求。COMMON-OQ-8 登记于
2026-07-31 跨域/consumer 排障：AR 周期窗口编年史、ESR RF 帧窗口匹配与零值配置、EOS 帧率-步长
耦合四处反直觉语义，违反时均静默拒绝，外部调用方难以察觉（两个 consumer 与一个集成测试的教训）。

## Common/Practice 非阻塞构建边界

### COMMON-OQ-1：Windows/MSVC 全链支持验收

- **现状证据**：仓库存在 Windows Conan/no-Conan presets 和 `scripts/fetch_third_party.bat`，但当前 CI
  只在 macOS 运行；`.bat` 还包含 GitLab/archives.boost.io 来源，且没有对下载内容做 hash 校验。
- **未决问题**：如何实现已冻结的 Windows shell/GitHub bootstrap，并在不依赖 Windows Conan 的前提下
  形成可重复的依赖、configure、build、install 和外部 consumer 闭环。
- **当前边界**：这些 presets 和脚本只视为未验收脚手架；文档不得据此宣称 Windows 已受支持，也不得
  把 Conan 路径自动提升为正式 Windows 方案。
- **Stage A 进入条件**：提交锁定版本/提交与下载校验矩阵，提供 shell bootstrap 原型，并在真实 Windows
  runner 上依次证明 configure、Debug/Release build、install、独立 consumer build/run；随后再决定保留、
  删除或重命名现有 presets 与 `.bat` 入口。

## Common 跨模块设计边界

### COMMON-OQ-3：`CreateWithValidation` 非阻断语义命名一致但反直觉

- **现状证据**：五模块（AR/ESR/SAR/EOS/SBIRS）会话工厂签名与行为完全一致——`CreateWithValidation`
  在实现中**无论校验是否产生 error 都会 `Create(config)` 返回会话**，`issues` 仅为咨询性诊断输出
  （AR `ArSession.cpp`、ESR `EsrSession.cpp`、SAR `SarSession.cpp`、EOS `EosSession.cpp:63-70`、
  SBIRS `SbirsSession.cpp`）。五模块头文件甚至使用逐字相同的中文 docstring
  （"无论 issues 是否为空，都会构造并返回会话（不阻断）"）。命名 `WithValidation` 暗示"校验通过才创建"，
  实际只附加诊断信息，是跨模块共享的反直觉点；不存在 `CreateWithDiagnostics` 命名。
- **未决问题**：是否在跨模块层面统一重命名（如 `CreateWithDiagnostics`）或改为返回
  `std::optional<Session>` / 失败时不构造，以消除命名误导。
- **当前边界**：五模块保持现有"非阻断 + 咨询性 issues"契约。docstring 已明确语义，调用方据
  `issues->empty()` 或 `HasValidationError` 决策。不得在文档中宣称校验失败会阻断创建。
- **Stage A 进入条件**：出现真实场景需要"校验失败即不构造"语义时，先评估跨模块统一重命名/
  返回类型变更的向后兼容成本与下游消费方影响，再决定是否一次性推广到五模块。

### COMMON-OQ-4：运行时补丁 mission.power_on 与叶子 sensor_enabled 双层冗余控制（已收敛 2026-07-31）

- **收敛决议**：采用"字段提升"方案（COMMON-OQ-2 单一来源原则的编译期实现）——
  `power_on` 从 AR/ESR/EOS/SBIRS 四模块 `*MissionConfig` 中**整体删除**，电源状态唯一由
  `*SessionConfig` 顶层 `sensor_enabled` 承载；运行时补丁电源入口唯一为
  `has_sensor_enabled` 叶子（SBIRS 命名从 `has_power_on`/`WithPowerOn` 统一对齐）。
  `mission` 域在类型层面已无电源字段，二重状态在编译期不可表达，不依赖注释约束。
  实施要点：resolver 的 `has_mission` 整块域不再映射电源（AR/EOS 删除映射行、ESR/SBIRS
  全量拷贝天然隔离）；`*InternalExecutionConfig` 顶层 `sensor_enabled` 同步提升；
  replay schema 的 `power_on` 字段移至 session config 表；`check_cross_domain_naming.cmake`
  阻断 7 更新为"mission 禁止 power_on、SessionConfig 必须 sensor_enabled、补丁必须
  has_sensor_enabled、builder 必须 WithSensorEnabled"。
- **与 COMMON-OQ-2 收敛模式的关联**：本议题与 OQ-2（SessionConfigBuilder Profile+dirty flag
  隐式覆写）同属"二重状态 + 隐式优先级"反模式家族。OQ-2 收敛（2026-07-31）沉淀的三原则中
  **单一来源**在此以更强的形式落地：不是"运行时拒绝/警告"，而是删字段使二重状态在类型层面
  不存在。显式顺序与可观测化原则仍适用于补丁的几何/模式字段（整块先、叶子后，docnote+测试
  固化）。
- **迁移记录**：`config.mission.power_on = X` 一律改写为 `config.sensor_enabled = X`；
  编译期强制（mission 结构已无该字段），全仓库 69 处构造点与 replay schema/codec 同步迁移，
  四模块单元/集成/replay/contract 测试全绿，9 个 consumer 全 exit=0。

### COMMON-OQ-5：`Step()` 在校验失败/关机时静默复用上一帧

- **现状证据**：`Step()`（返回 output frame）与 `StepWithResult()`（返回完整 result）两入口在五模块
  均存在。失败/关机时是否复用上一帧分两组：
  - **复用组**（SAR/EOS/SBIRS）：校验失败或关机时将 `latest_output` 保留为上一帧，`Step()` 直接返回，
    仅在 `StepWithResult()` 的 `executed_this_cycle`/`reused_previous_output`/`abort_reason` 中体现
    （SAR `SarController.cpp`、EOS `EosController.cpp:77-100`、SBIRS `SbirsController.cpp:21-27`）。
  - **不复用组**（AR/ESR）：失败周期不复用、不回传最近有效输出，状态经 `ArCycleStatus`/`EsrCycleExecutionStatus`
    枚举表达（AR `ArController.cpp`、ESR `EsrSession.cpp:53-54`）。
  公共 `OutputFrame`/`CycleResult` 无 `executed_this_cycle` 字段，仅在 result 层暴露。
- **未决问题**：是否统一五模块为同一组语义（要么全部复用、要么全部不复用）；若统一为复用组，是否在
  `OutputFrame` 增加失败标识使 `Step()` 也可区分"本轮计算"与"复用旧值"。
- **当前边界**：两组保持各自现有行为。复用组成员的 `Step()` 静默返回旧帧为已知设计，调用方须用
  `StepWithResult()` 获取失败/复用信号。不得在文档中暗示 `Step()` 返回值含本轮执行状态。
- **Stage A 进入条件**：出现真实消费方因无法从 `Step()` 区分新旧帧而误用，或跨模块集成要求统一周期
  失败语义时，先选定目标组（复用 vs 不复用），再评估迁移五模块的下游影响与测试矩阵。

### COMMON-OQ-6：`ApplyRuntimeConfig` 吞掉 `TryApplyRuntimeConfig` 返回值

- **现状证据**：五模块会话均提供 `ApplyRuntimeConfig(patch)`（void）与 `TryApplyRuntimeConfig(patch)`
  （bool）两方法；`ApplyRuntimeConfig` 内部 `(void)TryApplyRuntimeConfig(patch)` 显式丢弃成功与否
  （AR `ArSession.cpp`、ESR `EsrSession.cpp`、SAR `SarSession.cpp`、EOS `EosSession.cpp:83-85`、
  SBIRS `SbirsSession.cpp`）。ESR 额外提供 `ApplyRuntimeConfigWithResult` 返回结构化
  `EsrRuntimeConfigApplyResult`（含 `EsrRuntimeConfigApplyStatus` 枚举），其余四模块无此变体。
  docstring 警告不一致：SAR/SBIRS 标注"不返回成功与否"，AR/EOS 无此警告。
- **未决问题**：(1) 是否废弃 void 版、统一强制使用 Try/WithResult；(2) 是否向其余四模块推广
  `ApplyRuntimeConfigWithResult` 结构化结果；(3) 是否统一 docstring 警告。
- **与 COMMON-OQ-2 收敛模式的关联**：本议题与 OQ-2（Builder 静默覆盖用户赋值）及 COMMON-OQ-8
  （周期窗口违规静默拒绝）同属"静默语义"反模式家族——错误发生在调用方看不到的地方。
  OQ-2 收敛沉淀的**可观测化**原则在此已有局部先例：ESR 的 `ApplyRuntimeConfigWithResult`
  返回结构化结果（含 `EsrRuntimeConfigApplyStatus`），其余四模块可评估推广；弃 void 版强制
  Try/WithResult 与"消除隐式状态"同向。与 OQ-4 不同，本议题是纯 API 形态问题，不涉及状态
  冗余，推广成本主要在下游调用方迁移。
- **当前边界**：五模块保持 void+Try 双方法，void 版吞返回值为已知设计。ESR 的 WithResult 变体为
  ESR 独有增强，不构成跨模块契约。
- **Stage A 进入条件**：出现真实场景要求 void 版失败必须可观测，或跨模块集成要求统一结果返回形态时，
  先评估推广 `ApplyRuntimeConfigWithResult` 的 API 成本与四模块补丁结构差异，再决定是否统一。

### COMMON-OQ-7：CycleResult.input_cycle_index 与 OutputFrame.cycle_index 冗余

- **现状证据**：五模块 `CycleResult` 均同时携带 `input_cycle_index`（本次输入周期号）与内嵌
  `OutputFrame.cycle_index`（AR `ArCycleResult.h`、ESR `EsrCycleResult.h`、SAR `SarCycleResult.h`、
  EOS `EosCycleResult.h:30-41`、SBIRS `SbirsCycleResult.h`）。成功路径两者均取自 `input.cycle_index`，
  数值恒等。仅在复用/失败路径（SAR/EOS/SBIRS）二者分歧：`output_frame.cycle_index` 保留上一周期号、
  `input_cycle_index` 为本次输入号（AR/ESR 不复用，故不发生此分歧）。
- **未决问题**：是否移除 `input_cycle_index`、统一用 `output_frame.cycle_index`（但需为复用组另寻失败
  周期号归属），或反向统一为仅保留 `input_cycle_index`。
- **当前边界**：五模块保留双字段。`input_cycle_index` 在复用组承载"本次失败周期的归属号"语义，
  不可简单删除；去重须与 COMMON-OQ-5 的周期失败语义统一一并决策。
- **Stage A 进入条件**：与 COMMON-OQ-5 一同推进——先选定统一的周期失败/复用语义，再据此评估
  单一周期号字段的可行性及其对 trace/replay 归属的影响。

### COMMON-OQ-8：周期输入时间/窗口字段无统一契约，违反时静默拒绝

- **现状证据**：三模块各自为政的周期时间/窗口校验，外部调用方违反时多数表现为"静默
  不生效"而非显式错误，2026-07-31 一次 consumer 排障暴露三条：
  - **AR 编年史校验**：`PrepareRfCycle` 拒绝 `window_start_time_s < 上一周期窗口结束`
    （`ArSession.cpp:473`，`PreparedCycleLedger` 记录窗口编年史）。调用方若只递增
    `cycle_index` 而忘记推进 `cycle_start_time_s`，整周期在决策消费点
    （`PrepareEmissionControl`，`ArController.cpp:505` 调用链）**之前**被 `kRejected`——
    即使 `SubmitExternalDecision` 已返回 `kAccepted`，外部覆盖也从未被应用，
    `applied_decision_source` 保持 `kNone`（ar_extension_consumer 教训）。
  - **ESR 周期输入完整性**：`ValidateEsrCycleInput` 要求非零 `platform_entity_id` +
    有限且可定位的 ECEF 运动学（`EsrInputValidation.cpp:35-59`），并要求 `rf_emissions`
    的 `world_cycle_index`/`window_start_time_s`/`window_duration_s` 与周期 input 的
    `cycle_index`/`cycle_start_time_s`/`dt_sec` **精确相等**（`EsrInputValidation.cpp:80-89`）。
    空 RF 帧也必须填这三个窗口字段（esr_extension_consumer 教训）。
  - **EOS 帧率-步长耦合**：`ValidateEosCycleInput` 拒绝 `dt_sec > 10/frame_rate_hz`
    （`EosInputValidation.cpp:122-132`，53c56e21 收紧），默认 30 Hz → dt 上限约 0.333 s。
    1 s 步长的集成场景必须把帧率降到 1 Hz 才合法（multi_model_scenario_test 修复），
    帧率与步长的匹配义务完全由调用方承担，跨模块无统一约定。
  - **配置侧不对称**：ESR 零值 `EsrSessionConfig{}` 不合法（scan_rate、receiver_band_lower、
    receiver_equipment_id、检测策略全零均触发 `ValidateEsrSessionConfig`，
    `EsrSessionConfigBuilder.cpp:25-141`），而 AR/SBIRS 的 struct 默认即合法档位。
    叠加 COMMON-OQ-3 的非阻断语义，`EsrSession::Create({})` 静默携带 issues 返回会话，
    首周期才暴露为 `kRejected`（esr_extension_consumer 教训）。
- **未决问题**：(1) 是否跨模块统一周期时间/窗口契约——如共享"时间戳必须单调前进"与
  "RF 帧窗口必须匹配周期"的校验 helper，使违反在输入校验即显式可观测（validation issues），
  而非运行期静默拒绝；(2) 是否统一"零值配置"语义，或为 ESR 补 `kDefaultEsrSessionConfig`
  显式默认常量（对齐 SAR：struct 默认即合法档位）；(3) 各 `CycleInput` docstring 是否
  显式注明时间戳推进义务与窗口匹配要求（当前 AR/ESR 均无此警示）。
- **当前边界**：各模块保持现有校验。调用方必须：AR 推进 `cycle_start_time_s`（≥ 上一窗口
  结束）；ESR 填完整平台运动学 + 与周期匹配的 RF 帧窗口字段，并使用语义档位常量而非零值
  配置；EOS 保证 `dt_sec ≤ 10/frame_rate_hz`（1 s 步长须 1 Hz 帧率）。不得在文档中宣称
  周期时间戳可任意重复，或零值配置为合法默认。
- **Stage A 进入条件**：出现第二个真实消费方因时间戳未推进或 RF 帧窗口不匹配而静默失败
  （当前已有 ar/esr 两个 consumer 教训），或跨模块集成要求统一周期时间契约时，先盘点
  四模块 CycleInput 的窗口字段与校验差异，设计共享校验与 docstring 警示，再评估跨模块推广。

## Airborne Radar 非阻塞设计边界

### AR-OQ-1：假目标鉴别跨域命名双轨

- **现状证据**：反假目标鉴别的判据（同方向多脉冲列）天然属于接收机观测域
  （`ArInterferenceObservation`，含方位与波形），但其消费点在航迹生命周期域
  （`TrackLifecycleManager::PromoteState`，抑制 tentative→confirmed）。当前实现由
  `ArInterferenceObservationResolver` 在波束宽度与接收频率分辨单元内建立连通分量，对
  ≥2 成员的分量逐成员设置 `deception_class=kLikelyFalseTarget` 并生成一条内部
  `ArDeceptionMeasurementCandidate`（per-member 结构性收敛，不二次聚类）；`ArSession` 在
  `CompleteRfCycle` 把干扰观测与候选列表作为 `SignalCycleInput` 的一部分一次性显式传入 pipeline，
  `DeceptionMeasurementGenerator` 逐候选合成带 `classified_as_false_target` 的假目标量测注入
  `track_measurements`（候选关联键由正常位置关联产生，不预分配）。随后由 `PromoteState` 消费
  该标注。

- **已收敛的子问题**：跨域传递曾以 `ArController::SetPreparedInterferenceObservations` 与
  `SignalPipeline::SetNextRfV2DetectionContext` 等 mutable setter 旁路进行，存在调用顺序、
  失败后残留与 observation/cluster 不同步风险。该子问题已收敛为 `SignalCycleInput`
  （`CompleteRfCycle` 单点构造、`RunOnce`/`RunCycle` 显式按值传递），旁路 setter 已全部删除；
  见 `ar_deception_measurement_generator_test`、`ar_signal_pipeline_test`、
  `ar_core_controller_test`。

- **未决问题**：同一"疑似假目标"概念跨域用了两套命名：观测域
  `ArInterferenceObservation.deception_class`（枚举）与量测域
  `RawTrackMeasurement.classified_as_false_target`（bool），跨域阅读增加认知负担。

- **当前边界**：`SignalCycleInput` 是周期输入端口（内部 `ISignalPipeline` API），不构成公开契约；
  内部 `ArDeceptionMeasurementCandidate` 不进入 public result 或 replay，公开观测仍是唯一持久化事实。
  两套命名保留，待跨域标注契约收敛时统一。

### AR-OQ-2：SyncRuntimeTuning 字段同步的手工列表脆弱性（已收敛）

- **现状证据**：`TrackLifecycleManager::SyncRuntimeTuning` 曾用手工逐字段拷贝从 `LifecycleConfig`
  同步阈值到内部 `config_`。该列表当前覆盖 8 个字段中的 7 个，刻意排除 `track_pool_thread_safety_mode`
  （构造期决定、运行期不可变）。反欺骗三个字段（`enable_anti_false_target_discrimination`、
  `enable_anti_vgpo_acceleration_bound`、`max_acceleration_mps2`）曾被遗漏，导致开关无效——本次修复
  才补上。这种"想全量同步、但有一个例外"的手工列表没有编译期保证，每新增一个 `LifecycleConfig`
  字段都必须记得在此加一行，否则成为静默 latent bug。
- **收敛决议**：已把手工逐字段列表替换为整体赋值 `config_ = lifecycle_config`。整体赋值安全的两个前提：
  (1) `track_pool_thread_safety_mode` 进入 `LifecycleConfigSignature`，其变化触发
  `ShouldRebuildLifecycleAssembly` 的重建路径（而非同步路径），故同步路径上其值恒等于 `config_`；
  (2) 生命周期管理器从不读取 `config_.track_pool_thread_safety_mode`，即便被覆盖也无副作用。
  由此未来新增任何可同步 `LifecycleConfig` 字段都会随整体赋值自动覆盖，手工遗漏风险在根因上消除。
  见 `tests/unit/airborne_radar/ar_track_lifecycle_test.cpp::SyncRuntimeTuningConfirmHitsChangesPromotionBehavior`
  （真实生效断言，取代旧的仅 `SUCCEED()` 用例）。
- **未决子问题（更广的控制效果传播闭包）**：本次同时收敛了 directive→profile 映射的单一权威来源
  （`ControlReducer` 的 `IsLpiDirective`/`IsEccmDirective`/`IsValidDirectiveValue` 提升为静态方法，
  `ArController` 不再重复 `==` 链）并新增 `ControlDirectiveType::kCount` 哨兵；但 profile→effect 仍由
  两个消费者分别翻译：`ControlProfileEffects`（内部 detector runtime config）与 `ArSession` 的
  RF 场景构造（对外发布的发射/接收方向图）。二者服务于两个不同物理面，常量（旁瓣 6 vs 12 dB、
  自适应波束 0.60/+2.0dB vs 0.75/6.0dB）有意保持差异，已由
  `ar_core_controller_test.cpp::ExternalAdaptiveBeamformingRaisesNextPhysicalDetectionMargin` 与
  `ar_rf_session_test.cpp::SidelobeCancellerLeavesPublishedEmissionSidelobeUnchanged` 固化现状。
  若未来要统一为单一权威翻译点，需先证明两个物理面应使用相同常量。

## Electronic Surveillance Radar 非阻塞设计边界

### ESR-OQ-1：压制干扰感知与 ECCM 决策链路缺失

- **现状证据**：ECM 模块（`EcmSession`）支持四种干扰技术（瞄准、阻塞、扫频、欺骗），输出
  `EcmCycleResult.emission_frame`（`RfEmissionFrame`）。集成测试
  `multi_model_scenario_test.cpp:1287` 将 ECM 输出直接赋值给 `EsrCycleInput.rf_emissions`，
  ESR 通过 `EsrResolutionCellLedger` 将非最强信号功率作为 `interference_power_w` 叠加到 SNR
  分母，物理层干扰功率计算已完成。但 ESR 存在两条未消费的配置链路：
  (1) `InterceptSuppressionModelConfig`（`suppression_noise_scale` / `suppression_mark_threshold_w`）
  由 `BuildPipelineConfig` 填充到 `InterceptPipelineConfig.suppression_model`，但
  `InterceptDetectionExecutor` 从未读取；
  (2) `EsrEnvironmentSnapshot.spectrum_occupancy_ratio` 注释声称"检测链按 1+9ρ 计算环境噪声倍率"，
  但该计算代码不存在。对比 AR 模块，AR 拥有独立的 `interference` 输入字段、
  `ArInterferenceObservationResolver`（结构化干扰观测输出）、`EccmEvaluator`（8 项 ECCM 措施评分
  与提案）和 `TacticalCoordinator`（闭环决策反馈），而 ESR 无结构化干扰观测输出、无 ECCM 决策引擎、
  无工作模式自适应切换。

- **未决问题**：
  1. 是否激活 `suppression_model` 和 `spectrum_occupancy_ratio` 的消费逻辑，将压制干扰对等效噪声底
     的影响纳入 SNR 和检测门限计算；
  2. 是否定义 `EsrInterferenceObservation` 结构化输出（bearing、频谱、J/N、deception_class），
     为下游消费方提供干扰态势感知；
  3. 是否实现 ESR 特有的 ECCM 决策措施（接收机重调谐、扫描优先级调整、检测门限自适应、工作模式降级、
     欺骗标记与置信度降级），参考 AR 的评分-提案-执行架构但使用被动侦察机的措施集合。

- **当前边界**：ECM 压制干扰在 ESR 接收端仅以通用 RF emission 身份参与分辨单元竞争和 SNR 计算，
  不产生结构化干扰观测输出，不触发 ECCM 反制措施。`suppression_model` 和
  `spectrum_occupancy_ratio` 为死字段，不得在文档中声称 ESR 具备压制干扰感知或自适应抗干扰能力。

- **Stage A 进入条件**：
  1. 先激活死字段消费（修改 `InterceptDetectionExecutor` 噪声计算），并提供 unit test 证明
     `suppression_noise_scale` 和 `spectrum_occupancy_ratio` 的变化可被检测结果观测到；
  2. 定义 `EsrInterferenceObservation` 公开类型并提供 unit test 覆盖；
  3. 实现 ECCM 评分与提案机制，提供集成测试覆盖"ECM 发射 → ESR 感知 → ECCM 反制 → 检测效果变化"
     全链路。

### ESR-OQ-2：运行时补丁扫描中心静默关闭显式扫描边界

- **现状证据**：`EsrRuntimeConfigResolver.cpp` 在应用 `has_scan_center_az_deg` 或
  `has_scan_center_el_deg` 补丁时，会同时设置 `use_explicit_scan_bounds = false`，静默将扫描
  模式从显式边界切换为中心驱动。用户仅调整扫描中心意图不会预期丢失之前配置的四个扫描边界角。
  单独补丁 azimuth center 也会导致 elevation 侧跟着切模式。
- **未决问题**：是否应将模式切换设为显式补丁字段（`has_use_explicit_scan_bounds`），而非由
  scan center 补丁隐含触发；或是否应保留当前行为但增加返回值/日志提示。
- **当前边界**：当前行为为 scan center 补丁隐式关闭显式边界模式。消费方必须知晓此副作用。
- **Stage A 进入条件**：出现真实场景要求"调整扫描中心但保留显式边界模式"，先评估将模式切换
  提取为独立补丁字段的 API 变更成本和向后兼容性。

### ESR-OQ-3：扫描策略跨域耦合（mission.scan + hardware mount 偏移）

- **现状证据**：`EsrScanPolicyConfig`（mission 域）中的 `scan_center_az_deg` 经
  `ApplyScanPolicy` 解算时会减去 `EsrHardwareConfig::antenna_mount_az_deg`（hardware 域）。
  mission 域的值被 hardware 域静默偏移，用户只看 mission 配置无法推断实际扫描方向。
  同理，`scan_start_az_deg` / `scan_end_az_deg` 在 `use_explicit_scan_bounds` 模式下
  也会被 mount 偏移。
- **未决问题**：是否应在公开 API 中将扫描中心语义定义为"天线坐标系"（已含 mount 偏移）或
  "平台坐标系"（需显式减去 mount），或是否应提供查询实际解算扫描几何的 API。
- **当前边界**：扫描配置语义为"天线坐标系"，mount 偏移在内部解算时扣除。文档未明确说明此语义。
- **Stage A 进入条件**：出现因 mount 偏移导致的集成问题或用户误配，先明确公开 API 的坐标系
  语义并在 design.md 中固化，再评估是否需要查询 API。

## SBIRS 非阻塞仿真边界

### SBIRS-OQ-1：诊断距离的物理语义

- **现状证据**：`SbirsDetectionAttributionRecord.estimated_range_m` 明确只属于 cue/诊断层，不代表被动红外
  测距能力；Strict/Estimated 当前使用真值距离，Sensor-like 使用真值距离叠加比例误差。
- **未决问题**：字段名称和三模式取值来源是否足以防止调用方把它误解为正式传感器测距输出。
- **当前边界**：不得进入 `SbirsOutputFrame` raw output；消费方只能把它当作仿真归属与诊断辅助量。
- **Stage A 进入条件**：出现真实下游消费者需要区分 truth-derived、filter-derived 或不可用距离，先盘点消费路径，
  再评估重命名、增加来源枚举或显式有效性字段。

### SBIRS-OQ-2：WFOV、Estimated 与 Sensor-like 的分阶段误差统计

- **现状证据**：三条用途使用独立随机子流，但共同读取 `SbirsErrorModelConfig` 的同一组角度/距离统计参数。
- **未决问题**：是否需要分别表达 WFOV 搜索、Estimated 校正量测和 NFOV Sensor-like 输出的精度等级。
- **当前边界**：共享参数是当前确定性简化，不得宣称代表真实 WFOV/NFOV 载荷精度差异。
- **Stage A 进入条件**：取得可追溯的分阶段参数依据，或构造出共享参数无法满足的 SBIRS 场景验收矩阵后，
  再评估拆分配置；不得仅为形式完整扩大 public API。

### SBIRS-OQ-3：多目标随机样本与 scene 输入顺序

- **现状证据**：WFOV、Estimated、Sensor-like 各自是一条全局用途随机流；同一 trace 可确定性 replay，
  但多目标在同一周期获得哪个样本取决于 `scene` 遍历顺序。
- **未决问题**：SBIRS 是否需要保证目标列表置换后，每个 `target_id` 仍获得相同的量测随机序列。
- **当前边界**：replay 只保证相同输入字节和顺序的确定性，不承诺 scene permutation invariance。
- **Stage A 进入条件**：外部场景源无法稳定排序，或批量验证明确要求按 target 不受输入顺序影响时，比较
  按 target/channel 派生子流与现有全局用途子流的 snapshot、热更和目标生命周期成本。

### SBIRS-OQ-4：Estimated 航迹的真值初始化

- **现状证据**：Estimated 首次捕获后用输入场景真值 ECEF 位置和速度初始化滤波均值，后续才使用带误差角度量测。
- **未决问题**：是否需要改为仅由被动角度 cue 和显式距离/运动先验初始化，以形成无真值航迹起始链。
- **当前边界**：`Estimated` 是生产仿真链，但当前仍包含 truth-seeded track initiation 简化，不得描述为完全
  无真值辅助的真实载荷跟踪器。
- **Stage A 进入条件**：先定义被动角度不可观测距离的初始化先验、收敛时间和失败判据，并提供与当前方案的
  捕获率、位置协方差、丢锁率及 replay 对比证据，再决定是否替换。
