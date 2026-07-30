# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

## 当前状态（2026-07-24 实时代码复核）

原 OQ-1、OQ-3、OQ-8、OQ-9、OQ-10a 至 OQ-10m 均已完成、拒绝或冻结；对应结论和测试证据已迁入
`docs/common/contract.md` 及各模块 `design.md`。ESR runtime validation 边界也已完成并迁入 ESR design。
当前保留一项 Common/Practice 构建边界和四项 SBIRS 非阻塞仿真边界，均不构成已批准实现要求。

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
